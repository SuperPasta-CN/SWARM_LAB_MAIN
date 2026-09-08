"""Ground-vehicle actuator combining control and UDP transport.

The controller maps a planner velocity to wheel commands according to
``ExecutionConfig.mode``: ``"omni"`` (open-loop mecanum, default),
``"omni_pid"`` (mecanum + body-velocity PI feedback) or ``"diff"``
(legacy differential-drive).  All expose the same
``step(...)``/``stop_command()`` surface so the actuator stays agnostic.

The actuator additionally applies the motor dead-zone compensation
(``wheel_command_min_effective``): a nonzero command that is too small to
overcome static friction is lifted to the measured minimum.  Exact zeros
are left untouched so stop commands and the formation deadband still
produce a true standstill.
"""

from __future__ import annotations

from typing import Optional

from swarm.domain.models import ActuationResult, VehicleState, VelocityCommand
from swarm.infrastructure.udp_chassis import ChassisSocketDriver

_WHEEL_KEYS = ("front_left", "front_right", "rear_left", "rear_right")


def apply_min_command(values, min_effective: float):
    """Lift nonzero wheel commands below ``min_effective`` up to it.

    Zero commands stay zero; commands beyond the threshold pass through
    unchanged.  Values are rounded to integers for the datagram format.
    """

    if min_effective <= 0.0:
        return values
    lifted = dict(values)
    for key in _WHEEL_KEYS:
        command = float(lifted[key])
        if 0.0 < abs(command) < min_effective:
            lifted[key] = min_effective if command > 0.0 else -min_effective
    return lifted


class DeadzoneCompensator:
    """Configurable motor dead-zone compensation (oscillation A/B module).

    Modes (``ExecutionConfig.deadzone_mode``):

    - ``"lift"`` (default): legacy constant lift via :func:`apply_min_command`.
      Known issue: quantizes the low-speed region to {0, +-min_eff}, which
      creates a bang-bang limit cycle when the cruise command sits below
      the threshold.
    - ``"affine"``: dead-zone inverse —
      ``out = sign(c) * (min_eff + |c| * (max - min_eff) / max)``.
      Continuous, monotone authority above the threshold; exact zeros stay
      zero, so stop commands and the formation deadband still stand still.
    - ``"pwm"``: when the largest wheel command is below ``min_eff``, the
      whole vector is scaled so its largest entry equals +-min_eff and
      emitted only on a duty fraction ``max|c|/min_eff`` of the carrier
      period — the time average equals the request and the wheel ratios
      (motion direction) are preserved exactly on every ON cycle.
    """

    def __init__(
        self,
        mode: str = "lift",
        min_effective: float = 0.0,
        command_max: float = 100.0,
        pwm_period_cycles: int = 10,
        v_on_mps: Optional[float] = None,
        calib_mps: float = 0.5,
    ) -> None:
        if mode not in ("lift", "affine", "pwm"):
            raise ValueError("deadzone_mode must be 'lift', 'affine' or 'pwm'")
        if pwm_period_cycles < 1:
            raise ValueError("pwm_period_cycles must be >= 1")
        if v_on_mps is not None and v_on_mps <= 0.0:
            raise ValueError("pwm_v_on_mps must be positive when set")
        self.mode = mode
        self.min_effective = min_effective
        self.command_max = command_max
        self.pwm_period_cycles = pwm_period_cycles
        self.v_on_mps = v_on_mps
        self.calib_mps = calib_mps
        self._cycle = 0

    def apply(self, values):
        if self.min_effective <= 0.0:
            return values
        if self.mode == "lift":
            return apply_min_command(values, self.min_effective)
        if self.mode == "affine":
            return self._apply_affine(values)
        return self._apply_pwm(values)

    def _apply_affine(self, values):
        out = dict(values)
        span = self.command_max - self.min_effective
        for key in _WHEEL_KEYS:
            command = float(out[key])
            if command != 0.0:
                magnitude = min(abs(command), self.command_max)
                out[key] = (
                    (self.min_effective + magnitude * span / self.command_max)
                    * (1.0 if command > 0.0 else -1.0)
                )
        return out

    def _apply_pwm(self, values):
        self._cycle = (self._cycle + 1) % self.pwm_period_cycles
        commands = {key: float(values[key]) for key in _WHEEL_KEYS}
        strongest = max(abs(command) for command in commands.values())
        if strongest == 0.0 or strongest >= self.min_effective:
            return values  # exact zeros stay zero; beyond-threshold pass through
        if self.v_on_mps is not None:
            # Per-vehicle normalized duty: the average speed equals the
            # intended speed (strongest * calib/100) regardless of how
            # strong this chassis is at the ON amplitude.
            duty = (strongest * self.calib_mps / 100.0) / self.v_on_mps
        else:
            duty = strongest / self.min_effective
        if duty >= 1.0:
            return values
        on = self._cycle < duty * self.pwm_period_cycles
        if not on:
            out = dict(values)
            for key in _WHEEL_KEYS:
                out[key] = 0.0
            return out
        scale = self.min_effective / strongest
        out = dict(values)
        for key in _WHEEL_KEYS:
            out[key] = commands[key] * scale
        return out


class GroundVehicleActuator:
    """Translate a planner velocity into wheels and deliver it to one chassis."""

    platform = "ground_vehicle"

    def __init__(
        self,
        vehicle_id: str,
        controller,
        driver: ChassisSocketDriver,
        min_command: float = 0.0,
        compensator: Optional[DeadzoneCompensator] = None,
    ) -> None:
        self.vehicle_id = vehicle_id
        self.controller = controller
        self.driver = driver
        self.min_command = min_command
        self.compensator = compensator

    def execute(
        self,
        command: VelocityCommand,
        state: VehicleState,
        dt: float,
    ) -> ActuationResult:
        values = self.controller.step(
            target_vx=command.vx,
            target_vy=command.vy,
            target_vz=command.vz,
            yaw=state.yaw,
            measured_vx=state.vx,
            measured_vy=state.vy,
            measured_vz=state.vz,
            dt=dt,
        )
        if self.compensator is not None:
            values = self.compensator.apply(values)
        else:
            values = apply_min_command(values, self.min_command)
        sent = self.driver.send(values)
        return ActuationResult(self.vehicle_id, self.platform, values, sent)

    def stop(self) -> bool:
        return self.driver.send(self.controller.stop_command())
