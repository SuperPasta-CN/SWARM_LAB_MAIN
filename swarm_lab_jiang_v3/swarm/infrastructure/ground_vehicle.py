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


class GroundVehicleActuator:
    """Translate a planner velocity into wheels and deliver it to one chassis."""

    platform = "ground_vehicle"

    def __init__(
        self,
        vehicle_id: str,
        controller,
        driver: ChassisSocketDriver,
        min_command: float = 0.0,
    ) -> None:
        self.vehicle_id = vehicle_id
        self.controller = controller
        self.driver = driver
        self.min_command = min_command

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
        values = apply_min_command(values, self.min_command)
        sent = self.driver.send(values)
        return ActuationResult(self.vehicle_id, self.platform, values, sent)

    def stop(self) -> bool:
        return self.driver.send(self.controller.stop_command())
