"""Mecanum holonomic mapping with closed-loop body-velocity PI feedback.

Kinematics identical to the open-loop variant (mecanum.py); only the
body-frame command generation differs::

    u_b = v*_b + PI(v*_b - v_meas_b)

feedforward plus a PI correction on the mocap-estimated body velocity
(position finite difference + EMA, so keep kd = 0).  Friction and
wheel-calibration errors that persist as steady-state residuals in
open-loop mode are integrated away.  Heading hold (P on yaw), the
X-pattern inverse kinematics, wheel flip, and the proportional rescale
are unchanged.

A zero target bypasses feedback and outputs exact zeros, so stop commands
and the formation deadband still produce a true standstill (the motor
dead-zone compensation lifts any small nonzero command).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, sin, sqrt
from typing import Dict, Optional, Tuple

from swarm.algorithms.mecanum import wrap_to_pi
from swarm.algorithms.pid import PID
from swarm.domain.config import ExecutionConfig


@dataclass
class MecanumPidControllerState:
    """Observable state retained by one closed-loop mecanum controller."""

    vx_body: float = 0.0
    vy_body: float = 0.0
    omega: float = 0.0
    yaw_target: Optional[float] = None
    wheel_speeds_mps: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


class MecanumPidController:
    """Mecanum mapping whose body velocity is PI-corrected by mocap feedback."""

    def __init__(self, config: ExecutionConfig) -> None:
        self.config = config
        self.vx_pid = PID(**vars(config.omni_velocity_pid))
        self.vy_pid = PID(**vars(config.omni_velocity_pid))
        self.state = MecanumPidControllerState()

    def reset(self) -> None:
        self.vx_pid.reset()
        self.vy_pid.reset()
        self.state = MecanumPidControllerState()

    def _heading_omega(self, yaw: float) -> float:
        if not self.config.heading_hold:
            self.state.yaw_target = None
            return 0.0
        if self.config.heading_target_rad is not None:
            # Configured world-frame heading: rotate to face it after unlock.
            self.state.yaw_target = self.config.heading_target_rad
        elif self.state.yaw_target is None:
            # Captured when the first command arrives (experiment unlock).
            self.state.yaw_target = yaw
        error = wrap_to_pi(self.state.yaw_target - yaw)
        # Heading deadband: suppress micro-corrections that would otherwise
        # be lifted by the dead-zone compensation into a rotation limit cycle.
        if abs(error) < self.config.heading_deadband_rad:
            return 0.0
        omega = self.config.k_omega * error
        return max(-self.config.omega_max, min(self.config.omega_max, omega))

    def step(
        self,
        target_vx: float,
        target_vy: float,
        target_vz: float,
        yaw: float,
        measured_vx: float,
        measured_vy: float,
        measured_vz: float,
        dt: float,
    ) -> Dict[str, float]:
        """Run one closed-loop update: PI correction, then the usual mapping."""

        _ = target_vz, measured_vz
        # Feedforward: world-frame target rotated into the body frame.
        vx_ff = cos(yaw) * target_vx + sin(yaw) * target_vy
        vy_ff = -sin(yaw) * target_vx + cos(yaw) * target_vy

        if hypot(target_vx, target_vy) < 1e-9:
            # Standstill: exact zeros survive the dead-zone lift untouched.
            self.vx_pid.reset()
            self.vy_pid.reset()
            vx_b = 0.0
            vy_b = 0.0
        else:
            # Feedback: measured world velocity rotated the same way.
            vx_meas = cos(yaw) * measured_vx + sin(yaw) * measured_vy
            vy_meas = -sin(yaw) * measured_vx + cos(yaw) * measured_vy
            vx_b = vx_ff + self.vx_pid.update(vx_ff, vx_meas, dt)
            vy_b = vy_ff + self.vy_pid.update(vy_ff, vy_meas, dt)

        omega = self._heading_omega(yaw)
        self.state.vx_body = vx_b
        self.state.vy_body = vy_b
        self.state.omega = omega

        lever = self.config.mecanum_l_m
        spin = omega * lever
        wheel_speeds = (
            vx_b - vy_b - spin,  # front_left
            vx_b + vy_b + spin,  # front_right
            vx_b + vy_b - spin,  # rear_left
            vx_b - vy_b + spin,  # rear_right
        )
        flip = self.config.wheel_flip
        wheel_speeds = tuple(
            wheel_speeds[i] * float(flip[i]) for i in range(4)
        )  # type: ignore[assignment]
        self.state.wheel_speeds_mps = wheel_speeds

        limit = self.config.max_wheel_speed_mps
        fastest = max(abs(speed) for speed in wheel_speeds)
        scale = 1.0
        if fastest > limit:
            # Rescale all wheels equally so the motion direction is preserved.
            scale = limit / fastest

        span = self.config.wheel_command_max
        command_min = self.config.wheel_command_min
        command_max = self.config.wheel_command_max
        commands = [
            max(command_min, min(command_max, speed * scale / limit * span))
            for speed in wheel_speeds
        ]
        return {
            "front_left": commands[0],
            "front_right": commands[1],
            "rear_left": commands[2],
            "rear_right": commands[3],
            "vx_body": vx_b,
            "vy_body": vy_b,
            "omega": omega,
            "yaw_target": self.state.yaw_target if self.state.yaw_target is not None else yaw,
            "target_speed": sqrt(target_vx * target_vx + target_vy * target_vy),
            "target_heading": self.state.yaw_target
            if self.state.yaw_target is not None
            else yaw,
            "measured_speed": hypot(measured_vx, measured_vy),
        }

    @staticmethod
    def stop_command() -> Dict[str, float]:
        return {
            "front_left": 0.0,
            "front_right": 0.0,
            "rear_left": 0.0,
            "rear_right": 0.0,
            "vx_body": 0.0,
            "vy_body": 0.0,
            "omega": 0.0,
            "yaw_target": 0.0,
            "target_speed": 0.0,
            "target_heading": 0.0,
            "measured_speed": 0.0,
        }
