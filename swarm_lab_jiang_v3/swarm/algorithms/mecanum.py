"""Mecanum holonomic mapping: world velocity to four wheel commands.

The chassis is an X-pattern four-mecanum drive with wheel order
``FL, FR, RL, RR``::

    FL = vx_b - vy_b - omega * L      FR = vx_b + vy_b + omega * L
    RL = vx_b + vy_b - omega * L      RR = vx_b - vy_b + omega * L

where ``(vx_b, vy_b) = R(-yaw) @ (vx, vy)`` is the body-frame command and
``L = lx + ly`` the wheel-lever sum.  ``omega`` comes from a heading-hold
P controller so the mocap marker orientation stays fixed during translation.
If the wiring of one vehicle is mirrored, flip its wheels via
``ExecutionConfig.wheel_flip`` instead of touching these signs.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin, sqrt
from typing import Dict, Optional, Tuple

from swarm.domain.config import ExecutionConfig


def wrap_to_pi(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


@dataclass
class MecanumControllerState:
    """Observable state retained by one mecanum controller."""

    vx_body: float = 0.0
    vy_body: float = 0.0
    omega: float = 0.0
    yaw_target: Optional[float] = None
    wheel_speeds_mps: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


class MecanumController:
    """Convert a world-frame target velocity into four wheel commands."""

    def __init__(self, config: ExecutionConfig) -> None:
        self.config = config
        self.state = MecanumControllerState()

    def reset(self) -> None:
        self.state = MecanumControllerState()

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
        """Run one mecanum control update (feedback unused, P-law is open loop)."""

        _ = target_vz, measured_vx, measured_vy, measured_vz, dt
        vx_b = cos(yaw) * target_vx + sin(yaw) * target_vy
        vy_b = -sin(yaw) * target_vx + cos(yaw) * target_vy
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
            "measured_speed": 0.0,
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
