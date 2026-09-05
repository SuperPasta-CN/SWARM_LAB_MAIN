"""Unit tests for the differential-drive mapping (legacy port)."""

from __future__ import annotations

import unittest
from math import pi

from swarm.algorithms.differential import DifferentialDriveController
from swarm.domain.config import ExecutionConfig


SPEED_LIMIT = 0.25


def _controller() -> DifferentialDriveController:
    return DifferentialDriveController(ExecutionConfig(mode="diff"), SPEED_LIMIT)


def _step(controller, vx, vy, yaw, measured_vx=0.0, measured_vy=0.0):
    return controller.step(
        target_vx=vx,
        target_vy=vy,
        target_vz=0.0,
        yaw=yaw,
        measured_vx=measured_vx,
        measured_vy=measured_vy,
        measured_vz=0.0,
        dt=0.02,
    )


class DifferentialDriveTests(unittest.TestCase):
    def test_stationary_target_outputs_zero_and_resets(self) -> None:
        controller = _controller()
        values = _step(controller, 0.0, 0.0, 0.3)
        self.assertEqual(values["front_left"], 0.0)
        self.assertEqual(values["front_right"], 0.0)
        self.assertEqual(values["rear_left"], 0.0)
        self.assertEqual(values["rear_right"], 0.0)
        self.assertEqual(values["speed_mps"], 0.0)
        self.assertEqual(values["steer_rad"], 0.0)
        self.assertFalse(controller.speed_pid.initialized)
        self.assertFalse(controller.heading_pid.initialized)

    def test_aligned_forward_command_drives_both_sides_equally(self) -> None:
        controller = _controller()
        values = _step(controller, 0.2, 0.0, 0.0)
        self.assertGreater(values["speed_mps"], 0.0)
        self.assertAlmostEqual(values["front_left"], values["front_right"], places=9)
        self.assertGreater(values["front_left"], 0.0)

    def test_large_heading_error_gates_speed_to_zero(self) -> None:
        controller = _controller()
        # target heading 0 rad, vehicle faces the opposite direction
        values = _step(controller, 0.2, 0.0, pi)
        self.assertEqual(values["speed_mps"], 0.0)
        # pure turn in place: left and right commands have opposite signs
        self.assertGreater(values["front_left"], 0.0)
        self.assertLess(values["front_right"], 0.0)
        self.assertAlmostEqual(values["front_left"], -values["front_right"], places=9)

    def test_partial_heading_error_scales_speed_limit(self) -> None:
        controller = _controller()
        # 45 deg heading error: limit = SPEED_LIMIT * (1 - 0.5) = 0.125
        values = _step(controller, 0.2, 0.0, pi / 4.0)
        self.assertAlmostEqual(values["speed_mps"], SPEED_LIMIT * 0.5, places=9)

    def test_heading_error_wraps_across_pi(self) -> None:
        controller = _controller()
        # target heading 0, yaw = 3*pi/2 wraps to a -pi/2 heading error
        values = _step(controller, 0.2, 0.0, 3.0 * pi / 2.0)
        self.assertAlmostEqual(values["heading_error"], -pi / 2.0, places=9)

    def test_stop_command_is_all_zero(self) -> None:
        values = DifferentialDriveController.stop_command()
        self.assertEqual(values["front_left"], 0.0)
        self.assertEqual(values["front_right"], 0.0)
        self.assertEqual(values["rear_left"], 0.0)
        self.assertEqual(values["rear_right"], 0.0)


if __name__ == "__main__":
    unittest.main()
