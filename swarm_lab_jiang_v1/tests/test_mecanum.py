"""Unit tests for the mecanum holonomic velocity-to-wheel mapping."""

from __future__ import annotations

import unittest
from math import pi

from swarm.algorithms.mecanum import MecanumController, wrap_to_pi
from swarm.domain.config import ExecutionConfig


def _step(controller, vx, vy, yaw):
    return controller.step(
        target_vx=vx,
        target_vy=vy,
        target_vz=0.0,
        yaw=yaw,
        measured_vx=0.0,
        measured_vy=0.0,
        measured_vz=0.0,
        dt=0.02,
    )


def _wheels(values):
    return (
        values["front_left"],
        values["front_right"],
        values["rear_left"],
        values["rear_right"],
    )


class MecanumKinematicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = MecanumController(ExecutionConfig())

    def test_forward_at_zero_yaw_turns_all_wheels_forward_equally(self) -> None:
        values = _step(self.controller, 0.2, 0.0, 0.0)
        wheels = _wheels(values)
        for wheel in wheels:
            self.assertGreater(wheel, 0.0)
        self.assertAlmostEqual(wheels[0], wheels[1], places=9)
        self.assertAlmostEqual(wheels[0], wheels[2], places=9)
        self.assertAlmostEqual(wheels[0], wheels[3], places=9)

    def test_backward_at_zero_yaw_turns_all_wheels_backward(self) -> None:
        wheels = _wheels(_step(self.controller, -0.2, 0.0, 0.0))
        for wheel in wheels:
            self.assertLess(wheel, 0.0)

    def test_strafe_left_at_zero_yaw_x_pattern(self) -> None:
        # body +y translation: FL/RR backward, FR/RL forward
        wheels = _wheels(_step(self.controller, 0.0, 0.2, 0.0))
        fl, fr, rl, rr = wheels
        self.assertLess(fl, 0.0)
        self.assertGreater(fr, 0.0)
        self.assertGreater(rl, 0.0)
        self.assertLess(rr, 0.0)
        self.assertAlmostEqual(abs(fl), abs(fr), places=9)
        self.assertAlmostEqual(abs(rl), abs(rr), places=9)

    def test_strafe_right_at_zero_yaw_x_pattern(self) -> None:
        wheels = _wheels(_step(self.controller, 0.0, -0.2, 0.0))
        fl, fr, rl, rr = wheels
        self.assertGreater(fl, 0.0)
        self.assertLess(fr, 0.0)
        self.assertLess(rl, 0.0)
        self.assertGreater(rr, 0.0)

    def test_world_to_body_rotation_ninety_degrees(self) -> None:
        # Vehicle faces world +y; a world +x command is a body -y strafe.
        values = _step(self.controller, 0.2, 0.0, pi / 2.0)
        self.assertAlmostEqual(values["vx_body"], 0.0, places=9)
        self.assertAlmostEqual(values["vy_body"], -0.2, places=9)
        fl, fr, rl, rr = _wheels(values)
        self.assertGreater(fl, 0.0)
        self.assertLess(fr, 0.0)
        self.assertLess(rl, 0.0)
        self.assertGreater(rr, 0.0)

    def test_world_to_body_rotation_half_pi_yaw_gives_body_forward(self) -> None:
        # Vehicle faces world +y (yaw=pi/2); world +y is body +x.
        values = _step(self.controller, 0.0, 0.2, pi / 2.0)
        self.assertAlmostEqual(values["vx_body"], 0.2, places=9)
        self.assertAlmostEqual(values["vy_body"], 0.0, places=9)


class MecanumHeadingHoldTests(unittest.TestCase):
    def test_heading_hold_spins_to_recover_yaw(self) -> None:
        controller = MecanumController(ExecutionConfig())
        _step(controller, 0.0, 0.0, 0.0)  # captures yaw_target = 0.0
        values = _step(controller, 0.0, 0.0, 0.3)
        # yaw overshot by +0.3 rad: correct with clockwise spin (omega < 0),
        # left wheels forward, right wheels backward.
        self.assertLess(values["omega"], 0.0)
        fl, fr, rl, rr = _wheels(values)
        self.assertGreater(fl, 0.0)
        self.assertLess(fr, 0.0)
        self.assertGreater(rl, 0.0)
        self.assertLess(rr, 0.0)

    def test_heading_hold_counter_clockwise_correction(self) -> None:
        controller = MecanumController(ExecutionConfig())
        _step(controller, 0.0, 0.0, 0.0)
        values = _step(controller, 0.0, 0.0, -0.3)
        self.assertGreater(values["omega"], 0.0)
        fl, fr, rl, rr = _wheels(values)
        self.assertLess(fl, 0.0)
        self.assertGreater(fr, 0.0)
        self.assertLess(rl, 0.0)
        self.assertGreater(rr, 0.0)

    def test_heading_hold_disabled_outputs_no_omega(self) -> None:
        controller = MecanumController(ExecutionConfig(heading_hold=False))
        _step(controller, 0.0, 0.0, 0.0)
        values = _step(controller, 0.0, 0.0, 0.5)
        self.assertEqual(values["omega"], 0.0)
        self.assertEqual(_wheels(values), (0.0, 0.0, 0.0, 0.0))

    def test_omega_is_clamped(self) -> None:
        controller = MecanumController(ExecutionConfig())
        _step(controller, 0.0, 0.0, 0.0)
        values = _step(controller, 0.0, 0.0, 3.0)
        self.assertAlmostEqual(values["omega"], -ExecutionConfig().omega_max, places=9)


class MecanumScalingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = MecanumController(ExecutionConfig())

    def test_overspeed_wheels_rescale_proportionally(self) -> None:
        values = _step(self.controller, 10.0, 5.0, 0.0)
        wheels = _wheels(values)
        fastest = max(abs(wheel) for wheel in wheels)
        self.assertAlmostEqual(fastest, 100.0, places=6)
        # direction preserved: wheel ratio keeps the kinematic pattern
        raw = (10.0 - 5.0, 10.0 + 5.0, 10.0 + 5.0, 10.0 - 5.0)
        for wheel, expected in zip(wheels, raw):
            self.assertAlmostEqual(wheel / 100.0, expected / max(raw), places=6)

    def test_commands_clamped_to_configured_range(self) -> None:
        controller = MecanumController(
            ExecutionConfig(wheel_command_min=-50.0, wheel_command_max=50.0)
        )
        values = _step(controller, 10.0, 0.0, 0.0)
        for wheel in _wheels(values):
            self.assertLessEqual(wheel, 50.0)
            self.assertGreaterEqual(wheel, -50.0)

    def test_wheel_flip_flips_command_sign(self) -> None:
        controller = MecanumController(ExecutionConfig(wheel_flip=(-1.0, 1.0, 1.0, 1.0)))
        values = _step(controller, 0.2, 0.0, 0.0)
        fl, fr, rl, rr = _wheels(values)
        self.assertLess(fl, 0.0)
        self.assertGreater(fr, 0.0)
        self.assertGreater(rl, 0.0)
        self.assertGreater(rr, 0.0)

    def test_stop_command_is_all_zero(self) -> None:
        values = MecanumController.stop_command()
        self.assertEqual(_wheels(values), (0.0, 0.0, 0.0, 0.0))

    def test_wrap_to_pi(self) -> None:
        self.assertAlmostEqual(wrap_to_pi(3.0 * pi), pi, places=9)
        self.assertAlmostEqual(wrap_to_pi(-3.0 * pi), -pi, places=9)
        self.assertAlmostEqual(wrap_to_pi(0.5), 0.5, places=9)


if __name__ == "__main__":
    unittest.main()
