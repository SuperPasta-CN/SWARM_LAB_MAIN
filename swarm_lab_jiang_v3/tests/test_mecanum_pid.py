"""Unit tests for the closed-loop (PI) mecanum velocity mapping."""

from __future__ import annotations

import unittest
from math import hypot

from swarm.algorithms.mecanum import MecanumController
from swarm.algorithms.mecanum_pid import MecanumPidController
from swarm.domain.config import (
    ExecutionConfig,
    ExperimentConfig,
    PIDConfig,
    TopologyConfig,
    VehicleConfig,
)


def _step(controller, vx, vy, yaw, meas_vx=0.0, meas_vy=0.0):
    return controller.step(
        target_vx=vx,
        target_vy=vy,
        target_vz=0.0,
        yaw=yaw,
        measured_vx=meas_vx,
        measured_vy=meas_vy,
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


def _minimal_config(execution: ExecutionConfig) -> ExperimentConfig:
    return ExperimentConfig(
        name="unit",
        algorithm="task_driven",
        vehicles=(VehicleConfig("car1", "ground_vehicle", "10.1.1.81"),),
        topology=TopologyConfig(
            adjacency_matrix=((0,),),
            formation={"car1": (0.0, 0.0)},
        ),
        execution=execution,
    )


class OpenLoopParityTests(unittest.TestCase):
    """With measured == target the PI correction is zero: open-loop twin."""

    def test_matching_feedback_matches_open_loop(self) -> None:
        config = ExecutionConfig(heading_hold=False)
        open_loop = MecanumController(config)
        closed_loop = MecanumPidController(config)
        expected = _step(open_loop, 0.2, 0.1, 0.3)
        actual = _step(closed_loop, 0.2, 0.1, 0.3, meas_vx=0.2, meas_vy=0.1)
        for key in (
            "front_left",
            "front_right",
            "rear_left",
            "rear_right",
            "vx_body",
            "vy_body",
        ):
            self.assertAlmostEqual(actual[key], expected[key], places=9)


class FeedbackCorrectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ExecutionConfig(heading_hold=False)

    def test_lagging_velocity_increases_command(self) -> None:
        open_loop = MecanumController(self.config)
        closed_loop = MecanumPidController(self.config)
        base = _step(open_loop, 0.2, 0.0, 0.0)
        corrected = _step(closed_loop, 0.2, 0.0, 0.0, meas_vx=0.1)
        self.assertGreater(corrected["vx_body"], base["vx_body"])
        for wheel, base_wheel in zip(_wheels(corrected), _wheels(base)):
            self.assertGreater(wheel, base_wheel)

    def test_overshooting_velocity_decreases_command(self) -> None:
        closed_loop = MecanumPidController(self.config)
        corrected = _step(closed_loop, 0.2, 0.0, 0.0, meas_vx=0.25)
        self.assertLess(corrected["vx_body"], 0.2)

    def test_lateral_feedback_acts_on_lateral_axis(self) -> None:
        closed_loop = MecanumPidController(self.config)
        corrected = _step(closed_loop, 0.0, 0.2, 0.0, meas_vy=0.1)
        self.assertGreater(corrected["vy_body"], 0.2)
        self.assertAlmostEqual(corrected["vx_body"], 0.0, places=9)

    def test_correction_is_bounded_by_output_limit(self) -> None:
        closed_loop = MecanumPidController(self.config)
        values = None
        for _ in range(2000):
            values = _step(closed_loop, 0.2, 0.0, 0.0, meas_vx=0.0)
        limit = self.config.omni_velocity_pid.output_limit
        self.assertLessEqual(values["vx_body"], 0.2 + limit + 1e-9)

    def test_zero_target_outputs_exact_zeros_despite_drift(self) -> None:
        closed_loop = MecanumPidController(self.config)
        _step(closed_loop, 0.2, 0.0, 0.0, meas_vx=0.0)  # wind up the integrator
        values = _step(closed_loop, 0.0, 0.0, 0.0, meas_vx=0.1, meas_vy=-0.05)
        self.assertEqual(_wheels(values), (0.0, 0.0, 0.0, 0.0))

    def test_measured_speed_is_reported(self) -> None:
        closed_loop = MecanumPidController(self.config)
        values = _step(closed_loop, 0.2, 0.0, 0.0, meas_vx=0.12, meas_vy=0.05)
        self.assertAlmostEqual(values["measured_speed"], hypot(0.12, 0.05), places=9)


class HeadingHoldTests(unittest.TestCase):
    def test_heading_hold_matches_open_loop_behaviour(self) -> None:
        closed_loop = MecanumPidController(ExecutionConfig())
        _step(closed_loop, 0.0, 0.0, 0.0)  # captures yaw_target = 0.0
        values = _step(closed_loop, 0.0, 0.0, 0.3)
        self.assertLess(values["omega"], 0.0)
        fl, fr, rl, rr = _wheels(values)
        self.assertGreater(fl, 0.0)
        self.assertLess(fr, 0.0)


class ResetTests(unittest.TestCase):
    def test_reset_clears_integrators_and_yaw_target(self) -> None:
        closed_loop = MecanumPidController(ExecutionConfig())
        _step(closed_loop, 0.2, 0.0, 0.1, meas_vx=0.0)
        closed_loop.reset()
        self.assertIsNone(closed_loop.state.yaw_target)
        self.assertEqual(closed_loop.vx_pid.integral, 0.0)
        self.assertFalse(closed_loop.vx_pid.initialized)

    def test_stop_command_is_all_zero(self) -> None:
        self.assertEqual(
            _wheels(MecanumPidController.stop_command()), (0.0, 0.0, 0.0, 0.0)
        )


class OmniPidConfigTests(unittest.TestCase):
    def test_omni_pid_mode_validates(self) -> None:
        _minimal_config(ExecutionConfig(mode="omni_pid")).validate()

    def test_negative_gain_rejected(self) -> None:
        config = _minimal_config(
            ExecutionConfig(
                mode="omni_pid",
                omni_velocity_pid=PIDConfig(-0.1, 1.0, 0.0, 0.1, 0.15),
            )
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_non_positive_output_limit_rejected(self) -> None:
        config = _minimal_config(
            ExecutionConfig(
                mode="omni_pid",
                omni_velocity_pid=PIDConfig(0.6, 1.2, 0.0, 0.1, 0.0),
            )
        )
        with self.assertRaises(ValueError):
            config.validate()


class HeadingTargetConfigTests(unittest.TestCase):
    def test_configured_heading_target_overrides_capture(self) -> None:
        controller = MecanumPidController(ExecutionConfig(heading_target_rad=0.0))
        values = _step(controller, 0.0, 0.0, 1.6)  # parked facing +y, target +x
        self.assertEqual(values["yaw_target"], 0.0)
        self.assertLess(values["omega"], 0.0)  # rotate clockwise: yaw 1.6 -> 0

    def test_default_none_keeps_capture_behaviour(self) -> None:
        controller = MecanumPidController(ExecutionConfig())
        _step(controller, 0.0, 0.0, 1.6)  # captures yaw_target = 1.6
        values = _step(controller, 0.0, 0.0, 1.6)
        self.assertAlmostEqual(values["yaw_target"], 1.6, places=9)
        self.assertEqual(values["omega"], 0.0)


if __name__ == "__main__":
    unittest.main()
