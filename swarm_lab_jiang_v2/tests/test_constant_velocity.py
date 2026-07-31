"""Unit tests for the constant-velocity chassis step planner."""

from __future__ import annotations

import unittest

from swarm.algorithms.constant_velocity import ConstantVelocityPlanner
from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.config import (
    ConstantVelocityConfig,
    ExperimentConfig,
    RuntimeConfig,
    TopologyConfig,
    VehicleConfig,
)
from swarm.domain.models import MocapSnapshot, VehicleState


def _snapshot(*vehicle_ids: str, valid: bool = True) -> MocapSnapshot:
    states = {}
    for vehicle_id in vehicle_ids:
        state = VehicleState(vehicle_id, x=1.0, y=2.0)
        state.valid = valid
        states[vehicle_id] = state
    return MocapSnapshot(123.0, states)


def _experiment_config(
    constant: ConstantVelocityConfig, limit: float = 0.25
) -> ExperimentConfig:
    return ExperimentConfig(
        name="unit",
        algorithm="constant_velocity",
        vehicles=(VehicleConfig("car1", "captain", "ground_vehicle", "10.1.1.81"),),
        topology=TopologyConfig(
            adjacency_matrix=((0,),),
            bearing_matrix=(((0.0, 0.0, 0.0),),),
        ),
        constant_velocity=constant,
        runtime=RuntimeConfig(command_speed_limit_mps=limit),
    )


class ConstantVelocityPlannerTests(unittest.TestCase):
    def test_step_command_during_duration(self) -> None:
        planner = ConstantVelocityPlanner(["car1"], (0.15, 0.0), 6.0, 0.25)
        result = planner.step(_snapshot("car1"), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.15)
        self.assertTrue(command.valid)
        self.assertFalse(result.ready)

    def test_command_zero_after_duration(self) -> None:
        planner = ConstantVelocityPlanner(["car1"], (0.15, 0.0), 0.1, 0.25)
        planner.step(_snapshot("car1"), 0.05)
        planner.step(_snapshot("car1"), 0.05)  # elapsed = 0.10, still active
        result = planner.step(_snapshot("car1"), 0.05)  # elapsed = 0.15 > 0.1
        self.assertEqual(result.commands["car1"].vx, 0.0)
        self.assertEqual(result.commands["car1"].vy, 0.0)
        self.assertTrue(result.ready)

    def test_invalid_state_zeroes_command(self) -> None:
        planner = ConstantVelocityPlanner(["car1"], (0.15, 0.0), 6.0, 0.25)
        result = planner.step(_snapshot("car1", valid=False), 0.02)
        command = result.commands["car1"]
        self.assertEqual(command.vx, 0.0)
        self.assertFalse(command.valid)

    def test_missing_state_zeroes_command(self) -> None:
        planner = ConstantVelocityPlanner(["car1"], (0.15, 0.0), 6.0, 0.25)
        result = planner.step(MocapSnapshot(123.0, {}), 0.02)
        self.assertEqual(result.commands["car1"].vx, 0.0)

    def test_trivial_topology_supports_preflight(self) -> None:
        planner = ConstantVelocityPlanner(["car1", "car2"], (0.1, 0.0), 6.0, 0.25)
        self.assertEqual(planner.topology.vehicle_ids, ["car1", "car2"])
        self.assertEqual(planner.topology.adjacency_matrix, [[0, 0], [0, 0]])
        self.assertEqual(planner.topology.roles, ["crew", "crew"])

    def test_over_speed_step_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ConstantVelocityPlanner(["car1"], (0.3, 0.0), 6.0, 0.25)

    def test_non_positive_duration_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ConstantVelocityPlanner(["car1"], (0.1, 0.0), 0.0, 0.25)

    def test_reset_restarts_the_step(self) -> None:
        planner = ConstantVelocityPlanner(["car1"], (0.15, 0.0), 0.05, 0.25)
        planner.step(_snapshot("car1"), 0.1)
        planner.reset()
        result = planner.step(_snapshot("car1"), 0.02)
        self.assertAlmostEqual(result.commands["car1"].vx, 0.15)


class ConstantVelocityConfigTests(unittest.TestCase):
    def test_factory_builds_planner(self) -> None:
        planner = build_swarm_planner(
            _experiment_config(ConstantVelocityConfig(0.15, 0.0, 6.0))
        )
        self.assertIsInstance(planner, ConstantVelocityPlanner)

    def test_config_validates(self) -> None:
        _experiment_config(ConstantVelocityConfig(0.15, 0.0, 6.0)).validate()

    def test_over_limit_step_rejected_by_validate(self) -> None:
        config = _experiment_config(ConstantVelocityConfig(0.3, 0.0, 6.0))
        with self.assertRaises(ValueError):
            config.validate()

    def test_non_positive_duration_rejected_by_validate(self) -> None:
        config = _experiment_config(ConstantVelocityConfig(0.15, 0.0, 0.0))
        with self.assertRaises(ValueError):
            config.validate()


if __name__ == "__main__":
    unittest.main()
