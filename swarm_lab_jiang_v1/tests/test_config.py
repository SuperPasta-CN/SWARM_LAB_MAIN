"""Configuration loading and validation tests."""

from __future__ import annotations

import unittest
from dataclasses import replace

from swarm.application.planner_factory import available_algorithms, build_swarm_planner
from swarm.domain.config import (
    BearingControlConfig,
    ExecutionConfig,
    ExperimentConfig,
    PreflightConfig,
    RuntimeConfig,
    TopologyConfig,
    VehicleConfig,
)
from configs import bearing_four, bearing_six, bearing_three, bearing_two


class ExperimentConfigCatalogTests(unittest.TestCase):
    def test_all_catalog_configs_validate(self) -> None:
        for module in (bearing_two, bearing_three, bearing_four, bearing_six):
            with self.subTest(config=module.CONFIG.name):
                module.CONFIG.validate()
                self.assertEqual(module.CONFIG.algorithm, "bearing")

    def test_catalog_vehicle_addresses(self) -> None:
        self.assertEqual(
            {v.vehicle_id: v.address for v in bearing_two.CONFIG.vehicles},
            {"car1": "10.1.1.81", "car2": "10.1.1.82"},
        )
        self.assertEqual(
            {v.vehicle_id: v.address for v in bearing_four.CONFIG.vehicles},
            {
                "car1": "10.1.1.81",
                "car2": "10.1.1.82",
                "car3": "10.1.1.83",
                "car4": "10.1.1.84",
            },
        )
        self.assertEqual(
            {v.vehicle_id: v.address for v in bearing_six.CONFIG.vehicles},
            {"car%d" % i: "10.1.1.%d" % (80 + i) for i in range(1, 7)},
        )

    def test_bearing_four_topology_ported_unchanged(self) -> None:
        topology = bearing_four.CONFIG.topology
        self.assertEqual(
            tuple(tuple(row) for row in topology.adjacency_matrix),
            (
                (0, 1, 1, 1),
                (1, 0, 0, 1),
                (1, 0, 0, 1),
                (1, 1, 1, 0),
            ),
        )
        self.assertEqual(tuple(topology.leader_mask), (True, True, False, False))
        self.assertEqual(topology.bearing_matrix[0][1], (1.0, 0.0, 0.0))
        self.assertEqual(topology.bearing_matrix[0][2], (0.0, -1.0, 0.0))
        self.assertEqual(topology.bearing_matrix[3][0], (-1.0, 1.0, 0.0))

    def test_leader_masks(self) -> None:
        self.assertEqual(tuple(bearing_two.CONFIG.topology.leader_mask), (True, False))
        self.assertEqual(tuple(bearing_three.CONFIG.topology.leader_mask), (True, False, False))
        self.assertEqual(
            tuple(bearing_six.CONFIG.topology.leader_mask),
            (True, True, False, False, False, True),
        )


class DefaultConfigTests(unittest.TestCase):
    def test_new_defaults(self) -> None:
        bearing = BearingControlConfig()
        self.assertEqual(bearing.kp, 0.6)
        self.assertEqual(bearing.deadband_mps, 0.015)
        self.assertFalse(bearing.bearing_3d)
        self.assertEqual(bearing.agent_mode, "all_bearing")
        execution = ExecutionConfig()
        self.assertEqual(execution.mode, "omni")
        self.assertTrue(execution.heading_hold)
        self.assertEqual(execution.mecanum_l_m, 0.10)
        self.assertEqual(execution.max_wheel_speed_mps, 0.3)
        self.assertEqual(execution.wheel_command_min_effective, 0.0)
        self.assertEqual(execution.wheel_flip, (1.0, 1.0, 1.0, 1.0))
        runtime = RuntimeConfig()
        self.assertFalse(runtime.require_twist)
        self.assertFalse(runtime.stop_on_converge)
        self.assertEqual(runtime.converge_eps_rad, 0.05)
        preflight = PreflightConfig()
        self.assertEqual(preflight.anchor_tolerance_deg, 20.0)
        self.assertEqual(preflight.min_separation_m, 0.20)
        self.assertEqual(preflight.mocap_timeout_s, 10.0)
        self.assertTrue(preflight.require_confirmation)


def _minimal_config() -> ExperimentConfig:
    return ExperimentConfig(
        name="unit",
        algorithm="bearing",
        vehicles=(VehicleConfig("car1", "leader", "ground_vehicle", "10.1.1.81"),),
        topology=TopologyConfig(
            adjacency_matrix=((0,),),
            bearing_matrix=(((0.0, 0.0, 0.0),),),
            leader_mask=(True,),
        ),
        leader_target_velocities={"car1": (0.0, 0.0, 0.0)},
    )


class ValidationTests(unittest.TestCase):
    def test_minimal_config_validates(self) -> None:
        _minimal_config().validate()

    def test_invalid_execution_mode_rejected(self) -> None:
        config = replace(_minimal_config(), execution=ExecutionConfig(mode="hover"))
        with self.assertRaises(ValueError):
            config.validate()

    def test_non_positive_kp_rejected(self) -> None:
        config = replace(
            _minimal_config(), bearing_control=BearingControlConfig(kp=0.0)
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_negative_deadband_rejected(self) -> None:
        config = replace(
            _minimal_config(), bearing_control=BearingControlConfig(deadband_mps=-0.1)
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_bad_wheel_command_range_rejected(self) -> None:
        config = replace(
            _minimal_config(),
            execution=ExecutionConfig(wheel_command_min=100.0, wheel_command_max=-100.0),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_bad_wheel_flip_rejected(self) -> None:
        config = replace(
            _minimal_config(), execution=ExecutionConfig(wheel_flip=(1.0, 1.0, 1.0))
        )
        with self.assertRaises(ValueError):
            config.validate()
        config = replace(
            _minimal_config(),
            execution=ExecutionConfig(wheel_flip=(1.0, 1.0, 1.0, 0.5)),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_non_positive_mecanum_lever_rejected(self) -> None:
        config = replace(
            _minimal_config(), execution=ExecutionConfig(mecanum_l_m=0.0)
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_negative_min_separation_rejected(self) -> None:
        config = replace(
            _minimal_config(), preflight=PreflightConfig(min_separation_m=-0.1)
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_non_positive_anchor_tolerance_rejected(self) -> None:
        config = replace(
            _minimal_config(), preflight=PreflightConfig(anchor_tolerance_deg=0.0)
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_invalid_min_effective_command_rejected(self) -> None:
        for bad in (-1.0, 100.0, 150.0):
            config = replace(
                _minimal_config(),
                execution=ExecutionConfig(wheel_command_min_effective=bad),
            )
            with self.subTest(value=bad), self.assertRaises(ValueError):
                config.validate()

    def test_missing_leader_velocity_rejected(self) -> None:
        config = replace(
            _minimal_config(),
            leader_target_velocities={},
            bearing_control=BearingControlConfig(agent_mode="leader_velocity"),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_all_bearing_allows_missing_leader_velocities(self) -> None:
        config = replace(_minimal_config(), leader_target_velocities={})
        config.validate()  # all_bearing (default) never reads the velocity map

    def test_invalid_agent_mode_rejected(self) -> None:
        config = replace(
            _minimal_config(),
            bearing_control=BearingControlConfig(agent_mode="hover"),
        )
        with self.assertRaises(ValueError):
            config.validate()


class PlannerFactoryTests(unittest.TestCase):
    def test_bearing_algorithm_is_available(self) -> None:
        self.assertIn("bearing", available_algorithms())

    def test_unknown_algorithm_rejected(self) -> None:
        config = replace(
            _minimal_config(), algorithm="boids"
        )
        with self.assertRaises(ValueError):
            build_swarm_planner(config)

    def test_builds_controller_for_catalog_configs(self) -> None:
        for module in (bearing_two, bearing_three, bearing_four, bearing_six):
            planner = build_swarm_planner(module.CONFIG)
            self.assertEqual(len(planner.topology.vehicle_ids), len(module.CONFIG.vehicles))


if __name__ == "__main__":
    unittest.main()
