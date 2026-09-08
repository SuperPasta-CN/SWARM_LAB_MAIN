"""Configuration loading and validation tests (v3)."""

from __future__ import annotations

import unittest
import warnings
from dataclasses import replace

from swarm.application.planner_factory import available_algorithms, build_swarm_planner
from swarm.domain.config import (
    ExecutionConfig,
    ExperimentConfig,
    PreflightConfig,
    RuntimeConfig,
    TaskDrivenConfig,
    TopologyConfig,
    VehicleConfig,
)
from configs import bearing_two, task_four, task_three, task_two


class CatalogConfigTests(unittest.TestCase):
    def test_all_catalog_configs_validate(self) -> None:
        for module in (task_two, task_three, task_four, bearing_two):
            with self.subTest(config=module.CONFIG.name):
                module.CONFIG.validate()

    def test_task_configs_formation_keys_match_vehicles(self) -> None:
        for module in (task_two, task_three, task_four):
            with self.subTest(config=module.CONFIG.name):
                self.assertEqual(
                    set(module.CONFIG.topology.formation),
                    set(module.CONFIG.vehicle_ids),
                )
                # 二阶段默认：先收敛后机动
                self.assertTrue(module.CONFIG.control.converge_first)

    def test_task_two_vehicles(self) -> None:
        self.assertEqual(
            {v.vehicle_id: v.address for v in task_two.CONFIG.vehicles},
            {"car4": "10.1.1.84", "car5": "10.1.1.85"},
        )

    def test_fleet_sizes(self) -> None:
        self.assertEqual(len(task_three.CONFIG.vehicle_ids), 3)
        self.assertEqual(len(task_four.CONFIG.vehicle_ids), 4)

    def test_bearing_two_roles(self) -> None:
        roles = [v.role for v in bearing_two.CONFIG.vehicles]
        self.assertEqual(roles, ["captain", "first_mate"])
        self.assertIsNone(task_two.CONFIG.vehicles[0].role)


class BearingConfigValidationTests(unittest.TestCase):
    """The comparison bearing planner keeps v2's role validation rules."""

    def _bearing_config(self, **kwargs):
        return replace(bearing_two.CONFIG, **kwargs)

    def test_no_captain_rejected(self) -> None:
        vehicles = tuple(
            replace(v, role="crew") for v in bearing_two.CONFIG.vehicles
        )
        with self.assertRaises(ValueError):
            self._bearing_config(vehicles=vehicles).validate()

    def test_two_captains_rejected(self) -> None:
        vehicles = tuple(
            replace(v, role="captain") for v in bearing_two.CONFIG.vehicles
        )
        with self.assertRaises(ValueError):
            self._bearing_config(vehicles=vehicles).validate()

    def test_bad_role_rejected(self) -> None:
        vehicles = (
            replace(bearing_two.CONFIG.vehicles[0], role="pirate"),
            bearing_two.CONFIG.vehicles[1],
        )
        with self.assertRaises(ValueError):
            self._bearing_config(vehicles=vehicles).validate()

    def test_task_two_schedule(self) -> None:
        self.assertEqual(task_two.CONFIG.task_velocity, (0.10, 0.0))


class DefaultConfigTests(unittest.TestCase):
    def test_new_defaults(self) -> None:
        control = TaskDrivenConfig()
        self.assertEqual(control.k, 0.8)
        self.assertEqual(control.deadband_mps, 0.015)
        execution = ExecutionConfig()
        self.assertEqual(execution.mode, "omni_pid")
        self.assertIsNone(execution.heading_target_rad)
        runtime = RuntimeConfig()
        self.assertEqual(runtime.converge_eps_m, 0.03)
        preflight = PreflightConfig()
        self.assertEqual(preflight.min_separation_m, 0.20)
        self.assertEqual(preflight.formation_warn_m, 0.15)


def _minimal_config() -> ExperimentConfig:
    return ExperimentConfig(
        name="unit",
        algorithm="task_driven",
        vehicles=(
            VehicleConfig("car1", "ground_vehicle", "10.1.1.81"),
            VehicleConfig("car2", "ground_vehicle", "10.1.1.82"),
        ),
        topology=TopologyConfig(
            adjacency_matrix=((0, 1), (1, 0)),
            formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
        ),
    )


class ValidationTests(unittest.TestCase):
    def test_minimal_config_validates(self) -> None:
        _minimal_config().validate()

    def test_non_positive_gain_rejected(self) -> None:
        config = replace(_minimal_config(), control=TaskDrivenConfig(k=0.0))
        with self.assertRaises(ValueError):
            config.validate()

    def test_missing_formation_key_rejected(self) -> None:
        config = _minimal_config()
        bad_topology = TopologyConfig(
            adjacency_matrix=((0, 1), (1, 0)),
            formation={"car1": (0.0, 0.0)},
        )
        with self.assertRaises(ValueError):
            replace(config, topology=bad_topology).validate()

    def test_disconnected_graph_rejected(self) -> None:
        config = _minimal_config()
        bad_topology = TopologyConfig(
            adjacency_matrix=((0, 0), (0, 0)),
            formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
        )
        with self.assertRaises(ValueError):
            replace(config, topology=bad_topology).validate()

    def test_unknown_task_mode_rejected(self) -> None:
        config = replace(_minimal_config(), task_modes=("rotate",))
        with self.assertRaises(ValueError):
            config.validate()

    def test_task_velocity_dimension_mismatch_rejected(self) -> None:
        config = replace(_minimal_config(), task_velocity=(0.1,))
        with self.assertRaises(ValueError):
            config.validate()

    def test_over_limit_task_velocity_rejected(self) -> None:
        config = replace(_minimal_config(), task_velocity=(0.3, 0.0))
        with self.assertRaises(ValueError):
            config.validate()

    def test_non_monotonic_schedule_rejected(self) -> None:
        config = replace(
            _minimal_config(),
            task_velocity_schedule=((3.0, (0.1, 0.0)), (1.0, (0.0, 0.0))),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_negative_edge_weight_rejected(self) -> None:
        config = _minimal_config()
        bad_topology = TopologyConfig(
            adjacency_matrix=((0, 1), (1, 0)),
            formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
            edge_weight_matrix=((0.0, -1.0), (-1.0, 0.0)),
        )
        with self.assertRaises(ValueError):
            replace(config, topology=bad_topology).validate()

    def test_base_schedule_mismatch_warns(self) -> None:
        config = replace(
            _minimal_config(),
            task_velocity=(0.20, 0.0),
            task_velocity_schedule=((0.0, (0.0, 0.0)), (3.0, (0.10, 0.0))),
        )
        with self.assertWarnsRegex(UserWarning, "differs from the final scheduled"):
            config.validate()

    def test_matching_base_and_schedule_final_no_warning(self) -> None:
        config = replace(
            _minimal_config(),
            task_velocity=(0.10, 0.0),
            task_velocity_schedule=((0.0, (0.0, 0.0)), (3.0, (0.10, 0.0))),
        )
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            config.validate()

    def test_invalid_execution_mode_rejected(self) -> None:
        config = replace(_minimal_config(), execution=ExecutionConfig(mode="hover"))
        with self.assertRaises(ValueError):
            config.validate()


class PlannerFactoryTests(unittest.TestCase):
    def test_task_driven_algorithm_is_available(self) -> None:
        self.assertIn("task_driven", available_algorithms())

    def test_unknown_algorithm_rejected(self) -> None:
        config = replace(_minimal_config(), algorithm="magic")
        with self.assertRaises(ValueError):
            build_swarm_planner(config)

    def test_bearing_algorithm_is_available(self) -> None:
        self.assertIn("bearing", available_algorithms())

    def test_builds_controller_for_catalog_config(self) -> None:
        planner = build_swarm_planner(task_two.CONFIG)
        self.assertEqual(planner.topology.vehicle_ids, ["car4", "car5"])


if __name__ == "__main__":
    unittest.main()
