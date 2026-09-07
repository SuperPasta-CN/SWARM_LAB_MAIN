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
from configs import v3_four, v3_three, v3_two


class CatalogConfigTests(unittest.TestCase):
    def test_all_catalog_configs_validate(self) -> None:
        for module in (v3_two, v3_three, v3_four):
            with self.subTest(config=module.CONFIG.name):
                module.CONFIG.validate()
                self.assertEqual(module.CONFIG.algorithm, "task_driven")
                # formation 的键必须与车号一一对应
                self.assertEqual(
                    set(module.CONFIG.topology.formation),
                    set(module.CONFIG.vehicle_ids),
                )
                # schedule 末值 = 基值（无 v2 v_fm 陷阱，但保持语义一致）
                schedule = module.CONFIG.task_velocity_schedule
                if schedule:
                    self.assertEqual(schedule[-1][1], module.CONFIG.task_velocity)

    def test_v3_two_vehicles(self) -> None:
        self.assertEqual(
            {v.vehicle_id: v.address for v in v3_two.CONFIG.vehicles},
            {"car4": "10.1.1.84", "car5": "10.1.1.85"},
        )

    def test_v3_three_v3_four_vehicle_counts(self) -> None:
        self.assertEqual(len(v3_three.CONFIG.vehicle_ids), 3)
        self.assertEqual(len(v3_four.CONFIG.vehicle_ids), 4)


class VariantConfigTests(unittest.TestCase):
    """Oscillation A/B variants inherit v3_two and must stay valid."""

    def test_variants_validate(self) -> None:
        from configs import (
            v3_four_all,
            v3_three_all,
            v3_two_affine,
            v3_two_all,
            v3_two_ema,
            v3_two_hdb,
            v3_two_pwm,
        )
        # (deadzone_mode, heading_deadband_rad, velocity_ema_alpha)
        expected = {
            "v3_two_affine": ("affine", 0.03, 0.3),
            "v3_two_pwm": ("pwm", 0.03, 0.3),
            "v3_two_hdb": ("pwm", 0.035, 0.3),
            "v3_two_ema": ("pwm", 0.03, 0.5),
            "v3_two_all": ("pwm", 0.035, 0.3),
            "v3_three_all": ("pwm", 0.035, 0.3),
            "v3_four_all": ("pwm", 0.035, 0.3),
        }
        for module in (
            v3_two_affine, v3_two_pwm, v3_two_hdb, v3_two_ema, v3_two_all,
            v3_three_all, v3_four_all,
        ):
            with self.subTest(config=module.CONFIG.name):
                module.CONFIG.validate()
                mode, hdb, alpha = expected[module.CONFIG.name]
                self.assertEqual(module.CONFIG.execution.deadzone_mode, mode)
                self.assertEqual(module.CONFIG.execution.heading_deadband_rad, hdb)
                self.assertEqual(module.CONFIG.runtime.velocity_ema_alpha, alpha)

    def test_v3_two_schedule(self) -> None:
        self.assertEqual(v3_two.CONFIG.task_velocity, (0.10, 0.0))
        self.assertEqual(
            tuple(v3_two.CONFIG.task_velocity_schedule),
            ((0.0, (0.0, 0.0)), (3.0, (0.10, 0.0))),
        )


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
        config = replace(_minimal_config(), algorithm="bearing")
        with self.assertRaises(ValueError):
            build_swarm_planner(config)

    def test_builds_controller_for_catalog_config(self) -> None:
        planner = build_swarm_planner(v3_two.CONFIG)
        self.assertEqual(planner.topology.vehicle_ids, ["car4", "car5"])


if __name__ == "__main__":
    unittest.main()
