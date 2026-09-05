"""Configuration loading and validation tests."""

from __future__ import annotations

import unittest
from dataclasses import replace

from swarm.application.planner_factory import available_algorithms, build_swarm_planner
from swarm.domain.config import (
    BearingControlConfig,
    ExecutionConfig,
    ExperimentConfig,
    FirstMateParams,
    PreflightConfig,
    RuntimeConfig,
    TopologyConfig,
    VehicleConfig,
    VehicleExecutionOverride,
)
from configs import crew_four, crew_six, crew_three, crew_two


class ExperimentConfigCatalogTests(unittest.TestCase):
    def test_all_catalog_configs_validate(self) -> None:
        for module in (crew_two, crew_three, crew_four, crew_six):
            with self.subTest(config=module.CONFIG.name):
                module.CONFIG.validate()
                self.assertEqual(module.CONFIG.algorithm, "bearing")

    def test_catalog_vehicle_addresses(self) -> None:
        self.assertEqual(
            {v.vehicle_id: v.address for v in crew_two.CONFIG.vehicles},
            {"car1": "10.1.1.81", "car2": "10.1.1.82"},
        )
        self.assertEqual(
            {v.vehicle_id: v.address for v in crew_four.CONFIG.vehicles},
            {
                "car1": "10.1.1.81",
                "car2": "10.1.1.82",
                "car3": "10.1.1.83",
                "car4": "10.1.1.84",
            },
        )
        self.assertEqual(
            {v.vehicle_id: v.address for v in crew_six.CONFIG.vehicles},
            {"car%d" % i: "10.1.1.%d" % (80 + i) for i in range(1, 7)},
        )

    def test_crew_four_topology_ported_unchanged(self) -> None:
        topology = crew_four.CONFIG.topology
        self.assertEqual(
            tuple(tuple(row) for row in topology.adjacency_matrix),
            (
                (0, 1, 1, 1),
                (1, 0, 0, 1),
                (1, 0, 0, 1),
                (1, 1, 1, 0),
            ),
        )
        self.assertEqual(topology.bearing_matrix[0][1], (1.0, 0.0, 0.0))
        self.assertEqual(topology.bearing_matrix[0][2], (0.0, -1.0, 0.0))
        self.assertEqual(topology.bearing_matrix[3][0], (-1.0, 1.0, 0.0))

    def test_catalog_roles(self) -> None:
        def roles(module):
            return tuple(vehicle.role for vehicle in module.CONFIG.vehicles)

        self.assertEqual(roles(crew_two), ("captain", "first_mate"))
        self.assertEqual(roles(crew_three), ("captain", "first_mate", "crew"))
        self.assertEqual(roles(crew_four), ("captain", "first_mate", "crew", "crew"))
        self.assertEqual(
            roles(crew_six),
            ("captain", "first_mate", "crew", "crew", "crew", "crew"),
        )

    def test_catalog_captain_velocity_is_the_translational_maneuver(self) -> None:
        for module in (crew_two, crew_three, crew_four, crew_six):
            with self.subTest(config=module.CONFIG.name):
                self.assertEqual(module.CONFIG.captain_velocity, (0.10, 0.0, 0.0))


class DefaultConfigTests(unittest.TestCase):
    def test_new_defaults(self) -> None:
        bearing = BearingControlConfig()
        self.assertEqual(bearing.kp, 0.6)
        self.assertEqual(bearing.ki, 0.0)
        self.assertEqual(bearing.kd, 0.0)
        self.assertEqual(bearing.integral_limit, 0.5)
        self.assertEqual(bearing.deadband_mps, 0.015)
        self.assertFalse(bearing.bearing_3d)
        mate = FirstMateParams()
        self.assertIsNone(mate.velocity)
        self.assertEqual(mate.blend_weight, 0.5)
        self.assertEqual(mate.blend_mode, "weighted")
        execution = ExecutionConfig()
        self.assertEqual(execution.mode, "omni_pid")
        self.assertTrue(execution.heading_hold)
        self.assertEqual(execution.mecanum_l_m, 0.10)
        self.assertEqual(execution.max_wheel_speed_mps, 0.5)
        self.assertEqual(execution.wheel_command_min_effective, 0.0)
        self.assertEqual(execution.wheel_flip, (1.0, 1.0, 1.0, 1.0))
        runtime = RuntimeConfig()
        self.assertFalse(runtime.require_twist)
        self.assertFalse(runtime.stop_on_converge)
        self.assertEqual(runtime.converge_eps_rad, 0.05)
        self.assertEqual(runtime.velocity_diff_baseline_s, 0.2)
        preflight = PreflightConfig()
        self.assertEqual(preflight.captain_edge_warn_deg, 45.0)
        self.assertEqual(preflight.min_separation_m, 0.20)
        self.assertEqual(preflight.mocap_timeout_s, 10.0)
        self.assertTrue(preflight.require_confirmation)


def _minimal_config() -> ExperimentConfig:
    return ExperimentConfig(
        name="unit",
        algorithm="bearing",
        vehicles=(VehicleConfig("car1", "captain", "ground_vehicle", "10.1.1.81"),),
        topology=TopologyConfig(
            adjacency_matrix=((0,),),
            bearing_matrix=(((0.0, 0.0, 0.0),),),
        ),
    )


def _two_car_config() -> ExperimentConfig:
    return ExperimentConfig(
        name="unit",
        algorithm="bearing",
        vehicles=(
            VehicleConfig("car1", "captain", "ground_vehicle", "10.1.1.81"),
            VehicleConfig("car2", "first_mate", "ground_vehicle", "10.1.1.82"),
        ),
        topology=TopologyConfig(
            adjacency_matrix=((0, 1), (1, 0)),
            bearing_matrix=(
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            ),
        ),
    )


class ValidationTests(unittest.TestCase):
    def test_minimal_config_validates(self) -> None:
        _minimal_config().validate()

    def test_captain_plus_first_mate_without_crew_validates(self) -> None:
        _two_car_config().validate()

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

    def test_negative_ki_kd_rejected(self) -> None:
        for bad in (-0.1, -1.0):
            config = replace(
                _minimal_config(), bearing_control=BearingControlConfig(ki=bad)
            )
            with self.subTest(ki=bad), self.assertRaises(ValueError):
                config.validate()
        config = replace(
            _minimal_config(), bearing_control=BearingControlConfig(kd=-0.1)
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_non_positive_integral_limit_rejected(self) -> None:
        config = replace(
            _minimal_config(), bearing_control=BearingControlConfig(integral_limit=0.0)
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

    def test_non_positive_captain_edge_warn_rejected(self) -> None:
        config = replace(
            _minimal_config(), preflight=PreflightConfig(captain_edge_warn_deg=0.0)
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


class RoleValidationTests(unittest.TestCase):
    def test_two_captains_rejected(self) -> None:
        config = replace(
            _two_car_config(),
            vehicles=(
                VehicleConfig("car1", "captain", "ground_vehicle", "10.1.1.81"),
                VehicleConfig("car2", "captain", "ground_vehicle", "10.1.1.82"),
            ),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_zero_captains_rejected(self) -> None:
        config = replace(
            _two_car_config(),
            vehicles=(
                VehicleConfig("car1", "crew", "ground_vehicle", "10.1.1.81"),
                VehicleConfig("car2", "crew", "ground_vehicle", "10.1.1.82"),
            ),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_unknown_role_rejected(self) -> None:
        config = replace(
            _two_car_config(),
            vehicles=(
                VehicleConfig("car1", "captain", "ground_vehicle", "10.1.1.81"),
                VehicleConfig("car2", "admiral", "ground_vehicle", "10.1.1.82"),
            ),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_blend_weight_out_of_range_rejected(self) -> None:
        config = replace(
            _two_car_config(),
            first_mate_params={"car2": FirstMateParams(blend_weight=1.5)},
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_unknown_blend_mode_rejected(self) -> None:
        config = replace(
            _two_car_config(),
            first_mate_params={"car2": FirstMateParams(blend_mode="average")},
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_first_mate_params_key_must_be_a_first_mate(self) -> None:
        config = replace(
            _two_car_config(),
            first_mate_params={"car1": FirstMateParams()},
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_captain_velocity_over_limit_rejected(self) -> None:
        config = replace(_two_car_config(), captain_velocity=(0.3, 0.0, 0.0))
        with self.assertRaises(ValueError):
            config.validate()

    def test_bad_schedule_rejected(self) -> None:
        for schedule in (
            [(-1.0, (0.1, 0.0, 0.0))],  # negative time
            [(2.0, (0.1, 0.0, 0.0)), (1.0, (0.0, 0.1, 0.0))],  # decreasing
            [(1.0, (0.3, 0.0, 0.0))],  # over the speed limit
        ):
            config = replace(_two_car_config(), captain_velocity_schedule=schedule)
            with self.subTest(schedule=schedule), self.assertRaises(ValueError):
                config.validate()

    def test_non_opposite_bearing_edges_rejected(self) -> None:
        config = replace(
            _two_car_config(),
            topology=TopologyConfig(
                adjacency_matrix=((0, 1), (1, 0)),
                bearing_matrix=(
                    ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                    ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)),  # should be (0, -1, 0)
                ),
            ),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_bad_execution_override_rejected(self) -> None:
        config = replace(
            _two_car_config(),
            vehicles=(
                VehicleConfig(
                    "car1",
                    "captain",
                    "ground_vehicle",
                    "10.1.1.81",
                    execution_override=VehicleExecutionOverride(max_wheel_speed_mps=0.0),
                ),
                VehicleConfig("car2", "first_mate", "ground_vehicle", "10.1.1.82"),
            ),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_valid_execution_override_accepted(self) -> None:
        config = replace(
            _two_car_config(),
            vehicles=(
                VehicleConfig(
                    "car1",
                    "captain",
                    "ground_vehicle",
                    "10.1.1.81",
                    execution_override=VehicleExecutionOverride(
                        max_wheel_speed_mps=0.3,
                        wheel_command_min_effective=35.0,
                        wheel_flip=(-1.0, 1.0, 1.0, 1.0),
                    ),
                ),
                VehicleConfig("car2", "first_mate", "ground_vehicle", "10.1.1.82"),
            ),
        )
        config.validate()


class PlannerFactoryTests(unittest.TestCase):
    def test_bearing_algorithm_is_available(self) -> None:
        self.assertIn("bearing", available_algorithms())

    def test_unknown_algorithm_rejected(self) -> None:
        config = replace(_minimal_config(), algorithm="boids")
        with self.assertRaises(ValueError):
            build_swarm_planner(config)

    def test_builds_controller_for_catalog_configs(self) -> None:
        for module in (crew_two, crew_three, crew_four, crew_six):
            planner = build_swarm_planner(module.CONFIG)
            self.assertEqual(len(planner.topology.vehicle_ids), len(module.CONFIG.vehicles))

    def test_first_mate_velocity_defaults_to_captain_velocity(self) -> None:
        planner = build_swarm_planner(crew_four.CONFIG)
        self.assertEqual(
            planner.first_mate_params["car2"].velocity,
            crew_four.CONFIG.captain_velocity,
        )

    def test_first_mate_params_passed_through(self) -> None:
        config = replace(
            crew_four.CONFIG,
            first_mate_params={
                "car2": FirstMateParams(
                    velocity=(0.05, 0.05, 0.0), blend_weight=0.3, blend_mode="additive"
                )
            },
        )
        planner = build_swarm_planner(config)
        params = planner.first_mate_params["car2"]
        self.assertEqual(params.velocity, (0.05, 0.05, 0.0))
        self.assertEqual(params.blend_weight, 0.3)
        self.assertEqual(params.blend_mode, "additive")


if __name__ == "__main__":
    unittest.main()
