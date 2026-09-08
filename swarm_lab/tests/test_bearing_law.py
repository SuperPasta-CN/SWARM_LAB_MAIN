"""Unit tests for the projected bearing-only formation law with roles."""

from __future__ import annotations

import unittest
from math import cos, pi, sin, sqrt, tanh

from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.domain.config import BearingControlConfig, FirstMateParams
from swarm.domain.models import MocapSnapshot, VehicleState
from swarm.infrastructure.telemetry.bearing_metrics import bearing_angle_error_rad


def _snapshot(positions, invalid=()) -> MocapSnapshot:
    states = {
        vehicle_id: VehicleState(
            vehicle_id,
            x=xy[0],
            y=xy[1],
            z=xy[2] if len(xy) > 2 else 0.0,
            valid=vehicle_id not in invalid,
        )
        for vehicle_id, xy in positions.items()
    }
    return MocapSnapshot(1.0, states)


def _controller(
    adjacency,
    bearings,
    roles,
    captain_velocity=(0.0, 0.0, 0.0),
    captain_velocity_schedule=(),
    first_mate_params=None,
    control=None,
    speed_limit=0.25,
) -> BearingOnlyFormationController:
    vehicle_ids = ["car%d" % (i + 1) for i in range(len(adjacency))]
    topology = build_topology_from_matrices(vehicle_ids, adjacency, bearings, roles)
    return BearingOnlyFormationController(
        topology=topology,
        captain_velocity=captain_velocity,
        captain_velocity_schedule=captain_velocity_schedule,
        first_mate_params=first_mate_params,
        control=control or BearingControlConfig(),
        command_speed_limit_mps=speed_limit,
    )


_TWO_CAR_ADJACENCY = ((0, 1), (1, 0))
_TWO_CAR_BEARINGS = (
    ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
)
# car1 runs the law (crew); car2 is a zero-velocity captain, i.e. pinned.
_CREW_VS_PINNED = ("crew", "captain")


class BearingLawSignTests(unittest.TestCase):
    def test_self_check_control_rotates_bearing_towards_desired(self) -> None:
        # actual g = (1, 0), desired g* = (0, 1): the control on car1 must
        # point along -y so that g rotates towards g*.
        controller = _controller(_TWO_CAR_ADJACENCY, _TWO_CAR_BEARINGS, _CREW_VS_PINNED)
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.0, places=9)
        self.assertLess(command.vy, 0.0)

    def test_single_edge_control_is_perpendicular_to_line_of_sight(self) -> None:
        angle = pi / 6.0
        controller = _controller(
            adjacency=_TWO_CAR_ADJACENCY,
            bearings=(
                ((0.0, 0.0, 0.0), (cos(angle), sin(angle), 0.0)),
                ((-cos(angle), -sin(angle), 0.0), (0.0, 0.0, 0.0)),
            ),
            roles=_CREW_VS_PINNED,
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        g = (1.0, 0.0)
        dot = command.vx * g[0] + command.vy * g[1]
        magnitude = sqrt(command.vx * command.vx + command.vy * command.vy)
        self.assertGreater(magnitude, 0.0)
        self.assertAlmostEqual(dot, 0.0, places=9)

    def test_contributions_are_summed_not_averaged(self) -> None:
        control = BearingControlConfig(kp=1.0, deadband_mps=0.0)
        eps = 0.01
        desired = (cos(eps), sin(eps), 0.0)
        one_edge = _controller(
            adjacency=_TWO_CAR_ADJACENCY,
            bearings=(
                ((0.0, 0.0, 0.0), desired),
                ((-desired[0], -desired[1], 0.0), (0.0, 0.0, 0.0)),
            ),
            roles=("crew", "crew"),
            control=control,
            speed_limit=1.0,
        )
        two_edges = _controller(
            adjacency=((0, 1, 1), (1, 0, 0), (1, 0, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), desired, desired),
                ((-desired[0], -desired[1], 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                ((-desired[0], -desired[1], 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            roles=("crew", "crew", "crew"),
            control=control,
            speed_limit=1.0,
        )
        single = one_edge.step(
            _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02
        ).commands["car1"]
        double = two_edges.step(
            _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0), "car3": (2.0, 0.0)}), 0.02
        ).commands["car1"]
        # Small errors sit in the linear tanh region, so norms scale 1:2.
        self.assertAlmostEqual(abs(double.vy), 2.0 * abs(single.vy), delta=1e-4)


class BearingLawShapingTests(unittest.TestCase):
    def test_deadband_outputs_exactly_zero(self) -> None:
        eps = 0.005  # below deadband_mps / kp
        controller = _controller(
            adjacency=_TWO_CAR_ADJACENCY,
            bearings=(
                ((0.0, 0.0, 0.0), (cos(eps), sin(eps), 0.0)),
                ((-cos(eps), -sin(eps), 0.0), (0.0, 0.0, 0.0)),
            ),
            roles=_CREW_VS_PINNED,
            control=BearingControlConfig(kp=1.0, deadband_mps=0.015),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))

    def test_smooth_saturation_limits_speed_and_keeps_direction(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            _CREW_VS_PINNED,
            control=BearingControlConfig(kp=10.0, deadband_mps=0.0),
            speed_limit=0.25,
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        speed = sqrt(command.vx * command.vx + command.vy * command.vy)
        self.assertAlmostEqual(speed, 0.25, places=6)
        self.assertLess(command.vy, 0.0)
        self.assertAlmostEqual(command.vx, 0.0, places=9)

    def test_overlapping_vehicles_invalidate_the_edge(self) -> None:
        controller = _controller(_TWO_CAR_ADJACENCY, _TWO_CAR_BEARINGS, ("crew", "crew"))
        result = controller.step(_snapshot({"car1": (0.5, 0.5), "car2": (0.5, 0.5)}), 0.02)
        command = result.commands["car1"]
        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))
        invalid = [sample for sample in result.bearing_samples if not sample.valid]
        self.assertEqual(len(invalid), 2)  # car1->car2 and car2->car1
        for sample in invalid:
            self.assertIsNone(bearing_angle_error_rad(sample))

    def test_mean_error_is_angular_in_radians(self) -> None:
        controller = _controller(_TWO_CAR_ADJACENCY, _TWO_CAR_BEARINGS, _CREW_VS_PINNED)
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        self.assertAlmostEqual(result.mean_error, pi / 2.0, places=6)
        self.assertAlmostEqual(result.max_error, pi / 2.0, places=6)

    def test_bearing_3d_mode_uses_height(self) -> None:
        bearings = (
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        )
        snapshot = _snapshot({"car1": (0.0, 0.0, 0.0), "car2": (1.0, 0.0, 1.0)})
        planar = _controller(_TWO_CAR_ADJACENCY, bearings, _CREW_VS_PINNED)
        spatial = _controller(
            _TWO_CAR_ADJACENCY,
            bearings,
            _CREW_VS_PINNED,
            control=BearingControlConfig(bearing_3d=True),
        )
        planar_command = planar.step(snapshot, 0.02).commands["car1"]
        spatial_command = spatial.step(snapshot, 0.02).commands["car1"]
        self.assertEqual((planar_command.vx, planar_command.vy), (0.0, 0.0))
        self.assertGreater(
            sqrt(
                spatial_command.vx * spatial_command.vx
                + spatial_command.vy * spatial_command.vy
                + spatial_command.vz * spatial_command.vz
            ),
            0.0,
        )


class RoleDispatchTests(unittest.TestCase):
    """Who runs the bearing law: captain never, crew always, first mate blends."""

    def test_captain_outputs_configured_velocity_ignoring_bearing_error(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("captain", "crew"),
            captain_velocity=(0.1, -0.05, 0.0),
        )
        # Huge bearing error (90 deg): must not leak into the captain command.
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.1)
        self.assertAlmostEqual(command.vy, -0.05)
        self.assertEqual(command.role, "captain")

    def test_captain_zero_velocity_stays_pinned(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY, _TWO_CAR_BEARINGS, ("captain", "crew")
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))

    def test_crew_pair_acts_symmetrically(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("crew", "crew"),
            control=BearingControlConfig(kp=1.0, deadband_mps=0.0),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        first = result.commands["car1"]
        second = result.commands["car2"]
        # Equal and opposite: the centroid of the pair is not pushed around.
        self.assertAlmostEqual(first.vx + second.vx, 0.0, places=9)
        self.assertAlmostEqual(first.vy + second.vy, 0.0, places=9)

    def test_first_mate_weight_one_is_pure_fixed_velocity(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("first_mate", "crew"),
            first_mate_params={
                "car1": FirstMateParams(velocity=(0.1, 0.0, 0.0), blend_weight=1.0)
            },
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.1)
        self.assertAlmostEqual(command.vy, 0.0)

    def test_first_mate_weight_zero_is_pure_bearing_term(self) -> None:
        params = {"car1": FirstMateParams(velocity=(0.1, 0.0, 0.0), blend_weight=0.0)}
        blended = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("first_mate", "crew"),
            first_mate_params=params,
        )
        pure = _controller(_TWO_CAR_ADJACENCY, _TWO_CAR_BEARINGS, ("crew", "crew"))
        snapshot = _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)})
        blended_command = blended.step(snapshot, 0.02).commands["car1"]
        pure_command = pure.step(snapshot, 0.02).commands["car1"]
        self.assertAlmostEqual(blended_command.vx, pure_command.vx, places=9)
        self.assertAlmostEqual(blended_command.vy, pure_command.vy, places=9)

    def test_first_mate_weighted_blend_numeric(self) -> None:
        # Geometry: actual g = (1, 0), desired g* = (0, 1) -> raw = (0, -1).
        # kp = 1, speed_limit = 1: u = (0, -tanh(1)).  alpha = 0.5,
        # v_fm = (0.2, 0): v = 0.5 * v_fm + 0.5 * u = (0.1, -tanh(1) / 2).
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("first_mate", "crew"),
            first_mate_params={
                "car1": FirstMateParams(
                    velocity=(0.2, 0.0, 0.0), blend_weight=0.5, blend_mode="weighted"
                )
            },
            control=BearingControlConfig(kp=1.0, deadband_mps=0.0),
            speed_limit=1.0,
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.1, places=9)
        self.assertAlmostEqual(command.vy, -tanh(1.0) / 2.0, places=9)

    def test_first_mate_additive_blend_numeric(self) -> None:
        # Same geometry: additive gives v = v_fm + u = (0.2, -tanh(1)).
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("first_mate", "crew"),
            first_mate_params={
                "car1": FirstMateParams(velocity=(0.2, 0.0, 0.0), blend_mode="additive")
            },
            control=BearingControlConfig(kp=1.0, deadband_mps=0.0),
            speed_limit=1.0,
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.2, places=9)
        self.assertAlmostEqual(command.vy, -tanh(1.0), places=9)

    def test_blend_result_is_speed_limited(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("first_mate", "crew"),
            first_mate_params={
                "car1": FirstMateParams(velocity=(0.2, 0.0, 0.0), blend_mode="additive")
            },
            control=BearingControlConfig(kp=10.0, deadband_mps=0.0),
            speed_limit=0.25,
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        speed = sqrt(command.vx * command.vx + command.vy * command.vy)
        self.assertAlmostEqual(speed, 0.25, places=6)


class BearingPidTests(unittest.TestCase):
    """Optional integral/derivative terms on the bearing control term."""

    def _geometry_controller(self, control) -> BearingOnlyFormationController:
        return _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("crew", "captain"),
            control=control,
            speed_limit=1.0,
        )

    def test_integral_accumulates_raw_over_time(self) -> None:
        controller = self._geometry_controller(
            BearingControlConfig(kp=1.0, ki=0.5, deadband_mps=0.0, integral_limit=10.0)
        )
        snapshot = _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)})
        controller.step(snapshot, 0.02)
        controller.step(snapshot, 0.02)
        # raw = (0, -1) for this geometry, two steps of dt = 0.02.
        xi = controller._integrals["car1"]
        self.assertAlmostEqual(xi[0], 0.0, places=9)
        self.assertAlmostEqual(xi[1], -0.04, places=9)

    def test_integral_is_norm_clamped(self) -> None:
        controller = self._geometry_controller(
            BearingControlConfig(kp=1.0, ki=0.5, deadband_mps=0.0, integral_limit=0.01)
        )
        snapshot = _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)})
        for _ in range(10):
            controller.step(snapshot, 0.02)
        xi = controller._integrals["car1"]
        norm = sqrt(xi[0] * xi[0] + xi[1] * xi[1] + xi[2] * xi[2])
        self.assertAlmostEqual(norm, 0.01, places=9)

    def test_invalid_vehicle_clears_pid_state(self) -> None:
        controller = self._geometry_controller(
            BearingControlConfig(kp=1.0, ki=0.5, deadband_mps=0.0, integral_limit=10.0)
        )
        valid = _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)})
        controller.step(valid, 0.02)
        self.assertIn("car1", controller._integrals)
        invalid = _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}, invalid=("car1",))
        controller.step(invalid, 0.02)
        self.assertNotIn("car1", controller._integrals)
        self.assertNotIn("car1", controller._prev_raw)

    def test_derivative_term_is_zero_for_constant_raw(self) -> None:
        snapshot = _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)})
        with_kd = self._geometry_controller(
            BearingControlConfig(kp=1.0, kd=10.0, deadband_mps=0.0)
        )
        without_kd = self._geometry_controller(
            BearingControlConfig(kp=1.0, deadband_mps=0.0)
        )
        with_kd.step(snapshot, 0.02)  # first step: no prev_raw, kd term zero
        without_kd.step(snapshot, 0.02)
        # Second step: raw unchanged, so the kd term must still be zero.
        command_kd = with_kd.step(snapshot, 0.02).commands["car1"]
        command_p = without_kd.step(snapshot, 0.02).commands["car1"]
        self.assertAlmostEqual(command_kd.vx, command_p.vx, places=9)
        self.assertAlmostEqual(command_kd.vy, command_p.vy, places=9)

    def test_captain_never_accumulates_integral(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("captain", "crew"),
            captain_velocity=(0.1, 0.0, 0.0),
            control=BearingControlConfig(ki=0.5),
        )
        snapshot = _snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)})
        for _ in range(5):
            controller.step(snapshot, 0.02)
        self.assertNotIn("car1", controller._integrals)
        self.assertIn("car2", controller._integrals)


class CaptainScheduleTests(unittest.TestCase):
    def test_piecewise_constant_schedule_switches_velocity(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("captain", "crew"),
            captain_velocity=(0.1, 0.0, 0.0),
            captain_velocity_schedule=[(0.05, (0.0, 0.1, 0.0))],
        )
        snapshot = _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)})
        first = controller.step(snapshot, 0.02).commands["car1"]  # elapsed 0.02
        second = controller.step(snapshot, 0.02).commands["car1"]  # elapsed 0.04
        third = controller.step(snapshot, 0.02).commands["car1"]  # elapsed 0.06
        for command in (first, second):
            self.assertAlmostEqual(command.vx, 0.1)
            self.assertAlmostEqual(command.vy, 0.0)
        self.assertAlmostEqual(third.vx, 0.0)
        self.assertAlmostEqual(third.vy, 0.1)

    def test_reset_restarts_the_schedule_clock(self) -> None:
        controller = _controller(
            _TWO_CAR_ADJACENCY,
            _TWO_CAR_BEARINGS,
            ("captain", "crew"),
            captain_velocity=(0.1, 0.0, 0.0),
            captain_velocity_schedule=[(0.03, (0.0, 0.1, 0.0))],
        )
        snapshot = _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)})
        controller.step(snapshot, 0.02)
        controller.step(snapshot, 0.02)  # elapsed 0.04 -> scheduled velocity
        controller.reset()
        command = controller.step(snapshot, 0.02).commands["car1"]
        self.assertAlmostEqual(command.vx, 0.1)
        self.assertAlmostEqual(command.vy, 0.0)


if __name__ == "__main__":
    unittest.main()
