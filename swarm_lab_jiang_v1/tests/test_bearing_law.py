"""Unit tests for the projected bearing-only formation law."""

from __future__ import annotations

import unittest
from math import cos, pi, sin, sqrt

from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.domain.config import BearingControlConfig
from swarm.domain.models import MocapSnapshot, VehicleState
from swarm.infrastructure.telemetry.bearing_metrics import bearing_angle_error_rad


def _snapshot(positions) -> MocapSnapshot:
    states = {
        vehicle_id: VehicleState(vehicle_id, x=xy[0], y=xy[1], z=xy[2] if len(xy) > 2 else 0.0, valid=True)
        for vehicle_id, xy in positions.items()
    }
    return MocapSnapshot(1.0, states)


def _controller(
    adjacency,
    bearings,
    leader_mask,
    leader_velocities=None,
    control=None,
    speed_limit=0.25,
) -> BearingOnlyFormationController:
    vehicle_ids = ["car%d" % (i + 1) for i in range(len(adjacency))]
    topology = build_topology_from_matrices(vehicle_ids, adjacency, bearings, leader_mask)
    velocities = leader_velocities or {
        vehicle_id: (0.0, 0.0, 0.0)
        for vehicle_id, is_leader in zip(vehicle_ids, leader_mask)
        if is_leader
    }
    return BearingOnlyFormationController(
        topology=topology,
        leader_target_velocities=velocities,
        control=control or BearingControlConfig(),
        command_speed_limit_mps=speed_limit,
    )


class BearingLawSignTests(unittest.TestCase):
    def test_self_check_control_rotates_bearing_towards_desired(self) -> None:
        # actual g = (1, 0), desired g* = (0, 1): the control on car1 must
        # point along -y so that g rotates towards g*.
        controller = _controller(
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(False, True),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.0, places=9)
        self.assertLess(command.vy, 0.0)

    def test_single_edge_control_is_perpendicular_to_line_of_sight(self) -> None:
        angle = pi / 6.0
        controller = _controller(
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), (cos(angle), sin(angle), 0.0)),
                ((-cos(angle), -sin(angle), 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(False, True),
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
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), desired),
                ((-desired[0], -desired[1], 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(False, True),
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
            leader_mask=(False, True, True),
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
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), (cos(eps), sin(eps), 0.0)),
                ((-cos(eps), -sin(eps), 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(False, True),
            control=BearingControlConfig(kp=1.0, deadband_mps=0.015),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))

    def test_smooth_saturation_limits_speed_and_keeps_direction(self) -> None:
        controller = _controller(
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(False, True),
            control=BearingControlConfig(kp=10.0, deadband_mps=0.0),
            speed_limit=0.25,
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        speed = sqrt(command.vx * command.vx + command.vy * command.vy)
        self.assertAlmostEqual(speed, 0.25, places=6)
        self.assertLess(command.vy, 0.0)
        self.assertAlmostEqual(command.vx, 0.0, places=9)

    def test_leader_outputs_configured_target_velocity(self) -> None:
        controller = _controller(
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(True, False),
            leader_velocities={"car1": (0.1, -0.05, 0.0)},
            control=BearingControlConfig(agent_mode="leader_velocity"),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)}), 0.02)
        command = result.commands["car1"]
        self.assertAlmostEqual(command.vx, 0.1)
        self.assertAlmostEqual(command.vy, -0.05)

    def test_overlapping_vehicles_invalidate_the_edge(self) -> None:
        controller = _controller(
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(False, True),
        )
        result = controller.step(_snapshot({"car1": (0.5, 0.5), "car2": (0.5, 0.5)}), 0.02)
        command = result.commands["car1"]
        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))
        invalid = [sample for sample in result.bearing_samples if not sample.valid]
        self.assertEqual(len(invalid), 2)  # car1->car2 and car2->car1
        for sample in invalid:
            self.assertIsNone(bearing_angle_error_rad(sample))

    def test_mean_error_is_angular_in_radians(self) -> None:
        controller = _controller(
            adjacency=((0, 1), (1, 0)),
            bearings=(
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            leader_mask=(False, True),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        self.assertAlmostEqual(result.mean_error, pi / 2.0, places=6)
        self.assertAlmostEqual(result.max_error, pi / 2.0, places=6)

    def test_bearing_3d_mode_uses_height(self) -> None:
        bearings = (
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        )
        adjacency = ((0, 1), (1, 0))
        leader_mask = (False, True)
        snapshot = _snapshot({"car1": (0.0, 0.0, 0.0), "car2": (1.0, 0.0, 1.0)})
        planar = _controller(adjacency, bearings, leader_mask)
        spatial = _controller(
            adjacency,
            bearings,
            leader_mask,
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


class AgentModeTests(unittest.TestCase):
    """Who runs the bearing law: all agents (default) vs velocity leaders."""

    def _two_car(self, leader_mask, leader_velocities=None, control=None):
        bearings = (
            ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
        )
        return _controller(
            adjacency=((0, 1), (1, 0)),
            bearings=bearings,
            leader_mask=leader_mask,
            leader_velocities=leader_velocities,
            control=control,
        )

    def test_all_bearing_is_the_default_mode(self) -> None:
        self.assertEqual(BearingControlConfig().agent_mode, "all_bearing")

    def test_all_bearing_moves_leaders_too(self) -> None:
        controller = self._two_car(leader_mask=(False, True))
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car2"]  # masked as leader, must still move
        magnitude = sqrt(command.vx * command.vx + command.vy * command.vy)
        self.assertGreater(magnitude, 0.0)

    def test_all_bearing_acts_symmetrically_on_both_endpoints(self) -> None:
        controller = self._two_car(
            leader_mask=(False, True),
            control=BearingControlConfig(kp=1.0, deadband_mps=0.0),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        first = result.commands["car1"]
        second = result.commands["car2"]
        # Equal and opposite: the centroid of the pair is not pushed around.
        self.assertAlmostEqual(first.vx + second.vx, 0.0, places=9)
        self.assertAlmostEqual(first.vy + second.vy, 0.0, places=9)

    def test_leader_bias_adds_configured_velocity_to_the_law(self) -> None:
        controller = self._two_car(
            leader_mask=(True, False),
            leader_velocities={"car1": (0.1, 0.0, 0.0)},
            control=BearingControlConfig(agent_mode="leader_bias"),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertGreater(command.vx, 0.09)  # bias shows up in +x
        self.assertLess(command.vy, 0.0)  # bearing-law part still rotates g

    def test_leader_velocity_mode_keeps_zero_velocity_leader_pinned(self) -> None:
        controller = self._two_car(
            leader_mask=(True, False),
            control=BearingControlConfig(agent_mode="leader_velocity"),
        )
        result = controller.step(_snapshot({"car1": (0.0, 0.0), "car2": (1.0, 0.0)}), 0.02)
        command = result.commands["car1"]
        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
