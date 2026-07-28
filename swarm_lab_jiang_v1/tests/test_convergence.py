"""End-to-end convergence proofs of the real bearing law and configuration.

Uses the genuine ``configs/bearing_four`` topology and the genuine
``BearingOnlyFormationController`` driving ideal single-integrator stubs
(``p += v * dt``).  Two regimes are covered:

- ``leader_velocity``: leaders are pinned on a baseline consistent with the
  desired bearings; followers start at random separated positions.
- ``all_bearing`` (the project default): every vehicle runs the bearing law
  and moves; all four start at random separated positions.  The formation
  must still converge (free translation/rotation/scale) and no vehicle may
  stay stationary.

In both regimes the directed-edge bearing RMS angular error must drop below
5 degrees within 30 seconds.
"""

from __future__ import annotations

import random
import unittest
from dataclasses import replace
from math import atan2, degrees, pi, sqrt

from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.models import MocapSnapshot, VehicleState
from configs.bearing_four import CONFIG


DT = 0.02
DURATION_S = 30.0
RMS_TARGET_RAD = 5.0 * pi / 180.0

PINNED_CONFIG = replace(
    CONFIG,
    bearing_control=replace(CONFIG.bearing_control, agent_mode="leader_velocity"),
)


def _sample_positions(rng: random.Random, count: int, min_separation: float, taken=()):
    """Random start positions inside the lab area, pairwise well separated."""

    positions = []
    while len(positions) < count:
        candidate = (rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
        others = list(taken) + positions
        if all(
            sqrt((candidate[0] - other[0]) ** 2 + (candidate[1] - other[1]) ** 2)
            >= min_separation
            for other in others
        ):
            positions.append(candidate)
    return positions


def _edge_angle_errors_rad(result) -> list:
    errors = []
    for sample in result.bearing_samples:
        if not sample.valid:
            continue
        dot = sum(sample.desired[k] * sample.actual[k] for k in range(3))
        dot = min(1.0, max(-1.0, dot))
        cross = sample.desired[0] * sample.actual[1] - sample.desired[1] * sample.actual[0]
        errors.append(atan2(abs(cross), dot))
    return errors


def _rms(values) -> float:
    return sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


class BearingFourConvergenceTests(unittest.TestCase):
    def _run(self, config, positions):
        planner = build_swarm_planner(config)
        steps = int(DURATION_S / DT)
        first_below_s = None
        final_rms_rad = None
        rms_history = []
        path_lengths = {vehicle_id: 0.0 for vehicle_id in config.vehicle_ids}
        for step_index in range(steps):
            states = {
                vehicle_id: VehicleState(
                    vehicle_id, x=positions[vehicle_id][0], y=positions[vehicle_id][1], valid=True
                )
                for vehicle_id in config.vehicle_ids
            }
            result = planner.step(MocapSnapshot(step_index * DT, states), DT)
            rms_rad = _rms(_edge_angle_errors_rad(result))
            rms_history.append(rms_rad)
            if first_below_s is None and rms_rad < RMS_TARGET_RAD:
                first_below_s = step_index * DT
            final_rms_rad = rms_rad
            for vehicle_id, command in result.commands.items():
                x, y = positions[vehicle_id]
                positions[vehicle_id] = (x + command.vx * DT, y + command.vy * DT)
                path_lengths[vehicle_id] += (
                    sqrt(command.vx * command.vx + command.vy * command.vy) * DT
                )
        return rms_history, first_below_s, final_rms_rad, path_lengths

    def _assert_converged(self, label, rms_history, first_below_s, final_rms_rad):
        print(
            "\nconvergence[%s] | initial RMS=%.2f deg, first time below 5 deg: %s, "
            "final RMS=%.4f deg (t=%.1f s)"
            % (
                label,
                degrees(rms_history[0]),
                ("%.2f s" % first_below_s) if first_below_s is not None else "NEVER",
                degrees(final_rms_rad),
                DURATION_S,
            )
        )
        self.assertIsNotNone(
            first_below_s,
            "bearing RMS never dropped below 5 deg within %s s" % DURATION_S,
        )
        self.assertLessEqual(first_below_s, DURATION_S)
        self.assertLess(
            final_rms_rad,
            RMS_TARGET_RAD,
            "bearing RMS did not stay below 5 deg at the end of the run",
        )

    def test_rms_bearing_error_converges_below_five_degrees(self) -> None:
        PINNED_CONFIG.validate()
        leaders = {"car1": (0.0, 0.0), "car2": (1.0, 0.0)}
        rng = random.Random(20260727)
        follower_xy = _sample_positions(
            rng, 2, CONFIG.preflight.min_separation_m + 0.1, taken=leaders.values()
        )
        positions = dict(leaders)
        positions["car3"] = follower_xy[0]
        positions["car4"] = follower_xy[1]
        rms_history, first_below_s, final_rms_rad, _ = self._run(PINNED_CONFIG, positions)
        self._assert_converged("leader_velocity", rms_history, first_below_s, final_rms_rad)

    def test_all_agents_move_and_the_shape_still_converges(self) -> None:
        CONFIG.validate()
        self.assertEqual(CONFIG.bearing_control.agent_mode, "all_bearing")
        rng = random.Random(20260728)
        starts = _sample_positions(
            rng, 4, CONFIG.preflight.min_separation_m + 0.1
        )
        positions = {
            vehicle_id: xy for vehicle_id, xy in zip(CONFIG.vehicle_ids, starts)
        }
        rms_history, first_below_s, final_rms_rad, path_lengths = self._run(
            CONFIG, positions
        )
        self._assert_converged("all_bearing", rms_history, first_below_s, final_rms_rad)
        print(
            "convergence[all_bearing] | path lengths: "
            + ", ".join(
                "%s=%.3f m" % (vehicle_id, length)
                for vehicle_id, length in sorted(path_lengths.items())
            )
        )
        for vehicle_id, length in path_lengths.items():
            self.assertGreater(
                length, 0.1, "%s stayed (nearly) stationary in all_bearing mode" % vehicle_id
            )


if __name__ == "__main__":
    unittest.main()
