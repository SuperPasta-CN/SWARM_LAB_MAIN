"""End-to-end convergence proofs of the real bearing law and configuration.

Uses the genuine ``configs/crew_four`` topology and the genuine
``BearingOnlyFormationController`` driving ideal single-integrator stubs
(``p += v * dt``).  Three regimes are covered:

- static formation (``captain_velocity = 0``): the captain is pinned, the
  first mate blends a zero fixed velocity, and the crew run the pure
  bearing law; the directed-edge bearing RMS must drop below 5 degrees
  within 30 seconds, and every non-captain vehicle must actually move.
- translational maneuver (captain moving at 0.05 m/s): with the pure P
  law (``ki = 0``) the formation tracks with a constant steady-state
  error; with the integral term enabled (``ki > 0``) that error must be
  eliminated (Zhao & Zelazo, bearing-based formation maneuvering).
"""

from __future__ import annotations

import random
import unittest
from dataclasses import replace
from math import atan2, degrees, pi, sqrt

from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.config import BearingControlConfig
from swarm.domain.models import MocapSnapshot, VehicleState
from configs.crew_four import CONFIG


DT = 0.02
STATIC_DURATION_S = 30.0
MANEUVER_DURATION_S = 60.0
RMS_TARGET_RAD = 5.0 * pi / 180.0

# Fixtures pin every field they depend on (catalog crew_four carries a
# rotate-first schedule and ki>0 since 08-14): nothing is inherited by accident.
STATIC_CONFIG = replace(
    CONFIG, captain_velocity=(0.0, 0.0, 0.0), captain_velocity_schedule=()
)
MANEUVER_P_CONFIG = replace(
    CONFIG,
    captain_velocity=(0.05, 0.0, 0.0),
    captain_velocity_schedule=(),
    bearing_control=BearingControlConfig(kp=0.6, ki=0.0),
)
MANEUVER_PI_CONFIG = replace(
    CONFIG,
    captain_velocity=(0.05, 0.0, 0.0),
    captain_velocity_schedule=(),
    bearing_control=BearingControlConfig(kp=0.6, ki=0.3),
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


class CrewFourConvergenceTests(unittest.TestCase):
    def _run(self, config, positions, duration_s):
        planner = build_swarm_planner(config)
        steps = int(duration_s / DT)
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
            rms_history.append(_rms(_edge_angle_errors_rad(result)))
            for vehicle_id, command in result.commands.items():
                x, y = positions[vehicle_id]
                positions[vehicle_id] = (x + command.vx * DT, y + command.vy * DT)
                path_lengths[vehicle_id] += (
                    sqrt(command.vx * command.vx + command.vy * command.vy) * DT
                )
        return rms_history, path_lengths

    def test_static_formation_converges_below_five_degrees(self) -> None:
        STATIC_CONFIG.validate()
        rng = random.Random(20260728)
        starts = _sample_positions(
            rng, 4, CONFIG.preflight.min_separation_m + 0.1
        )
        positions = {
            vehicle_id: xy for vehicle_id, xy in zip(CONFIG.vehicle_ids, starts)
        }
        rms_history, path_lengths = self._run(STATIC_CONFIG, positions, STATIC_DURATION_S)
        first_below_s = next(
            (
                index * DT
                for index, rms_rad in enumerate(rms_history)
                if rms_rad < RMS_TARGET_RAD
            ),
            None,
        )
        print(
            "\nconvergence[static] | initial RMS=%.2f deg, first time below 5 deg: %s, "
            "final RMS=%.4f deg"
            % (
                degrees(rms_history[0]),
                ("%.2f s" % first_below_s) if first_below_s is not None else "NEVER",
                degrees(rms_history[-1]),
            )
        )
        self.assertIsNotNone(
            first_below_s,
            "bearing RMS never dropped below 5 deg within %s s" % STATIC_DURATION_S,
        )
        self.assertLess(rms_history[-1], RMS_TARGET_RAD)
        # The pinned captain must not move; everyone else must actually move.
        self.assertLess(path_lengths["car1"], 1e-9)
        for vehicle_id in ("car2", "car3", "car4"):
            self.assertGreater(
                path_lengths[vehicle_id],
                0.1,
                "%s stayed (nearly) stationary in the static run" % vehicle_id,
            )

    def test_maneuvering_pure_p_has_larger_steady_error_than_pi(self) -> None:
        MANEUVER_P_CONFIG.validate()
        MANEUVER_PI_CONFIG.validate()
        # Start from the exact desired square: the transient is then purely
        # the captain starting to move, and the last 10 s show the steady
        # tracking error of each law.
        square = {"car1": (0.0, 0.0), "car2": (1.0, 0.0), "car3": (0.0, -1.0), "car4": (1.0, -1.0)}
        window = int(10.0 / DT)
        rms_p, _ = self._run(MANEUVER_P_CONFIG, dict(square), MANEUVER_DURATION_S)
        rms_pi, _ = self._run(MANEUVER_PI_CONFIG, dict(square), MANEUVER_DURATION_S)
        steady_p = sum(rms_p[-window:]) / window
        steady_pi = sum(rms_pi[-window:]) / window
        print(
            "\nconvergence[maneuver] | steady RMS (last 10 s): pure P=%.3f deg, PI=%.3f deg"
            % (degrees(steady_p), degrees(steady_pi))
        )
        # The paper's claim: pure P tracks a moving leader only with a
        # constant error; the integral term removes it.
        self.assertGreater(steady_p, steady_pi + 0.8 * pi / 180.0)
        self.assertLess(steady_pi, RMS_TARGET_RAD)


if __name__ == "__main__":
    unittest.main()
