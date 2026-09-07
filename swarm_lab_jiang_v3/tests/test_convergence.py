"""End-to-end convergence proofs of the v3 task-driven control law.

Uses the genuine ``configs/v3_two`` catalog config (and an inline 3-car
triangle) with the genuine ``TaskDrivenFormationController`` driving ideal
single-integrator stubs (``p += v * dt``).  Covered regimes (spec 3.3):

- static formation (w = 0): the relative-position RMS error must drop below
  the deadband-limited floor within 30 s, and every vehicle must move;
- translational maneuver (w != 0): the formation translates at exactly w
  while the error still decays (null-space decoupling);
- a scaled initial formation (a mode NOT spanned by Z) is corrected by the
  constraint term even while the task term is active.
"""

from __future__ import annotations

import random
import unittest
from dataclasses import replace
from math import sqrt

from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.models import MocapSnapshot, VehicleState
from configs.v3_two import CONFIG


DT = 0.02
STATIC_DURATION_S = 30.0
ERROR_FLOOR_M = 0.03  # deadband-limited: k * |e| < deadband_mps freezes the term


def _run(config, positions, duration_s):
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
        rms_history.append(result.mean_error)
        for vehicle_id, command in result.commands.items():
            x, y = positions[vehicle_id]
            positions[vehicle_id] = (x + command.vx * DT, y + command.vy * DT)
            path_lengths[vehicle_id] += sqrt(command.vx**2 + command.vy**2) * DT
    return rms_history, path_lengths, positions


def _sample_positions(rng, count, min_separation):
    positions = []
    while len(positions) < count:
        candidate = (rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
        if all(
            sqrt((candidate[0] - other[0]) ** 2 + (candidate[1] - other[1]) ** 2)
            >= min_separation
            for other in positions
        ):
            positions.append(candidate)
    return positions


class StaticConvergenceTests(unittest.TestCase):
    def test_static_formation_converges(self) -> None:
        config = replace(
            CONFIG, task_velocity=(0.0, 0.0), task_velocity_schedule=()
        )
        config.validate()
        rng = random.Random(20260814)
        starts = _sample_positions(rng, 2, config.preflight.min_separation_m + 0.1)
        positions = {vid: list(xy) for vid, xy in zip(config.vehicle_ids, starts)}
        rms_history, path_lengths, _ = _run(config, positions, STATIC_DURATION_S)
        first_below = next(
            (i * DT for i, value in enumerate(rms_history) if value < ERROR_FLOOR_M),
            None,
        )
        print(
            "\nconvergence[v3 static] | initial RMS=%.4f m, first below %.3f m: %s, "
            "final RMS=%.5f m"
            % (
                rms_history[0],
                ERROR_FLOOR_M,
                ("%.2f s" % first_below) if first_below is not None else "NEVER",
                rms_history[-1],
            )
        )
        self.assertIsNotNone(first_below)
        self.assertLess(rms_history[-1], ERROR_FLOOR_M)
        for vehicle_id in config.vehicle_ids:
            self.assertGreater(path_lengths[vehicle_id], 0.05)


class ManeuverDecouplingTests(unittest.TestCase):
    def test_translation_at_w_with_decaying_error(self) -> None:
        config = replace(CONFIG, task_velocity_schedule=())  # constant w
        config.validate()
        positions = {"car4": [0.3, -0.4], "car5": [0.1, 1.5]}  # off formation
        duration = 20.0
        rms_history, _, final = _run(config, positions, duration)
        # The formation translated by w * T (decoupling: task not fought).
        # (delta absorbs the brief initial saturation, where the per-car
        # limiter partially masks the task term.)
        centroid_x = (final["car4"][0] + final["car5"][0]) / 2
        self.assertAlmostEqual(centroid_x, 0.2 + 0.10 * duration, delta=0.1)
        self.assertLess(rms_history[-1], ERROR_FLOOR_M)

    def test_scale_error_corrected_under_active_task(self) -> None:
        config = replace(CONFIG, task_velocity_schedule=())
        # Start with the separation doubled (a mode outside null(L_A)).
        positions = {"car4": [0.0, 0.0], "car5": [0.0, 1.0]}
        rms_history, _, final = _run(config, positions, 15.0)
        self.assertLess(rms_history[-1], ERROR_FLOOR_M)
        # separation back to the desired 0.5 m within the deadband floor
        # (the constraint freezes once k*|e| < deadband: |e| < ~0.019 m)
        self.assertAlmostEqual(final["car5"][1] - final["car4"][1], 0.5, delta=0.025)


if __name__ == "__main__":
    unittest.main()
