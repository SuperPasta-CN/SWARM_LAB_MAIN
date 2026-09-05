"""Preflight checks tests (v3, meters)."""

from __future__ import annotations

import unittest

from swarm.algorithms.task_driven import build_formation_spec
from swarm.application.preflight import (
    confirm_or_abort,
    run_preflight_checks,
)
from swarm.domain.config import PreflightConfig
from swarm.domain.models import MocapSnapshot, VehicleState


def _spec():
    return build_formation_spec(
        vehicle_ids=["car1", "car2"],
        adjacency_matrix=((0, 1), (1, 0)),
        formation={"car1": (0.0, 0.0), "car2": (0.0, 1.0)},
    )


def _snapshot(positions, valid=True):
    states = {}
    for vehicle_id, (x, y) in positions.items():
        state = VehicleState(vehicle_id, x=x, y=y)
        state.valid = valid
        states[vehicle_id] = state
    return MocapSnapshot(0.0, states)


class RunPreflightChecksTests(unittest.TestCase):
    def test_all_valid_on_formation(self) -> None:
        report = run_preflight_checks(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)}),
            _spec(),
            PreflightConfig(),
        )
        self.assertTrue(report.ok)
        self.assertAlmostEqual(report.formation_rms_m, 0.0, places=9)
        self.assertFalse(report.formation_warn)
        self.assertEqual(len(report.edge_rows), 2)  # both directed edges listed

    def test_missing_mocap_blocks(self) -> None:
        report = run_preflight_checks(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)}, valid=False),
            _spec(),
            PreflightConfig(),
        )
        self.assertFalse(report.ok)
        self.assertEqual(sorted(report.waiting_for), ["car1", "car2"])

    def test_close_pair_is_hard_failure(self) -> None:
        report = run_preflight_checks(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 0.10)}),
            _spec(),
            PreflightConfig(),
        )
        self.assertFalse(report.ok)
        self.assertEqual(len(report.separation_failures), 1)
        self.assertAlmostEqual(report.separation_failures[0].distance_m, 0.10)

    def test_large_initial_error_is_soft_warning_only(self) -> None:
        report = run_preflight_checks(
            # 0.5 m off in x on the car2 edge -> RMS ~0.35 m > 0.15 warn
            _snapshot({"car1": (0.0, 0.0), "car2": (0.5, 1.0)}),
            _spec(),
            PreflightConfig(),
        )
        self.assertTrue(report.ok)
        self.assertTrue(report.formation_warn)
        self.assertGreater(report.formation_rms_m, 0.15)

    def test_edge_error_matches_relative_position(self) -> None:
        report = run_preflight_checks(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.3, 1.0)}),
            _spec(),
            PreflightConfig(),
        )
        row = next(r for r in report.edge_rows if r.source_id == "car1")
        self.assertAlmostEqual(row.desired[1], 1.0)
        self.assertAlmostEqual(row.actual[0], 0.3)
        self.assertAlmostEqual(row.error_m, 0.3)


class ConfirmOrAbortTests(unittest.TestCase):
    def test_failed_report_cannot_be_confirmed(self) -> None:
        report = run_preflight_checks(
            _snapshot({"car1": (0.0, 0.0)}, valid=True),
            _spec(),
            PreflightConfig(),
        )
        self.assertFalse(confirm_or_abort(report, require_confirmation=False, assume_yes=True))

    def test_assume_yes_passes_good_report(self) -> None:
        report = run_preflight_checks(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)}),
            _spec(),
            PreflightConfig(),
        )
        self.assertTrue(confirm_or_abort(report, require_confirmation=True, assume_yes=True))

    def test_operator_abort(self) -> None:
        report = run_preflight_checks(
            _snapshot({"car1": (0.0, 0.0), "car2": (0.0, 1.0)}),
            _spec(),
            PreflightConfig(),
        )

        def _abort(_prompt):
            raise KeyboardInterrupt

        self.assertFalse(
            confirm_or_abort(report, require_confirmation=True, assume_yes=False, input_fn=_abort)
        )


if __name__ == "__main__":
    unittest.main()
