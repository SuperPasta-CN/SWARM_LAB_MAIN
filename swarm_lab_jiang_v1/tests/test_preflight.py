"""Unit tests for the preflight checks, including the incident replay."""

from __future__ import annotations

import unittest

from swarm.algorithms.bearing import build_topology_from_matrices
from swarm.application.preflight import (
    PreflightReport,
    confirm_or_abort,
    run_preflight_checks,
    wait_for_mocap,
)
from swarm.domain.config import PreflightConfig
from swarm.domain.models import MocapSnapshot, VehicleState
from configs.bearing_four import CONFIG


def _topology():
    return build_swarm_topology(CONFIG)


def build_swarm_topology(config):
    return build_topology_from_matrices(
        vehicle_ids=config.vehicle_ids,
        adjacency_matrix=config.topology.adjacency_matrix,
        bearing_matrix=config.topology.bearing_matrix,
        leader_mask=config.topology.leader_mask,
    )


def _snapshot(positions, invalid=()) -> MocapSnapshot:
    states = {}
    for vehicle_id, xy in positions.items():
        states[vehicle_id] = VehicleState(
            vehicle_id, x=xy[0], y=xy[1], valid=vehicle_id not in invalid
        )
    return MocapSnapshot(1.0, states)


class PreflightAnchorTests(unittest.TestCase):
    def test_incident_scenario_is_rejected(self) -> None:
        # Replay of the real accident: car2 placed on the +y side of car1
        # while the desired car1->car2 bearing is world +x (rotated ~87 deg).
        snapshot = _snapshot(
            {
                "car1": (0.399, 0.689),
                "car2": (0.407, 0.878),
                "car3": (0.4, -0.3),
                "car4": (1.4, -0.3),
            }
        )
        report = run_preflight_checks(snapshot, _topology(), CONFIG.preflight)
        self.assertFalse(report.ok)
        failing_edges = {
            (failure.source_id, failure.target_id) for failure in report.anchor_failures
        }
        self.assertIn(("car1", "car2"), failing_edges)
        self.assertIn(("car2", "car1"), failing_edges)
        for failure in report.anchor_failures:
            self.assertGreater(failure.deviation_deg, CONFIG.preflight.anchor_tolerance_deg)
        forward = [f for f in report.anchor_failures if f.source_id == "car1"][0]
        self.assertGreater(forward.deviation_deg, 80.0)  # measured ~87.6 deg
        self.assertAlmostEqual(forward.desired[0], 1.0, places=6)
        self.assertAlmostEqual(forward.desired[1], 0.0, places=6)
        self.assertGreater(forward.actual[1], 0.9)  # actual bearing points +y

    def test_consistent_leader_baseline_passes(self) -> None:
        snapshot = _snapshot(
            {
                "car1": (0.0, 0.0),
                "car2": (1.0, 0.0),
                "car3": (0.0, -1.0),
                "car4": (1.0, -1.0),
            }
        )
        report = run_preflight_checks(snapshot, _topology(), CONFIG.preflight)
        self.assertTrue(report.ok)
        self.assertFalse(report.anchor_failures)
        self.assertFalse(report.separation_failures)
        self.assertEqual(len(report.positions), 4)
        self.assertTrue(report.edge_rows)
        for row in report.edge_rows:
            self.assertIsNotNone(row.error_deg)
            self.assertLess(row.error_deg, 1.0)

    def test_anchor_tolerance_boundary(self) -> None:
        tolerance = 20.0
        config = PreflightConfig(anchor_tolerance_deg=tolerance)
        # rotate the actual car1->car2 bearing by slightly more than tolerated
        import math

        angle = math.radians(tolerance + 1.0)
        snapshot = _snapshot(
            {
                "car1": (0.0, 0.0),
                "car2": (math.cos(angle), math.sin(angle)),
                "car3": (0.0, -1.0),
                "car4": (1.0, -1.0),
            }
        )
        report = run_preflight_checks(snapshot, _topology(), config)
        self.assertFalse(report.ok)
        self.assertTrue(report.anchor_failures)

    def test_rotated_baseline_accepted_when_anchors_unchecked(self) -> None:
        # all_bearing mode: the formation is free to rotate, so a baseline
        # rotated ~90 deg from the desired bearings is no longer fatal.
        snapshot = _snapshot(
            {
                "car1": (0.0, 0.0),
                "car2": (0.0, 1.0),  # desired car1->car2 is +x, actual is +y
                "car3": (0.0, -1.0),
                "car4": (1.0, -1.0),
            }
        )
        report = run_preflight_checks(
            snapshot, _topology(), CONFIG.preflight, check_anchors=False
        )
        self.assertTrue(report.ok)
        self.assertFalse(report.anchor_failures)


class PreflightSeparationTests(unittest.TestCase):
    def test_minimum_separation_violation_is_rejected(self) -> None:
        snapshot = _snapshot(
            {
                "car1": (0.0, 0.0),
                "car2": (0.1, 0.0),  # direction fine, distance too small
                "car3": (0.0, -1.0),
                "car4": (1.0, -1.0),
            }
        )
        report = run_preflight_checks(snapshot, _topology(), CONFIG.preflight)
        self.assertFalse(report.ok)
        self.assertFalse(report.anchor_failures)
        self.assertEqual(len(report.separation_failures), 1)
        failure = report.separation_failures[0]
        self.assertEqual({failure.first_id, failure.second_id}, {"car1", "car2"})
        self.assertAlmostEqual(failure.distance_m, 0.1, places=9)

    def test_missing_mocap_is_reported(self) -> None:
        snapshot = _snapshot(
            {
                "car1": (0.0, 0.0),
                "car2": (1.0, 0.0),
                "car3": (0.0, -1.0),
                "car4": (1.0, -1.0),
            },
            invalid=("car4",),
        )
        report = run_preflight_checks(snapshot, _topology(), CONFIG.preflight)
        self.assertFalse(report.ok)
        self.assertEqual(report.waiting_for, ["car4"])


class _FakeSource:
    def __init__(self, valid):
        self._valid = valid
        self.states = {
            vehicle_id: VehicleState(vehicle_id, valid=valid)
            for vehicle_id in ("car1", "car2")
        }

    def all_valid(self) -> bool:
        return self._valid

    def get_snapshot(self) -> MocapSnapshot:
        return MocapSnapshot(1.0, self.states)


class PreflightWaitTests(unittest.TestCase):
    def test_wait_for_mocap_returns_immediately_when_valid(self) -> None:
        self.assertTrue(
            wait_for_mocap(
                _FakeSource(True),
                ("car1", "car2"),
                timeout_s=0.01,
                progress=False,
            )
        )

    def test_wait_for_mocap_times_out(self) -> None:
        now = [100.0]

        def clock() -> float:
            return now[0]

        def sleep(seconds: float) -> None:
            now[0] += seconds

        self.assertFalse(
            wait_for_mocap(
                _FakeSource(False),
                ("car1", "car2"),
                timeout_s=0.2,
                clock=clock,
                sleep=sleep,
                progress=False,
            )
        )
        self.assertGreaterEqual(now[0], 100.2)


class PreflightConfirmationTests(unittest.TestCase):
    def _ok_report(self) -> PreflightReport:
        report = PreflightReport()
        report.ok = True
        return report

    def test_hard_failure_cannot_be_skipped_by_assume_yes(self) -> None:
        report = PreflightReport()  # ok == False
        self.assertFalse(
            confirm_or_abort(report, require_confirmation=False, assume_yes=True)
        )

    def test_assume_yes_skips_only_the_confirmation(self) -> None:
        self.assertTrue(
            confirm_or_abort(
                self._ok_report(),
                require_confirmation=True,
                assume_yes=True,
                input_fn=lambda prompt: (_ for _ in ()).throw(AssertionError("must not prompt")),
            )
        )

    def test_enter_unlocks_and_eof_aborts(self) -> None:
        self.assertTrue(
            confirm_or_abort(
                self._ok_report(),
                require_confirmation=True,
                assume_yes=False,
                input_fn=lambda prompt: "",
            )
        )

        def _eof(prompt):
            raise EOFError

        self.assertFalse(
            confirm_or_abort(
                self._ok_report(),
                require_confirmation=True,
                assume_yes=False,
                input_fn=_eof,
            )
        )


if __name__ == "__main__":
    unittest.main()
