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
from configs.crew_four import CONFIG


def _topology():
    return build_swarm_topology(CONFIG)


def build_swarm_topology(config):
    return build_topology_from_matrices(
        vehicle_ids=config.vehicle_ids,
        adjacency_matrix=config.topology.adjacency_matrix,
        bearing_matrix=config.topology.bearing_matrix,
        roles=[vehicle.role for vehicle in config.vehicles],
    )


def _snapshot(positions, invalid=()) -> MocapSnapshot:
    states = {}
    for vehicle_id, xy in positions.items():
        states[vehicle_id] = VehicleState(
            vehicle_id, x=xy[0], y=xy[1], valid=vehicle_id not in invalid
        )
    return MocapSnapshot(1.0, states)


class PreflightCaptainEdgeWarningTests(unittest.TestCase):
    def test_incident_scenario_is_now_a_soft_warning(self) -> None:
        # Replay of the v1 accident geometry: car2 placed on the +y side of
        # car1 while the desired car1->car2 bearing is world +x (~90 deg
        # off).  v2 has a single captain and a self-correcting first mate,
        # so this must warn but never block takeoff.
        snapshot = _snapshot(
            {
                "car1": (0.0, 0.0),
                "car2": (0.0, 1.0),
                "car3": (0.0, -1.0),
                "car4": (1.0, -1.0),
            }
        )
        report = run_preflight_checks(snapshot, _topology(), CONFIG.preflight)
        self.assertTrue(report.ok)
        warned_edges = {
            (warning.source_id, warning.target_id)
            for warning in report.captain_edge_warnings
        }
        self.assertIn(("car1", "car2"), warned_edges)
        self.assertIn(("car2", "car1"), warned_edges)
        for warning in report.captain_edge_warnings:
            self.assertGreater(
                warning.deviation_deg, CONFIG.preflight.captain_edge_warn_deg
            )
        forward = [
            warning
            for warning in report.captain_edge_warnings
            if warning.source_id == "car1"
        ][0]
        self.assertGreater(forward.deviation_deg, 80.0)
        self.assertAlmostEqual(forward.desired[0], 1.0, places=6)
        self.assertAlmostEqual(forward.desired[1], 0.0, places=6)
        self.assertGreater(forward.actual[1], 0.9)  # actual bearing points +y

    def test_consistent_placement_passes_without_warnings(self) -> None:
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
        self.assertFalse(report.captain_edge_warnings)
        self.assertFalse(report.separation_failures)
        self.assertEqual(len(report.positions), 4)
        self.assertTrue(report.edge_rows)
        for row in report.edge_rows:
            self.assertIsNotNone(row.error_deg)
            self.assertLess(row.error_deg, 1.0)

    def test_warning_threshold_boundary(self) -> None:
        tolerance = 20.0
        config = PreflightConfig(captain_edge_warn_deg=tolerance)
        # Rotate the actual car1->car2 bearing slightly past the threshold:
        # warned, but still not blocked.
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
        self.assertTrue(report.ok)
        self.assertTrue(report.captain_edge_warnings)

    def test_crew_only_edges_never_warn(self) -> None:
        # Chain topology car1-captain -> car2-first_mate -> car3-crew ->
        # car4-crew: car4 has no edge to the captain, so misplacing it must
        # not produce any soft warning regardless of the bearing error.
        topology = build_topology_from_matrices(
            vehicle_ids=["car1", "car2", "car3", "car4"],
            adjacency_matrix=(
                (0, 1, 0, 0),
                (1, 0, 1, 0),
                (0, 1, 0, 1),
                (0, 0, 1, 0),
            ),
            bearing_matrix=(
                ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                ((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            roles=["captain", "first_mate", "crew", "crew"],
        )
        snapshot = _snapshot(
            {
                "car1": (0.0, 0.0),
                "car2": (1.0, 0.0),
                "car3": (2.0, 0.0),
                "car4": (2.0, -1.0),  # desired car3->car4 is +x, actual -y
            }
        )
        report = run_preflight_checks(snapshot, topology, CONFIG.preflight)
        self.assertTrue(report.ok)
        self.assertFalse(report.captain_edge_warnings)


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
        self.assertFalse(report.captain_edge_warnings)
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
