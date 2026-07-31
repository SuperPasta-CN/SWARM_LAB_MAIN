"""Pre-flight safety checks decoupled from ROS and vehicle IO.

The checks exist because of a real incident: in v1 the leaders were placed
with their baseline rotated ~87 deg from the desired bearing, the
followers' reachable configuration degenerated into an 8 mm slit and the
formation never converged.  v2 replaces v1's pinned leader-leader anchor
hard check with a *soft* captain-edge warning: there is exactly one captain
and the first mates can correct themselves, so a rotated initial placement
no longer degenerates the reachable set — it only slows convergence.  The
minimum-separation check below remains a hard gate and cannot be skipped
with ``--yes``.

The core :func:`run_preflight_checks` is a pure function of a snapshot plus
configuration so unit tests can exercise it without any hardware.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from math import atan2, degrees, sqrt
from typing import Callable, List, Optional, Sequence, Tuple

from swarm.algorithms.bearing import FormationTopology
from swarm.application.interfaces import StateSource
from swarm.domain.config import PreflightConfig
from swarm.domain.models import MocapSnapshot, VehicleState


@dataclass(frozen=True)
class CaptainEdgeWarning:
    """One captain-incident edge whose initial bearing error is large.

    Soft by design: printed for the operator, never blocks takeoff.
    """

    source_id: str
    target_id: str
    desired: Tuple[float, float]
    actual: Tuple[float, float]
    deviation_deg: float


@dataclass(frozen=True)
class SeparationFailure:
    """One vehicle pair closer than the minimum separation."""

    first_id: str
    second_id: str
    distance_m: float


@dataclass(frozen=True)
class BearingEdgeRow:
    """Initial bearing error of one directed topology edge (for the table)."""

    source_id: str
    target_id: str
    desired: Tuple[float, float]
    actual: Tuple[float, float]
    error_deg: Optional[float]


@dataclass
class PreflightReport:
    """Structured outcome of all pre-flight checks."""

    ok: bool = False
    waiting_for: List[str] = field(default_factory=list)
    captain_edge_warnings: List[CaptainEdgeWarning] = field(default_factory=list)
    separation_failures: List[SeparationFailure] = field(default_factory=list)
    positions: List[Tuple[str, float, float]] = field(default_factory=list)
    edge_rows: List[BearingEdgeRow] = field(default_factory=list)


def _normalize2(vector: Tuple[float, float]) -> Tuple[float, float]:
    norm = sqrt(vector[0] * vector[0] + vector[1] * vector[1])
    if norm < 1e-12:
        return 0.0, 0.0
    return vector[0] / norm, vector[1] / norm


def _angle_between_deg(first: Tuple[float, float], second: Tuple[float, float]) -> float:
    dot = first[0] * second[0] + first[1] * second[1]
    cross = first[0] * second[1] - first[1] * second[0]
    return degrees(atan2(abs(cross), dot))


def run_preflight_checks(
    snapshot: MocapSnapshot,
    topology: FormationTopology,
    config: PreflightConfig,
) -> PreflightReport:
    """Evaluate every check against one mocap snapshot.

    Planar (x, y) bearings are used: the vehicles drive on the floor and the
    desired topology is expressed in the world x/y plane.

    Edges incident to the captain whose initial bearing error exceeds
    ``captain_edge_warn_deg`` are reported as soft warnings only.
    """

    report = PreflightReport()
    vehicle_ids = topology.vehicle_ids

    # 1. mocap validity
    states = {}
    for vehicle_id in vehicle_ids:
        state = snapshot.states.get(vehicle_id)
        if state is None or not state.valid:
            report.waiting_for.append(vehicle_id)
        else:
            states[vehicle_id] = state
    report.positions = [
        (vehicle_id, states[vehicle_id].x, states[vehicle_id].y)
        for vehicle_id in vehicle_ids
        if vehicle_id in states
    ]

    # 2. soft warning on captain-incident edges with large initial error
    for i, source_id in enumerate(vehicle_ids):
        for j, target_id in enumerate(vehicle_ids):
            if not topology.adjacency_matrix[i][j]:
                continue
            if topology.roles[i] != "captain" and topology.roles[j] != "captain":
                continue
            desired = _normalize2(
                (
                    float(topology.bearing_matrix[i][j][0]),
                    float(topology.bearing_matrix[i][j][1]),
                )
            )
            source = states.get(source_id)
            target = states.get(target_id)
            if source is None or target is None:
                continue
            actual = _normalize2((target.x - source.x, target.y - source.y))
            deviation = _angle_between_deg(desired, actual)
            if deviation > config.captain_edge_warn_deg:
                report.captain_edge_warnings.append(
                    CaptainEdgeWarning(
                        source_id=source_id,
                        target_id=target_id,
                        desired=desired,
                        actual=actual,
                        deviation_deg=deviation,
                    )
                )

    # 3. minimum pairwise separation
    for i, first_id in enumerate(vehicle_ids):
        first = states.get(first_id)
        if first is None:
            continue
        for second_id in vehicle_ids[i + 1 :]:
            second = states.get(second_id)
            if second is None:
                continue
            distance = sqrt((first.x - second.x) ** 2 + (first.y - second.y) ** 2)
            if distance < config.min_separation_m:
                report.separation_failures.append(
                    SeparationFailure(
                        first_id=first_id,
                        second_id=second_id,
                        distance_m=distance,
                    )
                )

    # 4. per-edge initial bearing error table (informational)
    for i, source_id in enumerate(vehicle_ids):
        source = states.get(source_id)
        for j, target_id in enumerate(vehicle_ids):
            if not topology.adjacency_matrix[i][j]:
                continue
            desired = _normalize2(
                (
                    float(topology.bearing_matrix[i][j][0]),
                    float(topology.bearing_matrix[i][j][1]),
                )
            )
            target = states.get(target_id)
            if source is None or target is None:
                continue
            actual = _normalize2((target.x - source.x, target.y - source.y))
            overlap = sqrt((target.x - source.x) ** 2 + (target.y - source.y) ** 2) < 1e-9
            error_deg = None if overlap else _angle_between_deg(desired, actual)
            report.edge_rows.append(
                BearingEdgeRow(
                    source_id=source_id,
                    target_id=target_id,
                    desired=desired,
                    actual=actual,
                    error_deg=error_deg,
                )
            )

    report.ok = not report.waiting_for and not report.separation_failures
    return report


def wait_for_mocap(
    state_source: StateSource,
    vehicle_ids: Sequence[str],
    timeout_s: float,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    progress: bool = True,
) -> bool:
    """Block until every vehicle reports a valid mocap state or time out."""

    deadline = clock() + timeout_s
    last_print = 0.0
    while True:
        if state_source.all_valid():
            return True
        now = clock()
        if now >= deadline:
            return False
        if progress and now - last_print >= 1.0:
            last_print = now
            missing = [
                vehicle_id
                for vehicle_id in vehicle_ids
                if not _state_valid(state_source, vehicle_id)
            ]
            print(
                "preflight | waiting for mocap (%d/%d valid), missing: %s"
                % (
                    len(vehicle_ids) - len(missing),
                    len(vehicle_ids),
                    ", ".join(missing) if missing else "-",
                )
            )
        sleep(0.05)


def _state_valid(state_source: StateSource, vehicle_id: str) -> bool:
    snapshot = state_source.get_snapshot()
    state: Optional[VehicleState] = snapshot.states.get(vehicle_id)
    return state is not None and state.valid


def print_preflight_report(report: PreflightReport) -> None:
    """Render positions, the per-edge bearing table, warnings, and failures."""

    print("preflight | vehicle positions (world frame):")
    for vehicle_id, x, y in report.positions:
        print("  %-8s x=%+.3f m  y=%+.3f m" % (vehicle_id, x, y))
    if report.waiting_for:
        print("preflight | mocap INVALID for: %s" % ", ".join(report.waiting_for))

    if report.edge_rows:
        print("preflight | initial directed-edge bearing errors:")
        for row in report.edge_rows:
            error_text = "%.2f deg" % row.error_deg if row.error_deg is not None else "INVALID"
            print(
                "  %s -> %s  desired=(%+.3f,%+.3f) actual=(%+.3f,%+.3f)  error=%s"
                % (
                    row.source_id,
                    row.target_id,
                    row.desired[0],
                    row.desired[1],
                    row.actual[0],
                    row.actual[1],
                    error_text,
                )
            )

    for warning in report.captain_edge_warnings:
        print(
            "preflight WARN | captain edge %s -> %s: "
            "desired=(%+.3f,%+.3f) actual=(%+.3f,%+.3f) deviation=%.1f deg "
            "(soft warning, takeoff not blocked)"
            % (
                warning.source_id,
                warning.target_id,
                warning.desired[0],
                warning.desired[1],
                warning.actual[0],
                warning.actual[1],
                warning.deviation_deg,
            )
        )
    for failure in report.separation_failures:
        print(
            "preflight FAIL | %s and %s are %.3f m apart (below minimum separation)"
            % (failure.first_id, failure.second_id, failure.distance_m)
        )
    if report.ok:
        print("preflight | all checks passed")


def confirm_or_abort(
    report: PreflightReport,
    require_confirmation: bool,
    assume_yes: bool,
    input_fn: Callable[[str], str] = input,
) -> bool:
    """Return True only when the hard checks passed and the operator agreed."""

    if not report.ok:
        print("preflight | hard checks failed, refusing to start the motors")
        return False
    if not require_confirmation or assume_yes:
        return True
    try:
        input_fn("preflight | checks passed, press ENTER to unlock the motors (Ctrl-C to abort) ")
    except (KeyboardInterrupt, EOFError):
        print("\npreflight | aborted by operator")
        return False
    return True
