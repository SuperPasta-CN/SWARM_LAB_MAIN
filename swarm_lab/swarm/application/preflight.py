"""Pre-flight safety checks decoupled from ROS and vehicle IO (v3).

Hard gates (cannot be skipped with ``--yes``): mocap validity and minimum
pairwise separation.  The initial formation RMS error (meters) is reported
as a *soft* warning only — the v3 constraint law pulls the formation to the
desired coordinates from any consistent placement, so a large initial error
only costs convergence time.

The core :func:`run_preflight_checks` is a pure function of a snapshot plus
configuration so unit tests can exercise it without any hardware.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from math import sqrt
from typing import Callable, List, Optional, Sequence, Tuple

from swarm.algorithms.task_driven import FormationSpec
from swarm.application.interfaces import StateSource
from swarm.domain.config import PreflightConfig
from swarm.domain.models import MocapSnapshot, VehicleState


@dataclass(frozen=True)
class SeparationFailure:
    """One vehicle pair closer than the minimum separation."""

    first_id: str
    second_id: str
    distance_m: float


@dataclass(frozen=True)
class EdgeErrorRow:
    """Initial relative-position error of one directed edge (meters)."""

    source_id: str
    target_id: str
    desired: Tuple[float, float]
    actual: Tuple[float, float]
    error_m: float


@dataclass
class PreflightReport:
    """Structured outcome of all pre-flight checks."""

    ok: bool = False
    waiting_for: List[str] = field(default_factory=list)
    separation_failures: List[SeparationFailure] = field(default_factory=list)
    positions: List[Tuple[str, float, float]] = field(default_factory=list)
    edge_rows: List[EdgeErrorRow] = field(default_factory=list)
    formation_rms_m: float = 0.0
    formation_warn: bool = False


def run_preflight_checks(
    snapshot: MocapSnapshot,
    spec: FormationSpec,
    config: PreflightConfig,
) -> PreflightReport:
    """Evaluate every check against one mocap snapshot (planar, meters)."""

    report = PreflightReport()
    vehicle_ids = spec.vehicle_ids

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

    # 2. minimum pairwise separation (hard gate)
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

    # 3. per-edge initial error table (task_driven: relative-position error in
    # meters; the comparison bearing planner has no distance metric, so the
    # table is skipped when the spec carries no b_matrix)
    adjacency = getattr(spec, "adjacency", None)
    if adjacency is None:
        adjacency = spec.adjacency_matrix
    b_matrix = getattr(spec, "b_matrix", None)
    squared = []
    if b_matrix is not None:
        for i, source_id in enumerate(vehicle_ids):
            source = states.get(source_id)
            for j, target_id in enumerate(vehicle_ids):
                if not adjacency[i][j]:
                    continue
                target = states.get(target_id)
                if source is None or target is None:
                    continue
                desired = (float(b_matrix[i, j, 0]), float(b_matrix[i, j, 1]))
                actual = (target.x - source.x, target.y - source.y)
                error = sqrt(
                    (actual[0] - desired[0]) ** 2 + (actual[1] - desired[1]) ** 2
                )
                squared.append(error * error)
                report.edge_rows.append(
                    EdgeErrorRow(
                        source_id=source_id,
                        target_id=target_id,
                        desired=desired,
                        actual=actual,
                        error_m=error,
                    )
                )
    if squared:
        report.formation_rms_m = sqrt(sum(squared) / len(squared))
        report.formation_warn = report.formation_rms_m > config.formation_warn_m

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
    """Render positions, the per-edge error table, warnings, and failures."""

    print("preflight | vehicle positions (world frame):")
    for vehicle_id, x, y in report.positions:
        print("  %-8s x=%+.3f m  y=%+.3f m" % (vehicle_id, x, y))
    if report.waiting_for:
        print("preflight | mocap INVALID for: %s" % ", ".join(report.waiting_for))

    if report.edge_rows:
        print("preflight | initial directed-edge relative-position errors:")
        for row in report.edge_rows:
            print(
                "  %s -> %s  desired=(%+.3f,%+.3f) actual=(%+.3f,%+.3f)  error=%.3f m"
                % (
                    row.source_id,
                    row.target_id,
                    row.desired[0],
                    row.desired[1],
                    row.actual[0],
                    row.actual[1],
                    row.error_m,
                )
            )
    print("preflight | initial formation RMS error: %.3f m" % report.formation_rms_m)
    if report.formation_warn:
        print(
            "preflight WARN | initial formation error above threshold "
            "(soft warning, takeoff not blocked; convergence just takes longer)"
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
