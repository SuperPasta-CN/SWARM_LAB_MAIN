"""Matrix-weighted Laplacian formation constraint + task-driven control (v3).

The control law reproduced here (paper/spec form):

    u_i = -k * sum_{j in N_i} A_ij (p_i - p_j + b_ij) + Z_i w

- ``p_i``: current world-frame position of robot ``i`` (planar, mocap x/y).
- ``b_ij = p_j* - p_i*``: desired relative position ("target minus source",
  consistent with the v1/v2 ``target - source`` convention).  The constraint
  ``p_i - p_j + b_ij = 0`` then holds exactly at the desired formation.
- ``A_ij``: edge weight matrix.  Scalar weights ``a_ij`` (i.e. ``a_ij * I_2``)
  for now; :class:`FormationSpec` stores full 2x2 blocks so matrix weights
  are a drop-in extension.
- ``k > 0``: formation gain.
- ``Z_i w``: task-driven term.  Columns of ``Z`` span allowed motion modes
  inside ``null(L_A)`` — for a connected graph with scalar weights the
  matrix-weighted Laplacian's null space is exactly global x/y translation
  (spec section 2.4) — so the task motion is never fought by the
  constraint term (decoupling).  ``Z_i`` is robot ``i``'s 2xq block of the
  global ``Z``; ``w`` holds the per-mode desired velocities (scheduleable).

Self-check (sign convention): desired ``p_1*=(0,0)``, ``p_2*=(0,1)``, hence
``b_12 = (0,1)``.  Place ``p_1=(0,0)``, ``p_2=(0.4,1)``::

    e_12 = (p_1 - p_2) + b_12 = (-0.4, 0)
    u_1  = -k * e_12 = (+0.4k, 0)   -> car1 moves +x, shrinking the error
    u_2  = -k * e_21 = (-0.4k, 0)   -> symmetric, centroid preserved

so the relative-position error decreases.  Unit tests encode this example.

With the displacement error (not a bearing) the constraint term drives the
error to zero *exactly*, and a uniform task term never disturbs the shape:
no integral term is needed for moving formations (contrast with v2).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt, tanh
from typing import List, Mapping, Optional, Sequence, Tuple

import numpy as np

from swarm.domain.config import TaskDrivenConfig
from swarm.domain.models import (
    EdgeSample,
    MocapSnapshot,
    PlannerResult,
    Vector3,
    VelocityCommand,
)

NAMED_TASK_MODES = ("translate_x", "translate_y")


def build_task_modes(count: int, modes: Sequence) -> np.ndarray:
    """Build the global task-mode matrix ``Z`` of shape ``(2n, q)``.

    Each entry of ``modes`` is either a name from ``NAMED_TASK_MODES``
    (canonical global-translation mode) or an explicit length-2n vector.
    """

    if count == 0:
        raise ValueError("vehicle count must be positive")
    columns = []
    for mode in modes:
        if isinstance(mode, str):
            if mode not in NAMED_TASK_MODES:
                raise ValueError(
                    "unknown task mode %r; available: %s" % (mode, NAMED_TASK_MODES)
                )
            column = np.zeros(2 * count)
            column[0 if mode == "translate_x" else 1 :: 2] = 1.0
            columns.append(column)
        else:
            column = np.asarray(mode, dtype=float).ravel()
            if column.size != 2 * count:
                raise ValueError(
                    "explicit task mode must have %d entries (2 per vehicle)"
                    % (2 * count)
                )
            columns.append(column)
    if not columns:
        return np.zeros((2 * count, 0))
    return np.column_stack(columns)


@dataclass(frozen=True)
class FormationSpec:
    """Topology + desired formation + edge weights + task modes for v3.

    ``b_matrix[i][j]`` is the desired relative position ``b_ij = p_j* - p_i*``
    ("target minus source"); ``weight_blocks[i][j]`` the 2x2 edge weight
    matrix ``A_ij``.  ``Z`` is the global ``(2n, q)`` task-mode matrix;
    robot ``i`` executes the block ``Z[2*i:2*i+2, :] @ w``.
    """

    vehicle_ids: List[str]
    adjacency: np.ndarray        # (n, n) of 0/1
    weight_blocks: np.ndarray    # (n, n, 2, 2)
    b_matrix: np.ndarray         # (n, n, 2)
    Z: np.ndarray                # (2n, q)

    def __post_init__(self) -> None:
        count = len(self.vehicle_ids)
        if count == 0:
            raise ValueError("vehicle_ids must not be empty")
        if self.adjacency.shape != (count, count):
            raise ValueError("adjacency must be square and match vehicle count")
        if self.weight_blocks.shape != (count, count, 2, 2):
            raise ValueError("weight_blocks must have shape (n, n, 2, 2)")
        if self.b_matrix.shape != (count, count, 2):
            raise ValueError("b_matrix must have shape (n, n, 2)")
        if self.Z.shape[0] != 2 * count:
            raise ValueError("Z must have 2n rows")

    def z_block(self, index: int) -> np.ndarray:
        """Robot ``index``'s 2xq block of the task-mode matrix."""

        return self.Z[2 * index : 2 * index + 2, :]


def build_formation_spec(
    vehicle_ids: Sequence[str],
    adjacency_matrix: Sequence[Sequence[int]],
    formation: Mapping[str, Tuple[float, float]],
    edge_weight_matrix: Optional[Sequence[Sequence[float]]] = None,
    task_modes: Sequence = ("translate_x", "translate_y"),
) -> FormationSpec:
    """Assemble a FormationSpec from config-level matrices and coordinates."""

    ids = list(vehicle_ids)
    count = len(ids)
    if count == 0:
        raise ValueError("vehicle_ids must not be empty")
    missing = [vid for vid in ids if vid not in formation]
    if missing:
        raise ValueError("formation is missing coordinates for: %s" % missing)
    if len(adjacency_matrix) != count or any(len(row) != count for row in adjacency_matrix):
        raise ValueError("adjacency_matrix must be square and match vehicle_ids")

    adjacency = np.asarray(adjacency_matrix, dtype=int)
    if edge_weight_matrix is None:
        weights = adjacency.astype(float)
    else:
        if len(edge_weight_matrix) != count or any(
            len(row) != count for row in edge_weight_matrix
        ):
            raise ValueError("edge_weight_matrix must be square and match vehicle_ids")
        weights = np.asarray(edge_weight_matrix, dtype=float)
        if (weights < 0.0).any():
            raise ValueError("edge weights must be non-negative")

    weight_blocks = np.zeros((count, count, 2, 2))
    for i in range(count):
        for j in range(count):
            weight_blocks[i, j] = weights[i, j] * np.eye(2)

    b_matrix = np.zeros((count, count, 2))
    for i, source_id in enumerate(ids):
        for j, target_id in enumerate(ids):
            ps = formation[source_id]
            pt = formation[target_id]
            b_matrix[i, j] = (pt[0] - ps[0], pt[1] - ps[1])

    Z = build_task_modes(count, task_modes)
    return FormationSpec(
        vehicle_ids=ids,
        adjacency=adjacency,
        weight_blocks=weight_blocks,
        b_matrix=b_matrix,
        Z=Z,
    )


def trivial_spec(vehicle_ids: Sequence[str]) -> FormationSpec:
    """No-edge spec for planners without formation content (calibration)."""

    count = len(vehicle_ids)
    return FormationSpec(
        vehicle_ids=list(vehicle_ids),
        adjacency=np.zeros((count, count), dtype=int),
        weight_blocks=np.zeros((count, count, 2, 2)),
        b_matrix=np.zeros((count, count, 2)),
        Z=np.zeros((2 * count, 0)),
    )


class TaskDrivenFormationController:
    """Matrix-weighted Laplacian constraint law + task-driven term."""

    def __init__(
        self,
        spec: FormationSpec,
        control: TaskDrivenConfig,
        task_velocity: Vector3 = (0.0, 0.0, 0.0),
        task_velocity_schedule: Sequence[Tuple[float, Sequence[float]]] = (),
        command_speed_limit_mps: float = 0.25,
    ) -> None:
        if command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        if control.k <= 0.0:
            raise ValueError("formation gain k must be positive")
        if control.deadband_mps < 0.0:
            raise ValueError("deadband_mps must be non-negative")
        q = spec.Z.shape[1]
        for name, velocity in [("task_velocity", task_velocity)] + [
            ("schedule entry", scheduled) for _, scheduled in task_velocity_schedule
        ]:
            if len(velocity) != q:
                raise ValueError(
                    "%s velocity dimension %d must match the task-mode count %d"
                    % (name, len(velocity), q)
                )
            if sqrt(sum(component * component for component in velocity)) > command_speed_limit_mps:
                raise ValueError(
                    "%s speed must not exceed command_speed_limit_mps" % name
                )
        previous_t = 0.0
        for t, _ in task_velocity_schedule:
            if t < previous_t:
                raise ValueError("task_velocity_schedule times must be non-decreasing")
            previous_t = t
        self.spec = spec
        self.control = control
        self.task_velocity = tuple(float(v) for v in task_velocity)
        self.task_velocity_schedule = tuple(task_velocity_schedule)
        self.command_speed_limit_mps = command_speed_limit_mps
        self._elapsed = 0.0
        # Two-phase state machine (converge -> maneuver); see config docs.
        self._phase = "converge"
        self._below_eps_since: Optional[float] = None
        self._maneuver_started_at: Optional[float] = None

    @property
    def topology(self) -> FormationSpec:
        """Bootstrap runs the standard preflight against the spec."""

        return self.spec

    def _task_velocity_now(self) -> np.ndarray:
        """Piecewise-constant task velocity on the *maneuver-phase* clock.

        With ``converge_first`` the schedule clock starts when the maneuver
        phase begins (the standstill window is automatic before that);
        otherwise the legacy since-construction clock applies.
        """

        w: Sequence[float] = self.task_velocity
        clock = self._elapsed
        if self.control.converge_first:
            if self._phase == "converge":
                return np.zeros(len(self.task_velocity))
            clock = self._elapsed - (self._maneuver_started_at or self._elapsed)
        for t, scheduled in self.task_velocity_schedule:
            if t <= clock:
                w = scheduled
            else:
                break
        return np.asarray(w, dtype=float)

    def _update_phase(self, mean_error: float, ready: bool) -> None:
        """Advance the converge -> maneuver state machine (see config docs)."""

        if not self.control.converge_first or self._phase == "maneuver":
            return
        if ready and mean_error < self.control.phase_converge_eps_m:
            if self._below_eps_since is None:
                self._below_eps_since = self._elapsed
            elif self._elapsed - self._below_eps_since >= self.control.phase_converge_hold_s:
                self._phase = "maneuver"
                self._maneuver_started_at = self._elapsed
                print("task_driven | converged (edge error < %.3f m for %.1f s), maneuver phase"
                      % (self.control.phase_converge_eps_m, self.control.phase_converge_hold_s))
        else:
            self._below_eps_since = None
        if self._phase == "converge" and self._elapsed >= self.control.phase_converge_timeout_s:
            # Never hold the fleet forever on a bad day: proceed anyway.
            self._phase = "maneuver"
            self._maneuver_started_at = self._elapsed
            print("task_driven | converge timeout %.1f s, maneuver phase anyway"
                  % self.control.phase_converge_timeout_s)

    def _shape(self, ux: float, uy: float) -> Tuple[float, float]:
        """Smooth tanh saturation followed by the deadband (v2-style)."""

        magnitude = sqrt(ux * ux + uy * uy)
        if magnitude < 1e-12:
            return 0.0, 0.0
        v_max = self.command_speed_limit_mps
        speed = v_max * tanh(magnitude / v_max)
        if speed < self.control.deadband_mps:
            return 0.0, 0.0
        scale = speed / magnitude
        return ux * scale, uy * scale

    def _limit(self, vx: float, vy: float) -> Tuple[float, float]:
        speed = sqrt(vx * vx + vy * vy)
        if speed <= self.command_speed_limit_mps or speed < 1e-9:
            return vx, vy
        scale = self.command_speed_limit_mps / speed
        return vx * scale, vy * scale

    def step(self, snapshot: MocapSnapshot, dt: float) -> PlannerResult:
        self._elapsed += dt
        spec = self.spec
        result = PlannerResult(timestamp=snapshot.timestamp)

        # pass 1: constraint totals and edge errors
        totals: List[np.ndarray] = []
        valids: List[bool] = []
        errors: List[float] = []
        valid_count = 0
        for index, vehicle_id in enumerate(spec.vehicle_ids):
            state = snapshot.states.get(vehicle_id)
            valid = state is not None and state.valid
            valids.append(valid)
            total = np.zeros(2)
            if valid:
                valid_count += 1
                for target_index, target_id in enumerate(spec.vehicle_ids):
                    if not spec.adjacency[index, target_index]:
                        continue
                    target = snapshot.states.get(target_id)
                    if target is None or not target.valid:
                        continue
                    error = (
                        np.array([state.x - target.x, state.y - target.y])
                        + spec.b_matrix[index, target_index]
                    )
                    result.edge_samples.append(
                        EdgeSample(
                            source_id=vehicle_id,
                            target_id=target_id,
                            desired=(spec.b_matrix[index, target_index, 0],
                                     spec.b_matrix[index, target_index, 1], 0.0),
                            actual=(target.x - state.x, target.y - state.y, 0.0),
                        )
                    )
                    errors.append(float(np.linalg.norm(error)))
                    total += spec.weight_blocks[index, target_index] @ error
            totals.append(total)

        result.mean_error = sum(errors) / len(errors) if errors else 0.0
        result.max_error = max(errors) if errors else 0.0
        result.ready = valid_count == len(spec.vehicle_ids)

        # pass 2: phase transition, then compose commands
        self._update_phase(result.mean_error, result.ready)
        w = self._task_velocity_now()
        for index, vehicle_id in enumerate(spec.vehicle_ids):
            total = totals[index]
            valid = valids[index]
            cx, cy = self._shape(-self.control.k * total[0], -self.control.k * total[1])
            task = spec.z_block(index) @ w  # (2,)
            if valid:
                vx, vy = self._limit(cx + float(task[0]), cy + float(task[1]))
            else:
                # Never drive blind: an invalid vehicle gets a true zero.
                vx, vy = 0.0, 0.0
            result.commands[vehicle_id] = VelocityCommand(
                vehicle_id=vehicle_id,
                vx=vx,
                vy=vy,
                vz=0.0,
                error_vector=(float(total[0]), float(total[1]), 0.0),
                valid=valid,
            )
        return result

    def reset(self) -> None:
        """Reset the schedule clock and the phase state machine."""

        self._elapsed = 0.0
        self._phase = "converge"
        self._below_eps_since = None
        self._maneuver_started_at = None
