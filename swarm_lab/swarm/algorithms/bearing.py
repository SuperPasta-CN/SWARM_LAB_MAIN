"""Topology-driven bearing-only formation algorithm with projection matrix.

For agent ``i`` the bearing control term is::

    raw_i = sum_{j in N_i} P_ij @ (g_ij - g*_ij)
    u_i   = shape(kp * raw_i + ki * xi_i + kd * draw_i)

where ``g_ij = normalize(p_j - p_i)`` is the measured unit bearing,
``g*_ij`` the desired one, and ``P_ij = I - g_ij g_ij^T`` the orthogonal
projector onto the line-of-sight complement.  The projection removes the
radial error component so the control only rotates bearings (and does not
fight scale drift); the per-edge contributions are *summed*, not averaged.

``xi_i`` is an optional per-vehicle integral state (``xi += raw * dt``,
norm-clamped to ``integral_limit``) and ``draw_i = (raw - prev_raw) / dt``
the optional derivative term.  A pure P law tracks a moving leader only
with a constant steady-state error; ``ki > 0`` removes it.  Both states
are cleared whenever the vehicle state is invalid.

What each vehicle does with the law is decided by its role (the single
source of truth is ``VehicleConfig.role``):

- ``captain`` (exactly one): ignores the law and outputs its own fixed
  world-frame velocity ``v_c`` (possibly zero; a piecewise-constant
  schedule is supported).  Its bearing error is still computed for
  telemetry.
- ``first_mate`` (zero or more): blends a fixed velocity ``v_fm`` with the
  bearing term ``u`` — ``"weighted"`` (default): ``v = a*v_fm + (1-a)*u``,
  or ``"additive"``: ``v = v_fm + u`` — then speed-limits the blend.  With
  ``v_fm = v_c`` the formation ideally translates rigidly behind the
  captain (Zhao & Zelazo, bearing-based formation maneuvering, Prop. 1).
- ``crew`` (zero or more): runs the pure bearing term ``v = u``.

Bearings are planar (2D) by default to ignore mocap height noise; set
``BearingControlConfig.bearing_3d`` to use full 3D vectors.

Self-check (sign convention): with ``g = (1, 0)`` and ``g* = (0, 1)`` the
control on ``i`` points along ``-y``, i.e. it rotates ``g`` towards ``g*``::

    e = g - g* = (1, -1);  P = I - g g^T = diag(0, 1);  P @ e = (0, -1)
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, sqrt, tanh
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from swarm.domain.config import VEHICLE_ROLES, BearingControlConfig, FirstMateParams
from swarm.domain.models import (
    BearingSample,
    MocapSnapshot,
    PlannerResult,
    Vector3,
    VehicleState,
    VelocityCommand,
)


@dataclass(frozen=True)
class FormationTopology:
    """Directed matrix topology, roles, and desired world-frame bearings."""

    vehicle_ids: List[str]
    adjacency_matrix: List[List[int]]
    bearing_matrix: List[List[Vector3]]
    roles: List[str]

    def __post_init__(self) -> None:
        count = len(self.vehicle_ids)
        if count == 0:
            raise ValueError("vehicle_ids must not be empty")
        if len(self.adjacency_matrix) != count or len(self.bearing_matrix) != count:
            raise ValueError("topology matrices must match vehicle_ids length")
        if len(self.roles) != count:
            raise ValueError("roles must match vehicle_ids length")
        if any(role not in VEHICLE_ROLES for role in self.roles):
            raise ValueError("roles entries must be one of %s" % (VEHICLE_ROLES,))
        if any(len(row) != count for row in self.adjacency_matrix):
            raise ValueError("adjacency_matrix must be square")
        if any(len(row) != count for row in self.bearing_matrix):
            raise ValueError("bearing_matrix must be square")


_ZERO: Vector3 = (0.0, 0.0, 0.0)


class BearingOnlyFormationController:
    """Projected bearing-only law with roles, optional PID, and blending."""

    def __init__(
        self,
        topology: FormationTopology,
        captain_velocity: Vector3 = _ZERO,
        captain_velocity_schedule: Sequence[Tuple[float, Vector3]] = (),
        first_mate_params: Optional[Mapping[str, FirstMateParams]] = None,
        control: Optional[BearingControlConfig] = None,
        command_speed_limit_mps: float = 0.25,
    ) -> None:
        if command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        control = control or BearingControlConfig()
        if control.kp <= 0.0:
            raise ValueError("bearing kp must be positive")
        if control.ki < 0.0 or control.kd < 0.0:
            raise ValueError("bearing ki/kd must be non-negative")
        if control.integral_limit <= 0.0:
            raise ValueError("bearing integral_limit must be positive")
        if control.deadband_mps < 0.0:
            raise ValueError("deadband_mps must be non-negative")
        first_mates = {
            vehicle_id
            for vehicle_id, role in zip(topology.vehicle_ids, topology.roles)
            if role == "first_mate"
        }
        params = dict(first_mate_params or {})
        for key in params:
            if key not in first_mates:
                raise ValueError("first_mate_params key %r is not a first_mate" % key)
        for key, value in params.items():
            if value.velocity is None:
                raise ValueError(
                    "first_mate_params[%s].velocity must be resolved before "
                    "the controller is built" % key
                )
        self.topology = topology
        self.captain_velocity = captain_velocity
        self.captain_velocity_schedule = tuple(captain_velocity_schedule)
        self.first_mate_params = params
        self.control = control
        self.command_speed_limit_mps = command_speed_limit_mps
        self._elapsed = 0.0
        self._integrals: Dict[str, Vector3] = {}
        self._prev_raw: Dict[str, Vector3] = {}

    @property
    def _dim(self) -> int:
        return 3 if self.control.bearing_3d else 2

    @staticmethod
    def _normalize(vector: Vector3) -> Tuple[Vector3, float]:
        norm = sqrt(sum(component * component for component in vector))
        if norm < 1e-9:
            return (0.0, 0.0, 0.0), 0.0
        return (
            vector[0] / norm,
            vector[1] / norm,
            vector[2] / norm,
        ), norm

    def _planar(self, vector: Vector3) -> Vector3:
        if self.control.bearing_3d:
            return vector
        return vector[0], vector[1], 0.0

    def _measured_bearing(
        self,
        source: VehicleState,
        target: VehicleState,
    ) -> Tuple[Vector3, float]:
        """Unit bearing from source to target and the norm before scaling."""

        relative = self._planar(
            (target.x - source.x, target.y - source.y, target.z - source.z)
        )
        return self._normalize(relative)

    def _vehicle_control(
        self,
        snapshot: MocapSnapshot,
        index: int,
        bearing_samples: List[BearingSample],
    ) -> Vector3:
        """Summed projected bearing error for one vehicle (zero if invalid)."""

        source_id = self.topology.vehicle_ids[index]
        source = snapshot.states.get(source_id)
        if source is None or not source.valid:
            return 0.0, 0.0, 0.0

        total = [0.0, 0.0, 0.0]
        for target_index, target_id in enumerate(self.topology.vehicle_ids):
            if not self.topology.adjacency_matrix[index][target_index]:
                continue
            target = snapshot.states.get(target_id)
            if target is None or not target.valid:
                continue
            actual, distance = self._measured_bearing(source, target)
            desired, _ = self._normalize(
                self._planar(self.topology.bearing_matrix[index][target_index])
            )
            if distance < 1e-9:
                # Vehicles overlap: the bearing is undefined, skip the edge.
                bearing_samples.append(
                    BearingSample(source_id, target_id, desired, actual, valid=False)
                )
                continue
            bearing_samples.append(BearingSample(source_id, target_id, desired, actual))
            error = (
                actual[0] - desired[0],
                actual[1] - desired[1],
                actual[2] - desired[2],
            )
            # P = I - g g^T; projected = P @ error = error - g (g . error)
            dot = sum(actual[k] * error[k] for k in range(3))
            for k in range(3):
                total[k] += error[k] - actual[k] * dot
        return total[0], total[1], total[2]

    def _shape_command(self, ux: float, uy: float, uz: float) -> Vector3:
        """Smooth tanh saturation followed by the deadband."""

        magnitude = sqrt(ux * ux + uy * uy + uz * uz)
        if magnitude < 1e-12:
            return 0.0, 0.0, 0.0
        v_max = self.command_speed_limit_mps
        speed = v_max * tanh(magnitude / v_max)
        if speed < self.control.deadband_mps:
            return 0.0, 0.0, 0.0
        scale = speed / magnitude
        return ux * scale, uy * scale, uz * scale

    def _limit_speed_3d(self, vx: float, vy: float, vz: float) -> Vector3:
        speed = sqrt(vx * vx + vy * vy + vz * vz)
        if speed <= self.command_speed_limit_mps or speed < 1e-9:
            return vx, vy, vz
        scale = self.command_speed_limit_mps / speed
        return vx * scale, vy * scale, vz * scale

    @staticmethod
    def _angle_error_rad(sample: BearingSample) -> Optional[float]:
        if not sample.valid:
            return None
        dot = sum(sample.desired[k] * sample.actual[k] for k in range(3))
        return atan2(
            sqrt(max(0.0, 1.0 - min(1.0, max(-1.0, dot)) ** 2)),
            min(1.0, max(-1.0, dot)),
        )

    def _captain_velocity_now(self) -> Vector3:
        """Piecewise-constant captain speed: the last entry with t <= elapsed.

        Before the first scheduled time the constant ``captain_velocity``
        applies; an empty schedule means constant speed for the whole run.
        """

        velocity = self.captain_velocity
        for t, scheduled in self.captain_velocity_schedule:
            if t <= self._elapsed:
                velocity = scheduled
            else:
                break
        return velocity

    def _update_pid_state(self, vehicle_id: str, raw: Vector3, dt: float, valid: bool) -> Vector3:
        """Advance the per-vehicle integral/derivative states, return PID sum."""

        if not valid:
            # Forget the past: a lost vehicle must not accumulate windup.
            self._integrals.pop(vehicle_id, None)
            self._prev_raw.pop(vehicle_id, None)
            return self.control.kp * raw[0], self.control.kp * raw[1], self.control.kp * raw[2]

        total = [self.control.kp * raw[k] for k in range(3)]
        if dt > 0.0:
            if self.control.ki > 0.0:
                xi = self._integrals.get(vehicle_id, _ZERO)
                xi = tuple(xi[k] + raw[k] * dt for k in range(3))
                norm = sqrt(sum(component * component for component in xi))
                if norm > self.control.integral_limit:
                    scale = self.control.integral_limit / norm
                    xi = tuple(component * scale for component in xi)
                self._integrals[vehicle_id] = xi
                for k in range(3):
                    total[k] += self.control.ki * xi[k]
            if self.control.kd > 0.0:
                previous = self._prev_raw.get(vehicle_id)
                if previous is not None:
                    for k in range(3):
                        total[k] += self.control.kd * (raw[k] - previous[k]) / dt
        self._prev_raw[vehicle_id] = raw
        return total[0], total[1], total[2]

    def step(self, snapshot: MocapSnapshot, dt: float) -> PlannerResult:
        self._elapsed += dt
        result = PlannerResult(timestamp=snapshot.timestamp)
        valid_count = 0
        error_sum = 0.0
        max_error = 0.0

        for index, vehicle_id in enumerate(self.topology.vehicle_ids):
            state = snapshot.states.get(vehicle_id)
            valid = state is not None and state.valid
            raw = self._vehicle_control(snapshot, index, result.bearing_samples)
            role = self.topology.roles[index]

            if role == "captain":
                # The captain never runs the law: fixed world-frame velocity
                # (piecewise-constant when a schedule is configured).
                vx, vy, vz = self._limit_speed_3d(*self._captain_velocity_now())
                self._integrals.pop(vehicle_id, None)
                self._prev_raw.pop(vehicle_id, None)
            else:
                ux, uy, uz = self._update_pid_state(vehicle_id, raw, dt, valid)
                ux, uy, uz = self._shape_command(ux, uy, uz)
                if role == "first_mate":
                    params = self.first_mate_params[vehicle_id]
                    v_fm = params.velocity
                    assert v_fm is not None  # resolved before construction
                    if params.blend_mode == "additive":
                        vx, vy, vz = self._limit_speed_3d(
                            v_fm[0] + ux, v_fm[1] + uy, v_fm[2] + uz
                        )
                    else:  # "weighted"
                        alpha = params.blend_weight
                        vx, vy, vz = self._limit_speed_3d(
                            alpha * v_fm[0] + (1.0 - alpha) * ux,
                            alpha * v_fm[1] + (1.0 - alpha) * uy,
                            alpha * v_fm[2] + (1.0 - alpha) * uz,
                        )
                else:  # "crew": the pure bearing term
                    vx, vy, vz = ux, uy, uz

            result.commands[vehicle_id] = VelocityCommand(
                vehicle_id=vehicle_id,
                vx=vx,
                vy=vy,
                vz=vz,
                error_vector=raw,
                valid=valid,
                role=role,
            )
            if valid:
                valid_count += 1

        angle_errors = [
            angle
            for sample in result.bearing_samples
            for angle in (self._angle_error_rad(sample),)
            if angle is not None
        ]
        if angle_errors:
            error_sum = sum(angle_errors)
            max_error = max(angle_errors)
        result.ready = valid_count == len(self.topology.vehicle_ids)
        result.mean_error = error_sum / len(angle_errors) if angle_errors else 0.0
        result.max_error = max_error
        return result

    def reset(self) -> None:
        """Clear the schedule clock and every per-vehicle PID state."""

        self._elapsed = 0.0
        self._integrals.clear()
        self._prev_raw.clear()


def build_topology_from_matrices(
    vehicle_ids: Sequence[str],
    adjacency_matrix: Sequence[Sequence[int]],
    bearing_matrix: Sequence[Sequence[Sequence[float]]],
    roles: Sequence[str],
) -> FormationTopology:
    count = len(vehicle_ids)
    if count == 0:
        raise ValueError("vehicle_ids must not be empty")
    if len(adjacency_matrix) != count or len(bearing_matrix) != count:
        raise ValueError("matrix dimensions must match vehicle_ids")
    if len(roles) != count:
        raise ValueError("roles must match vehicle_ids")
    if any(role not in VEHICLE_ROLES for role in roles):
        raise ValueError("roles entries must be one of %s" % (VEHICLE_ROLES,))

    adjacency = [[int(adjacency_matrix[i][j]) for j in range(count)] for i in range(count)]
    bearings: List[List[Vector3]] = []
    for i in range(count):
        row: List[Vector3] = []
        if len(bearing_matrix[i]) != count:
            raise ValueError("bearing_matrix must be square")
        for j in range(count):
            vector = bearing_matrix[i][j]
            if len(vector) < 2:
                raise ValueError("each desired bearing must contain at least x and y")
            row.append(
                (
                    float(vector[0]),
                    float(vector[1]),
                    float(vector[2]) if len(vector) > 2 else 0.0,
                )
            )
        bearings.append(row)

    return FormationTopology(
        vehicle_ids=list(vehicle_ids),
        adjacency_matrix=adjacency,
        bearing_matrix=bearings,
        roles=[str(role) for role in roles],
    )
