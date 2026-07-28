"""Topology-driven bearing-only formation algorithm with projection matrix.

For agent ``i`` the control law is::

    u_i = kp * sum_{j in N_i} P_ij @ (g_ij - g*_ij)

where ``g_ij = normalize(p_j - p_i)`` is the measured unit bearing,
``g*_ij`` the desired one, and ``P_ij = I - g_ij g_ij^T`` the orthogonal
projector onto the line-of-sight complement.  The projection removes the
radial error component so the control only rotates bearings (and does not
fight scale drift); the per-edge contributions are *summed*, not averaged.

Who runs the law depends on ``BearingControlConfig.agent_mode``:

- ``"all_bearing"`` (default): every vehicle moves under the bearing law.
  Undirected edges then act symmetrically on both endpoints (the net force
  is zero, so the centroid is ideally stationary), and the formation
  converges to the desired shape up to translation/rotation/scale.
- ``"leader_velocity"``: leaders output their configured velocity (pinned
  when zero); only followers run the law.
- ``"leader_bias"``: leaders run the law plus their configured velocity.

Bearings are planar (2D) by default to ignore mocap height noise; set
``BearingControlConfig.bearing_3d`` to use full 3D vectors.

Self-check (sign convention): with ``g = (1, 0)`` and ``g* = (0, 1)`` the
control on ``i`` points along ``-y``, i.e. it rotates ``g`` towards ``g*``::

    e = g - g* = (1, -1);  P = I - g g^T = diag(0, 1);  P @ e = (0, -1)
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, sqrt, tanh
from typing import List, Mapping, Optional, Sequence, Tuple

from swarm.domain.config import BearingControlConfig
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
    """Directed matrix topology and desired world-frame bearings."""

    vehicle_ids: List[str]
    adjacency_matrix: List[List[int]]
    bearing_matrix: List[List[Vector3]]
    leader_mask: List[bool]
    follower_mask: List[bool]

    def __post_init__(self) -> None:
        count = len(self.vehicle_ids)
        if count == 0:
            raise ValueError("vehicle_ids must not be empty")
        if len(self.adjacency_matrix) != count or len(self.bearing_matrix) != count:
            raise ValueError("topology matrices must match vehicle_ids length")
        if len(self.leader_mask) != count or len(self.follower_mask) != count:
            raise ValueError("role masks must match vehicle_ids length")
        if any(len(row) != count for row in self.adjacency_matrix):
            raise ValueError("adjacency_matrix must be square")
        if any(len(row) != count for row in self.bearing_matrix):
            raise ValueError("bearing_matrix must be square")


class BearingOnlyFormationController:
    """Projected bearing-only law with smooth saturation and a deadband."""

    def __init__(
        self,
        topology: FormationTopology,
        leader_target_velocities: Mapping[str, Vector3],
        control: BearingControlConfig,
        command_speed_limit_mps: float,
    ) -> None:
        if command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        if control.kp <= 0.0:
            raise ValueError("bearing kp must be positive")
        if control.deadband_mps < 0.0:
            raise ValueError("deadband_mps must be non-negative")
        self.topology = topology
        self.leader_target_velocities = dict(leader_target_velocities)
        self.control = control
        self.command_speed_limit_mps = command_speed_limit_mps

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
        """Summed projected bearing error for one follower (zero if invalid)."""

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

    def step(self, snapshot: MocapSnapshot, dt: float) -> PlannerResult:
        _ = dt  # the projected P-law is memoryless; dt kept for the protocol
        result = PlannerResult(timestamp=snapshot.timestamp)
        valid_count = 0
        error_sum = 0.0
        max_error = 0.0

        mode = self.control.agent_mode
        for index, vehicle_id in enumerate(self.topology.vehicle_ids):
            state = snapshot.states.get(vehicle_id)
            valid = state is not None and state.valid
            raw = self._vehicle_control(snapshot, index, result.bearing_samples)
            is_leader = self.topology.leader_mask[index]

            if is_leader and mode == "leader_velocity":
                # Legacy behaviour: the leader ignores the bearing law and
                # tracks its configured velocity (pinned when zero).
                vx, vy, vz = self._limit_speed_3d(*self.leader_target_velocities[vehicle_id])
            else:
                vx, vy, vz = self._shape_command(
                    self.control.kp * raw[0],
                    self.control.kp * raw[1],
                    self.control.kp * raw[2],
                )
                if is_leader and mode == "leader_bias":
                    bias = self.leader_target_velocities[vehicle_id]
                    vx, vy, vz = self._limit_speed_3d(
                        bias[0] + vx, bias[1] + vy, bias[2] + vz
                    )

            result.commands[vehicle_id] = VelocityCommand(
                vehicle_id=vehicle_id,
                vx=vx,
                vy=vy,
                vz=vz,
                error_vector=raw,
                valid=valid,
                is_leader=self.topology.leader_mask[index],
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
        """Stateless law: nothing to reset (kept for the planner protocol)."""

        return None


def build_topology_from_matrices(
    vehicle_ids: Sequence[str],
    adjacency_matrix: Sequence[Sequence[int]],
    bearing_matrix: Sequence[Sequence[Sequence[float]]],
    leader_mask: Optional[Sequence[bool]] = None,
) -> FormationTopology:
    count = len(vehicle_ids)
    if count == 0:
        raise ValueError("vehicle_ids must not be empty")
    if len(adjacency_matrix) != count or len(bearing_matrix) != count:
        raise ValueError("matrix dimensions must match vehicle_ids")

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

    if leader_mask is None:
        leaders = [False] * count
    else:
        if len(leader_mask) != count:
            raise ValueError("leader_mask must match vehicle_ids")
        leaders = [bool(value) for value in leader_mask]
    return FormationTopology(
        vehicle_ids=list(vehicle_ids),
        adjacency_matrix=adjacency,
        bearing_matrix=bearings,
        leader_mask=leaders,
        follower_mask=[not value for value in leaders],
    )
