"""Shared data structures passed between architecture layers (v3)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Tuple

Vector2 = Tuple[float, float]
Vector3 = Tuple[float, float, float]


@dataclass
class VehicleState:
    """Latest pose and twist for one vehicle in the world frame."""

    vehicle_id: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    yaw: float = 0.0
    timestamp: float = 0.0
    frame_id: str = "world"
    valid: bool = False
    pose_ready: bool = False
    twist_ready: bool = False

    def position(self) -> Vector3:
        return self.x, self.y, self.z

    def velocity(self) -> Vector3:
        return self.vx, self.vy, self.vz


@dataclass(frozen=True)
class MocapSnapshot:
    """Stable control-cycle snapshot containing all configured vehicles."""

    timestamp: float
    states: Mapping[str, VehicleState] = field(default_factory=dict)


@dataclass(frozen=True)
class EdgeSample:
    """Desired and measured relative position of one directed edge (meters).

    ``desired`` is ``b_ij = p_j* - p_i*`` and ``actual`` is ``p_j - p_i``
    (both "target minus source"); the edge error is ``actual - desired``.
    Unlike bearings, relative positions stay well-defined when vehicles
    overlap, so ``valid`` is True whenever both endpoint states are valid.
    """

    source_id: str
    target_id: str
    desired: Vector3
    actual: Vector3
    valid: bool = True


@dataclass
class VelocityCommand:
    """Translational command produced by a swarm planner (v3: no roles)."""

    vehicle_id: str
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    error_vector: Vector3 = (0.0, 0.0, 0.0)
    valid: bool = False


@dataclass
class PlannerResult:
    """Planner output consumed by platform-specific vehicle actuators."""

    timestamp: float
    commands: Dict[str, VelocityCommand] = field(default_factory=dict)
    mean_error: float = 0.0  # meters: mean ||edge error||
    max_error: float = 0.0
    ready: bool = False
    edge_samples: List[EdgeSample] = field(default_factory=list)


@dataclass(frozen=True)
class ActuationResult:
    """Platform command and delivery status for one vehicle."""

    vehicle_id: str
    platform: str
    values: Mapping[str, float]
    sent: bool


@dataclass(frozen=True)
class ControlFrame:
    """Complete result of one valid control iteration."""

    monotonic_time: float
    dt: float
    snapshot: MocapSnapshot
    planner: PlannerResult
    actuations: Mapping[str, ActuationResult]
    nominal_planner: Optional[PlannerResult] = None
