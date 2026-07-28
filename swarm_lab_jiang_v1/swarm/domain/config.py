"""Typed configuration grouped by algorithm, experiment, and runtime concerns."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence, Tuple

from swarm.domain.models import Vector3


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float


@dataclass(frozen=True)
class BearingControlConfig:
    """Parameters of the projected bearing-only formation law.

    ``agent_mode`` selects who runs the bearing law:

    - ``"all_bearing"`` (default): every vehicle runs the bearing law and
      moves; no agent is pinned.  The formation converges to the desired
      shape up to a free translation/rotation/scale, and the anchor
      preflight check is skipped (any consistent initial placement works).
    - ``"leader_velocity"``: leaders output their configured target
      velocity (pinned when it is zero); only followers run the bearing
      law.  This is the legacy behaviour that requires the leader baseline
      to be consistent with the desired bearings.
    - ``"leader_bias"``: leaders run the bearing law PLUS their configured
      target velocity, dragging the whole formation along (maneuvering).
    """

    kp: float = 0.6
    deadband_mps: float = 0.015
    bearing_3d: bool = False
    agent_mode: str = "all_bearing"


@dataclass(frozen=True)
class StaticObstacleConfig:
    """Circular static obstacle expressed in the world frame."""

    obstacle_id: str
    x: float
    y: float
    radius_m: float


@dataclass(frozen=True)
class ObstacleAvoidanceConfig:
    """Parameters for the planar artificial-potential-field filter."""

    enabled: bool = False
    vehicle_radius_m: float = 0.03
    influence_distance_m: float = 0.1
    repulsive_gain: float = 0.0005
    min_distance_epsilon_m: float = 0.01
    avoid_other_vehicles: bool = True


@dataclass(frozen=True)
class ExecutionConfig:
    """Velocity-to-wheel mapping shared by omni (mecanum) and diff modes."""

    mode: str = "omni"  # "omni" (mecanum holonomic) or "diff" (differential)
    heading_hold: bool = True
    k_omega: float = 2.0
    omega_max: float = 1.5
    mecanum_l_m: float = 0.10  # lx + ly of the X-pattern mecanum chassis
    # Wheel-speed <-> command calibration: command 100 corresponds to this
    # wheel speed (m/s).  Measure it (full command for 2 s, distance / time);
    # 0.3 matches the lab fleet, 0.5 systematically under-drives the motors.
    max_wheel_speed_mps: float = 0.3
    wheel_command_min: float = -100.0
    wheel_command_max: float = 100.0
    # Motor dead-zone compensation: nonzero wheel commands smaller than this
    # magnitude are lifted to it (exact zeros stay zero).  Measure per fleet
    # with tools/check_wheels.py --ramp; 0 disables the compensation.
    wheel_command_min_effective: float = 0.0
    wheel_flip: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    # diff mode only (ported from the legacy GroundVehicleControlConfig)
    track_width_m: float = 0.053
    speed_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(1.2, 0.0, 0.0, 1.0, 0.3)
    )
    heading_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(3.2, 0.0, 0.0, 1.0, 11.32)
    )


@dataclass(frozen=True)
class VehicleConfig:
    vehicle_id: str
    role: str
    platform: str
    address: str


@dataclass(frozen=True)
class TopologyConfig:
    adjacency_matrix: Sequence[Sequence[int]]
    bearing_matrix: Sequence[Sequence[Sequence[float]]]
    leader_mask: Sequence[bool]


@dataclass(frozen=True)
class TelemetryConfig:
    console_enabled: bool = True
    console_hz: float = 10.0
    live_plot_enabled: bool = True
    live_plot_hz: float = 10.0
    live_plot_queue_size: int = 64
    live_plot_max_points: int = 2000
    record_enabled: bool = True
    record_hz: float = 5.0
    record_root: str = "runs"
    record_flush_interval_s: float = 1.0


@dataclass(frozen=True)
class PreflightConfig:
    """Hard checks that must pass before wheels are allowed to spin."""

    anchor_tolerance_deg: float = 20.0
    min_separation_m: float = 0.20
    mocap_timeout_s: float = 10.0
    require_confirmation: bool = True


@dataclass(frozen=True)
class RuntimeConfig:
    control_hz: float = 50.0
    # ROS2 vrpn_client_ros publishes directly to /{name}/pose, no prefix needed
    mocap_prefix: str = ""
    require_twist: bool = False
    stop_on_loss_of_mocap: bool = True
    udp_port: int = 12345
    udp_timeout_s: float = 0.2
    command_speed_limit_mps: float = 0.25
    stop_on_converge: bool = False
    converge_eps_rad: float = 0.05
    converge_hold_s: float = 3.0
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    algorithm: str
    vehicles: Tuple[VehicleConfig, ...]
    topology: TopologyConfig
    leader_target_velocities: Mapping[str, Vector3]
    bearing_control: BearingControlConfig = field(default_factory=BearingControlConfig)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    preflight: PreflightConfig = field(default_factory=PreflightConfig)
    obstacle_avoidance: ObstacleAvoidanceConfig = field(default_factory=ObstacleAvoidanceConfig)
    static_obstacles: Tuple[StaticObstacleConfig, ...] = ()

    @property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    def validate(self) -> None:
        """Reject inconsistent configuration before external resources start."""

        if not isinstance(self.algorithm, str) or not self.algorithm.strip():
            raise ValueError("algorithm must be a non-empty string")
        vehicle_ids = self.vehicle_ids
        if not vehicle_ids:
            raise ValueError("vehicles must not be empty")
        if len(set(vehicle_ids)) != len(vehicle_ids):
            raise ValueError("vehicle IDs must be unique")
        n = len(vehicle_ids)
        if len(self.topology.adjacency_matrix) != n or len(self.topology.bearing_matrix) != n:
            raise ValueError("topology dimensions must match vehicle count")
        if len(self.topology.leader_mask) != n:
            raise ValueError("leader mask must match vehicle count")
        agent_mode = self.bearing_control.agent_mode
        if agent_mode not in ("all_bearing", "leader_velocity", "leader_bias"):
            raise ValueError("bearing_control.agent_mode must be 'all_bearing', 'leader_velocity' or 'leader_bias'")
        if agent_mode != "all_bearing":
            for index, vehicle in enumerate(self.vehicles):
                is_leader = bool(self.topology.leader_mask[index])
                if is_leader and vehicle.vehicle_id not in self.leader_target_velocities:
                    raise ValueError("missing target velocity for leader: %s" % vehicle.vehicle_id)
        if self.runtime.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if self.runtime.command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        if self.bearing_control.kp <= 0.0:
            raise ValueError("bearing kp must be positive")
        if self.bearing_control.deadband_mps < 0.0:
            raise ValueError("bearing deadband_mps must be non-negative")
        execution = self.execution
        if execution.mode not in ("omni", "diff"):
            raise ValueError("execution.mode must be 'omni' or 'diff'")
        if execution.mecanum_l_m <= 0.0:
            raise ValueError("mecanum_l_m must be positive")
        if execution.max_wheel_speed_mps <= 0.0:
            raise ValueError("max_wheel_speed_mps must be positive")
        if not 0.0 <= execution.wheel_command_min_effective < execution.wheel_command_max:
            raise ValueError("wheel_command_min_effective must be in [0, wheel_command_max)")
        if execution.omega_max <= 0.0:
            raise ValueError("omega_max must be positive")
        if execution.k_omega < 0.0:
            raise ValueError("k_omega must be non-negative")
        if execution.wheel_command_min >= execution.wheel_command_max:
            raise ValueError("wheel_command_min must be less than wheel_command_max")
        if len(execution.wheel_flip) != 4:
            raise ValueError("wheel_flip must contain four signs")
        if any(sign not in (-1.0, 1.0, -1, 1) for sign in execution.wheel_flip):
            raise ValueError("wheel_flip entries must be +1 or -1")
        if execution.track_width_m <= 0.0:
            raise ValueError("track_width_m must be positive")
        if self.runtime.converge_eps_rad <= 0.0:
            raise ValueError("converge_eps_rad must be positive")
        if self.runtime.converge_hold_s <= 0.0:
            raise ValueError("converge_hold_s must be positive")
        preflight = self.preflight
        if preflight.anchor_tolerance_deg <= 0.0:
            raise ValueError("anchor_tolerance_deg must be positive")
        if preflight.min_separation_m < 0.0:
            raise ValueError("min_separation_m must be non-negative")
        if preflight.mocap_timeout_s <= 0.0:
            raise ValueError("mocap_timeout_s must be positive")
        avoidance = self.obstacle_avoidance
        if avoidance.vehicle_radius_m < 0.0:
            raise ValueError("vehicle_radius_m must be non-negative")
        if avoidance.influence_distance_m <= 0.0:
            raise ValueError("influence_distance_m must be positive")
        if avoidance.repulsive_gain < 0.0:
            raise ValueError("repulsive_gain must be non-negative")
        if avoidance.min_distance_epsilon_m <= 0.0:
            raise ValueError("min_distance_epsilon_m must be positive")
        if avoidance.min_distance_epsilon_m >= avoidance.influence_distance_m:
            raise ValueError("min_distance_epsilon_m must be less than influence_distance_m")
        obstacle_ids = tuple(obstacle.obstacle_id for obstacle in self.static_obstacles)
        if len(set(obstacle_ids)) != len(obstacle_ids):
            raise ValueError("static obstacle IDs must be unique")
        for obstacle in self.static_obstacles:
            if not obstacle.obstacle_id:
                raise ValueError("static obstacle ID must not be empty")
            if obstacle.radius_m < 0.0:
                raise ValueError("static obstacle radius_m must be non-negative")
