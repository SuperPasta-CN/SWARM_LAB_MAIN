"""Typed configuration grouped by algorithm, experiment, and runtime concerns."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import sqrt
from typing import Mapping, Optional, Sequence, Tuple

from swarm.domain.models import Vector3

VEHICLE_ROLES = ("captain", "first_mate", "crew")


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

    The pure proportional law (``ki = kd = 0``) tracks a moving leader only
    with a constant steady-state error; the optional integral term removes
    it (``ki > 0``) and the derivative term adds damping (``kd > 0``).  The
    integral state is one vector per vehicle, ``xi += raw * dt`` with
    ``raw = sum P(g - g*)``, clamped to ``integral_limit`` in norm.
    """

    kp: float = 0.6
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 0.5
    deadband_mps: float = 0.015
    bearing_3d: bool = False


@dataclass(frozen=True)
class FirstMateParams:
    """Blending parameters for one first-mate vehicle.

    The first mate combines its fixed velocity ``v_fm`` with the bearing
    control term ``u``; the blended result is speed-limited afterwards:

    - ``"weighted"`` (default): ``v = alpha * v_fm + (1 - alpha) * u``
    - ``"additive"``:           ``v = v_fm + u``

    ``velocity=None`` resolves to the captain velocity (Zhao & Zelazo,
    bearing-based formation maneuvering, Proposition 1: with all leaders
    sharing v_c the formation translates without changing scale).
    """

    velocity: Optional[Vector3] = None
    blend_weight: float = 0.5
    blend_mode: str = "weighted"  # "weighted" | "additive"


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
class ConstantVelocityConfig:
    """Fixed world-frame velocity step for chassis tracking tests.

    Used by ``algorithm="constant_velocity"``: every vehicle is commanded
    (vx_mps, vy_mps) for ``duration_s`` seconds.  Kept as the controlled
    step input for chassis calibration; there is deliberately no catalog
    config for it in v2.
    """

    vx_mps: float = 0.0
    vy_mps: float = 0.0
    duration_s: float = 5.0


@dataclass(frozen=True)
class ExecutionConfig:
    """Velocity-to-wheel mapping shared by omni / omni_pid / diff modes."""

    mode: str = "omni"  # "omni" (open-loop mecanum), "omni_pid" (mecanum + velocity PI), "diff" (differential)
    heading_hold: bool = True
    k_omega: float = 2.0
    omega_max: float = 1.5
    mecanum_l_m: float = 0.10  # lx + ly of the X-pattern mecanum chassis
    # Wheel-speed <-> command calibration: command 100 corresponds to this
    # wheel speed (m/s).  Measure it (full command for 2 s, distance / time).
    # 0.5 is an estimate from the 2026-07-31 open-loop chassis_step run
    # (target 0.15 m/s, actual ~0.257 m/s with the old 0.3 value); refine
    # with a dedicated calibration run when possible.
    max_wheel_speed_mps: float = 0.5
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
    # omni_pid mode only: feedforward + PI correction on the mocap-estimated
    # body velocity (position difference + EMA, so keep kd = 0).  Error and
    # correction are both in m/s.  Conservative on-site starting points;
    # tune with a constant-velocity step experiment before any formation run.
    omni_velocity_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(0.6, 1.2, 0.0, 0.10, 0.15)
    )


@dataclass(frozen=True)
class VehicleExecutionOverride:
    """Per-vehicle execution-side calibration (pays off the v1 debt of
    treating every chassis alike although e.g. car1/car2 ran weaker).

    Any field left ``None`` inherits the experiment-wide ExecutionConfig.
    """

    max_wheel_speed_mps: Optional[float] = None
    wheel_command_min_effective: Optional[float] = None
    wheel_flip: Optional[Tuple[float, float, float, float]] = None


@dataclass(frozen=True)
class VehicleConfig:
    vehicle_id: str
    role: str  # "captain" | "first_mate" | "crew" — the only role source of truth
    platform: str
    address: str
    execution_override: Optional[VehicleExecutionOverride] = None


@dataclass(frozen=True)
class TopologyConfig:
    adjacency_matrix: Sequence[Sequence[int]]
    bearing_matrix: Sequence[Sequence[Sequence[float]]]


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
    """Hard checks that must pass before wheels are allowed to spin.

    ``captain_edge_warn_deg`` is a *soft* warning threshold: edges incident
    to the captain whose initial bearing error exceeds it are printed as
    warnings but never block takeoff (v2 has no pinned leader baseline, so
    the v1 anchor hard check no longer applies).
    """

    captain_edge_warn_deg: float = 45.0
    min_separation_m: float = 0.20
    mocap_timeout_s: float = 10.0
    require_confirmation: bool = True


@dataclass(frozen=True)
class RuntimeConfig:
    control_hz: float = 50.0
    # ROS2 vrpn_client_ros publishes directly to /{name}/pose, no prefix needed
    mocap_prefix: str = ""
    require_twist: bool = False
    # Velocity estimates above this speed are rejected as mocap glitches
    # (pose-only finite-difference mode; the fleet tops out near 0.3 m/s).
    max_plausible_speed_mps: float = 1.0
    # Baseline window for the pose finite-difference velocity estimate: the
    # raw difference is taken against the oldest sample inside this trailing
    # window, so position noise / dt stays below the glitch threshold even
    # at high mocap rates.  Larger is smoother but adds feedback lag.
    velocity_diff_baseline_s: float = 0.2
    stop_on_loss_of_mocap: bool = True
    udp_port: int = 12345
    udp_timeout_s: float = 0.2
    command_speed_limit_mps: float = 0.25
    stop_on_converge: bool = False
    converge_eps_rad: float = 0.05
    converge_hold_s: float = 3.0
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)


def _speed(vector: Vector3) -> float:
    return sqrt(sum(component * component for component in vector))


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    algorithm: str
    vehicles: Tuple[VehicleConfig, ...]
    topology: TopologyConfig
    captain_velocity: Vector3 = (0.0, 0.0, 0.0)
    # Piecewise-constant captain speed schedule [(t_seconds, velocity), ...],
    # empty = constant captain_velocity for the whole run.
    captain_velocity_schedule: Sequence[Tuple[float, Vector3]] = ()
    # Keys must be vehicles with role="first_mate"; missing entries get the
    # default FirstMateParams (v_fm = captain velocity, alpha = 0.5, weighted).
    first_mate_params: Mapping[str, FirstMateParams] = field(default_factory=dict)
    bearing_control: BearingControlConfig = field(default_factory=BearingControlConfig)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    preflight: PreflightConfig = field(default_factory=PreflightConfig)
    obstacle_avoidance: ObstacleAvoidanceConfig = field(default_factory=ObstacleAvoidanceConfig)
    static_obstacles: Tuple[StaticObstacleConfig, ...] = ()
    constant_velocity: ConstantVelocityConfig = field(default_factory=ConstantVelocityConfig)

    @property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    def _validate_roles(self) -> None:
        roles = [vehicle.role for vehicle in self.vehicles]
        for vehicle in self.vehicles:
            if vehicle.role not in VEHICLE_ROLES:
                raise ValueError(
                    "vehicle %s role must be one of %s"
                    % (vehicle.vehicle_id, VEHICLE_ROLES)
                )
        if roles.count("captain") != 1:
            raise ValueError("exactly one vehicle must have role 'captain'")
        first_mates = {
            vehicle.vehicle_id for vehicle in self.vehicles if vehicle.role == "first_mate"
        }
        for key in self.first_mate_params:
            if key not in first_mates:
                raise ValueError(
                    "first_mate_params key %r is not a first_mate vehicle" % key
                )
        limit = self.runtime.command_speed_limit_mps
        for key, params in self.first_mate_params.items():
            if not 0.0 <= params.blend_weight <= 1.0:
                raise ValueError(
                    "first_mate_params[%s].blend_weight must be in [0, 1]" % key
                )
            if params.blend_mode not in ("weighted", "additive"):
                raise ValueError(
                    "first_mate_params[%s].blend_mode must be 'weighted' or 'additive'"
                    % key
                )
            if params.velocity is not None and _speed(params.velocity) > limit:
                raise ValueError(
                    "first_mate_params[%s].velocity must not exceed "
                    "command_speed_limit_mps" % key
                )

    def _validate_captain_velocity(self) -> None:
        limit = self.runtime.command_speed_limit_mps
        if _speed(self.captain_velocity) > limit:
            raise ValueError(
                "captain_velocity must not exceed command_speed_limit_mps"
            )
        previous_t = 0.0
        for t, velocity in self.captain_velocity_schedule:
            if t < 0.0:
                raise ValueError("captain_velocity_schedule times must be >= 0")
            if t < previous_t:
                raise ValueError(
                    "captain_velocity_schedule times must be non-decreasing"
                )
            previous_t = t
            if _speed(velocity) > limit:
                raise ValueError(
                    "captain_velocity_schedule speeds must not exceed "
                    "command_speed_limit_mps"
                )

    def _validate_bearing_matrix_consistency(self) -> None:
        """Opposite edges of the topology must carry opposite bearings."""

        adjacency = self.topology.adjacency_matrix
        bearings = self.topology.bearing_matrix
        cos_tolerance = 0.9998476952  # cos(1 deg): normalized directions only
        n = len(self.vehicle_ids)
        for i in range(n):
            for j in range(i + 1, n):
                if not adjacency[i][j] and not adjacency[j][i]:
                    continue
                forward = tuple(float(c) for c in bearings[i][j])
                backward = tuple(float(c) for c in bearings[j][i])
                forward_norm = _speed(forward)
                backward_norm = _speed(backward)
                if forward_norm < 1e-9 or backward_norm < 1e-9:
                    continue
                # bearing[j][i] should be approximately -bearing[i][j]
                dot = sum(
                    forward[k] * backward[k] for k in range(3)
                ) / (forward_norm * backward_norm)
                if dot > -cos_tolerance:
                    raise ValueError(
                        "bearing_matrix edges (%d,%d) and (%d,%d) must be "
                        "approximately opposite" % (i, j, j, i)
                    )

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
        if self.runtime.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if self.runtime.max_plausible_speed_mps <= 0.0:
            raise ValueError("max_plausible_speed_mps must be positive")
        if self.runtime.velocity_diff_baseline_s <= 0.0:
            raise ValueError("velocity_diff_baseline_s must be positive")
        if self.runtime.command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        control = self.bearing_control
        if control.kp <= 0.0:
            raise ValueError("bearing kp must be positive")
        if control.ki < 0.0 or control.kd < 0.0:
            raise ValueError("bearing ki/kd must be non-negative")
        if control.integral_limit <= 0.0:
            raise ValueError("bearing integral_limit must be positive")
        if control.deadband_mps < 0.0:
            raise ValueError("bearing deadband_mps must be non-negative")
        self._validate_roles()
        self._validate_captain_velocity()
        self._validate_bearing_matrix_consistency()
        execution = self.execution
        if execution.mode not in ("omni", "omni_pid", "diff"):
            raise ValueError("execution.mode must be 'omni', 'omni_pid' or 'diff'")
        velocity_pid = execution.omni_velocity_pid
        if velocity_pid.kp < 0.0 or velocity_pid.ki < 0.0 or velocity_pid.kd < 0.0:
            raise ValueError("omni_velocity_pid gains must be non-negative")
        if velocity_pid.integral_limit <= 0.0:
            raise ValueError("omni_velocity_pid integral_limit must be positive")
        if velocity_pid.output_limit <= 0.0:
            raise ValueError("omni_velocity_pid output_limit must be positive")
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
        for vehicle in self.vehicles:
            override = vehicle.execution_override
            if override is None:
                continue
            if override.max_wheel_speed_mps is not None and override.max_wheel_speed_mps <= 0.0:
                raise ValueError(
                    "execution_override.max_wheel_speed_mps must be positive"
                )
            if override.wheel_command_min_effective is not None and not (
                0.0 <= override.wheel_command_min_effective < execution.wheel_command_max
            ):
                raise ValueError(
                    "execution_override.wheel_command_min_effective must be in "
                    "[0, wheel_command_max)"
                )
            if override.wheel_flip is not None:
                if len(override.wheel_flip) != 4:
                    raise ValueError("execution_override.wheel_flip must contain four signs")
                if any(sign not in (-1.0, 1.0, -1, 1) for sign in override.wheel_flip):
                    raise ValueError("execution_override.wheel_flip entries must be +1 or -1")
        if self.runtime.converge_eps_rad <= 0.0:
            raise ValueError("converge_eps_rad must be positive")
        if self.runtime.converge_hold_s <= 0.0:
            raise ValueError("converge_hold_s must be positive")
        preflight = self.preflight
        if preflight.captain_edge_warn_deg <= 0.0:
            raise ValueError("captain_edge_warn_deg must be positive")
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
        constant = self.constant_velocity
        if constant.duration_s <= 0.0:
            raise ValueError("constant_velocity duration_s must be positive")
        constant_speed = (constant.vx_mps ** 2 + constant.vy_mps ** 2) ** 0.5
        if constant_speed > self.runtime.command_speed_limit_mps:
            raise ValueError(
                "constant_velocity speed must not exceed command_speed_limit_mps"
            )
