"""Typed configuration grouped by algorithm, experiment, and runtime concerns (v3).

v3 replaces the bearing-only projection law and the captain/first_mate/crew
role system with the matrix-weighted Laplacian constraint law plus a
task-driven term:

    u_i = -k * sum_j A_ij (p_i - p_j + b_ij) + Z_i w

The desired formation is given as per-vehicle coordinates ``p_i*`` (the
``b_ij`` blocks are derived from them), task modes ``Z`` by name, and the
mode velocities ``w`` support a piecewise-constant schedule.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field
from math import sqrt
from typing import Mapping, Optional, Sequence, Tuple

from swarm.domain.models import Vector2, Vector3


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float


@dataclass(frozen=True)
class TaskDrivenConfig:
    """Parameters of the matrix-weighted Laplacian constraint law.

    ``k`` is the formation gain (constraint term ``-k * sum_j A_ij e_ij``
    with the relative-position error ``e_ij = p_i - p_j + b_ij`` in meters).
    ``deadband_mps`` zeroes the *shaped* constraint term below it (the task
    term is never deadbanded).
    """

    k: float = 0.8
    deadband_mps: float = 0.015


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
    """Velocity-to-wheel mapping shared by omni / omni_pid / diff modes."""

    mode: str = "omni_pid"  # "omni_pid" (mecanum + velocity PI, default), "omni" (open-loop mecanum), "diff" (differential)
    heading_hold: bool = True
    k_omega: float = 2.0
    omega_max: float = 1.5
    # World-frame heading setpoint for heading_hold (rad).  None = capture the
    # yaw at unlock (legacy behavior).  Set it (e.g. to the maneuver
    # direction) and each car rotates to face it after unlock, whatever its
    # parked yaw: mecanum oblique driving is far less efficient than
    # straight driving (08-14 run: 47-73% speed achievement at ~60 deg).
    heading_target_rad: Optional[float] = None
    # 航向保持死区：|yaw 误差| 小于该值时 omega = 0（抑制航向通道经死区
    # 抬升产生的极限环；0.03 rad ≈ 1.7°，09-07 实车验证 yaw 摆动显著下降；
    # 0 = 不启用）。
    heading_deadband_rad: float = 0.03
    # 死区补偿策略（振荡治理 A/B 模块）："pwm" 占空比脉冲调制（默认，
    # 09-07 实车+仿真双证：消灭低速 bang-bang，整向量按比例缩放后以
    # min_eff 脉冲输出，时间平均等于请求值，瞬时方向保持）；
    # "lift" 固定抬升（旧行为，留作对照）；"affine" 死区逆
    # （实车证伪：去量化后振荡同频同幅，不推荐）。
    deadzone_mode: str = "pwm"
    # pwm 模式的载波周期（控制拍数；50 Hz 下 10 拍 = 5 Hz 载波）。
    pwm_period_cycles: int = 10
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
    """Per-vehicle execution-side calibration (weak cars get their own).

    Any field left ``None`` inherits the experiment-wide ExecutionConfig.
    """

    max_wheel_speed_mps: Optional[float] = None
    wheel_command_min_effective: Optional[float] = None
    wheel_flip: Optional[Tuple[float, float, float, float]] = None


@dataclass(frozen=True)
class VehicleConfig:
    """One vehicle.  v3 has no roles: every agent runs the same law."""

    vehicle_id: str
    platform: str
    address: str
    execution_override: Optional[VehicleExecutionOverride] = None


@dataclass(frozen=True)
class TopologyConfig:
    """Adjacency + desired formation coordinates + optional scalar weights.

    ``formation`` maps each vehicle_id to its desired world-frame position
    ``p_i*`` (meters); the controller derives ``b_ij = p_j* - p_i*``.
    ``edge_weight_matrix`` holds scalar weights ``a_ij`` (default 1.0 on
    adjacent pairs); matrix weights are an extension point of the spec.
    """

    adjacency_matrix: Sequence[Sequence[int]]
    formation: Mapping[str, Vector2]
    edge_weight_matrix: Optional[Sequence[Sequence[float]]] = None


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

    ``formation_warn_m`` is a *soft* warning threshold on the initial
    formation RMS error (meters): reported, never blocks takeoff.
    """

    min_separation_m: float = 0.20
    mocap_timeout_s: float = 10.0
    require_confirmation: bool = True
    formation_warn_m: float = 0.15


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
    # EMA smoothing factor for the pose-only velocity estimate: larger =
    # less lag and less smoothing.  The feedback delay of this estimator
    # (baseline/2 + EMA lag) is a phase-lag source for the omni_pid loop;
    # reduce it (or set require_twist=True) to damp the cruise limit cycle.
    velocity_ema_alpha: float = 0.3
    stop_on_loss_of_mocap: bool = True
    udp_port: int = 12345
    udp_timeout_s: float = 0.2
    command_speed_limit_mps: float = 0.25
    stop_on_converge: bool = False
    # Convergence in meters (mean relative-position edge error).
    converge_eps_m: float = 0.03
    converge_hold_s: float = 3.0
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)


@dataclass(frozen=True)
class ConstantVelocityConfig:
    """Fixed world-frame velocity step for chassis tracking tests."""

    vx_mps: float = 0.0
    vy_mps: float = 0.0
    duration_s: float = 5.0


def _speed(vector) -> float:
    return sqrt(sum(component * component for component in vector))


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    algorithm: str
    vehicles: Tuple[VehicleConfig, ...]
    topology: TopologyConfig
    # Task-driven term: w (per-mode desired velocity, meters/second per
    # translation mode) with an optional piecewise-constant schedule
    # [(t_seconds, w), ...]; empty schedule = constant task_velocity.
    task_velocity: Tuple[float, ...] = (0.0, 0.0)
    task_velocity_schedule: Sequence[Tuple[float, Tuple[float, ...]]] = ()
    # Task modes of the Z basis: names from NAMED_TASK_MODES
    # ("translate_x", "translate_y") or explicit 2n vectors.
    task_modes: Tuple = ("translate_x", "translate_y")
    control: TaskDrivenConfig = field(default_factory=TaskDrivenConfig)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    preflight: PreflightConfig = field(default_factory=PreflightConfig)
    obstacle_avoidance: ObstacleAvoidanceConfig = field(default_factory=ObstacleAvoidanceConfig)
    static_obstacles: Tuple[StaticObstacleConfig, ...] = ()
    constant_velocity: ConstantVelocityConfig = field(default_factory=ConstantVelocityConfig)

    @property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    def _validate_topology(self) -> None:
        from swarm.algorithms.task_driven import NAMED_TASK_MODES

        n = len(self.vehicle_ids)
        adjacency = self.topology.adjacency_matrix
        if len(adjacency) != n or any(len(row) != n for row in adjacency):
            raise ValueError("adjacency_matrix must be square and match vehicle count")
        formation = self.topology.formation
        missing = [vid for vid in self.vehicle_ids if vid not in formation]
        if missing:
            raise ValueError("formation is missing coordinates for: %s" % missing)
        weights = self.topology.edge_weight_matrix
        if weights is not None:
            if len(weights) != n or any(len(row) != n for row in weights):
                raise ValueError("edge_weight_matrix must be square and match vehicle count")
            for row in weights:
                for value in row:
                    if value < 0.0:
                        raise ValueError("edge weights must be non-negative")
        # The task modes are meaningful only when null(L_A) is exactly the
        # global translations, which requires a connected graph.
        seen = {0}
        frontier = [0]
        while frontier:
            node = frontier.pop()
            for other in range(n):
                if other not in seen and (adjacency[node][other] or adjacency[other][node]):
                    seen.add(other)
                    frontier.append(other)
        if len(seen) != n:
            raise ValueError("adjacency graph must be connected")
        for mode in self.task_modes:
            if isinstance(mode, str) and mode not in NAMED_TASK_MODES:
                raise ValueError(
                    "unknown task mode %r; available: %s" % (mode, NAMED_TASK_MODES)
                )

    def _validate_task_velocity(self) -> None:
        limit = self.runtime.command_speed_limit_mps
        q = len(self.task_modes)
        if len(self.task_velocity) != q:
            raise ValueError(
                "task_velocity dimension %d must match the task-mode count %d"
                % (len(self.task_velocity), q)
            )
        if _speed(self.task_velocity) > limit:
            raise ValueError("task_velocity must not exceed command_speed_limit_mps")
        previous_t = 0.0
        for t, velocity in self.task_velocity_schedule:
            if t < 0.0:
                raise ValueError("task_velocity_schedule times must be >= 0")
            if t < previous_t:
                raise ValueError("task_velocity_schedule times must be non-decreasing")
            previous_t = t
            if len(velocity) != q:
                raise ValueError(
                    "task_velocity_schedule velocities must have %d entries" % q
                )
            if _speed(velocity) > limit:
                raise ValueError(
                    "task_velocity_schedule speeds must not exceed "
                    "command_speed_limit_mps"
                )
        # Consistency guard: when a schedule exists, its final entry (not the
        # base task_velocity) is the effective cruise speed — warn loudly
        # instead of silently ignoring the base value (09-07 field incident).
        if self.task_velocity_schedule:
            final = tuple(float(v) for v in self.task_velocity_schedule[-1][1])
            base = tuple(float(v) for v in self.task_velocity)
            if final != base:
                warnings.warn(
                    "task_velocity %s differs from the final scheduled velocity %s: "
                    "the schedule takes precedence after its last entry, so %s is "
                    "the effective cruise speed (align both to change cruise speed)"
                    % (base, final, final),
                    UserWarning,
                    stacklevel=3,
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
        self._validate_topology()
        self._validate_task_velocity()
        if self.runtime.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if self.runtime.max_plausible_speed_mps <= 0.0:
            raise ValueError("max_plausible_speed_mps must be positive")
        if self.runtime.velocity_diff_baseline_s <= 0.0:
            raise ValueError("velocity_diff_baseline_s must be positive")
        if not 0.0 < self.runtime.velocity_ema_alpha <= 1.0:
            raise ValueError("velocity_ema_alpha must be in (0, 1]")
        if self.runtime.command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        control = self.control
        if control.k <= 0.0:
            raise ValueError("formation gain k must be positive")
        if control.deadband_mps < 0.0:
            raise ValueError("deadband_mps must be non-negative")
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
        if execution.heading_deadband_rad < 0.0:
            raise ValueError("heading_deadband_rad must be non-negative")
        if execution.deadzone_mode not in ("lift", "affine", "pwm"):
            raise ValueError("deadzone_mode must be 'lift', 'affine' or 'pwm'")
        if execution.pwm_period_cycles < 1:
            raise ValueError("pwm_period_cycles must be >= 1")
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
        if self.runtime.converge_eps_m <= 0.0:
            raise ValueError("converge_eps_m must be positive")
        if self.runtime.converge_hold_s <= 0.0:
            raise ValueError("converge_hold_s must be positive")
        preflight = self.preflight
        if preflight.min_separation_m < 0.0:
            raise ValueError("min_separation_m must be non-negative")
        if preflight.mocap_timeout_s <= 0.0:
            raise ValueError("mocap_timeout_s must be positive")
        if preflight.formation_warn_m <= 0.0:
            raise ValueError("formation_warn_m must be positive")
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
