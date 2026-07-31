"""Build production adapters from a validated experiment configuration.

``rclpy`` is imported lazily inside :func:`build_control_loop` so that this
module (and everything importing it) loads on machines without ROS2; only
actually connecting to the fleet requires the ROS2 runtime.
"""

from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

from swarm.algorithms.differential import DifferentialDriveController
from swarm.algorithms.mecanum import MecanumController
from swarm.algorithms.mecanum_pid import MecanumPidController
from swarm.algorithms.obstacle_avoidance import ArtificialPotentialField
from swarm.application.control_loop import ControlLoop
from swarm.application.interfaces import TelemetrySink
from swarm.application.planner_factory import build_swarm_planner
from swarm.application.preflight import (
    confirm_or_abort,
    print_preflight_report,
    run_preflight_checks,
    wait_for_mocap,
)
from swarm.domain.config import ExecutionConfig, ExperimentConfig, VehicleConfig
from swarm.infrastructure.ground_vehicle import GroundVehicleActuator
from swarm.infrastructure.telemetry.composite import CompositeTelemetrySink
from swarm.infrastructure.telemetry.console import ConsoleTelemetrySink
from swarm.infrastructure.telemetry.live_plot import LiveTrajectoryPlotSink
from swarm.infrastructure.telemetry.recorder import ExperimentRecorder
from swarm.infrastructure.udp_chassis import ChassisSocketDriver, SocketConfig


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _run_directory(root: str, experiment_name: str) -> Path:
    root_path = Path(root)
    if not root_path.is_absolute():
        root_path = _project_root() / root_path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return root_path / (experiment_name + "_" + timestamp)


def _build_telemetry(config: ExperimentConfig) -> CompositeTelemetrySink:
    telemetry = config.runtime.telemetry
    sinks: List[TelemetrySink] = []
    if telemetry.console_enabled:
        sinks.append(ConsoleTelemetrySink(telemetry.console_hz))
    run_directory = None
    if telemetry.record_enabled or telemetry.live_plot_enabled:
        # One run directory per experiment: the CSV recorder and the final
        # trajectory PNG share it.
        run_directory = _run_directory(telemetry.record_root, config.name)
    if telemetry.record_enabled:
        metadata: Dict[str, Any] = {
            "schema_version": 4,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "experiment": asdict(config),
        }
        recorder = ExperimentRecorder(
            run_directory=run_directory,
            metadata=metadata,
            record_hz=telemetry.record_hz,
            flush_interval_s=telemetry.record_flush_interval_s,
        )
        print("recording | %s" % recorder.run_directory)
        sinks.append(recorder)
    if telemetry.live_plot_enabled:
        sinks.append(
            LiveTrajectoryPlotSink(
                vehicle_ids=config.vehicle_ids,
                plot_hz=telemetry.live_plot_hz,
                queue_size=telemetry.live_plot_queue_size,
                max_points=telemetry.live_plot_max_points,
                save_path=str(run_directory / "trajectory.png"),
            )
        )
    return CompositeTelemetrySink(sinks)


def _vehicle_execution(config: ExperimentConfig, vehicle: VehicleConfig) -> ExecutionConfig:
    """Execution config for one vehicle with its per-vehicle override applied."""

    override = vehicle.execution_override
    if override is None:
        return config.execution
    updates = {
        field: getattr(override, field)
        for field in (
            "max_wheel_speed_mps",
            "wheel_command_min_effective",
            "wheel_flip",
        )
        if getattr(override, field) is not None
    }
    return replace(config.execution, **updates)


def _build_vehicle_controller(config: ExperimentConfig, vehicle: VehicleConfig):
    execution = _vehicle_execution(config, vehicle)
    if execution.mode == "diff":
        return DifferentialDriveController(
            execution,
            config.runtime.command_speed_limit_mps,
        )
    if execution.mode == "omni_pid":
        return MecanumPidController(execution)
    return MecanumController(execution)


def _run_preflight(config: ExperimentConfig, state_source, topology, assume_yes: bool) -> None:
    """Gate motor unlock on mocap and separation checks (captain-edge warnings are soft)."""

    state_source.start()
    try:
        if not wait_for_mocap(
            state_source,
            config.vehicle_ids,
            config.preflight.mocap_timeout_s,
        ):
            raise SystemExit(
                "preflight | timed out after %.1f s waiting for valid mocap"
                % config.preflight.mocap_timeout_s
            )
        report = run_preflight_checks(
            state_source.get_snapshot(),
            topology,
            config.preflight,
        )
        print_preflight_report(report)
        if not confirm_or_abort(
            report,
            require_confirmation=config.preflight.require_confirmation,
            assume_yes=assume_yes,
        ):
            raise SystemExit("preflight | aborted, motors stay locked")
    except BaseException:
        state_source.stop()
        raise


def build_control_loop(config: ExperimentConfig, assume_yes: bool = False) -> ControlLoop:
    """Construct the planner, runtime, and vehicles after a passed preflight."""

    config.validate()

    try:
        import rclpy
    except ImportError:
        raise SystemExit(
            "rclpy is required for live experiments but is not installed. "
            "Install ROS2 (e.g. 'sudo apt install ros-humble-rclpy "
            "ros-humble-geometry-msgs ros-humble-vrpn-client-ros') on the "
            "experiment machine. Unit tests run without ROS2."
        ) from None

    # Initialize ROS2 context (safe to call multiple times)
    if not rclpy.ok():
        rclpy.init()

    from swarm.infrastructure.ros2_mocap import Ros2MocapSource

    state_source = Ros2MocapSource(
        config.vehicle_ids,
        config.runtime.mocap_prefix,
        require_twist=config.runtime.require_twist,
        max_plausible_speed_mps=config.runtime.max_plausible_speed_mps,
        velocity_diff_baseline_s=config.runtime.velocity_diff_baseline_s,
    )
    try:
        planner = build_swarm_planner(config)
        _run_preflight(config, state_source, planner.topology, assume_yes)

        velocity_filter = ArtificialPotentialField(
            config=config.obstacle_avoidance,
            command_speed_limit_mps=config.runtime.command_speed_limit_mps,
            static_obstacles=config.static_obstacles,
        )
        socket_config = SocketConfig(
            port=config.runtime.udp_port,
            timeout_s=config.runtime.udp_timeout_s,
        )
        actuators = {}
        for vehicle in config.vehicles:
            if vehicle.platform != "ground_vehicle":
                raise ValueError(
                    "no actuator factory registered for platform: %s" % vehicle.platform
                )
            execution = _vehicle_execution(config, vehicle)
            actuators[vehicle.vehicle_id] = GroundVehicleActuator(
                vehicle_id=vehicle.vehicle_id,
                controller=_build_vehicle_controller(config, vehicle),
                driver=ChassisSocketDriver(vehicle.address, socket_config),
                min_command=execution.wheel_command_min_effective,
            )
        return ControlLoop(
            control_hz=config.runtime.control_hz,
            stop_on_loss_of_mocap=config.runtime.stop_on_loss_of_mocap,
            state_source=state_source,
            planner=planner,
            actuators=actuators,
            telemetry=_build_telemetry(config),
            shutdown_requested=lambda: not rclpy.ok(),
            velocity_filter=velocity_filter,
            stop_on_converge=config.runtime.stop_on_converge,
            converge_eps_rad=config.runtime.converge_eps_rad,
            converge_hold_s=config.runtime.converge_hold_s,
        )
    except BaseException:
        state_source.stop()
        raise
