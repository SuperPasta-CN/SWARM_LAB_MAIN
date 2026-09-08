"""Rate-limited console telemetry for live experiment tuning (v3)."""

from __future__ import annotations

from typing import Optional

from swarm.domain.models import ControlFrame
from swarm.infrastructure.telemetry.edge_metrics import edge_error_m


class ConsoleTelemetrySink:
    """Print compact planner, velocity, heading, wheel, and delivery state."""

    def __init__(self, output_hz: float) -> None:
        self.interval = 1.0 / max(output_hz, 1e-6)
        self._next_output_time: Optional[float] = None

    def emit(self, frame: ControlFrame) -> None:
        if self._next_output_time is not None and frame.monotonic_time < self._next_output_time:
            return
        self._next_output_time = frame.monotonic_time + self.interval
        edge_errors_m = [
            error
            for sample in frame.planner.edge_samples
            for error in (edge_error_m(sample),)
            if error is not None
        ]
        mean_edge_error_m = (
            sum(edge_errors_m) / len(edge_errors_m) if edge_errors_m else 0.0
        )
        max_edge_error_m = max(edge_errors_m) if edge_errors_m else 0.0
        print(
            "control | t=%.3f ready=%s mean_edge_error_m=%.4f "
            "max_edge_error_m=%.4f valid_edges=%d dt=%.4f"
            % (
                frame.snapshot.timestamp,
                frame.planner.ready,
                mean_edge_error_m,
                max_edge_error_m,
                len(edge_errors_m),
                frame.dt,
            )
        )
        for vehicle_id, planner_command in frame.planner.commands.items():
            nominal_command = planner_command
            if frame.nominal_planner is not None:
                nominal_command = frame.nominal_planner.commands.get(
                    vehicle_id,
                    planner_command,
                )
            state = frame.snapshot.states.get(vehicle_id)
            actuation = frame.actuations.get(vehicle_id)
            if state is None or actuation is None:
                print("vehicle | %s state_or_actuation=NA" % vehicle_id)
                continue
            values = actuation.values
            print(
                "vehicle | %s formation=(%.3f,%.3f,%.3f) "
                "apf_target=(%.3f,%.3f,%.3f) apf_delta=(%.3f,%.3f,%.3f) "
                "measured=(%.3f,%.3f,%.3f) "
                "heading=%.3f/%.3f speed=%.3f/%.3f cmd=(%d,%d,%d,%d) sent=%s"
                % (
                    vehicle_id,
                    nominal_command.vx,
                    nominal_command.vy,
                    nominal_command.vz,
                    planner_command.vx,
                    planner_command.vy,
                    planner_command.vz,
                    planner_command.vx - nominal_command.vx,
                    planner_command.vy - nominal_command.vy,
                    planner_command.vz - nominal_command.vz,
                    state.vx,
                    state.vy,
                    state.vz,
                    state.yaw,
                    values.get("target_heading", 0.0),
                    values.get("measured_speed", 0.0),
                    values.get("target_speed", 0.0),
                    int(values.get("front_left", 0.0)),
                    int(values.get("front_right", 0.0)),
                    int(values.get("rear_left", 0.0)),
                    int(values.get("rear_right", 0.0)),
                    actuation.sent,
                )
            )
        for sample in frame.planner.edge_samples:
            error_m = edge_error_m(sample)
            error_text = "%.4f m" % error_m if error_m is not None else "NA"
            print(
                "edge | %s->%s desired=(%.3f,%.3f,%.3f) "
                "actual=(%.3f,%.3f,%.3f) error=%s"
                % (
                    sample.source_id,
                    sample.target_id,
                    *sample.desired,
                    *sample.actual,
                    error_text,
                )
            )

    @staticmethod
    def close() -> None:
        return None
