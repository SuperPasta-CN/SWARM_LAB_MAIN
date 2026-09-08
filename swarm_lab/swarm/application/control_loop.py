"""Deterministic control-loop orchestration independent of ROS and UDP."""

from __future__ import annotations

import time
from typing import Callable, Dict, Mapping, Optional

from swarm.application.interfaces import (
    Clock,
    StateSource,
    SwarmPlanner,
    TelemetrySink,
    VehicleActuator,
    VelocityCommandFilter,
)
from swarm.domain.models import ActuationResult, ControlFrame


class SystemClock:
    """Production monotonic clock adapter."""

    @staticmethod
    def monotonic() -> float:
        return time.monotonic()

    @staticmethod
    def sleep(seconds: float) -> None:
        time.sleep(seconds)


class ControlLoop:
    """Run sensing, planning, platform execution, and telemetry in order."""

    def __init__(
        self,
        control_hz: float,
        stop_on_loss_of_mocap: bool,
        state_source: StateSource,
        planner: SwarmPlanner,
        actuators: Mapping[str, VehicleActuator],
        telemetry: TelemetrySink,
        shutdown_requested: Callable[[], bool],
        clock: Optional[Clock] = None,
        velocity_filter: Optional[VelocityCommandFilter] = None,
        stop_on_converge: bool = False,
        converge_eps: float = 0.03,
        converge_hold_s: float = 3.0,
    ) -> None:
        if control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if not actuators:
            raise ValueError("actuators must not be empty")
        if converge_eps <= 0.0:
            raise ValueError("converge_eps must be positive")
        if converge_hold_s <= 0.0:
            raise ValueError("converge_hold_s must be positive")
        self.nominal_dt = 1.0 / control_hz
        self.stop_on_loss_of_mocap = stop_on_loss_of_mocap
        self.state_source = state_source
        self.planner = planner
        self.actuators = dict(actuators)
        self.telemetry = telemetry
        self.shutdown_requested = shutdown_requested
        self.clock = clock or SystemClock()
        self.velocity_filter = velocity_filter
        self.stop_on_converge = stop_on_converge
        self.converge_eps = converge_eps
        self.converge_hold_s = converge_hold_s
        self.previous_control_time: Optional[float] = None
        self.cycle = 0
        self.converged = False
        self._below_eps_since: Optional[float] = None

    def _compute_control_dt(self, current_time: float) -> float:
        if self.previous_control_time is None:
            return self.nominal_dt
        measured_dt = current_time - self.previous_control_time
        return measured_dt if measured_dt > 0.0 else self.nominal_dt

    def _sleep_remaining_period(self, cycle_started_at: float) -> None:
        remaining = self.nominal_dt - (self.clock.monotonic() - cycle_started_at)
        if remaining > 0.0:
            self.clock.sleep(remaining)

    def _stop_all(self) -> None:
        for actuator in self.actuators.values():
            try:
                actuator.stop()
            except Exception:
                pass

    def _update_convergence(self, mean_error: float, ready: bool, now: float) -> None:
        if not self.stop_on_converge:
            return
        # Only a fully observed formation may accumulate convergence time.
        if ready and mean_error < self.converge_eps:
            if self._below_eps_since is None:
                self._below_eps_since = now
            elif now - self._below_eps_since >= self.converge_hold_s and not self.converged:
                self.converged = True
                print(
                    "converged | mean edge error < %.4f m for %.1f s, stopping"
                    % (self.converge_eps, self.converge_hold_s)
                )
        else:
            self._below_eps_since = None

    def run_cycle(self) -> Optional[ControlFrame]:
        """Run one cycle and return its frame, or ``None`` while mocap is unready."""

        cycle_started_at = self.clock.monotonic()
        snapshot = self.state_source.get_snapshot()
        if self.stop_on_loss_of_mocap and not self.state_source.all_valid():
            self._stop_all()
            self._sleep_remaining_period(cycle_started_at)
            return None

        control_dt = self._compute_control_dt(cycle_started_at)
        self.previous_control_time = cycle_started_at
        nominal_planner_result = self.planner.step(snapshot, control_dt)
        planner_result = nominal_planner_result
        if self.velocity_filter is not None:
            planner_result = self.velocity_filter.apply(snapshot, planner_result)
        actuations: Dict[str, ActuationResult] = {}
        for vehicle_id, actuator in self.actuators.items():
            command = planner_result.commands[vehicle_id]
            state = snapshot.states.get(vehicle_id)
            if state is None:
                continue
            actuations[vehicle_id] = actuator.execute(command, state, control_dt)

        self._update_convergence(
            planner_result.mean_error,
            planner_result.ready,
            cycle_started_at,
        )

        frame = ControlFrame(
            monotonic_time=cycle_started_at,
            dt=control_dt,
            snapshot=snapshot,
            planner=planner_result,
            actuations=actuations,
            nominal_planner=nominal_planner_result,
        )
        self.telemetry.emit(frame)
        self.cycle += 1
        self._sleep_remaining_period(cycle_started_at)
        return frame

    def run(self) -> None:
        """Run until shutdown/convergence and always issue final stop commands."""

        self.state_source.start()
        try:
            while not self.shutdown_requested() and not self.converged:
                self.run_cycle()
        except KeyboardInterrupt:
            pass
        finally:
            self._stop_all()
            self.telemetry.close()
            self.state_source.stop()
