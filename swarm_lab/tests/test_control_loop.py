"""Application orchestration tests using in-memory adapters."""

from __future__ import annotations

import unittest

from swarm.application.control_loop import ControlLoop
from swarm.domain.models import (
    ActuationResult,
    MocapSnapshot,
    PlannerResult,
    VehicleState,
    VelocityCommand,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeSource:
    def __init__(self, valid: bool = True) -> None:
        self.valid = valid
        self.started = False
        self.stopped = False
        self.states = {
            "car1": VehicleState("car1", x=0.0, y=0.0, valid=valid),
            "car2": VehicleState("car2", x=0.5, y=0.2, valid=valid),
        }

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def get_snapshot(self) -> MocapSnapshot:
        return MocapSnapshot(1.0, self.states)

    def all_valid(self) -> bool:
        return self.valid


class FakePlanner:
    def __init__(self, mean_error: float = 0.5) -> None:
        self.mean_error = mean_error
        self.calls = 0

    def step(self, snapshot: MocapSnapshot, dt: float) -> PlannerResult:
        self.calls += 1
        result = PlannerResult(
            timestamp=snapshot.timestamp, mean_error=self.mean_error, ready=True
        )
        for vehicle_id in snapshot.states:
            result.commands[vehicle_id] = VelocityCommand(
                vehicle_id=vehicle_id, vx=0.1, vy=0.0, valid=True
            )
        return result

    def reset(self) -> None:
        return None


class FakeActuator:
    def __init__(self, vehicle_id: str) -> None:
        self.vehicle_id = vehicle_id
        self.executed = 0
        self.stopped = 0
        self.target_velocities = []

    def execute(self, command, state, dt):
        self.executed += 1
        self.target_velocities.append((command.vx, command.vy, command.vz))
        return ActuationResult(self.vehicle_id, "fake", {"front_left": command.vx}, True)

    def stop(self) -> bool:
        self.stopped += 1
        return True


class MemoryTelemetry:
    def __init__(self) -> None:
        self.frames = []
        self.closed = False

    def emit(self, frame) -> None:
        self.frames.append(frame)

    def close(self) -> None:
        self.closed = True


def _loop(source, planner, actuators, telemetry, clock, shutdown_after=3, **kwargs):
    cycles = {"count": 0}

    def shutdown_requested() -> bool:
        return cycles["count"] >= shutdown_after

    loop = ControlLoop(
        control_hz=50.0,
        stop_on_loss_of_mocap=True,
        state_source=source,
        planner=planner,
        actuators=actuators,
        telemetry=telemetry,
        shutdown_requested=shutdown_requested,
        clock=clock,
        **kwargs,
    )
    original_run_cycle = loop.run_cycle

    def run_cycle():
        frame = original_run_cycle()
        if frame is not None:
            cycles["count"] += 1
        return frame

    loop.run_cycle = run_cycle
    return loop


class ControlLoopTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = FakeSource()
        self.planner = FakePlanner()
        self.actuators = {"car1": FakeActuator("car1"), "car2": FakeActuator("car2")}
        self.telemetry = MemoryTelemetry()
        self.clock = FakeClock()

    def test_run_cycle_executes_and_emits(self) -> None:
        loop = _loop(self.source, self.planner, self.actuators, self.telemetry, self.clock)
        frame = loop.run_cycle()
        self.assertIsNotNone(frame)
        self.assertEqual(self.actuators["car1"].executed, 1)
        self.assertEqual(self.actuators["car2"].executed, 1)
        self.assertEqual(len(self.telemetry.frames), 1)
        self.assertAlmostEqual(frame.dt, 1.0 / 50.0)

    def test_mocap_loss_stops_actuators_and_skips_planning(self) -> None:
        self.source.valid = False
        for state in self.source.states.values():
            state.valid = False
        loop = _loop(self.source, self.planner, self.actuators, self.telemetry, self.clock)
        frame = loop.run_cycle()
        self.assertIsNone(frame)
        self.assertEqual(self.planner.calls, 0)
        self.assertEqual(self.actuators["car1"].stopped, 1)
        self.assertEqual(self.actuators["car2"].stopped, 1)
        self.assertEqual(len(self.telemetry.frames), 0)

    def test_run_stops_everything_in_finally(self) -> None:
        loop = _loop(self.source, self.planner, self.actuators, self.telemetry, self.clock)
        loop.run()
        self.assertTrue(self.source.started)
        self.assertTrue(self.source.stopped)
        self.assertEqual(self.actuators["car1"].executed, 3)
        self.assertGreaterEqual(self.actuators["car1"].stopped, 1)
        self.assertTrue(self.telemetry.closed)
        self.assertFalse(loop.converged)

    def test_stop_on_converge_exits_by_itself(self) -> None:
        planner = FakePlanner(mean_error=0.01)  # below converge_eps
        loop = _loop(
            self.source,
            planner,
            self.actuators,
            self.telemetry,
            self.clock,
            shutdown_after=10 ** 9,
            stop_on_converge=True,
            converge_eps=0.05,
            converge_hold_s=0.1,
        )
        loop.run()
        self.assertTrue(loop.converged)
        self.assertGreaterEqual(self.actuators["car1"].stopped, 1)
        self.assertTrue(self.telemetry.closed)

    def test_error_above_eps_never_converges(self) -> None:
        planner = FakePlanner(mean_error=0.5)
        loop = _loop(
            self.source,
            planner,
            self.actuators,
            self.telemetry,
            self.clock,
            shutdown_after=5,
            stop_on_converge=True,
            converge_eps=0.05,
            converge_hold_s=0.1,
        )
        loop.run()
        self.assertFalse(loop.converged)


if __name__ == "__main__":
    unittest.main()
