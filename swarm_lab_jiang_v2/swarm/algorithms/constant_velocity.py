"""Constant-velocity step planner for chassis-level tracking experiments.

Every configured vehicle receives the same fixed world-frame velocity for
``duration_s`` seconds, then zero.  Formation planners close an outer loop
on shape, so their time-varying commands cannot serve as a known test
input; this planner issues a controlled step instead, and the recorded
``target_vx/vy`` against the actual velocity (positions, central
difference) directly exposes chassis tracking quality.  Used to compare
the three execution modes: omni (open loop) / omni_pid (velocity PI) /
diff.

The planner carries a trivial no-edge topology so the standard preflight
(mocap validity + minimum separation) works unchanged, and reports
``ready`` once the step ends so ``runtime.stop_on_converge`` stops the
loop automatically after the standstill hold time.
"""

from __future__ import annotations

from math import hypot
from typing import Sequence, Tuple

from swarm.algorithms.bearing import build_topology_from_matrices
from swarm.domain.models import MocapSnapshot, PlannerResult, VelocityCommand


class ConstantVelocityPlanner:
    """Issue one fixed world-frame velocity step to every vehicle."""

    def __init__(
        self,
        vehicle_ids: Sequence[str],
        velocity: Tuple[float, float],
        duration_s: float,
        command_speed_limit_mps: float,
    ) -> None:
        if not vehicle_ids:
            raise ValueError("vehicle_ids must not be empty")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if hypot(*velocity) > command_speed_limit_mps:
            raise ValueError("step speed must not exceed command_speed_limit_mps")
        self.vehicle_ids = tuple(vehicle_ids)
        self.vx, self.vy = float(velocity[0]), float(velocity[1])
        self.duration_s = duration_s
        # Trivial no-edge topology: lets bootstrap run the standard preflight.
        count = len(self.vehicle_ids)
        self.topology = build_topology_from_matrices(
            self.vehicle_ids,
            [[0] * count for _ in range(count)],
            [[(0.0, 0.0, 0.0)] * count for _ in range(count)],
            ["crew"] * count,
        )
        self._elapsed = 0.0

    def step(self, snapshot: MocapSnapshot, dt: float) -> PlannerResult:
        self._elapsed += dt
        active = self._elapsed <= self.duration_s
        result = PlannerResult(timestamp=snapshot.timestamp)
        for vehicle_id in self.vehicle_ids:
            state = snapshot.states.get(vehicle_id)
            valid = state is not None and state.valid
            vx, vy = (self.vx, self.vy) if active and valid else (0.0, 0.0)
            result.commands[vehicle_id] = VelocityCommand(
                vehicle_id=vehicle_id,
                vx=vx,
                vy=vy,
                vz=0.0,
                valid=valid,
            )
        # ready=True once the step is over: stop_on_converge then ends the run.
        result.ready = not active
        result.mean_error = 0.0
        result.max_error = 0.0
        return result

    def reset(self) -> None:
        self._elapsed = 0.0
