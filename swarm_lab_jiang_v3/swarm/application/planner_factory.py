"""Construct swarm planners selected by experiment configuration (v3)."""

from __future__ import annotations

from typing import Callable, Dict, Tuple

from swarm.algorithms.constant_velocity import ConstantVelocityPlanner
from swarm.algorithms.task_driven import (
    TaskDrivenFormationController,
    build_formation_spec,
)
from swarm.application.interfaces import SwarmPlanner
from swarm.domain.config import ExperimentConfig


PlannerBuilder = Callable[[ExperimentConfig], SwarmPlanner]


def _build_task_driven_planner(config: ExperimentConfig) -> SwarmPlanner:
    spec = build_formation_spec(
        vehicle_ids=config.vehicle_ids,
        adjacency_matrix=config.topology.adjacency_matrix,
        formation=config.topology.formation,
        edge_weight_matrix=config.topology.edge_weight_matrix,
        task_modes=config.task_modes,
    )
    return TaskDrivenFormationController(
        spec=spec,
        control=config.control,
        task_velocity=config.task_velocity,
        task_velocity_schedule=config.task_velocity_schedule,
        command_speed_limit_mps=config.runtime.command_speed_limit_mps,
    )


def _build_constant_velocity_planner(config: ExperimentConfig) -> SwarmPlanner:
    return ConstantVelocityPlanner(
        vehicle_ids=config.vehicle_ids,
        velocity=(config.constant_velocity.vx_mps, config.constant_velocity.vy_mps),
        duration_s=config.constant_velocity.duration_s,
        command_speed_limit_mps=config.runtime.command_speed_limit_mps,
    )


# Each selectable planner has one builder registered under its config value.
_PLANNER_BUILDERS: Dict[str, PlannerBuilder] = {
    "task_driven": _build_task_driven_planner,
    "constant_velocity": _build_constant_velocity_planner,
}


def available_algorithms() -> Tuple[str, ...]:
    """Return the algorithm names accepted by :func:`build_swarm_planner`."""

    return tuple(sorted(_PLANNER_BUILDERS))


def build_swarm_planner(config: ExperimentConfig) -> SwarmPlanner:
    """Build the planner selected by ``config.algorithm``."""

    try:
        builder = _PLANNER_BUILDERS[config.algorithm]
    except KeyError:
        supported = ", ".join(available_algorithms())
        raise ValueError(
            "unsupported swarm algorithm %r; available algorithms: %s"
            % (config.algorithm, supported)
        ) from None
    return builder(config)
