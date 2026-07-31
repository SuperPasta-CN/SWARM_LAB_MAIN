"""Construct swarm planners selected by experiment configuration."""

from __future__ import annotations

from dataclasses import replace
from typing import Callable, Dict, Tuple

from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.algorithms.constant_velocity import ConstantVelocityPlanner
from swarm.application.interfaces import SwarmPlanner
from swarm.domain.config import ExperimentConfig, FirstMateParams


PlannerBuilder = Callable[[ExperimentConfig], SwarmPlanner]


def _build_bearing_planner(config: ExperimentConfig) -> SwarmPlanner:
    topology = build_topology_from_matrices(
        vehicle_ids=config.vehicle_ids,
        adjacency_matrix=config.topology.adjacency_matrix,
        bearing_matrix=config.topology.bearing_matrix,
        roles=[vehicle.role for vehicle in config.vehicles],
    )
    # Resolve every first mate's fixed velocity: default FirstMateParams for
    # missing entries, and velocity=None means "use the captain velocity".
    first_mate_params: Dict[str, FirstMateParams] = {}
    for vehicle in config.vehicles:
        if vehicle.role != "first_mate":
            continue
        params = config.first_mate_params.get(vehicle.vehicle_id, FirstMateParams())
        if params.velocity is None:
            params = replace(params, velocity=config.captain_velocity)
        first_mate_params[vehicle.vehicle_id] = params
    return BearingOnlyFormationController(
        topology=topology,
        captain_velocity=config.captain_velocity,
        captain_velocity_schedule=config.captain_velocity_schedule,
        first_mate_params=first_mate_params,
        control=config.bearing_control,
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
    "bearing": _build_bearing_planner,
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
