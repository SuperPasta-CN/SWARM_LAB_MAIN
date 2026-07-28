"""Six-vehicle flexible formation experiment.

World frame and desired formation:

          +y
           |
    car1 *----* car2  -> +x
         |  \ |
    car3 *----* car4
         |    |
    car5 *----* car6

The lines only aid intuition; the effective edges and directions are
defined by the topology matrices below (ported unchanged from the legacy
project).

ROS2 mocap topics (vrpn_client_ros default):
    /car1/pose  /car1/twist  ...  /car6/pose  /car6/twist
"""

from swarm.domain.config import (
    ExperimentConfig,
    ExecutionConfig,
    ObstacleAvoidanceConfig,
    TopologyConfig,
    VehicleConfig,
)


_BEARINGS = [[[0.0, 0.0, 0.0] for _ in range(6)] for _ in range(6)]
for source, target, vector in (
    (0, 1, (1.0, 0.0, 0.0)),
    (0, 2, (0.0, -1.0, 0.0)),
    (0, 3, (1.0, -1.0, 0.0)),
    (1, 3, (0.0, -1.0, 0.0)),
    (2, 3, (1.0, 0.0, 0.0)),
    (2, 4, (0.0, -1.0, 0.0)),
    (3, 5, (0.0, -1.0, 0.0)),
    (4, 5, (1.0, 0.0, 0.0)),
):
    _BEARINGS[source][target] = list(vector)
    _BEARINGS[target][source] = [-value for value in vector]

_ADJACENCY = tuple(
    tuple(int(any(abs(value) > 0.0 for value in _BEARINGS[i][j])) for j in range(6))
    for i in range(6)
)

CONFIG = ExperimentConfig(
    name="bearing_six",
    algorithm="bearing",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "leader" if index in (1, 2, 6) else "follower",
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in range(1, 7)
    ),
    topology=TopologyConfig(
        adjacency_matrix=_ADJACENCY,
        bearing_matrix=tuple(tuple(tuple(vector) for vector in row) for row in _BEARINGS),
        leader_mask=(True, True, False, False, False, True),
    ),
    leader_target_velocities={
        "car1": (0, 0.0, 0.0),
        "car2": (0, 0.0, 0.0),
        "car6": (0, 0.0, 0.0),
    },
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
    # 电机死区补偿：非零轮指令低于该值时抬升。用 tools/check_wheels.py --ramp
    # 实测"刚滚动"的指令值后填它（略放大），仍卡再按 5 一档上调。
    execution=ExecutionConfig(wheel_command_min_effective=25.0),
)
