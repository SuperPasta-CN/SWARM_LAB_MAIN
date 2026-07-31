r"""Six-vehicle flexible formation maneuvering experiment.

World frame and desired formation:

          +y
           |
    car1 *----* car2  -> +x
         |  \ |
    car3 *----* car4
         |    |
    car5 *----* car6

The lines only aid intuition; the effective edges and directions are
defined by the topology matrices below (ported unchanged from v1).

Roles: car1 = captain (fixed velocity only), car2 = first_mate (fixed
velocity blended with the bearing term), car3--car6 = crew (pure
bearing law).

captain_velocity=(0.10, 0, 0): constant translational maneuver; set to
(0, 0, 0) for a static formation (v1-equivalent behaviour).

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
    name="crew_six",
    algorithm="bearing",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "captain" if index == 1 else ("first_mate" if index == 2 else "crew"),
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in range(1, 7)
    ),
    topology=TopologyConfig(
        adjacency_matrix=_ADJACENCY,
        bearing_matrix=tuple(tuple(tuple(vector) for vector in row) for row in _BEARINGS),
    ),
    # 定速平移机动；改 (0.0, 0.0, 0.0) 即退化为静态编队。变速用
    # captain_velocity_schedule=[(t秒, (vx, vy, vz)), ...]（分段常值）。
    captain_velocity=(0.10, 0.0, 0.0),
    # first_mate 默认参数：v_fm=captain 速度（共享 v_c → 纯平移、尺度不变）、
    # α=0.5、weighted；需要 additive 或自定义时在 first_mate_params 里覆盖。
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
    # bearing 项默认纯 P（ki=0）。leader 速度非零时纯 P 有恒定跟踪误差，
    # 开 PI：BearingControlConfig(ki=0.2, integral_limit=0.5)。
    execution=ExecutionConfig(wheel_command_min_effective=25.0),
)
