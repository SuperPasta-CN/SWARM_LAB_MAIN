"""Two-vehicle leader/follower experiment.

World frame and desired formation:

    car2 *
         | +y
         |
    car1 *--------> +x

The desired bearing of car1 towards car2 is world +y.  Bearing-only
control constrains relative directions only; the absolute distance
between the two vehicles is not regulated.

ROS2 mocap topics (vrpn_client_ros default):
    /car1/pose  /car1/twist
    /car2/pose  /car2/twist
"""

from swarm.domain.config import (
    ExperimentConfig,
    ExecutionConfig,
    ObstacleAvoidanceConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="bearing_two",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car1", "leader", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car2", "follower", "ground_vehicle", "10.1.1.82"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=((0, 1), (1, 0)),
        bearing_matrix=(
            ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
        ),
        leader_mask=(True, False),
    ),
    leader_target_velocities={"car1": (0.0, 0.0, 0.0)},
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    # 电机死区补偿：非零轮指令低于该值时抬升。用 tools/check_wheels.py --ramp
    # 实测"刚滚动"的指令值后填它（略放大），仍卡再按 5 一档上调。
    execution=ExecutionConfig(wheel_command_min_effective=25.0),
)
