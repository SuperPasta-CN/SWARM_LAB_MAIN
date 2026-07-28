"""Three-vehicle right-isosceles-triangle formation experiment.

World frame and desired formation:

    car6 * (0, +d)
         |\
      +y | \
         |  \
    car3 *---* car5  -> +x
        (0, 0) (+d, 0)

with ``d > 0``.  car5 sits in the +x direction of car3, car6 in the +y
direction, and both legs have equal length.  Bearing-only control fixes
relative directions and the formation scale ratio, not the absolute ``d``.

ROS2 mocap topics (vrpn_client_ros default):
    /car3/pose  /car3/twist
    /car5/pose  /car5/twist
    /car6/pose  /car6/twist
"""

from swarm.domain.config import (
    ExperimentConfig,
    ExecutionConfig,
    ObstacleAvoidanceConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="bearing_three",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car3", "leader", "ground_vehicle", "10.1.1.83"),
        VehicleConfig("car5", "follower", "ground_vehicle", "10.1.1.85"),
        VehicleConfig("car6", "follower", "ground_vehicle", "10.1.1.86"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=(
            (0, 1, 1),
            (1, 0, 1),
            (1, 1, 0),
        ),
        bearing_matrix=(
            (
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
            ),
            (
                (-1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (-1.0, 1.0, 0.0),
            ),
            (
                (0.0, -1.0, 0.0),
                (1.0, -1.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        leader_mask=(True, False, False),
    ),
    leader_target_velocities={"car3": (0.0, 0.0, 0.0)},
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
    # 电机死区补偿：非零轮指令低于该值时抬升。用 tools/check_wheels.py --ramp
    # 实测"刚滚动"的指令值后填它（略放大），仍卡再按 5 一档上调。
    execution=ExecutionConfig(wheel_command_min_effective=25.0),
)
