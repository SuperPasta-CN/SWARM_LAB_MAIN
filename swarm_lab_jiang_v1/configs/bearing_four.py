"""Four-vehicle square formation experiment.

World frame and desired formation:

          +y
           |
    car1 *----* car2  -> +x
         |  \ |
    car3 *----* car4

This formation is the upper ``2x2`` sub-formation of the six-vehicle
layout.  car1 and car2 are stationary leaders, car3 and car4 followers.
Bearing-only control constrains relative directions and the scale ratio,
not the side length of the square.

Preflight note: the car1->car2 desired bearing is world +x, so car2 must
be placed in the +x direction of car1 (within anchor_tolerance_deg).

ROS2 mocap topics (vrpn_client_ros default):
    /car1/pose  /car1/twist  ...  /car4/pose  /car4/twist
"""

from swarm.domain.config import (
    ExperimentConfig,
    ExecutionConfig,
    ObstacleAvoidanceConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="bearing_four",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car1", "leader", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car2", "leader", "ground_vehicle", "10.1.1.82"),
        VehicleConfig("car3", "follower", "ground_vehicle", "10.1.1.83"),
        VehicleConfig("car4", "follower", "ground_vehicle", "10.1.1.84"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=(
            (0, 1, 1, 1),
            (1, 0, 0, 1),
            (1, 0, 0, 1),
            (1, 1, 1, 0),
        ),
        bearing_matrix=(
            (
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
                (1.0, -1.0, 0.0),
            ),
            (
                (-1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
            ),
            (
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
            ),
            (
                (-1.0, 1.0, 0.0),
                (0.0, 1.0, 0.0),
                (-1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        leader_mask=(True, True, False, False),
    ),
    leader_target_velocities={
        "car1": (0.0, 0.0, 0.0),
        "car2": (0.0, 0.0, 0.0),
    },
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    # 电机死区补偿：非零轮指令低于该值时抬升。用 tools/check_wheels.py --ramp
    # 实测"刚滚动"的指令值后填它（略放大），仍卡再按 5 一档上调。
    execution=ExecutionConfig(wheel_command_min_effective=25.0),
)
