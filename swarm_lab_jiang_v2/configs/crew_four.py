r"""Four-vehicle square formation maneuvering experiment.

World frame and desired formation:

          +y
           |
    car1 *----* car2  -> +x
         |  \ |
    car3 *----* car4

This formation is the upper ``2x2`` sub-formation of the six-vehicle
layout.  Roles: car1 = captain (fixed velocity only), car2 = first_mate
(fixed velocity blended with the bearing term), car3/car4 = crew (pure
bearing law).  Bearing-only control constrains relative directions and
the scale ratio, not the side length of the square.

captain_velocity=(0.10, 0, 0): constant translational maneuver; set to
(0, 0, 0) for a static formation (v1-equivalent behaviour).

Preflight note: the car1->car2 desired bearing is world +x.  A rotated
initial placement no longer blocks takeoff (soft warning only), but
placing car2 roughly in the +x direction of car1 converges faster.

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
    name="crew_four",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car1", "captain", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car2", "first_mate", "ground_vehicle", "10.1.1.82"),
        VehicleConfig("car3", "crew", "ground_vehicle", "10.1.1.83"),
        VehicleConfig("car4", "crew", "ground_vehicle", "10.1.1.84"),
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
    ),
    # 定速平移机动；改 (0.0, 0.0, 0.0) 即退化为静态编队。变速用
    # captain_velocity_schedule=[(t秒, (vx, vy, vz)), ...]（分段常值）。
    captain_velocity=(0.10, 0.0, 0.0),
    # first_mate 默认参数：v_fm=captain 速度（共享 v_c → 纯平移、尺度不变）、
    # α=0.5、weighted；需要 additive 或自定义时在 first_mate_params 里覆盖。
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    # bearing 项默认纯 P（ki=0）。leader 速度非零时纯 P 有恒定跟踪误差，
    # 开 PI：BearingControlConfig(ki=0.2, integral_limit=0.5)。
    # 电机死区补偿：非零轮指令低于该值时抬升。用 tools/check_wheels.py --ramp
    # 实测"刚滚动"的指令值后填它（略放大），仍卡再按 5 一档上调。
    # 个别车偏弱时用 VehicleConfig(execution_override=...) 按车标定。
    execution=ExecutionConfig(wheel_command_min_effective=25.0),
)
