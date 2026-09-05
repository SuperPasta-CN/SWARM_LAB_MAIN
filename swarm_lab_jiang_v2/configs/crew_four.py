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
    BearingControlConfig,
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
    # 巡航速度（同时也是 first_mate v_fm 的默认引用，装配期一次性解析——
    # 用 schedule 变速时必须与 schedule 末值保持一致）。
    captain_velocity=(0.10, 0.0, 0.0),
    # 先转向后行驶：t∈[0,3) 全员静止（航向自对准 + 静帧纠偏），t=3 s 起巡航。
    captain_velocity_schedule=[(0.0, (0.0, 0.0, 0.0)), (3.0, (0.10, 0.0, 0.0))],
    # first_mate 默认参数：v_fm=captain 速度（共享 v_c → 纯平移、尺度不变）、
    # α=0.5、weighted；需要 additive 或自定义时在 first_mate_params 里覆盖。
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    # bearing 项开 PI：ki 消除机动纯 P 稳态误差（8/14 crew_two 实证稳态 ~1°）。
    bearing_control=BearingControlConfig(ki=0.2, integral_limit=0.5),
    # 电机死区补偿：非零轮指令低于该值时抬升。用 tools/check_wheels.py --ramp
    # 实测"刚滚动"的指令值后填它（略放大），仍卡再按 5 一档上调。
    # 个别车偏弱时用 VehicleConfig(execution_override=...) 按车标定。
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        # 航向对准机动方向 +x：摆放朝向随意，解锁后各车自转对准再巡航。
        heading_target_rad=0.0,
    ),
)
