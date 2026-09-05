"""Two-vehicle captain/first-mate maneuvering experiment.

World frame and desired formation:

    car2 *
         | +y
         |
    car1 *--------> +x

Roles: car1 = captain (fixed velocity only), car2 = first_mate (fixed
velocity blended with the bearing term).  Two vehicles is the minimal
maneuvering formation of Zhao & Zelazo (two "leaders", zero crew): the
captain carries the translation reference and the first mate's fixed
velocity pins the formation scale; pure-crew vehicles do not exist here.

The desired bearing of car1 towards car2 is world +y.  Bearing-only
control constrains relative directions only; the absolute distance
between the two vehicles is not regulated.

captain_velocity=(0.10, 0, 0): constant translational maneuver; set to
(0, 0, 0) for a static formation (v1-equivalent behaviour).

ROS2 mocap topics (vrpn_client_ros default):
    /car1/pose  /car1/twist
    /car2/pose  /car2/twist
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
    name="crew_two",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car1", "captain", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car2", "first_mate", "ground_vehicle", "10.1.1.82"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=((0, 1), (1, 0)),
        bearing_matrix=(
            ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
        ),
    ),
    # 巡航速度（同时也是 first_mate v_fm 的默认引用，装配期一次性解析——
    # 用 schedule 变速时必须与 schedule 末值保持一致）。
    captain_velocity=(0.10, 0.0, 0.0),
    # 先转向后行驶：t∈[0,3) 全员静止（航向自对准 + 静帧纠偏），t=3 s 起巡航
    # （治 08-14 起步段小指令被 min_eff 抬升导致的方向畸变/近撞车）。
    captain_velocity_schedule=[(0.0, (0.0, 0.0, 0.0)), (3.0, (0.10, 0.0, 0.0))],
    # first_mate 默认参数：v_fm=captain 速度（共享 v_c → 纯平移、尺度不变）、
    # α=0.5、weighted；需要 additive 或自定义时在 first_mate_params 里覆盖。
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    # bearing 项开 PI：ki 消除机动纯 P 稳态误差（论文命题；8/3、8/14 数据实证）。
    bearing_control=BearingControlConfig(ki=0.2, integral_limit=0.5),
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        # 航向对准机动方向 +x：摆放朝向随意，解锁后各车自转对准再巡航
        # （麦轮斜行效率远低于直行，08-14 斜行段速度达成率仅 47-73%）。
        heading_target_rad=0.0,
    ),
)
