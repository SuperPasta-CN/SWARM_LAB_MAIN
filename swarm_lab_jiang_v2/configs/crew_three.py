"""Three-vehicle right-isosceles-triangle maneuvering experiment.

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

Roles: car3 = captain (fixed velocity only), car5 = first_mate (fixed
velocity blended with the bearing term), car6 = crew (pure bearing law).

captain_velocity=(0.10, 0, 0): constant translational maneuver; set to
(0, 0, 0) for a static formation (v1-equivalent behaviour).

ROS2 mocap topics (vrpn_client_ros default):
    /car3/pose  /car3/twist
    /car5/pose  /car5/twist
    /car6/pose  /car6/twist
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
    name="crew_three",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car3", "captain", "ground_vehicle", "10.1.1.83"),
        VehicleConfig("car5", "first_mate", "ground_vehicle", "10.1.1.85"),
        VehicleConfig("car6", "crew", "ground_vehicle", "10.1.1.86"),
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
    ),
    # 巡航速度（同时也是 first_mate v_fm 的默认引用，装配期一次性解析——
    # 用 schedule 变速时必须与 schedule 末值保持一致）。
    captain_velocity=(0.10, 0.0, 0.0),
    # 先转向后行驶：t∈[0,3) 全员静止（航向自对准 + 静帧纠偏），t=3 s 起巡航。
    captain_velocity_schedule=[(0.0, (0.0, 0.0, 0.0)), (3.0, (0.10, 0.0, 0.0))],
    # first_mate 默认参数：v_fm=captain 速度（共享 v_c → 纯平移、尺度不变）、
    # α=0.5、weighted；需要 additive 或自定义时在 first_mate_params 里覆盖。
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
    # bearing 项开 PI：ki 消除机动纯 P 稳态误差（8/14 crew_two 实证稳态 ~1°）。
    bearing_control=BearingControlConfig(ki=0.2, integral_limit=0.5),
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        # 航向对准机动方向 +x：摆放朝向随意，解锁后各车自转对准再巡航。
        heading_target_rad=0.0,
    ),
)
