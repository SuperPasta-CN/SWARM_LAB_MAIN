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
    # 定速平移机动；改 (0.0, 0.0, 0.0) 即退化为静态编队。变速用
    # captain_velocity_schedule=[(t秒, (vx, vy, vz)), ...]（分段常值）。
    captain_velocity=(0.10, 0.0, 0.0),
    # first_mate 默认参数：v_fm=captain 速度（共享 v_c → 纯平移、尺度不变）、
    # α=0.5、weighted；需要 additive 或自定义时在 first_mate_params 里覆盖。
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    # bearing 项默认纯 P（ki=0）。leader 速度非零时纯 P 有恒定跟踪误差，
    # 开 PI：BearingControlConfig(ki=0.2, integral_limit=0.5)。
    execution=ExecutionConfig(wheel_command_min_effective=25.0),
)
