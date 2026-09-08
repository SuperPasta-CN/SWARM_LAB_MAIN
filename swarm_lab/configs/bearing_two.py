"""Two-vehicle bearing comparison experiment (v2 role-based law).

对照配置：v2 的投影 bearing 律 + 角色机制（captain 定速 + first_mate
合成），用于与 task_two 做同队形、同车队、同速度的 A/B 对比。
队形几何与 task_two 相同（car5 在 car4 的 +y 方向；bearing-only 不管距离）。

ROS2 mocap topics (vrpn_client_ros default):
    /car4/pose  /car5/pose
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
    name="bearing_two",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car4", "ground_vehicle", "10.1.1.84", role="captain"),
        VehicleConfig("car5", "ground_vehicle", "10.1.1.85", role="first_mate"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=((0, 1), (1, 0)),
        bearing_matrix=(
            ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
        ),
    ),
    # captain 定速 0.10 m/s 向 +x；first_mate 默认共享该速度（加权合成）。
    captain_velocity=(0.10, 0.0, 0.0),
    # 机动下纯 P 有恒定跟踪误差，开 PI 消除（论文命题）。
    bearing_control=BearingControlConfig(ki=0.2, integral_limit=0.5),
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        heading_target_rad=0.0,
    ),
)
