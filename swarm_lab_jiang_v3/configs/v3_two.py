"""Two-vehicle task-driven formation maneuvering experiment (v3).

v3 版 crew_two 等价配置：两车、任务速度 w=(0.1, 0)、先静止对准再巡航。

World frame and desired formation (v3 fixes scale as well as shape):

    car5 * (0, +0.8)
         | +y
         |
    car6 *----------> +x   (机动方向)

控制律：u_i = -k * sum_j a_ij (p_i - p_j + b_ij) + Z_i w
- b_ij = p_j* - p_i* 由 formation 坐标自动计算（含距离的完整位移）；
- Z = [平移x, 平移y]（标量权重连通图的零空间恰为整体平移，spec 2.4 节），
  每车 Z_i = I_2，任务项即全员叠加同一个世界系速度 w；
- 任务项位于零空间内，不会被约束项抵消（解耦）。

时序：t∈[0,3) 全员静止（航向自对准 + 静帧纠偏），t=3 s 起整体 +x 巡航
0.10 m/s。车号按现场可用车辆调整（当前 car6/car5 为 8-14 验证过的组合）。

ROS2 mocap topics (vrpn_client_ros default):
    /car6/pose  /car6/twist
    /car5/pose  /car5/twist
"""

from swarm.domain.config import (
    ExperimentConfig,
    ExecutionConfig,
    ObstacleAvoidanceConfig,
    TaskDrivenConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="v3_two",
    algorithm="task_driven",
    vehicles=(
        VehicleConfig("car4", "ground_vehicle", "10.1.1.84"),
        VehicleConfig("car5", "ground_vehicle", "10.1.1.85"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=((0, 1), (1, 0)),
        # 期望队形坐标 p_i*（世界系，米）；b_ij 由坐标自动计算。
        # 0.8 m 间距可按场地调整。
        formation={
            "car4": (0.0, 0.0),
            "car5": (0.0, 0.5),
        },
        # 标量边权（默认 1.0）；矩阵权重在 FormationSpec 预留接口。
        edge_weight_matrix=None,
    ),
    # 任务驱动项：w = (wx, wy)，每模态期望速度（m/s）。
    task_velocity=(0.10, 0.0),
    # 先静止对准再巡航：t∈[0,3) 静止，t=3 s 起 w=(0.10, 0)。
    task_velocity_schedule=[(0.0, (0.0, 0.0)), (3.0, (0.10, 0.0))],
    task_modes=("translate_x", "translate_y"),
    control=TaskDrivenConfig(k=0.8, deadband_mps=0.015),
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        # 航向对准机动方向 +x：摆放朝向随意，解锁后各车自转对准再巡航。
        heading_target_rad=0.0,
    ),
)