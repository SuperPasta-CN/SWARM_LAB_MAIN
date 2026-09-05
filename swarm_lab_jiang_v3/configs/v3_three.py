"""Three-vehicle triangle task-driven formation maneuvering experiment (v3).

World frame and desired formation (right isosceles triangle, legs d = 0.8 m):

    car6 * (0, +d)
         |\
      +y | \
         |  \
    car3 *---* car5  -> +x   (机动方向)
       (0,0)  (+d,0)

控制律与 v3_two 相同：u_i = -k·Σ a_ij (p_i − p_j + b_ij) + Z_i w，
全连接拓扑（每车与另两车互为邻居），b_ij 由 formation 坐标自动计算。
车号按现场可用车辆调整（当前 car3/car5/car6 为 8-14 验证过的组合）。

ROS2 mocap topics (vrpn_client_ros default):
    /car3/pose  /car5/pose  /car6/pose
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
    name="v3_three",
    algorithm="task_driven",
    vehicles=(
        VehicleConfig("car2", "ground_vehicle", "10.1.1.82"),
        VehicleConfig("car5", "ground_vehicle", "10.1.1.85"),
        VehicleConfig("car4", "ground_vehicle", "10.1.1.84"),
    ),
    topology=TopologyConfig(
        # 全连接：位移约束下每对邻居互相约束（连通性是零空间正确性的前提）。
        adjacency_matrix=(
            (0, 1, 1),
            (1, 0, 1),
            (1, 1, 0),
        ),
        # 期望队形坐标 p_i*（世界系，米）：等腰直角三角形，直角在 car3。
        formation={
            "car2": (0.0, 0.0),
            "car5": (0.5, 0.0),
            "car4": (0.0, 0.5),
        },
        edge_weight_matrix=None,
    ),
    task_velocity=(0.10, 0.0),
    # 先静止对准再巡航：t∈[0,3) 静止，t=3 s 起 w=(0.10, 0)。
    task_velocity_schedule=[(0.0, (0.0, 0.0)), (3.0, (0.10, 0.0))],
    task_modes=("translate_x", "translate_y"),
    control=TaskDrivenConfig(k=0.8, deadband_mps=0.015),
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        # 航向对准机动方向 +x：摆放朝向随意，解锁后各车自转对准再巡航。
        heading_target_rad=0.0,
    ),
)