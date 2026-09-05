"""Four-vehicle square task-driven formation maneuvering experiment (v3).

World frame and desired formation (square, side d = 0.8 m):

          +y
           |
    car4 *----* car5  (y=+d)
         |    |
    car1 *----* car2  -> +x   (机动方向)
       (0,0)  (+d,0)

控制律与 v3_two 相同：u_i = -k·Σ a_ij (p_i − p_j + b_ij) + Z_i w，
全连接拓扑（K4），b_ij 由 formation 坐标自动计算。
车号按现场可用车辆调整（当前 car2/car3/car5/car6 为 8-14 下午跑过的组合）。

ROS2 mocap topics (vrpn_client_ros default):
    /car2/pose  /car3/pose  /car5/pose  /car6/pose
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
    name="v3_four",
    algorithm="task_driven",
    vehicles=(
        VehicleConfig("car1", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car2", "ground_vehicle", "10.1.1.82"),
        VehicleConfig("car4", "ground_vehicle", "10.1.1.84"),
        VehicleConfig("car5", "ground_vehicle", "10.1.1.85"),
    ),
    topology=TopologyConfig(
        # 全连接 K4：位移约束下每对邻居互相约束。
        adjacency_matrix=(
            (0, 1, 1, 1),
            (1, 0, 1, 1),
            (1, 1, 0, 1),
            (1, 1, 1, 0),
        ),
        # 期望队形坐标 p_i*（世界系，米）：0.8 m 边长正方形。
        formation={
            "car1": (0.0, 0.0),
            "car2": (0.8, 0.0),
            "car4": (0.0, 0.8),
            "car5": (0.8, 0.8),
        },
        edge_weight_matrix=None,
    ),
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