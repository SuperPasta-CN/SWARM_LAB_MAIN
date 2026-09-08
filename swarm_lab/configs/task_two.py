"""Two-vehicle task-driven formation maneuvering experiment.

主力配置（二阶段范式）：先收敛编队（converge 阶段，自动静止纠偏 +
航向对准），边误差稳定低于阈值后切入机动阶段（maneuver），全员统一
叠加任务速度 w=(0.10, 0) 向 +x 巡航。

Desired formation (car5 在 car4 的 +y 方向 0.5 m)：

    car5 * (0, +0.5)
         | +y
         |
    car4 *----------> +x   (机动方向)

车号按现场可用车辆调整（当前 car4/car5 为 9-04/9-07 验证过的组合）。

ROS2 mocap topics (vrpn_client_ros default):
    /car4/pose  /car5/pose
"""

from swarm.domain.config import (
    ExperimentConfig,
    ExecutionConfig,
    ObstacleAvoidanceConfig,
    TaskDrivenConfig,
    TopologyConfig,
    VehicleConfig,
)

try:
    # 逐车标定参数（tools/calibrate_car.py 自动生成；不存在时回退全局参数）
    from configs.calibrated_overrides import CALIBRATED_OVERRIDES
except ImportError:
    CALIBRATED_OVERRIDES = {}


CONFIG = ExperimentConfig(
    name="task_two",
    algorithm="task_driven",
    vehicles=(
        VehicleConfig(
            "car4", "ground_vehicle", "10.1.1.84",
            execution_override=CALIBRATED_OVERRIDES.get("car4"),
        ),
        VehicleConfig(
            "car5", "ground_vehicle", "10.1.1.85",
            execution_override=CALIBRATED_OVERRIDES.get("car5"),
        ),
    ),
    topology=TopologyConfig(
        adjacency_matrix=((0, 1), (1, 0)),
        formation={"car4": (0.0, 0.0), "car5": (0.0, 0.5)},
    ),
    # 机动速度：二阶段状态机在收敛后自动切入；无需 schedule。
    task_velocity=(0.10, 0.0),
    task_modes=("translate_x", "translate_y"),
    control=TaskDrivenConfig(k=0.8),
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        # 航向对准机动方向 +x（默认 pwm 死区补偿与航向死区继承全局默认）。
        heading_target_rad=0.0,
    ),
)
