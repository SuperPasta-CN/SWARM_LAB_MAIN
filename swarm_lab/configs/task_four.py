"""Four-vehicle square task-driven formation maneuvering experiment.

正方形队形（边长 0.8 m）：

          +y
           |
    car4 *----* car5  (y=+d)
         |    |
    car1 *----* car2  -> +x   (机动方向)
       (0,0)  (+d,0)

全连接拓扑（K4）；控制律与 task_two 相同（二阶段：先收敛后巡航）。

ROS2 mocap topics (vrpn_client_ros default):
    /car1/pose  /car2/pose  /car4/pose  /car5/pose
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
    from configs.calibrated_overrides import CALIBRATED_OVERRIDES
except ImportError:
    CALIBRATED_OVERRIDES = {}


CONFIG = ExperimentConfig(
    name="task_four",
    algorithm="task_driven",
    vehicles=(
        VehicleConfig(
            "car1", "ground_vehicle", "10.1.1.81",
            execution_override=CALIBRATED_OVERRIDES.get("car1"),
        ),
        VehicleConfig(
            "car2", "ground_vehicle", "10.1.1.82",
            execution_override=CALIBRATED_OVERRIDES.get("car2"),
        ),
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
        adjacency_matrix=(
            (0, 1, 1, 1),
            (1, 0, 1, 1),
            (1, 1, 0, 1),
            (1, 1, 1, 0),
        ),
        formation={
            "car1": (0.0, 0.0),
            "car2": (0.8, 0.0),
            "car4": (0.0, 0.8),
            "car5": (0.8, 0.8),
        },
    ),
    task_velocity=(0.10, 0.0),
    task_modes=("translate_x", "translate_y"),
    control=TaskDrivenConfig(k=0.8),
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    execution=ExecutionConfig(
        wheel_command_min_effective=35.0,
        heading_target_rad=0.0,
    ),
)
