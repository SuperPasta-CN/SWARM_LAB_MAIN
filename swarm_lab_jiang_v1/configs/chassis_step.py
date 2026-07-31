"""Single-vehicle constant-velocity step test for chassis-mode comparison.

Commands car1 a fixed world-frame velocity for a few seconds, then stops
(``stop_on_converge`` ends the run automatically after the standstill
hold).  Run the same step with ``execution.mode`` = "omni" / "omni_pid" /
"diff" and compare steady-state velocity residuals with
``report_v1/chassis_comparison/postprocess_chassis.py``.

现场只需改三处：阶跃速度 (vx, vy)、时长、执行模式。速度不超过
``command_speed_limit_mps``（0.25），且应远大于编队死区 0.015 m/s；
diff 为非完整约束底盘，只能执行纵向（车头方向）阶跃。

ROS2 mocap topics (vrpn_client_ros default):
    /car1/pose  /car1/twist
"""

from swarm.domain.config import (
    ConstantVelocityConfig,
    ExperimentConfig,
    ExecutionConfig,
    ObstacleAvoidanceConfig,
    RuntimeConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="chassis_step",
    algorithm="constant_velocity",
    vehicles=(
        VehicleConfig("car1", "leader", "ground_vehicle", "10.1.1.81"),
    ),
    topology=TopologyConfig(
        # 恒速 planner 自带无边拓扑，此处仅为通过维度校验。
        adjacency_matrix=((0,),),
        bearing_matrix=(((0.0, 0.0, 0.0),),),
        leader_mask=(False,),
    ),
    leader_target_velocities={},
    constant_velocity=ConstantVelocityConfig(
        vx_mps=0.15,
        vy_mps=0.0,
        duration_s=6.0,
    ),
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
    execution=ExecutionConfig(
        # 三种底盘运动学对比："omni"（开环）/ "omni_pid"（速度 PI 闭环）/ "diff"（差速双 PID）
        mode="diff",
        mode="omni",
        mode="omni_pid",
        wheel_command_min_effective=35.0,
    ),
    runtime=RuntimeConfig(stop_on_converge=True),
)