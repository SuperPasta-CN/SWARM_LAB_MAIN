# v2 → v3 迁移说明

v3 = v2 的骨架（动捕/执行/遥测/装配全部沿用）+ 控制律内核替换。
v2 代码原样保留在 `../swarm_lab_jiang_v2`，实验可随时复跑作对比。

## 模块复用与替换清单

**原样复用（零改动）**：

- `infrastructure/ros2_mocap.py`、`udp_chassis.py`、`ground_vehicle.py`
- `algorithms/mecanum.py`、`mecanum_pid.py`、`differential.py`、`pid.py`、
  `obstacle_avoidance.py`
- `application/interfaces.py`、`application/control_loop.py`（仅收敛判据
  参数改名 `converge_eps_rad` → `converge_eps`，单位米）
- `telemetry/composite.py`、`live_plot.py`
- `tools/check_wheels.py`

**适配性修改**：

- `application/bootstrap.py`：去掉 first_mate 解析；planner 由
  `task_driven` 算法构建；metadata schema_version = 5
- `application/preflight.py`：锚定/角色逻辑删除；新增逐边米制初始误差表 +
  初始队形 RMS 软警告（`formation_warn_m`）
- `telemetry/console.py`、`recorder.py`：bearing 角误差（deg）→ 相对位置
  边误差（m）；CSV 去掉 v2 的 `role` 列；`bearing_edges.csv` →
  `edge_errors.csv`
- `tools/postprocess.py`：米制稳态统计（steady_state 节结构不变）

**整体替换**：

- `algorithms/bearing.py` → `algorithms/task_driven.py`
  （`FormationSpec` + `TaskDrivenFormationController` + numpy 构建的 Z）
- 配置：`configs/crew_*.py` → `configs/v3_two.py`

## 参数映射表

| v2 | v3 | 说明 |
| --- | --- | --- |
| `topology.bearing_matrix`（单位方向） | `topology.formation`（每车期望坐标 p_i*） | b_ij = p_j* − p_i* 自动计算；v3 约束含距离/尺度 |
| `topology` 中的角色 / `VehicleConfig.role` | 无（VehicleConfig 删除 role 字段） | 全员同一控制律 |
| `captain_velocity` | `task_velocity`（w 常值） | 全员统一叠加，不再只给 leader |
| `captain_velocity_schedule` | `task_velocity_schedule` | 分段常值语义相同；**不再有 v2 的 v_fm 装配期解析陷阱**（w 是全局量，无角色合成） |
| `first_mate_params`（α/blend） | 无 | 合成机制删除 |
| `bearing_control.kp/ki/kd` | `control.k` | 位移误差结构性收敛，无需积分项 |
| `converge_eps_rad`（0.05 rad） | `converge_eps_m`（0.03 m） | 收敛判据从角度改为相对位置误差 |
| `preflight.captain_edge_warn_deg` | `preflight.formation_warn_m` | 软警告从角度改为队形 RMS（米） |
| `summary.json` 的 `*_bearing_error_*` | `*_edge_error_*`（米） | 后处理口径同步切换 |

## 理论对应（论文复现核对点）

- 约束项：`-k Σ_j A_ij (p_i − p_j + b_ij)`，`A_ij = a_ij·I₂`（标量权重，
  矩阵权重接口预留在 `FormationSpec.weight_blocks`）。
- 任务项：`Z = [z_1, …, z_q]`，`z_ℓ ∈ null(L_A)`；标量权重 + 连通图时
  null(L_A) 恰为整体平移 x/y（规格书 2.4 节的两车例子：
  Z = [[1,0],[0,1],[1,0],[0,1]]，Z_i = I₂），
  `build_task_modes` 按模态名生成，单元测试逐值核对。
- 解耦：Z_i w ∈ null(L_A) → 约束项不抵消任务运动；
  反之尺度/形变类初始偏差（不在零空间）仍由约束项消除（有单元测试与
  端到端回归覆盖）。
