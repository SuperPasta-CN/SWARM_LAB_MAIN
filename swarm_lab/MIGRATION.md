# 版本迭代史与迁移说明（v1 / v2 / v3 → swarm_lab）

本工程（`swarm_lab/`）是三轮实车迭代的收敛产物。旧工程目录原样保留、
保持可复跑，但**不再维护**；新实验一律基于 swarm_lab。

## 版本对照总表

| 版本 | 控制律内核 | 角色 | 机动机制 | 误差口径 | 状态 |
| --- | --- | --- | --- | --- | --- |
| `swarm_lab_jiang_v1` | bearing 投影律（静态编队） | 无 | 无（仅静态收敛） | 角度 | 归档 |
| `swarm_lab_jiang_v2` | bearing 投影律 + PID | captain / first_mate / crew | captain 定速 + first_mate 合成 + crew 纯约束 | 角度 | 归档（对照数据源） |
| `swarm_lab_jiang_v3` | 矩阵加权拉普拉斯 + 任务驱动 | 无 | 全员叠加 Z_i·w（schedule 手写静止窗） | 米 | 归档（被本工程取代） |
| `swarm_lab`（本工程） | task_driven（主力）+ bearing（对照） | 仅 bearing 用 | **二阶段状态机** + Z_i·w | 米（对照为角度） | 当前维护 |

## 关键迭代节点（问题 → 对策）

- **v1**：验证 bearing 静态编队可行性。
- **v2**：加机动（角色机制）。实车发现：crew 斜向行走速度损失严重
  （麦轮 60° 斜行效率仅 47–73%，跟不上直线行驶的 captain）→ 引出
  "航向对准 + 先转后动"思路；机动下纯 P 有恒定跟踪滞后 → first_mate
  合成 + ki。
- **v3**：控制律内核替换为矩阵加权拉普拉斯 + 任务驱动，取消角色。
  位移误差结构性收敛（无需积分项）。实车暴露低速振荡 → 归因机械
  死区（静摩擦）→ **PWM 占空比补偿**（消灭低速 bang-bang）+ 航向死区
  + **逐车标定**（calibrate_car）。
- **swarm_lab**：v3 手写的"schedule 前 3 秒静止"升级为显式二阶段
  状态机（收敛判据驱动，超时兜底）；v2 的 bearing 律原样回归为
  对照 planner（`bearing_two` 与 `task_two` 同几何 A/B）；标定链固化
  进配置体系。

## v3 → swarm_lab 映射

| v3 | swarm_lab | 说明 |
| --- | --- | --- |
| `configs/v3_two.py` | `configs/task_two.py` | 状态机取代手写静止窗 |
| `configs/v3_three.py` / `v3_four.py` | `configs/task_three.py` / `task_four.py` | 同 |
| `task_velocity_schedule` 首段 `(0, (0,0))` | 删除 | `converge_first` 自动提供静止段；schedule 时钟从 maneuver 切入时刻起算 |
| 无 | `control.converge_first / phase_converge_*` | 新增二阶段参数（eps 0.03 m / hold 2 s / timeout 10 s） |
| `VehicleConfig` 无 role | `role` 字段恢复 | 仅 `algorithm="bearing"` 时校验使用；task_driven 一律 `None` |
| 无 bearing 链 | `algorithms/bearing.py` + `telemetry/bearing_metrics.py` + `configs/bearing_two.py` | v2 原样移植 |
| 118 个测试 | 184 个测试 | 新增：bearing 回归、二阶段 PhaseTests、配置校验 |

`run_experiment.py` 默认配置：`v3_two` → `task_two`。运行产物目录结构、
CSV schema（`trajectory.csv` / `edge_errors.csv` 米制）不变，v3 的
后处理脚本与结论口径直接适用。

## v2 → swarm_lab（对照实验）映射

| v2 | swarm_lab | 说明 |
| --- | --- | --- |
| `configs/crew_two.py` | `configs/bearing_two.py` | 同几何（car4/captain + car5/first_mate，0.5 m 间距，0.10 m/s 向 +x） |
| `topology.bearing_matrix` | 同名 | 单位方向，仅方向约束 |
| `captain_velocity` / `_schedule` | 同名（ExperimentConfig 顶层） | 仅 bearing 算法使用 |
| `first_mate_params`（α/blend） | 同名 | `velocity=None` 解析为 captain 速度（v2 语义不变） |
| `bearing_control.kp/ki/kd` | 同名 | bearing_two 默认 `ki=0.2` 消机动滞后 |
| `converge_eps_rad`（0.05 rad） | `runtime.converge_eps_m`（0.03 m） | 收敛判据从角度改为相对位置误差 |
| `bearing_edges.csv`（度） | `edge_errors.csv`（米，task_driven）；bearing 运行时仍产 bearing 口径 | 后处理注意换算 |

## 复跑旧实验

```bash
# 旧工程在同级目录，依赖与运行方式各自独立：
cd ../swarm_lab_jiang_v2 && python3 run_experiment.py --config crew_two
cd ../swarm_lab_jiang_v3 && python3 run_experiment.py --config v3_two
```

对比口径建议：队形残差收敛时间 + 稳态值 + 行进振荡三个维度
（v2 数据为角度误差，v3/swarm_lab 为米制误差，换算后陈述）。
