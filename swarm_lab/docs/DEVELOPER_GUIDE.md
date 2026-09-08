# swarm_lab 开发者指南

面向要改动/扩展本工程的人。现场实操请看 `docs/USER_GUIDE.md`，
版本沿革请看 `MIGRATION.md`。

## 1. 分层架构

```
domain          纯数据与配置：models.py（MocapSnapshot/PlannerResult/...）、
                config.py（全部 frozen dataclass + validate()）。无 IO。
algorithms      纯计算：控制律（task_driven/bearing/constant_velocity）、
                运动学（mecanum/mecanum_pid/differential/pid）、避障。
application     装配与编排：planner_factory（算法注册表）、bootstrap、
                control_loop（50 Hz 主循环）、preflight（起飞前硬校验）。
infrastructure  外部世界：ros2_mocap（rclpy 可选导入）、udp_chassis、
                ground_vehicle（执行器）、telemetry/（console/live_plot/recorder）。
```

依赖方向只允许向下（domain 不 import 任何上层）。`ros2_mocap` 的 rclpy
是可选导入，因此全部测试在无 ROS2 的机器上可跑。

## 2. 一拍控制周期（50 Hz）的数据流

```
ROS2 /carX/pose ──► MocapSnapshot{VehicleState(x,y,yaw,valid)}
  ──► planner.step(snapshot, dt) ──► PlannerResult{
        commands{vid: VelocityCommand(vx,vy,世界系 m/s)},
        edge_samples / bearing_samples, mean_error, ready}
  ──► obstacle_avoidance 过滤（可选，默认关）
  ──► ground_vehicle 执行器（逐车）：
        世界系速度 → 航向保持（k_omega，带 heading_deadband_rad 死区；
        heading_target_rad 设定时机动方向对准）→ 旋转到车体系
        ──► 按 execution.mode 分发：
            omni_pid（默认）麦轮映射 + 车体速度前馈+PI 闭环
                       （反馈 = mocap 位置差分 + EMA，kd=0）
            omni           麦轮映射，开环
            diff           差速：速度 PID + 航向 PID
        ──► 死区补偿 DeadzoneCompensator：
            pwm（默认） 占空比脉冲调制，ON 拍整向量按比例缩放到 min_eff，
                       时间平均 = 请求值，方向瞬时保持；可按 v_on 逐车归一化
            lift         固定抬升（旧行为，对照用）
            affine       死区逆（实车证伪，不推荐）
        ──► 轮指令限幅 [-100, 100] ──► UDP "<FL,FR,RL,RR>"
```

坐标/单位/符号约定：位置为世界系（动捕）米；`VelocityCommand` 为**世界系**
速度 m/s；`b_ij = p_j* − p_i*`（target − source），约束
`p_i − p_j + b_ij = 0` 在期望队形处精确成立，`u = −kΣAe` 是误差能量的
梯度下降（自检例见 `task_driven.py` docstring 与
`tests/test_task_driven.py::ControlLawSignTests`）。

planner 侧整形顺序（`task_driven.py`）：约束和 → **tanh 平滑饱和** →
**死区**（仅约束项，任务项永不被死区）→ **+ Z_i·w** → **限速**。
非线性环节顺序是有语义的一先饱和再死区，任务项在最后叠加以免被压掉。

## 3. 二阶段状态机（task_driven）

`swarm/algorithms/task_driven.py`（`_update_phase` / `_task_velocity_now`）：

- `converge`：`_task_velocity_now()` 返回零向量，车队只做约束纠偏
  （配合 `heading_target_rad` 同时完成航向对准）。
- 切换条件：`ready`（全员 mocap 有效）且 `mean_error < phase_converge_eps_m`
  持续 `phase_converge_hold_s`；误差冒头会重置保持计时
  （`test_error_spike_restarts_hold_timer`）。
- 兜底：`phase_converge_timeout_s` 到点必切，绝不在收敛阶段卡死。
- maneuver 阶段 schedule 时钟从切入时刻起算（`_maneuver_started_at`），
  因此配置里不再需要 v3 时代手写的"前 3 秒静止"schedule 段。
- `reset()` 回到 converge（重跑实验不复用相位）。
- 与 `control_loop` 的 `stop_on_converge`（`runtime.converge_eps_m` /
  `converge_hold_s`）是**两个独立机制**：状态机管"何时开始机动"，
  stop_on_converge 管"何时停车"。

## 4. 两种控制律对照

| 维度 | task_driven（主力） | bearing（对照） |
| --- | --- | --- |
| 误差量 | 相对位置误差 p_i−p_j+b_ij（米） | 单位方位误差 g−g*（经投影 P=I−ggᵀ） |
| 约束对象 | 完整相对位置（方向+距离+尺度） | 仅相对方向 |
| 零空间 | 仅整体平移（标量权重连通图） | 平移 + 尺度伸缩 |
| 机动机制 | 全员统一叠加 Z_i·w，无角色 | captain 定速 + first_mate 合成 + crew 纯约束 |
| 机动稳态误差 | 结构性为零 | 纯 P 恒定滞后，需 ki |
| 配置字段 | `formation`/`control.k`/`task_velocity`(+schedule)/`task_modes` | `bearing_matrix`/`role`/`captain_velocity`/`first_mate_params`/`bearing_control` |
| 遥测误差 | `edge_errors.csv`（米） | `bearing_edges.csv`（度） |

算法注册在 `application/planner_factory.py`（`task_driven` / `bearing` /
`constant_velocity`）；配置校验在 `domain/config.py::ExperimentConfig.validate`
（bearing 走角色/对边反向校验分支，task_driven 走 formation/连通性/
任务模态分支）。

## 5. 标定链（逐车执行参数）

```
tools/calibrate_car.py --address <ip> --mocap
  ├─ min_eff：斜坡 0→60，mocap 自动检测"开始滚动"（或 ENTER 手动），
  │           3 次取中位 + 裕量
  └─ v(35)：min_eff 指令处的持续车速，mocap 位移/窗口全自动
  ──► tools/calibrations/<车号>.json（原始记录）
  ──► configs/calibrated_overrides.py（可 import 的汇总，仅存 WSL 侧）
  ──► 配置中 VehicleConfig(execution_override=CALIBRATED_OVERRIDES.get("carX"))
  ──► ground_vehicle 逐车生效（VehicleExecutionOverride：
      max_wheel_speed_mps / wheel_command_min_effective / wheel_flip / pwm_v_on_mps）
```

已标定参考值：car4 min_eff 30.2 / v(35)=0.201 m/s；car5 23.5 / 0.283 m/s。
PWM 占空比归一化：`pwm_v_on_mps` 设置后 `duty = (指令×calib/100)/v_on`，
逐车消除速度增益差异；不设则退回 `duty = 指令/min_eff`。

## 6. 扩展点

- **新控制律**：`algorithms/` 新建文件实现 `SwarmPlanner` 接口
  （`step(snapshot, dt) -> PlannerResult`、`reset()`、`topology` 属性）→
  `planner_factory` 注册 builder → `config.py` 加算法专属 dataclass 与
  validate 分支 → 配单元测试 + 端到端收敛测试。
- **矩阵权重**：`FormationSpec.weight_blocks` 已是 (n,n,2,2) 存储，
  config 层 `edge_weight_matrix` 目前只暴露标量；扩展其语义或在
  `build_formation_spec` 加矩阵入口即可。
- **新任务模态**：`task_modes` 传显式 2n 向量（与命名模态可混用）；
  必须自行验证该模态 ⊂ null(L_A)，否则被约束项抵消。
- **bearing 3D**：`BearingControlConfig.bearing_3d` 已留口。

## 7. 测试策略

```bash
python -m unittest discover -s tests    # 当前 184 个，必须全绿才准交付
```

- 全部测试不依赖 ROS2/实车（mocap 用构造的 `MocapSnapshot`）。
- `test_task_driven.py`：裸控制律（helper 默认 `converge_first=False` 隔离
  状态机）+ `PhaseTests` 专测二阶段；`test_convergence.py`：理想单积分器
  stub 的端到端收敛/解耦回归；`test_bearing_law.py`：对照律回归。
- 改控制律必须同步改符号自检与收敛回归；改配置 schema 必须同步
  `test_config.py` 的校验用例。

## 8. 约定

- 配置一律 frozen dataclass，`validate()` 在任何外部资源启动前调用；
  非法配置必须在 preflight 之前就被拒绝。
- 文档语言：结构性 docstring 用英文，实验结论性注释（参数来历、实车
  教训）用中文并注明日期/数据源（沿用现状，例：`heading_target_rad`
  注释里的 08-14 斜行效率数据）。
- 每次实车实验的数据回传 Windows 侧 `report_v*` 归档分析，代码结论
  回写进对应注释/文档。
