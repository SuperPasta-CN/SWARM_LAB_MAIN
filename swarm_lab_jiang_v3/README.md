# swarm_lab_jiang_v3 —— 矩阵加权拉普拉斯 + 任务驱动的编队机动工程

ROS2 动捕 + 麦轮小车 + UDP 下发轮速的编队机动实车工程。
本工程是 `../swarm_lab_jiang_v2` 的**控制律内核替换版**（v2 代码原样保留，
可复跑，供论文对比）：bearing-only 投影律 + 角色机制 →
**相对位置误差的矩阵加权拉普拉斯约束律 + 任务驱动项**，取消全部角色：

```
u_i = -k * Σ_{j∈N_i} A_ij (p_i − p_j + b_ij) + Z_i w
```

- **约束项**：`b_ij = p_j* − p_i*`（含距离的完整期望相对位移，由
  `formation` 坐标自动计算）；`A_ij` 当前为标量权重 `a_ij·I₂`
  （`edge_weight_matrix`，矩阵权重在 `FormationSpec.weight_blocks` 预留接口）。
  约束的是完整相对位置（方向 + 距离 + 尺度），不是仅方向。
- **任务项**：`Z` 的列张成矩阵加权拉普拉斯零空间内的允许运动模态
  （标量权重连通图 = 整体平移 x/y），按车分块 `Z_i`，全员统一叠加 `w`。
  任务运动在零空间内，不被约束项抵消（解耦）；位移误差直接收敛，
  **无需积分项**（与 v2 机动需要 ki 形成对照）。
- 整形链路沿用 v2：tanh 平滑饱和 → 死区（仅作用于约束项）→ 限速；
  执行层默认 `omni_pid`（车体速度前馈 + PI 闭环）。

## 目录结构

```
swarm_lab_jiang_v3/
├── README.md
├── USAGE.md                # 使用说明（现场实操手册）
├── MIGRATION.md              # v2 → v3 迁移说明（复用/替换清单 + 参数映射表）
├── requirements.txt          # matplotlib（可选）+ numpy（Z 构建）
├── run_experiment.py         # 入口：--config/--yes/--no-plot/--no-record
├── configs/
│   └── v3_two.py             # 两车机动（等价 v2 crew_two；先转后动 + w=(0.1,0)）
├── swarm/
│   ├── domain/               # models.py（EdgeSample：米制相对位置误差） config.py
│   ├── algorithms/
│   │   ├── task_driven.py    # v3 内核：FormationSpec + TaskDrivenFormationController
│   │   ├── pid.py            # PID（差速模式内部使用）
│   │   ├── mecanum.py        # 麦轮全向映射 + 航向保持（平移开环）
│   │   ├── mecanum_pid.py    # 麦轮全向 + 车体速度 PI 闭环（默认）
│   │   ├── differential.py   # 差速模式（旧方案移植，对比用）
│   │   ├── constant_velocity.py  # 恒速阶跃 planner（底盘标定备用）
│   │   └── obstacle_avoidance.py # APF 人工势场过滤器
│   ├── application/
│   │   ├── control_loop.py   # 50 Hz 主循环；收敛判据为米制边误差
│   │   ├── preflight.py      # mocap 有效 + 最小车距（硬）+ 初始队形误差表（米）
│   │   ├── planner_factory.py  bootstrap.py  interfaces.py
│   └── infrastructure/
│       ├── ros2_mocap.py     # rclpy 可选导入；后台线程 spin；twist 可选
│       ├── udp_chassis.py    # 持久 socket，报文 <FL,FR,RL,RR> 不变
│       ├── ground_vehicle.py # 执行器：按 execution.mode 分发 omni/omni_pid/diff
│       └── telemetry/        # console / live_plot / recorder / composite
│                             # （edge_metrics：米制边误差）
├── tools/
│   ├── check_wheels.py       # 单车开环硬件校验
│   └── postprocess.py        # 运行目录 → analysis.json（米制稳态统计）+ analysis.png
└── tests/                    # 全部不依赖 ROS2/实车
```

## 与 v2 的本质区别

| 维度 | v2 | v3 |
| --- | --- | --- |
| 误差量 | 单位方位误差 g−g*，投影 P=I−ggᵀ | 相对位置误差 p_i−p_j+b_ij，加权 A_ij |
| 约束对象 | 仅相对方向（不管距离/尺度） | 完整相对位置（含距离、尺度） |
| 零空间 | 平移 + 尺度伸缩 | 仅整体平移（标量权重情形） |
| 机动机制 | captain 固定速度 + first_mate 合成 + crew 纯约束 | 全员统一叠加 Z_i w，无角色 |
| 机动稳态误差 | 纯 P 有恒定滞后（需 ki 消除） | 结构性为零（无需积分项） |
| 收敛判据 | 角度（rad） | 相对位置误差（m，默认 0.03） |

## 环境与运行

- Python 3（标准库 + numpy + 可选 matplotlib）：`pip install -r requirements.txt`
- 实车机器另需 ROS2（apt）：rclpy、geometry-msgs、vrpn-client-ros。
- 无 ROS2 的电脑可跑全部单元测试：

```
python -m unittest discover -s tests
```

实车实验：

```
python run_experiment.py --config v3_two     # preflight → 回车解锁 → 机动
python tools/postprocess.py runs/v3_two_<时间戳> --window 10   # 后处理
```

## 配置要点（configs/v3_two.py 为模板）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `topology.formation` | — | 期望队形坐标 p_i*（世界系，米），b_ij 自动计算 |
| `topology.edge_weight_matrix` | None | 标量边权 a_ij（默认邻接处 1.0） |
| `control.k` | 0.8 | 约束增益（1/s 量级；e 单位为米） |
| `task_modes` | ("translate_x","translate_y") | Z 的模态；当前支持命名平移模态或显式 2n 向量 |
| `task_velocity` / `task_velocity_schedule` | (0,0) / () | w 常值 / 分段常值表；注意 schedule 与基值的一致性 |
| `execution.deadzone_mode` | `"pwm"` | 死区补偿策略：pwm（默认，消灭低速 bang-bang）/ lift（旧行为对照）/ affine（不推荐） |
| `execution.heading_deadband_rad` | 0.03 | 航向保持死区：小于该角度的偏差不纠偏（压制航向微振） |
| `runtime.velocity_ema_alpha` | 0.3 | 速度估计 EMA 系数；大=滞后小，0.5 实车未见收益 |
| `runtime.converge_eps_m` | 0.03 | 收敛判据（米）——受死区限制，残差下限 ≈ deadband/k |

## 符号约定与自检

`b_ij = p_j* − p_i*`（target − source），约束 `p_i − p_j + b_ij = 0` 在期望
队形处精确成立；`u = −k·ΣAe` 是误差能量的梯度下降。docstring
（`swarm/algorithms/task_driven.py` 顶部）与 `tests/test_task_driven.py`
内置了两车自检例（p_2 偏 +0.4 m → u_1 朝 +x、u_2 朝 −x，误差缩小）。

## 实验室检查单（与 v2 相同的关键项）

- [ ] 动捕标定完成，世界系 +x/+y 与队形坐标一致
- [ ] 车距 ≥ 0.20 m；初始构型尽量贴近期望（preflight 打印逐边米制误差表）
- [ ] 每辆车跑过 `check_wheels` 且方向正确；弱车用 `execution_override` 按车标定
- [ ] 解锁后 3 秒内确认各车实际开始移动（console 的 measured 列）
- [ ] 急停：Ctrl-C 下发停车；必要时断电
