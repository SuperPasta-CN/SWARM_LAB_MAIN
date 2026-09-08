# swarm_lab —— 实验室多车编队控制工程

ROS2 动捕（vrpn_client_ros）+ 麦轮小车 + UDP 轮速下发的平面编队机动
实车工程。由 v1（静态 bearing 编队）→ v2（bearing 投影律 + 角色机动）→
v3（矩阵加权拉普拉斯 + 任务驱动）三轮实车迭代收敛而来，是实验室的
**当前正式版本**；旧工程 `swarm_lab_jiang_v1/v2/v3` 原样保留作历史对照，
可复跑（见 `MIGRATION.md`）。

## 控制律

**主力：`task_driven`** —— 矩阵加权拉普拉斯约束 + 任务驱动项：

```
u_i = −k · Σ_{j∈N_i} A_ij·(p_i − p_j + b_ij)  +  Z_i · w
```

- 约束项：期望相对位置 `b_ij = p_j* − p_i*` 由 `formation` 坐标自动计算；
  约束完整相对位置（方向 + 距离 + 尺度），不是仅方向。
- 任务项：`Z` 的列张成矩阵加权拉普拉斯零空间（标量权重连通图 = 整体
  平移 x/y），全员统一叠加 `w`；任务运动不被约束项抵消（结构性解耦，
  机动稳态误差为零，无需积分项）。
- **二阶段范式**（`control.converge_first=True`，默认开）：先收敛后机动。
  converge 阶段任务速度强制为零，只做编队纠偏 + 航向对准；平均边误差
  稳定低于 `phase_converge_eps_m`（默认 0.03 m）持续
  `phase_converge_hold_s`（默认 2 s）后切入 maneuver 阶段，任务 schedule
  时钟从切入时刻起算；`phase_converge_timeout_s`（默认 10 s）超时兜底，
  不会卡死在收敛阶段。

**对照：`bearing`** —— v2 的投影 bearing-only 律 + captain/first_mate/crew
角色机制（Zhao & Zelazo 形式），原样保留用于 A/B 对比；只约束相对方位
（不管距离/尺度），机动下纯 P 有恒定跟踪滞后（需 ki 消除）。

**工具：`constant_velocity`** —— 恒速阶跃 planner，底盘标定/测速用。

## 配置目录（configs/）

| 配置 | 算法 | 车辆 | 内容 |
| --- | --- | --- | --- |
| `task_two` | task_driven | car4/car5 | 两车 0.5 m 间距，收敛后 w=(0.10, 0) 向 +x 巡航 |
| `task_three` | task_driven | car2/car5/car4 | 三角队形（0.5 m 边） |
| `task_four` | task_driven | car1/car2/car4/car5 | 正方形队形（0.8 m 边） |
| `bearing_two` | bearing | car4(captain)/car5(first_mate) | 与 task_two 同几何的 A/B 对照 |

逐车执行参数（min_eff、v_on 等）由 `tools/calibrate_car.py` 标定后写入
`configs/calibrated_overrides.py`，配置经 `execution_override` 自动引用；
文件不存在时回退全局参数（见 `docs/DEVELOPER_GUIDE.md` 标定链一节）。

## 目录结构

```
swarm_lab/
├── README.md                 # 本文件
├── MIGRATION.md              # v1/v2/v3 → swarm_lab 迭代史与参数映射
├── docs/
│   ├── USER_GUIDE.md         # 使用者手册（现场实操）
│   └── DEVELOPER_GUIDE.md    # 开发者指南（架构/状态机/扩展点/标定链）
├── requirements.txt          # numpy + 可选 matplotlib
├── run_experiment.py         # 入口：--config/--yes/--no-plot/--no-record
├── configs/                  # 实验配置（上表）+ calibrated_overrides.py（标定产物）
├── swarm/
│   ├── domain/               # models.py / config.py（全部 dataclass 配置与校验）
│   ├── algorithms/
│   │   ├── task_driven.py    # 主力内核：FormationSpec + 二阶段状态机
│   │   ├── bearing.py        # 对照内核：v2 投影 bearing 律 + 角色机制
│   │   ├── constant_velocity.py
│   │   ├── mecanum.py / mecanum_pid.py / differential.py / pid.py
│   │   └── obstacle_avoidance.py
│   ├── application/
│   │   ├── control_loop.py   # 50 Hz 主循环
│   │   ├── preflight.py      # mocap 有效 + 最小车距（硬）+ 初始队形误差表（软）
│   │   └── planner_factory.py  bootstrap.py  interfaces.py
│   └── infrastructure/
│       ├── ros2_mocap.py     udp_chassis.py  ground_vehicle.py
│       └── telemetry/        # console / live_plot / recorder / composite
│                             # + edge_metrics（米制）/ bearing_metrics（角度，对照用）
├── tools/
│   ├── calibrate_car.py      # 逐车标定（min_eff 半自动 + v_on 全自动）
│   ├── check_wheels.py       # 单车开环硬件校验
│   └── postprocess.py        # runs 目录 → analysis.json/png（米制稳态统计）
└── tests/                    # 单元 + 端到端测试，全部不依赖 ROS2/实车
```

## 快速开始

```bash
pip install -r requirements.txt          # numpy + 可选 matplotlib
python -m unittest discover -s tests     # 无 ROS2 可跑：Ran 184 / OK

# 实车（在实验机器上，需 ROS2 + 动捕）：
python run_experiment.py --config task_two
python tools/postprocess.py runs/task_two_<时间戳> --window 10
```

## 实验室检查单

- [ ] 动捕标定完成，世界系 +x/+y 与 formation 坐标一致
- [ ] 车距 ≥ 0.20 m；初始构型尽量接近期望（preflight 打印逐边米制误差表）
- [ ] 每辆车跑过 `check_wheels` 且方向正确；弱车跑过 `calibrate_car`
- [ ] 解锁后确认各车实际开始移动（console measured 列）
- [ ] 急停：Ctrl-C 下发停车；必要时断电
