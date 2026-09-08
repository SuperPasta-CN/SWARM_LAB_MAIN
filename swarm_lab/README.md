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

## 底层运动控制逻辑（关键代码）

planner 输出的世界系速度 `VelocityCommand(vx, vy)` 到轮子的完整链路，
按执行顺序（每车每拍 50 Hz 走一遍）：

```
VelocityCommand(世界系 m/s)
  │  GroundVehicleActuator.execute            infrastructure/ground_vehicle.py:155
  ├─ 1) 航向保持 → 角速度 omega               algorithms/mecanum_pid.py:56 (_heading_omega)
  ├─ 2) 世界系→车体系旋转 + 速度前馈/PI 闭环   algorithms/mecanum_pid.py:74 (step)
  ├─ 3) 麦轮逆运动学 → 四轮线速度             algorithms/mecanum_pid.py:110
  ├─ 4) 等比缩放（保方向）+ 轮指令限幅        algorithms/mecanum_pid.py:124
  ├─ 5) 死区补偿（默认 PWM 占空比）           infrastructure/ground_vehicle.py:43 (DeadzoneCompensator)
  └─ 6) UDP 下发 "<FL,FR,RL,RR>"              infrastructure/udp_chassis.py:43
```

各环节要点：

- **执行模式三态**（`execution.mode`，分发在 `bootstrap.py` 装配）：
  `omni_pid`（默认，麦轮+速度闭环）/ `omni`（麦轮开环，
  `algorithms/mecanum.py:72`）/ `diff`（差速双 PID，
  `algorithms/differential.py`，对照用）。
- **航向保持**（`mecanum_pid.py:56-72`）：`heading_target_rad` 设定时
  各车解锁后原地转向该世界系航向（对准机动方向，避免麦轮 60° 斜行
  只剩 47–73% 效率的老问题）；未设定则捕获解锁瞬间的 yaw。
  `heading_deadband_rad`（默认 0.03 rad ≈ 1.7°）以内不纠偏——防止
  微小航向误差经死区抬升变成原地抖动极限环。
- **速度闭环**（`mecanum_pid.py:88-103`）：前馈 = 世界系目标速度旋转到
  车体系；反馈 = mocap 实测速度（位置差分+EMA）同旋转；车体系 PI
  校正（`execution.omni_velocity_pid`，kp=0.6/ki=1.2/kd=0——kd 必须
  为 0，反馈是差分噪声）。**目标为零时输出精确零**（92-97 行），
  绕过死区抬升，保证能真正停车。
- **麦轮逆运动学**（`mecanum_pid.py:110-117`）：X 型布置，
  `FL = vx−vy−ωL`，`FR = vx+vy+ωL`，`RL = vx+vy−ωL`，`RR = vx−vy+ωL`
  （L = `mecanum_l_m`），再乘 `wheel_flip` 逐车方向修正。
- **等比缩放 + 限幅**（`mecanum_pid.py:124-137`）：最快轮超
  `max_wheel_speed_mps` 时四轮**等比**缩放（运动方向不变），再映射到
  轮指令域 ±100 并硬限幅。
- **死区补偿**（`ground_vehicle.py:43-133`，`execution.deadzone_mode`）：
  电机死区（静摩擦）是低速振荡的主因。默认 `pwm`：指令低于 min_eff
  时不抬升，而是按占空比整周期脉冲输出——ON 拍四轮**整向量等比**
  放大到 min_eff（瞬时方向保持），OFF 拍全零，时间平均等于请求值；
  设了逐车 `pwm_v_on_mps` 后占空比按实测车速归一化
  （`duty = (指令×calib/100)/v_on`），消除车辆间增益差异。
  `lift`（固定抬升，旧行为）与 `affine`（死区逆，实车证伪）留作对照。
- **UDP 下发**（`udp_chassis.py`）：持久 socket，线格式
  `"<FL,FR,RL,RR>"`（与车队固件约定，历代未变）。

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
