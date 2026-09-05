# swarm_lab_jiang_v2 —— captain / first mate / crew 角色化 bearing 编队机动工程

ROS2 动捕 + 麦轮小车 + UDP 下发轮速的 bearing 编队**机动**实车工程。
本工程是 `../swarm_lab_jiang_v1` 的升级版：v1 所有智能体地位平等（全员只受
bearing 约束）或仅有二元 leader/follower；v2 引入三级角色，实现
Zhao & Zelazo《Bearing-Based Formation Maneuvering》(2015) 的编队机动结构：

- **captain**（恰好 1 个）：不跑 bearing 律，只输出自己固定的世界系速度 v_c
  （可为 0；预留分段常值变速接口）。captain 承载编队的平移参考。
- **first_mate**（0 个或多个）：固定速度 v_fm 与 bearing 控制项 u 的加权合成
  （`weighted`，默认）或相加合成（`additive`），合成后统一限速。
  v_fm 默认等于 captain 速度——论文 Proposition 1：所有 leader 共享 v_c 时
  编队纯平移且尺度不变。first_mate 是"第二个 leader"，负责钉死编队尺度。
- **crew**（0 个或多个）：纯 bearing 律（同 v1 follower 行为）。

理论依据（实现与调参前必读）：

1. 目标编队 infinitesimally bearing rigid 时，bearing 约束只能把构型确定到
   平移 + 尺度两个自由度，**至少两个带速度参考的 leader** 才能同时钉死——
   这就是 captain + first_mate 的角色分工（论文 Assumption 1）。
2. **leader 速度非零时纯 P 律必有恒定跟踪误差**，积分项才能消除（论文核心结论）。
   v2 给 crew 与 first_mate 的 bearing 项加了可选 PID 接口：`ki=0`（默认）退化为
   v1 纯 P 行为，`ki>0` 消除机动稳态误差。单元测试里有这条命题的端到端回归
   （`tests/test_convergence.py`：captain 匀速 0.05 m/s，纯 P 稳态 RMS ≈ 5.0°，
   PI ≈ 3.0°，且 PI 受死区下限约束）。
3. 期望 bearing 是世界系固定方向（同 v1）：形状锁定在世界系，平移/尺度由
   captain/first_mate 管理。

## 与 v1 的差异对照

| 项目 | v1 | v2 |
| --- | --- | --- |
| 角色模型 | 二元 `leader_mask` + `agent_mode` 字符串（all_bearing / leader_velocity / leader_bias），`VehicleConfig.role` 冗余 | 三级角色 `VehicleConfig.role`（captain / first_mate / crew）为**唯一真相源**，agent_mode 取消 |
| leader 速度 | `leader_target_velocities` 映射 | `captain_velocity` 常量 + `captain_velocity_schedule` 分段常值表（变速接口） |
| first_mate 合成 | 无（leader_bias 为写死相加） | `FirstMateParams`：α ∈ [0,1] 加权（默认 0.5）或 additive，合成后统一限速；v_fm 缺省 = captain 速度 |
| bearing 律 | 纯 P（kp） | 可选 PID：kp / ki / kd / integral_limit（ki=kd=0 退化为 v1） |
| preflight 锚定 | leader_velocity 模式下 leader-leader 边夹角硬校验 | 单一 captain 无"双钉死基线"机理，改为 **captain 相关边初始误差软警告**（默认 45°，不拒飞）；最小车距仍硬校验 |
| 执行端标定 | 全编队统一 | 支持**按车覆盖**（`VehicleExecutionOverride`：max_wheel_speed_mps / wheel_command_min_effective / wheel_flip），偿还 v1 car1/car2 偏弱欠账 |
| bearing 矩阵 | 无一致性校验 | 校验对边互逆（bearing[j][i] ≈ −bearing[i][j]，1° 容差） |
| 后处理 | 全程统计 | 新增**末 N 秒稳态统计**（mean/RMS/std，默认 N=5，`--window` 可调）与初始 RMS；稳态一律报末 N 秒窗口，禁用全程 mean（v1 报告教训） |
| 配置 | bearing_two/three/four/six + chassis_step | crew_two/three/four/six（拓扑矩阵逐字沿用 v1）；chassis_step 不入 v2（底盘选型已完成），constant_velocity 算法保留作标定备用 |
| CSV | `is_leader` 列 | `role` 列 |

## 目录结构

```
swarm_lab_jiang_v2/
├── README.md
├── requirements.txt            # 仅 matplotlib（可选）；ROS2 用 apt 装
├── run_experiment.py           # 入口：--config/--yes/--no-plot/--no-record
├── configs/                    # 四个机动拓扑（矩阵沿用 v1，角色 1+1+N）
│   ├── crew_two.py             # captain + first_mate（论文最小编队：2 leaders）
│   ├── crew_three.py           # captain + first_mate + crew
│   ├── crew_four.py            # captain + first_mate + 2 crew
│   └── crew_six.py             # captain + first_mate + 4 crew
├── swarm/
│   ├── domain/                 # models.py（数据结构） config.py（类型化配置 + 角色校验）
│   ├── algorithms/
│   │   ├── pid.py              # PID + VectorPIController（差速模式内部使用）
│   │   ├── bearing.py          # 角色分派控制律：P 投影 + 可选 PID + 合成 + 平滑饱和 + 死区
│   │   ├── mecanum.py          # 麦轮全向映射 + 航向保持（平移开环）
│   │   ├── mecanum_pid.py      # 麦轮全向 + 车体速度前馈/PI 闭环（omni_pid）
│   │   ├── differential.py     # 差速模式（旧 vehicle.py 移植）
│   │   ├── constant_velocity.py # 恒速阶跃 planner（底盘标定备用，无目录配置）
│   │   └── obstacle_avoidance.py  # APF 人工势场过滤器
│   ├── application/
│   │   ├── control_loop.py     # 50 Hz 主循环，mocap 丢失停车，可选收敛停车
│   │   ├── preflight.py        # 起飞前校验（软警告版，纯逻辑，与 ROS/IO 解耦）
│   │   ├── planner_factory.py  bootstrap.py  interfaces.py
│   └── infrastructure/
│       ├── ros2_mocap.py       # rclpy 可选导入；后台线程 spin；twist 可选
│       ├── udp_chassis.py      # 持久 socket，报文 <FL,FR,RL,RR> 不变
│       ├── ground_vehicle.py   # 执行器：按 execution.mode 分发 omni/omni_pid/diff
│       └── telemetry/          # console / live_plot / recorder / composite
├── tools/
│   ├── check_wheels.py         # 单车开环硬件校验
│   └── postprocess.py          # 运行目录 → analysis.json（含稳态窗口）+ analysis.png
└── tests/                      # 全部不依赖 ROS2/实车（139 项）
```

## 角色与运动行为

| 角色 | 数量 | 速度指令 | bearing 误差 |
| --- | --- | --- | --- |
| captain | 恰好 1 | 限速后的 v_c（固定或 schedule；0 即钉死） | 照常计算（遥测用），不影响指令 |
| first_mate | ≥0 | weighted：`α·v_fm + (1−α)·u`；additive：`v_fm + u`；合成后统一限速 | 参与 u 的计算 |
| crew | ≥0 | 纯 u（投影 bearing 律 + 可选 PID + tanh 饱和 + 死区） | 参与 u 的计算 |

注意两种合成的稳态差异：weighted 下 first_mate 的 bearing 项需恒输出 v_c
才能跟上编队平移，其相关边会稳定在非零误差上（量级 ≈ asin(|v_c|/((1−α)·kp))，
可由积分项消除）；additive 下稳态 u→0，first_mate 恰好以 v_c 运动。
两种模式都做成配置就是为了让这个对比成为可重复的实验。

## 环境与安装

- Python 3（仅标准库 + 可选 matplotlib）：`pip install -r requirements.txt`
- 实车机器另需 ROS2（apt 安装，不经 pip）：
  `sudo apt install ros-humble-rclpy ros-humble-geometry-msgs ros-humble-vrpn-client-ros`
- 无 ROS2 的电脑：`import` 不报错、单元测试全可跑；只有真正跑实验时才提示缺依赖。

运行测试：

```
cd swarm_lab_jiang_v2
python -m unittest discover -s tests -v
```

## 实验流程

1. **新车上电先做单机校验**（尤其是动过接线/滚子之后）：

   ```
   python tools/check_wheels.py --address 10.1.1.84
   ```

2. **摆车**：车距 ≥ 0.20 m。captain 相关边的相对方位尽量接近期望 bearing
   （如 crew_four 期望 car1→car2 为世界 +x）——摆偏了不再拒飞（软警告），
   但纠偏全靠 first_mate/crew，收敛慢。captain 速度方向要留出行驶空间。

3. **启动实验**：

   ```
   python run_experiment.py --config crew_four
   ```

   可选：`--yes` 跳过 preflight 后的回车确认（硬校验跳不过）；
   `--no-plot` 关实时轨迹窗；`--no-record` 关 CSV 记录。

4. **preflight 自动执行**：等待全部车辆 mocap valid → **captain 相关边初始
   误差软警告**（超 `captain_edge_warn_deg` 打印 WARN，不拒飞）→ **最小车距
   硬校验** → 逐边初始 bearing 误差表 → 回车解锁电机。

5. **运行中**：console 10 Hz 打印逐车指令/逐边误差；Ctrl-C 或
   `stop_on_converge` 触发后自动停车（finally 保证停车指令下发）。

6. **后处理**：

   ```
   python tools/postprocess.py runs/crew_four_20260727_120000_000000 [--window 5]
   ```

   生成 `analysis.json`（全程统计 + **`steady_state` 节：末 N 秒整体/逐边
   bearing 误差 mean/RMS/std、逐车速度跟踪 RMSE 稳态版** + `initial_rms_deg`
   初始 RMS）和 `analysis.png`。**报告稳态一律用 `steady_state` 节，禁用全程
   mean**（v1 报告 §5：短运行全程 mean 被瞬态污染，曾产生 15.6° 的伪影指标）。

## 调参指南

配置集中在 `swarm/domain/config.py`，在 `configs/*.py` 里按需覆盖：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `captain_velocity` | 配置内 (0.10, 0, 0) | captain 固定世界系速度；(0,0,0) 退化为静态编队。≤ command_speed_limit |
| `captain_velocity_schedule` | () | 分段常值变速表 `[(t秒, (vx,vy,vz)), ...]`，空=全程恒定；t 非递减 |
| `first_mate_params[车].velocity` | None→captain 速度 | first_mate 固定速度 v_fm；共享 v_c 时编队纯平移、尺度不变（论文 Prop. 1） |
| `first_mate_params[车].blend_weight` | 0.5 | α ∈ [0,1]：1=纯固定速度，0=纯 bearing 项 |
| `first_mate_params[车].blend_mode` | "weighted" | "weighted"（α 加权）/ "additive"（相加） |
| `bearing_control.kp` | 0.6 | bearing 律比例增益 |
| `bearing_control.ki` | 0.0 | 积分增益。**captain 速度非零时纯 P 有恒定跟踪误差，机动实验建议 ki>0**（0.2–0.3 起步）；0 退化为 v1 纯 P |
| `bearing_control.kd` | 0.0 | 微分增益（raw 差分）；动捕噪声下保持 0，需要阻尼再试小值 |
| `bearing_control.integral_limit` | 0.5 | 积分向量模长限幅（抗饱和） |
| `bearing_control.deadband_mps` | 0.015 | 死区。残差角 ≈ deadband/kp 弧度量级 |
| `runtime.command_speed_limit_mps` | 0.25 | 平滑饱和上限，也是 captain/first_mate 合成限速 |
| `execution.mode` | "omni_pid" | omni_pid 麦轮+速度 PI 闭环（默认，8/3 诊断后切换）/ omni 麦轮开环 / diff 差速 |
| `execution.heading_target_rad` | None | 航向目标（世界系 rad）：None=解锁时捕获当前朝向；设为机动方向（如 +x 即 0.0）后各车解锁自转对准，摆放朝向随意（麦轮斜行效率远低于直行，8/14 斜行段达成率 47–73%） |
| `VehicleConfig.execution_override` | None | **按车覆盖** max_wheel_speed_mps / wheel_command_min_effective / wheel_flip（弱车单独标定，v1 car1/car2 教训） |
| `preflight.captain_edge_warn_deg` | 45 | captain 相关边初始误差软警告阈值 |
| `preflight.min_separation_m` | 0.20 | 最小车距硬校验 |
| `runtime.stop_on_converge` / `converge_eps_rad` | False / 0.05 | 收敛自动停车（mean bearing 角误差，持续 3 s） |

典型顺序：先 `check_wheels` 排除硬件 → 单车 omni 平移/旋转方向正确 →
crew_two（captain+first_mate 最小机动对）→ crew_three → crew_four → crew_six。
每个尺度先静态（captain_velocity=0）后机动（定速），再开 PI 对比稳态误差。

**实验协议标准化**（v1 报告附录 B 教训，强制执行）：

- 同一批实验统一运行时长（建议 ≥30 s，机动实验 ≥60 s 让积分项充分建立）；
- 报告稳态一律用末 N 秒窗口（postprocess `steady_state` 节）；
- 初始 RMS 已自动记入 `analysis.json` 的 `initial_rms_deg`，报告时引用它
  解释收敛过程，不要用手估。

## 实验室检查单（每次开跑前逐项过）

- [ ] 动捕系统标定完成，世界系 +x/+y 方向与拓扑图一致（卷尺+直角尺校准）
- [ ] captain 相关边相对方位尽量接近期望 bearing（软警告不拒飞，但摆得准收敛快）
- [ ] captain 速度方向前方无遮挡（机动实验编队整体平移）
- [ ] 车距均 ≥ 0.20 m
- [ ] 每辆车跑过 `check_wheels` 且方向全部正确（尤其换过电池/动过接线后）
- [ ] 弱车已用 `execution_override` 单独标定（参考 v1：car1/car2 偏弱）
- [ ] 车轮滚子无缠发/异物，地面无电线；电池电量充足（死区阈值随电量漂移）
- [ ] 各车 IP 与 `configs/*.py` 一致（10.1.1.81–86），`ping` 全通
- [ ] 动捕 marker 牢固、朝向与车头一致（航向保持以此为准）
- [ ] `/car*/pose` 话题均有数据；首跑先 `--no-plot` 看 console 输出是否正常
- [ ] 急停预案：Ctrl-C 会下发停车；必要时直接断电

## 备注

- UDP 报文格式 `<FL,FR,RL,RR>`（int(round())）与固件保持一致，未改。
- CSV 列 `is_leader` 更名 `role`（captain / first_mate / crew），其余与 v1 一致。
- `tests/test_convergence.py` 用真实 `configs/crew_four` + 真实控制律 +
  理想单积分器桩做端到端回归：静态收敛（RMS < 5° / 30 s）与机动纯 P vs PI
  稳态误差对比（论文命题）。改控制律后必须保持全绿。
- 执行端标定方法（速度标定 / 死区 ramp 测量）与 v1 完全相同，见 v1 README
  "执行端标定"一节；v2 的区别只是标定结果可以按车填写。
