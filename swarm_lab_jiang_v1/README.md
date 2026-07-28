# swarm_lab_jiang_v1 —— bearing-only 编队控制实车工程（重构版）

ROS2 动捕 + 麦轮小车 + UDP 下发轮速的 bearing-only 编队控制工程。
本工程是对旧工程（`../jiang`）的重构，针对实车暴露的四类问题逐项修复：

1. **leader 摆放与期望 bearing 矛盾无检查**（旧日志：car1→car2 初始误差 87°，
   follower 可达构型退化成 8 mm 狭缝，永不收敛）
   → 新增 **preflight 起飞前校验**：锚定边 bearing 一致性、最小车距、逐边误差表 +
   人工确认后才解锁电机。
2. **car4 执行响应异常**（前进指令下横向漂移、转向反号）
   → 新增 `tools/check_wheels.py` 单车开环硬件校验工具，把轮序/极性/滚子装反
   这类问题暴露在编队实验之前。
3. **仿真=全向单积分器，实车=差速转向**，且麦轮全向能力被浪费
   → 执行器默认**麦轮全向模式**（世界系速度→车体系→逆运动学四轮分配），与理论
   模型一致；保留差速模式（`execution.mode="diff"`）便于 A/B 对比。
4. **控制律无投影矩阵、长期饱和、无死区**
   → 新控制律采用理论形式 `u_i = k_p · Σ P_ij(g_ij − g*_ij)`，平滑 tanh 饱和 +
   死区；bearing 默认 2D（忽略动捕 z 高度噪声），可切 3D。

## 目录结构

```
swarm_lab_jiang_v1/
├── README.md
├── requirements.txt            # 仅 matplotlib（可选）；ROS2 用 apt 装
├── run_experiment.py           # 入口：--config/--yes/--no-plot/--no-record
├── configs/                    # 四个实验拓扑（矩阵/leader_mask/IP 与旧工程一致）
│   ├── bearing_two.py  bearing_three.py  bearing_four.py  bearing_six.py
├── swarm/
│   ├── domain/                 # models.py（数据结构） config.py（类型化配置）
│   ├── algorithms/
│   │   ├── pid.py              # PID + VectorPIController（差速模式内部使用）
│   │   ├── bearing.py          # 新控制律：P 投影 + 求和 + 平滑饱和 + 死区
│   │   ├── mecanum.py          # 麦轮全向映射 + 航向保持
│   │   ├── differential.py     # 差速模式（旧 vehicle.py 移植）
│   │   └── obstacle_avoidance.py  # APF 人工势场过滤器
│   ├── application/
│   │   ├── control_loop.py     # 50 Hz 主循环，mocap 丢失停车，可选收敛停车
│   │   ├── preflight.py        # 起飞前校验（纯逻辑，与 ROS/IO 解耦）
│   │   ├── planner_factory.py  bootstrap.py  interfaces.py
│   └── infrastructure/
│       ├── ros2_mocap.py       # rclpy 可选导入；后台线程 spin；twist 可选
│       ├── udp_chassis.py      # 持久 socket，报文 <FL,FR,RL,RR> 不变
│       ├── ground_vehicle.py   # 执行器：按 execution.mode 分发 omni/diff
│       └── telemetry/          # console / live_plot / recorder / composite
├── tools/
│   ├── check_wheels.py         # 单车开环硬件校验
│   └── postprocess.py          # 运行目录 → analysis.json + analysis.png
└── tests/                      # 全部不依赖 ROS2/实车
```

## 与旧工程的差异对照

| 项目 | 旧工程 `jiang/` | 本工程 |
| --- | --- | --- |
| 控制律 | 逐边误差求**均值** + 向量 PI，无投影 | `Σ P_ij(g−g*)` 求和，P=I−g·gᵀ，tanh 平滑饱和 + 死区 |
| 运动模式 | leader 钉死、follower 跟随 | 默认 `all_bearing` **全员运动**（无静止锚点），另有 `leader_velocity`（旧行为）与 `leader_bias`（编队机动）可选 |
| bearing 维度 | 3D（含 z 噪声） | 默认 2D，`bearing_3d=True` 可切回 |
| 执行器 | 差速转向单车模型（航向PID+速度PID） | 默认麦轮全向（世界系→车体系→X 型逆运动学），差速可配置 |
| 航向 | 差速转向即航向 | 航向保持 P 控制（解锁时锁定各车初始 yaw） |
| 起飞检查 | 无 | preflight：mocap 有效 / 锚定边夹角 ≤ 20° / 车距 ≥ 0.20 m / 逐边误差表 + 确认 |
| UDP | 每条指令新建 socket | 持久 socket + 发送锁，报文格式不变 `<FL,FR,RL,RR>` |
| mocap | 主线程 `spin_once`，pose+twist 才 valid | 后台 daemon 线程 `spin`；`require_twist=False` 时 pose 即 valid，速度用位置差分 + EMA(α=0.3) 估计 |
| 收敛停车 | 无 | `stop_on_converge`：mean bearing 误差 < 0.05 rad 持续 3 s 自动停车 |
| rclpy | 顶层硬导入 | 可选导入；无 ROS2 的机器可跑全部单元测试 |
| 硬件校验 | 无 | `tools/check_wheels.py` |
| 轨迹图 | 只看实时窗口 | 实验结束自动把最终轨迹存为运行目录下 `trajectory.png`；无显示环境（headless）时自动跳过窗口只存图；绘图进程忽略 Ctrl-C 信号，存图不受退出时序影响 |

## 运动模式（bearing_control.agent_mode）

| 模式 | 行为 | 适用 |
| --- | --- | --- |
| `all_bearing`（默认） | **所有车都跑 bearing 控制律，无静止锚点**。无向边对两端对称作用（理想情况下净控制力为零，质心不动），编队收敛到目标形状，整体平移/旋转/尺度自由 | 纯编队成型实验；摆车无方向要求 |
| `leader_velocity` | leader 输出配置速度（为 0 即钉死），follower 跑 bearing 律 | 需要固定锚点/固定朝向的实验（旧行为） |
| `leader_bias` | leader 跑 bearing 律 **加上**配置速度偏置，拖着整个编队走 | 编队机动（maneuvering） |

注意：`all_bearing` 下编队的旋转和尺度不受控（bearing-only 固有自由度），
实际会有缓慢漂移；质心理想不动，但执行器误差会带来慢漂移。
`leader_velocity` 下 leader 基线把平移/旋转/尺度全部锁死，但基线方向必须
与期望 bearing 一致（preflight 锚定校验）。

## 环境与安装

- Python 3（仅标准库 + 可选 matplotlib）：`pip install -r requirements.txt`
- 实车机器另需 ROS2（apt 安装，不经 pip）：
  `sudo apt install ros-humble-rclpy ros-humble-geometry-msgs ros-humble-vrpn-client-ros`
- 无 ROS2 的电脑：`import` 不报错、单元测试全可跑；只有真正跑实验时才提示缺依赖。

运行测试：

```
cd swarm_lab_jiang_v1
python -m unittest discover -s tests -v
```

## 实验流程

1. **新车上电先做单机校验**（尤其是动过接线/滚子之后）：

   ```
   python tools/check_wheels.py --address 10.1.1.84
   ```

   依次执行：全轮正转/反转 → 左/右平移 → 顺/逆时针旋转 → 逐轮点动。
   每步打印期望运动方向，实际不符即说明轮序/极性/滚子方向有问题，先修硬件。

2. **摆车**：默认 `all_bearing` 模式对摆放方向无要求（编队整体可旋转），
   只需车距 ≥ 0.20 m、大致在场地内；`leader_velocity` 模式才要求 leader
   相对方向与期望 bearing 一致（见文末检查单）。

3. **启动实验**：

   ```
   python run_experiment.py --config bearing_four
   ```

   可选：`--yes` 跳过 preflight 后的回车确认（硬校验跳不过）；
   `--no-plot` 关实时轨迹窗；`--no-record` 关 CSV 记录。

4. **preflight 自动执行**，全部通过并回车后才解锁电机：
   - 等待全部车辆 mocap valid（超时 10 s，打印进度）；
   - **锚定一致性**（仅 `leader_velocity` 模式执行）：所有"两端都是 leader"的边，
     实际 bearing 与期望 bearing 夹角必须 ≤ `anchor_tolerance_deg`（默认 20°），
     否则逐边列出 source/target/期望方向/实际方向/偏差角并拒绝起飞；
     `all_bearing` 模式跳过此项（编队可自由旋转，摆放方向不再是约束）；
   - **最小车距**：任意两车距离 ≥ `min_separation_m`（默认 0.20 m）；
   - 打印各车位置 + 逐边初始 bearing 误差表，回车确认。

5. **运行中**：console 10 Hz 打印逐车指令/逐边误差；Ctrl-C 或
   `stop_on_converge` 触发后自动停车（finally 保证停车指令下发）。
   正常结束时实时轨迹窗口的最终图自动存为运行目录下的 `trajectory.png`
   （无显示环境下不弹窗、只存图）。

6. **后处理**：

   ```
   python tools/postprocess.py runs/bearing_four_20260727_120000_000000
   ```

   生成 `analysis.json`（逐边/全局 bearing 误差统计、轨迹长度、速度 RMSE、
   发送成功率）和 `analysis.png`（轨迹 + 逐边误差 + 速度跟踪）。

## 调参指南

配置集中在 `swarm/domain/config.py`，在 `configs/*.py` 里按需覆盖：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `bearing_control.agent_mode` | `"all_bearing"` | 运动模式：全员运动 / `leader_velocity` 旧行为 / `leader_bias` 编队机动 |
| `bearing_control.kp` | 0.6 | bearing 律增益。收敛慢可加大；抖动/超调先查执行端再降 kp |
| `bearing_control.deadband_mps` | 0.015 | 死区。收敛后微抖可略加大；过大留残差（残差角 ≈ deadband/kp 弧度量级） |
| `bearing_control.bearing_3d` | False | 默认忽略 z；确需 3D 再开 |
| `runtime.command_speed_limit_mps` | 0.25 | 平滑饱和上限（tanh），也是 APF/leader 限速 |
| `execution.mode` | `"omni"` | `"diff"` 切回旧差速模式做 A/B 对比 |
| `execution.heading_hold` | True | 航向保持；关闭则 ω=0（平移时车头会随阻力漂移） |
| `execution.k_omega` / `omega_max` | 2.0 / 1.5 | 航向保持增益与角速度上限（rad/s） |
| `execution.mecanum_l_m` | 0.10 | 轮距参数 lx+ly（m），按实车量 |
| `execution.max_wheel_speed_mps` | 0.3 | 单轮线速度→指令的标定（cmd 100 对应的轮速）；偏大会整体欠驱动，务必按"执行端标定"实测 |
| `execution.wheel_command_min_effective` | 0.0 | 电机死区补偿：非零指令低于该值时抬升到该值（零保持零）。用 `--ramp` 实测后填入；0 关闭 |
| `execution.wheel_flip` | (1,1,1,1) | 某车轮极性接反时的软件兜底（如 (-1,1,1,1) 翻 FL） |
| `preflight.anchor_tolerance_deg` | 20 | 锚定边容差；摆车精度高可收紧到 10° |
| `preflight.min_separation_m` | 0.20 | 最小车距硬校验 |
| `runtime.stop_on_converge` / `converge_eps_rad` | False / 0.05 | 收敛自动停车（mean bearing 角误差，持续 3 s） |
| `runtime.require_twist` | False | True 恢复旧行为：pose+twist 都到才 valid（diff 模式速度 PID 需要实测速度时更准） |

典型顺序：先 `check_wheels` 排除硬件 → 单车 omni 平移/旋转方向正确 →
两车（bearing_two）→ 四车（bearing_four）→ 六车（bearing_six）。

## 执行端标定（解决"跑一段后原地不动"）

症状：小车初期能动，误差变小后原地卡住不动，但 CLI 里指令仍在下发。
原因：误差缩小 → 指令速度缩小 → 低于电机静摩擦死区。若
`max_wheel_speed_mps` 标定偏大（如 0.5 而实际只有 0.3），所有指令再被
额外削弱 ~40%，卡死来得更早。标定两步：

1. **速度标定**（`max_wheel_speed_mps`）：单车放空旷地面，
   `python tools/check_wheels.py --address <IP> --speed 100 --duration 2 --yes`，
   全轮正转时量 2 s 内行驶距离 ÷ 2 = 实测轮速上限，填入配置。
2. **死区测量**（`wheel_command_min_effective`）：
   `python tools/check_wheels.py --address <IP> --ramp --yes`，
   指令从 0 缓慢爬升，记下小车**刚开始滚动**时的 cmd 值，配置时取其略大值
   （如测得 22 就填 25）。各车差异大时按最差的车取，或逐车分配不同配置。
   补偿保持零指令为零，不影响停车与编队死区。

## 实验室检查单（每次开跑前逐项过）

- [ ] 动捕系统标定完成，世界系 +x/+y 方向与拓扑图一致（卷尺+直角尺校准）
- [ ] **leader 摆放**（仅 `leader_velocity` 模式是硬约束；默认 `all_bearing`
      模式跳过，但摆得接近期望朝向收敛更快）：以 `bearing_four` 为例，期望
      `car1→car2` bearing 是世界 **+x**；`bearing_two` 期望 car2 在 car1 的
      **+y**；`bearing_three` 期望 car5 在 car3 的 +x、car6 在 car3 的 +y。
      **摆错方向是上次 87° 事故的根因**——默认模式下 preflight 不再因此拒绝，
      但 leader_velocity 模式下会直接拒绝起飞。
- [ ] 车距均 ≥ 0.20 m；follower 初始位置与期望构型大致相称（bearing-only
      不定尺度，但初始别压到 leader 基线内侧）
- [ ] 每辆车跑过 `check_wheels` 且方向全部正确（尤其换过电池/动过接线后）
- [ ] 车轮滚子无缠发/异物，地面无电线
- [ ] 各车 IP 与 `configs/*.py` 一致（10.1.1.81–86），`ping` 全通
- [ ] 动捕 marker 牢固、朝向与车头一致（航向保持以此为准）
- [ ] `/car*/pose` 话题均有数据；首跑先 `--no-plot` 看 console 输出是否正常
- [ ] 急停预案：Ctrl-C 会下发停车；必要时直接断电

## 备注

- UDP 报文格式 `<FL,FR,RL,RR>`（int(round())）与固件保持一致，未改。
- CSV 列与旧版一致，`tools/postprocess.py` 与旧分析习惯兼容。
- `tests/test_convergence.py` 用真实 `configs/bearing_four` + 真实控制律 +
  理想单积分器桩做端到端收敛回归（`all_bearing` 与 `leader_velocity` 两种模式，
  RMS < 5° / 30 s，且全员模式下断言每辆车都实际运动），改控制律后必须保持全绿。
