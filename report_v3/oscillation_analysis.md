# v3 巡航振荡：归因分析与解决方案（含对比实验操作流程）

> 数据：`report_v3/runs/`（2026-09-04 七轮）+ `report_v3/analysis/`
> （fig1–fig7）。代码改动：`swarm_lab_jiang_v3`（三个可开关模块，
> 默认全部关闭 = 与 9-04 行为逐字节一致）。

## 0. TL;DR

巡航振荡是**两个耦合极限环**叠加，根子在执行端的死区补偿量化
（min_eff=35 高于巡航原始指令 ≈20），反馈延迟（速度估计器 ≈0.12 s）
决定其频率与幅度；动捕网络延迟与编队控制律均被数据排除。
三个可开关模块分别治理：死区量化（消灭）、速度反馈时延（削减）、
航向环微纠偏（死区抑制）。

## 1. 现象与量化

9-04 七轮巡航段实测（`analysis/results/fig6_oscillation.json`、
`fig7_oscillation_attribution.json`）：

- 速度波纹中位 31–77%（围绕 w=0.10 m/s）；
- 朝向摆动中位 1.6–11.2°；
- 轮指令换向 1.3–2.8 次/s/轮，53–85% 的指令被钉死在 ±35。

注意量级换算：1.5 Hz、±0.07 m/s 的极限环折算到位置层仅 ~1 cm
振幅——所以队形指标（稳态 0.022–0.030 m）没被打坏，但视觉上
"抖得厉害"，且电机频繁正反转对硬件不友好。

## 2. 假设检验（逐条判定）

| 假设 | 判定 | 证据 |
| --- | --- | --- |
| H1 网络/动捕延迟 → bearing 相位差 | **对编队环不成立；在执行环内以"速度估计器延迟"的形式成立** | mocap 时钟全程漂移 <2.2 ms、抖动 std 7–8 ms；位姿时效 ~10–20 ms ≈ 2 mm 位置误差，对位置级约束律无影响。但 omni_pid 的速度反馈（差分基线 0.2 s + EMA α=0.3）有效延迟 ≈0.12 s，在 1.5 Hz 处贡献 ~65° 相位滞后 |
| H2 控制律整体一致性牺牲同步 | **否** | 边误差 RMS 频谱只有 0.07–0.14 Hz 慢漂移，无周期峰；v3 约束律是梯度下降结构，无保守振荡模态 |
| H3 min_eff 抬升副作用 | **成立，但是两个极限环而非一个** | 频谱分离：速度波纹与轮指令同峰锁相 1.46–1.5 Hz（推进环）；yaw 摆动在 0.07–0.77 Hz 更低频（航向环） |

## 3. 完整机理

```
巡航目标 0.10 m/s → 原始轮指令 ≈20（< min_eff 35）
  → 抬升到 ±35（≈0.175 m/s 推进）→ 超速
  → PI 持 ~0.12 s 前的 EMA 速度纠偏 → 指令压过零 → −35 刹车
  → 欠冲 → 回正……        【推进 bang-bang 极限环，1.5 Hz，主振荡】

yaw 偏差几度 → 航向保持 P → spin 指令 ~3（< 35）→ 抬升到 ±35
  → 旋转过冲 → 翻转……    【航向保持极限环，0.1–0.8 Hz，次振荡】
```

- min_eff 决定振荡**是否存在**（指令空间被量化出断档）；
- 反馈延迟决定振荡的**频率与幅度**（延迟占极限环相位预算 ~40%）；
- 麦轮低速滚子颤振是公开记载的物理现象（见参考文献），
  在 5 Hz 记录中不可分辨，只可能是高频底噪而非 1.5 Hz 主振荡。

## 4. 解决方案：三个可开关模块（默认全关 = 原行为）

全部在配置层开关，`ExecutionConfig` / `RuntimeConfig` 新增字段，
不破坏现有行为，便于 A/B 对比：

| 模块 | 配置项 | 取值 | 原理 |
| --- | --- | --- | --- |
| 消灭量化 | `execution.deadzone_mode` | `"lift"`（默认，现状）/ `"affine"` / `"pwm"` | affine：死区逆 `out=sign(c)·(min_eff+|c|·(100−min_eff)/100)`，指令→推进恢复连续单调，bang-bang 失去结构基础；pwm：整向量按比例缩放后以 min_eff 脉冲、占空比=请求/阈值，时间平均等于请求且瞬时方向严格保持（顺带治起步畸变） |
| 削减时延 | `runtime.velocity_ema_alpha`（新配置化，原硬编码 0.3）+ 既有 `velocity_diff_baseline_s` / `require_twist` | α 0.3→0.5，baseline 0.2→0.1 s；或 `require_twist=True` 用 VRPN 原生 twist（零估计延迟） | 把 PI 反馈的 ~65° 相位滞后砍到 ~30°，极限环频率上移、幅度下降 |
| 航向环 | `execution.heading_deadband_rad` | 0（默认）→ 0.03–0.05 rad（≈2–3°） | 小角度偏差不纠偏，航向极限环失去触发源 |

代码位置：`swarm/infrastructure/ground_vehicle.py`（`DeadzoneCompensator`）、
`swarm/algorithms/mecanum.py` / `mecanum_pid.py`（航向死区）、
`swarm/domain/config.py` + `application/bootstrap.py`（接线）。
新增 18 项单元测试，全套 136 项全绿。

## 5. 对比实验操作流程（现场）

基线与三个模块的开启都在 `configs/v3_two.py`（WSL 侧副本可直接改）：

```python
execution=ExecutionConfig(
    wheel_command_min_effective=35.0,
    heading_target_rad=0.0,
    deadzone_mode="lift",            # A 组改 "affine"；B 组改 "pwm"
    heading_deadband_rad=0.0,        # C 组改 0.035
),
runtime=RuntimeConfig(
    velocity_ema_alpha=0.3,          # D 组改 0.5；或 require_twist=True
),
```

实验矩阵（同车同场地同起点，每组跑 v3_two，≥30 s）：

| 组 | 配置 | 验证目标 |
| --- | --- | --- |
| 基线 | 全默认 | 复现 9-04 形态（对照组） |
| A | `deadzone_mode="affine"` | 主振荡是否消失 |
| B | `deadzone_mode="pwm"` | 与 A 对比（若 A 的"刚滚动速度"高于巡航速度，B 更优） |
| C | `heading_deadband_rad=0.035` | yaw 摆动是否消失 |
| D | `velocity_ema_alpha=0.5` | 极限环频率上移/幅度下降 |
| A+C+D | 全开 | 终态组合 |

判读指标（`python3 tools/postprocess.py runs/<目录> --window 10` +
console 目测）：

- 轮指令换向率（目标 <0.5 次/s，基线 ~2.4）；
- 速度波纹中位（目标 <15%，基线 ~31–77%）；
- 朝向摆动中位（目标 <2°，基线 1.6–11.2°）；
- **队形指标不得变差**：稳态边误差 ≤ 0.04 m（基线 0.022–0.030）——
  振荡治理不能以收敛质量为代价。

## 6. 参考文献与外部依据

- [AMC：闭环中增益或延迟过大导致自激振荡](https://www.a-m-c.com/closed-loop-control/)
- [Paine et al., 执行延迟下机器人稳定性分析（UT Austin HCRL）](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/ds_138_05_051005.pdf)
- [Bitcraze：通信延迟下四旋翼 NMPC 位置控制](https://www.bitcraze.io/2020/11/an-efficient-real-time-nmpc-for-quadrotor-position-control-under-communication-time-delay/)
- [Chief Delphi：麦轮振动问题讨论（滚子/安装/电机差异）](https://www.chiefdelphi.com/t/problem-with-mecanum-wheels/126198)
- [麦轮全向平台控制综述（固有非线性与动态耦合）](https://stumejournals.com/journals/tm/2025/2/51.full.pdf)
- 电机死区 bang-bang 振荡为运动控制常识（例：[Zbotic：on-off 控制围绕设定点持续振荡](https://zbotic.in/servo-motor-pid-control-with-arduino-smooth-positioning/)）。

## 附录：遗留与边界

- `v3_two_152637` 离群轮：前 10 s 收敛后稳在 0.026 m，t=30 s 后升到
  0.048 m——后段存在一次未定位的漂移/扰动事件，非系统性问题；
- 振荡归因的"平移型 bang-bang"是统计与机理论证，未做"降 min_eff 重跑"
  的直接消融（A 组实验即此消融的软替代）；
- 5 Hz 记录分辨不出 >2.5 Hz 的物理颤振分量，如需确认用动捕原始高帧率
  数据（`v3_*1/` 目录）补一段高频分析。
