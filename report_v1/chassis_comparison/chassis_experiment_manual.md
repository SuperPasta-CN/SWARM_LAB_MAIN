# 底盘运动学对比实验手册（chassis step 三方对比）

> 实验日期：2026-07-31　　实验人：＿＿＿＿　　场地：动捕实验室
> 状态：**已完成**（选型结论：omni_pid，见第八节）
> 被测对象：执行层"期望速度 → 轮指令"的三种底盘运动学控制逻辑
> 工程：`swarm_lab_jiang_v1`（WSL 副本 `~/swarm_lab_jiang_v1`）

---

## 一、实验目的

1. **主目的**：在相同输入、相同车辆、相同场地条件下，测量三种底盘运动学
   控制逻辑的**稳态速度跟踪残差**，为执行层选型提供定量依据：
   - **A 组 `diff`**：差速转向 + 速度/航向双 PID 闭环（旧方案移植）；
   - **B 组 `omni`**：麦轮全向 + 平移开环 + 航向保持 P（现行默认）；
   - **C 组 `omni_pid`**：麦轮全向 + 平移前馈&PI 闭环 + 航向保持 P（本次新增）。
2. **次目的**：若 C 组胜出，记录其默认 PI 参数下的表现，为后续
   `omni_velocity_pid` 增益整定积累初值。
3. **参考指标**：编队层 bearing 稳态残差（本实验不涉及编队，仅了解；
   选型后以 `bearing_two` 编队实验复核）。

**选型问题**：开环（B）结构最简单但存在摩擦/标定造成的系统性残差；
闭环（A/C）理论上能消除稳态残差，但引入反馈噪声与调参成本。
本实验用数据回答：残差到底多大？闭环能否、以及在多大代价下消除它？

## 二、实验原理

### 2.1 被测环节在控制链路中的位置

```
动捕 (x,y,yaw) ──> 恒速 planner（已知输入 v*）──> 【被测：执行控制器】──> min_eff 补偿 ──> UDP <FL,FR,RL,RR>
        │                                              │
        └──────── 反馈（仅 A、C 使用）：速度估计 ←──────┘
```

编队控制律（bearing）在本实验中**不使用**——它会随时改变 v*，
使"输入"本身未知，无法计算跟踪残差。改用恒速阶跃 planner
（`swarm/algorithms/constant_velocity.py`），输出已知的恒定世界系速度。

### 2.2 三种模式的控制结构

| 组 | 结构 | 反馈 |
| --- | --- | --- |
| B `omni` | v* → R(−yaw) → X 型逆运动学 → 轮指令 | 无（开环）；ω 通道为航向保持 P |
| C `omni_pid` | u_b = v*_b + PI(v*_b − v_实测b)，再进同一逆运动学 | 车体双轴速度 PI（kd=0） |
| A `diff` | v* → (|v*|, atan2) → 速度 PID + 航向 PID + 误差门控 → 左右轮分配 | 速度幅值 + 航向双 PID |

三组共用：轮速→指令标定（`max_wheel_speed_mps`=0.3）、min_eff 死区补偿
（25）、UDP 报文、50 Hz 控制周期。**反馈同源**：A、C 均使用位置差分
+EMA(α=0.3) 的速度估计（`require_twist=False`），传感器差异不混入对比。

### 2.3 观测量定义

- **稳态速度残差**：稳态窗口内 mean(|v_实际|) − mean(|v_目标|)，
  同时给出相对百分比；负值 = 欠驱动（实际偏慢），正值 = 过驱动。
- **方向误差**：窗口内平均实际速度向量与平均目标速度向量的夹角（deg）。
- **跟踪 RMSE**：‖v*(t) − v_实际(t)‖ 的均方根（含瞬态与抖动信息）。
- **实际速度口径**：由记录位置（5 Hz）做 0.4 s 中心差分重建；
  **不使用** CSV 的 measured_vx/vy 列（动捕差分噪声，含尖峰）。
- **稳态窗口**：自动取"目标速度 ≥ 峰值 50%"样本的后 60%（排除起动
  瞬态与停车尾段）；必要时手动 `--t-start/--t-end`。

### 2.4 公平性与局限性说明

- **交集与专项**：diff（A）为非完整约束底盘，横向指令物理上不可执行。
  故**纵向（车头方向）阶跃**为三方交集对比；**横向阶跃**仅 B vs C，
  用于暴露麦轮横移效率低/滚子打滑对开环的影响。
- diff 组摆放要求车头朝向指令方向（航向误差门控会因偏航压低车速，
  见附录 B）。

## 三、实验清单

### 3.1 硬件

- [ ] 麦轮小车 car1（10.1.1.81），电量充足（三组实验连贯做完，避免
      电量差异影响摩擦/电机响应；中途换电需在记录表注明）
- [ ] 动捕 marker 牢固、朝向与车头一致
- [ ] 空旷场地：指令方向 ≥ 1.5 m 无障碍（0.15 m/s × 6 s ≈ 0.9 m 行程
      + 余量）；速度扫描组（至 0.25 m/s）需 ≥ 2.5 m
- [ ] 地面胶带标记统一起点（每组同一位置、同一朝向发车）
- [ ] 卷尺（标定复核备用）、急停预案（Ctrl-C / 断电）

### 3.2 软件与环境（前一晚可完成）

- [ ] Windows 端代码为最新（含 `mecanum_pid.py`、`chassis_step.py`）
- [ ] 同步到 WSL：

  ```bash
  wsl.exe -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/ros2/bearing4cars/swarm_lab-main && \
    tar --exclude='__pycache__' --exclude='swarm_lab_jiang_v1/runs' -cf - swarm_lab_jiang_v1 | tar -xf - -C ~/"
  ```

- [ ] WSL 内基线测试全绿（应 Ran 113 / OK）：

  ```bash
  wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/swarm_lab_jiang_v1 && \
    python3 -m unittest discover -s tests 2>&1 | grep -E '^(Ran|OK|FAILED)'"
  ```

- [ ] 当天动过接线/滚子/换电池 → 先 `check_wheels`：

  ```bash
  wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/swarm_lab_jiang_v1 && \
    python3 tools/check_wheels.py --address 10.1.1.81 --yes --duration 1.0"
  ```

### 3.3 动捕链路

- [ ] 按实验室既有流程启动动捕与 VRPN（`ros_setup.sh` / `vrpn_ws`）
- [ ] 确认数据到达且方向正确：

  ```bash
  wsl.exe -d Ubuntu-22.04 -- bash -lc 'ros2 topic echo /car1/pose --once'
  wsl.exe -d Ubuntu-22.04 -- bash -lc 'ros2 topic hz /car1/pose'
  ```

  手推小车沿世界 +x 移动，确认 x 值增大；世界系方向与场地标记一致。

## 四、实验步骤与命令

> 以下命令均在 **WSL 内**执行（`wsl.exe -d Ubuntu-22.04 -- bash -lc '...'`，
> 或直接开在 WSL 终端）。**模式与速度的切换直接改 WSL 副本**
> `~/swarm_lab_jiang_v1/configs/chassis_step.py`，不用回 Windows 重同步；
> 定型后再把最终参数回写 Windows 端。

每次运行：preflight 自动等 mocap（回车确认解锁，或加 `--yes`），
阶跃 6 s 后自动停车退出，产出 `~/swarm_lab_jiang_v1/runs/chassis_step_<时间戳>/`。

### 第 1 组：纵向阶跃，三方对比（v* = (0.15, 0) m/s，6 s）

1. **B 组（omni）**：确认 `chassis_step.py` 中 `mode="omni"`、
   `vx_mps=0.15, vy_mps=0.0`；车放起点，朝向任意（全向）：

   ```bash
   cd ~/swarm_lab_jiang_v1 && python3 run_experiment.py --config chassis_step
   ```

2. **C 组（omni_pid）**：`mode="omni_pid"`，同一车同一起点，重跑同一命令。
3. **A 组（diff）**：`mode="diff"`，**车头朝世界 +x**（指令方向），重跑。

### 第 2 组：横向阶跃，B vs C 专项（v* = (0, 0.15) m/s，6 s）

4. `vx_mps=0.0, vy_mps=0.15`；`mode="omni"` 跑一次、`mode="omni_pid"`
   跑一次。（diff 不跑本组，物理上执行不了。）

### 第 3 组（可选）：速度扫描

5. 纵向，v* = 0.05 / 0.10 / 0.20 m/s 各跑一次 B 与 C（低速段暴露摩擦
   死区，高速段看标定斜率误差；勿低于 0.015 m/s 死区，勿超 0.25 限速）。

### 数据回收（每组跑完即可做）

```bash
# 方式一（推荐）：WSL 内直接对 WSL 的 runs 跑后处理（脚本为纯标准库）
python3 /mnt/d/ros2/bearing4cars/swarm_lab-main/report_v1/chassis_comparison/postprocess_chassis.py \
    ~/swarm_lab_jiang_v1/runs/ --json ~/chassis_compare.json

# 方式二：把 runs 拷回 Windows 再处理
wsl.exe -d Ubuntu-22.04 -- bash -lc 'cp -r ~/swarm_lab_jiang_v1/runs \
  /mnt/d/ros2/bearing4cars/swarm_lab-main/swarm_lab_jiang_v1/'
```

## 五、实验要求

1. **控制变量**：同一辆车、同一起点、同一朝向（diff 组除外，见 2.4）、
   同一阶跃参数；只改变 `execution.mode`（组间）或阶跃速度（扫描组）。
2. **每跑必记**：运行目录名、模式、设定速度、异常情况（打滑、卡顿、
   急停）当场填入第六节记录表；异常运行的数据**保留但标注**，不删除。
3. **原始记录只读**：不手工编辑任何 `runs/` 下的 CSV/JSON；
   分析一律用后处理脚本。
4. **安全**：解锁前确认场地无人无障碍；运行中手不离 Ctrl-C（会下发
   停车指令）；异常直接断电。
5. 若某次运行明显失败（如动捕中断），重跑并在记录表注明作废原因。

## 六、数据记录（实测，2026-07-31）

> 实验期间发生两次中途变更，按时间分三轮记录：
> **变更①**（13:42 后）：轮速标定 `max_wheel_speed_mps` 0.3 → 0.5
> （由第 1 轮 B 组开环 +71% 残差反推，cmd 100 实际约 0.51 m/s）；
> **变更②**（14:10 后）：速度估计器加差分基线 `velocity_diff_baseline_s=0.2 s`
> （修复估计偏低 23% 导致的 PI 盲飞，见附录 B 新增条目）。
> 运行目录均已归档至 Windows 端 `swarm_lab_jiang_v1/runs/`；
> 后处理 JSON：`report_v1/chassis_comparison/results_chassis_step_*.json`。

### 第 1 组：纵向阶跃，三方对比（v* = (0.15, 0) m/s，6 s）

| 轮次 | mode | 运行目录 | 备注 |
| --- | --- | --- | --- |
| 1（标定 0.3） | omni | chassis_step_20260731_133708_omni | 残差 +71.3%，暴露标定错误 |
| 1（标定 0.3） | omni_pid | chassis_step_20260731_133746_omni_pid | +19.3%，PI 顶着输出限压偏差 |
| 1（标定 0.3） | diff | chassis_step_20260731_134235_diff | −15.6% |
| 2（标定 0.5） | omni | chassis_step_20260731_140756_omni_invalid_stuck | **无效**：全程仅位移 0.096 m（卡阻），保留标注 |
| 2（标定 0.5） | omni | chassis_step_20260731_140933_omni | +10.8%，标定修正确认 |
| 2（标定 0.5） | omni_pid | chassis_step_20260731_141017_omni_pid | +33.0%，暴露估计器偏差 |
| 2（标定 0.5） | diff | chassis_step_20260731_140702_diff | −27.2% |
| 3（+基线 0.2 s，×3） | diff | ..._143941_084563_diff1 / ..._144016_734887_diff2 / ..._144053_075897_diff3 | −22.9 / −25.7 / −24.9% |
| 3（+基线 0.2 s，×3） | omni | ..._144155_798760_omni1 / ..._144247_743096_omni2 / ..._144318_974340_omni3 | +20.0 / +15.4 / +16.1% |
| 3（+基线 0.2 s，×3） | omni_pid | ..._144400_829963_omni_pid1 / ..._144435_951119_omni_pid2 / ..._144508_070548_omni_pid3 | −1.1 / −2.0 / −1.2% |

### 第 2 组：斜向阶跃，B vs C（**实际执行为 (0.15, 0.15)**，|v*|=0.212 m/s，非计划的纯横向 (0, 0.15)）

| mode | 运行目录 | 备注 |
| --- | --- | --- |
| omni | ..._155725_839330_omni1 / ..._155813_259511_omni3 | 速率准（−5.2/+3.9%）但方向偏 −37.4/−32.7° |
| omni | ..._155747_041324_omni2 | **无效**：x 向仅位移 0.232 m（卡阻），剔除不平均 |
| omni_pid | ..._155911_598197_omni_pid1 / ..._160004_405842_omni_pid2 / ..._160059_878397_omni_pid3 | −15.7/−14.7/−13.5%，方向 −11.2/−18.2/−7.8° |

### 第 3 组：速度扫描（0.05 / 0.10 / 0.20 m/s 纵向，**较计划增加了 diff 组**，单次）

| mode | 运行目录 |
| --- | --- |
| diff | ..._160629_556056_diff0.05 / ..._160707_766685_diff0.10 / ..._160801_400805_diff0.20 |
| omni | ..._160843_075997_omni0.05 / ..._160910_635165_omni0.10 / ..._161007_196051_omni0.20 |
| omni_pid | ..._161052_339503_omni_pid0.05 / ..._161134_884815_omni_pid0.10 / ..._161202_089384_omni_pid0.20 |

### 后处理结果摘抄

**第 1 组第 3 轮（三次平均 ± 标准差）**：

| mode | 稳态残差 (m/s) | 相对 (%) | 方向误差 (deg) | RMSE (m/s) |
| --- | --- | --- | --- | --- |
| omni | +0.0257 ± 0.0037 | +17.2 ± 2.5 | −4.53 ± 1.49 | 0.0332 |
| omni_pid | −0.0021 ± 0.0007 | −1.4 ± 0.5 | +1.06 ± 0.93 | 0.0282 |
| diff | −0.0368 ± 0.0022 | −24.5 ± 1.5 | −2.94 ± 2.35 | 0.0511 |

**第 2 组斜向（omni 剔除无效次，n=2；omni_pid n=3 平均）**：

| mode | 速率残差 (m/s) | 相对 (%) | 方向误差 (deg) | RMSE (m/s) |
| --- | --- | --- | --- | --- |
| omni | −0.0014 | −0.7 | −35.1 | 0.131 |
| omni_pid | −0.0310 ± 0.0023 | −14.6 ± 1.1 | −12.4 ± 5.4 | 0.0581 |

**第 3 组速度扫描（单次，相对残差 %）**：

| mode | 0.05 m/s | 0.10 m/s | 0.20 m/s |
| --- | --- | --- | --- |
| omni | −81.7 | −99.7 | +7.3（方向 −18.0°） |
| omni_pid | −86.0 | −29.3 | −1.8（方向 −2.0°） |
| diff | −15.3 | −71.0 | −27.5（方向 −2.7°） |

## 七、数据分析

1. 运行第四节的后处理命令，得逐运行指标与 `compare | car1` 对比表；
   若自动稳态窗口不合理（如落入瞬态），加 `--t-start/--t-end` 手动指定。
2. **判读指南**：
   - B（开环）：预期残差为负（欠驱动，摩擦+标定误差），横向组更明显；
   - C（PI 闭环）：稳态残差应显著小于 B、趋近 0；若 RMSE 明显偏大或
     轮指令抖动，说明默认增益对 EMA 反馈偏激进，属整定问题而非结构问题；
   - A（diff，纵向）：旧双 PID 基准，作为 C 的对照；
   - 横向组重点看 B vs C 的**幅值残差**差多少——横移效率低是麦轮固有
     短板，闭环应能补上。
3. **决策准则（建议）**：以纵向组三方对比为主、横向组 B/C 专项为辅，
   选"稳态残差绝对值小、方向误差小、且无可感知抖动"的模式；
   若 C 与 B 残差接近，则选 B（结构更简单）。

### 实测判读（2026-07-31）

1. **B（开环）残差为正而非预期的负**：第 1 轮 +71% 实为轮速标定错误
   （0.3 → 实际约 0.5），修正后仍 +17%，且 0.20 m/s 时降到 +7.3%——
   标定斜率随速度非线性，单一标定值修不好开环。斜向组暴露更本质的
   短板：横向轴塌陷（目标 (0.15,0.15)，实际横向仅 0.03–0.05 m/s，
   方向偏 35°），速率指标掩盖了向量失控。
2. **C（PI 闭环）第 2 轮 +33% 是估计器问题而非结构/增益问题**：
   控制器自记车速 0.153 m/s（以为达标），位移真值 0.199 m/s——
   逐帧差分的 position noise/dt 在动捕高频下越过 1.0 m/s 毛刺阈值，
   筛样不对称把估计压低。加 0.2 s 差分基线后，C 纵向残差降到
   **−1.4% ± 0.5%**（接近编队死区 0.015 m/s 的 10% 量级，无需再整定），
   斜向方向误差 12.4° 仅为 B 的 1/3，RMSE 全面最低。
3. **A（diff）三轮稳定欠驱动 −15.6% → −27.2% → −24.5%±1.5%**：
   速度环 ki=0（纯 P）有结构性稳态误差；且其轮指令走
   `command_speed_limit_mps`（0.25）而非 `max_wheel_speed_mps`，
   与 B/C 不同标定路径，三方"共用标定"前提对 A 本就不完全成立。
4. **低速段（≤0.10 m/s）全体失控是死区问题而非控制问题**：
   标定 0.5 后 min_eff=25 ≈ 0.125 m/s 轮速，0.05/0.10 目标的轮指令
   （10/20）被抬到 25，恰卡静摩擦边界，动不动全凭运气（omni 0.10
   完全不动 −99.7%，diff 0.05 却仅 −15.3%，单次随机性大）。
   该段数据只能定性：**约 0.1 m/s 以下为死区-黏滑区**，对编队工况
   （速度常 <0.1 m/s）是首要瓶颈。

## 八、结论（2026-07-31 填写）

- 三种模式稳态残差对比结论：**omni_pid 全面最优**——纵向 −1.4%±0.5%
  （omni +17.2%，diff −24.5%），斜向方向误差 12.4°（omni 35.1°），
  高速 0.20 m/s −1.8% 且消除横向漂移；omni 的残差与横向塌陷、diff 的
  欠驱动均为开环/纯 P 环的结构性问题，无法靠标定修复。
- 选型结果：☐ diff　☐ omni　☑ **omni_pid**
- 理由：稳态残差绝对值最小（1.4%，已在编队死区量级以下）、方向误差
  最小、RMSE 最低且三次重复稳定；默认增益 (0.6, 1.2) 即达标，
  无需整定。前提是估计器差分基线修复（0.2 s）已就位。
- 后续动作：
  1. ~~整定 `omni_velocity_pid`~~（残差 1.4%，不必要）；
  2. **重测电机死区**（`check_wheels --ramp`）：min_eff=25 在新标定下
     ≈0.125 m/s 轮速，是低速/编队工况的首要瓶颈；
  3. `bearing_two` 编队实验复核 bearing 稳态残差（注意编队速度常
     <0.1 m/s，先确认死区再跑）；
  4. 可选补测：纯横向 (0, 0.15) 阶跃（本次第 2 组实跑为斜向，
     结论由斜向数据外推）；
  5. 可选改进：diff 指令尺度对齐 `max_wheel_speed_mps`，消除 A 组
     标定路径不一致（仅影响今后对比实验的公平性，不影响选型）。

## 附录 A：关键参数（实验时核对）

| 参数 | 值 | 位置 |
| --- | --- | --- |
| 阶跃速度/时长 | 0.15 m/s / 6 s | `configs/chassis_step.py` |
| 指令限速 | 0.25 m/s | `RuntimeConfig.command_speed_limit_mps` |
| 编队死区（阶跃须远大于此） | 0.015 m/s | `BearingControlConfig.deadband_mps` |
| min_eff 死区补偿 | 25 | `chassis_step.py` execution |
| omni_pid 增益 (kp,ki,kd,积分限,输出限) | (0.6, 1.2, 0, 0.10, 0.15) | `ExecutionConfig.omni_velocity_pid` |
| 轮速标定（cmd 100 对应轮速） | 0.5 m/s（B 组开环数据预估初值，待专项标定复核） | `ExecutionConfig.max_wheel_speed_mps` |
| 速度估计差分基线 | 0.2 s | `RuntimeConfig.velocity_diff_baseline_s` |
| 控制 / 记录频率 | 50 Hz / 5 Hz | `RuntimeConfig` |

## 附录 B：故障排查

| 症状 | 可能原因 | 处理 |
| --- | --- | --- |
| preflight 卡在等 mocap | VRPN 未启动 / marker 掉落 | 查 `ros2 topic hz /car1/pose` |
| 车完全不动 | min_eff 偏低 / 车轮卡阻 | `check_wheels --ramp` 重测死区 |
| 运动方向错 | wheel_flip / marker 朝向错 | 跑 `check_wheels` 逐步核对 |
| diff 组车速明显偏低 | 车头未朝指令方向（门控限速） | 重新对正摆放（differential.py:125-131） |
| omni_pid 抖动/超调 | 增益对 EMA 滞后偏激进 | 降 kp/ki 后重跑该组并注明 |
| 残差全组系统偏大 | 轮速标定漂移 | 复核 `max_wheel_speed_mps` 标定 |
| omni_pid 残差异常偏大且 `measured_speed` 与位移真值不符 | 逐帧差分 noise/dt 越过毛刺阈值，筛样不对称压低速度估计 | 加大 `velocity_diff_baseline_s`（本次 0.2 s 修复）；勿盲目降 `max_plausible_speed_mps`（截断球向原点收缩，反而加重低估） |
| 低速（≤0.1 m/s）阶跃车不动或时动时停 | 轮指令被 min_eff 抬到死区边界，黏滑区 | `check_wheels --ramp` 重测死区；低速对比数据仅定性使用 |
| 同参数重跑一次正常一次不动 | 指令恰在死区边界，静摩擦随机性 | 保留标注为 invalid，重跑；长期解是死区重标定 |

## 附录 C：相关文件索引

- 实验配置：`swarm_lab_jiang_v1/configs/chassis_step.py`
- 被测控制器：`swarm/algorithms/{mecanum.py, mecanum_pid.py, differential.py}`
- 阶跃 planner：`swarm/algorithms/constant_velocity.py`
- 后处理：`report_v1/chassis_comparison/postprocess_chassis.py`（口径说明见其 README.md）
- 本次结果 JSON：`report_v1/chassis_comparison/results_chassis_step_longitudinal.json`
  （第 1 轮，标定 0.3）、`results_chassis_step_longitudinal_calib05.json`（第 2 轮）、
  `results_chassis_step_rep3.json`（第 3 轮三次重复）、
  `results_chassis_step_diagonal_rep3.json`（斜向组）、
  `results_chassis_step_sweep.json`（速度扫描组）
- 本手册：`chassis_experiment_manual.md`（仓库根目录，本地文件，未入 git 白名单）
