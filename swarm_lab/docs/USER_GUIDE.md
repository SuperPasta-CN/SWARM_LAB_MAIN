# swarm_lab 使用者手册（现场实操）

面向实验现场：从同步代码到拿到实验数据的完整流程。
约定：开发在 Windows（`D:\ros2\bearing4cars\swarm_lab-main\swarm_lab`），
实验在 WSL（`~/swarm_lab`）里跑。历史对照工程 `~/swarm_lab_jiang_v2`、
`~/swarm_lab_jiang_v3` 仍在，可复跑。

## 0. 一句话概念

主力控制律 `task_driven`：`u_i = −k·Σ a_ij·(p_i − p_j + b_ij) + Z_i·w`。
编队形状由 `formation` 期望坐标决定（含距离），整体机动由任务速度 `w`
决定（全员统一叠加），两者解耦互不影响。默认**先收敛后机动**：解锁后
各车原地纠偏 + 转向对准，队形误差稳定达标后自动开始巡航。

## 1. 同步代码（Windows 改动后必做）

```bash
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/ros2/bearing4cars/swarm_lab-main && \
  tar --exclude='__pycache__' --exclude='swarm_lab/runs' \
      --exclude='swarm_lab/tools/calibrations' -cf - swarm_lab | tar -xf - -C ~/"
```

同步后验证（应为 Ran 184 / OK）：

```bash
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/swarm_lab && \
  python3 -m unittest discover -s tests 2>&1 | grep -E '^(Ran|OK|FAILED)'"
```

注意：

- 同步是全量覆盖——在 WSL 侧手动改过的源码/配置会被冲掉。现场调参
  建议直接改 WSL 副本，定型后再回写 Windows 端。
- **例外**：`configs/calibrated_overrides.py`（逐车标定产物）只存在于
  WSL 侧，tar 包里没有它，不会被覆盖，无需备份。

## 2. 开跑前（每次）

1. **动捕**：按实验室既有流程启动（`ros_setup.sh` / `vrpn_ws`），确认话题：

   ```bash
   ros2 topic echo /car4/pose --once
   ros2 topic hz /car4/pose
   ```

2. **摆车**：车距 ≥ 0.20 m；初始构型尽量接近 `formation` 期望（preflight
   会打印逐边米制误差表，大误差只警告不拒飞，但收敛要花时间）。
   车头朝向随意（`heading_target_rad=0.0` 会让各车解锁后自动转向 +x）。

3. **单车校验**（换电池/动过接线/新增车辆后）：

   ```bash
   python3 tools/check_wheels.py --address 10.1.1.84
   ```

4. **急停预案**：运行中 Ctrl-C 自动下发停车；必要时直接断电。

## 3. 运行实验

```bash
cd ~/swarm_lab
python3 run_experiment.py --config task_two      # 两车（默认配置）
python3 run_experiment.py --config task_three    # 三角
python3 run_experiment.py --config task_four     # 正方形
python3 run_experiment.py --config bearing_two   # v2 对照律
# 连跑可加 --yes 跳过回车确认（preflight 硬校验仍生效）
```

默认时序（task_driven 配置，二阶段状态机自动推进）：

- **converge 阶段**：任务速度强制为零——各车原地转向 +x 对准，
  约束项做编队纠偏；
- **切入 maneuver**：平均边误差 < 0.03 m 持续 2 s 后自动切换
  （console 打印 `converged ... maneuver phase`），全员叠加 w=(0.10, 0)
  向 +x 巡航；超过 10 s 未达标也会兜底切入；
- 结束：Ctrl-C（或配置 `stop_on_converge=True` 后机动中再次收敛自动停：
  平均边误差 < `runtime.converge_eps_m` 持续 `converge_hold_s`）。

运行产物在 `~/swarm_lab/runs/<配置名>_<时间戳>/`：
`trajectory.csv`（轨迹）、`edge_errors.csv`（逐边米制误差）、
`metadata.json`、`summary.json`、`trajectory.png`。

## 4. 标定新车 / 换电池后（逐车执行参数）

弱车与强车的死区、速度增益差异是低速振荡与队形漂移的主要来源，
**新车入网或换电池后必须逐车标定**：

```bash
python3 tools/calibrate_car.py --address 10.1.1.84 --mocap
```

- **min_eff（最小有效轮指令）**：轮指令斜坡 0→60，车开始滚动时用
  mocap 自动检测（无 mocap 则眼看按 ENTER）；重复 3 次取中位 + 安全
  裕量。
- **v(35)（min_eff 指令处的持续车速）**：有 `--mocap` 时全自动——
  工具驱动小车并按 mocap 位移/时间计算。这是 PWM 补偿的精度上限：
  平均速度 = 占空比 × v(min_eff)。
- 产物：原始记录 `tools/calibrations/<车号>.json`；汇总能被配置直接
  import 的 `configs/calibrated_overrides.py`（各配置经
  `execution_override=CALIBRATED_OVERRIDES.get("carX")` 自动引用）。

已标定参考值（2026-09 车队）：car4 min_eff 30.2 / v(35)=0.201 m/s；
car5 23.5 / 0.283 m/s。

## 5. 后处理与判读

```bash
python3 tools/postprocess.py runs/task_two_<时间戳> --window 10
```

看 `analysis.json` 的 **`steady_state` 节**（末 N 秒），不要看全程 mean
（含瞬态，系统性偏大）。关键指标：

- `edge_global.mean_error_m / rms_error_m`：队形残差（米）。
  理论下限 ≈ `deadband/k ≈ 0.019 m`（死区内约束项冻结，属正常）；
- `vehicles.*.velocity_rmse_mps`：逐车速度跟踪；
- `initial_rms_m`：初始队形误差，用于解释收敛过程。

## 6. 改配置（常见需求）

都在 `configs/task_*.py`（或照样子新建）里改：

| 需求 | 改哪里 | 示例 |
| --- | --- | --- |
| 换队形/换间距 | `topology.formation` 坐标 | `"car5": (0.0, 1.0)` |
| 换巡航速度/方向 | `task_velocity`（+ `task_velocity_schedule` 末值，若有 schedule） | `(0.15, 0.0)` |
| 调收敛快慢 | `control.k`（大=快但易饱和/抖） | `TaskDrivenConfig(k=1.2)` |
| 调收敛判据/保持/超时 | `control.phase_converge_*` | `phase_converge_eps_m=0.02` |
| 关掉二阶段（立即机动） | `control.converge_first=False` | 对比实验用 |
| 换车 | `vehicles` 车号/IP + `formation` 键同步改 | 三处一致 |

注意事项：

- **队形坐标是世界系绝对坐标**，但实际起作用的是相对位置（整体平移
  自由），摆放位置不用迁就坐标原点，形状对就行；
- 残差到 ~0.02 m 就到底了（死区下限），再调 k 也压不下去；
- 需要非各向同性边权（矩阵权重）时，改 `FormationSpec.weight_blocks`
  （`edge_weight_matrix` 只支持标量）；
- 任务模态目前只有整体平移（`translate_x/translate_y`）；要加内部形变
  模态，用显式 2n 向量传给 `task_modes`（需自行验证该模态在零空间内，
  否则会被约束项抵消）。

## 7. A/B 对照实验（bearing 律）

同一场地、同一批车，先跑 `--config bearing_two`（v2 投影律 + 角色），
再跑 `--config task_two`，后处理口径注意换算：bearing 的边误差是角度
（deg，`bearing_edges.csv`），task_driven 是相对位置误差（m，
`edge_errors.csv`）。对比时建议以"队形残差收敛时间 + 稳态值 + 行进
振荡"三个维度陈述。

## 8. 故障排查

| 症状 | 可能原因 | 处理 |
| --- | --- | --- |
| preflight 卡等 mocap | VRPN 未启动 / marker 掉落 | `ros2 topic hz /carX/pose` |
| 解锁后某车不动 | 没电 / 死区偏低 / 动力链断 | console 看该车 measured 恒 0；重跑 `calibrate_car` |
| 队形到位但原地抖 | k 偏大 / min_eff 未逐车标定 | 降 k；跑 `calibrate_car` |
| 巡航方向跑偏 | 动捕世界系方向错 | 手推车沿 +x 确认 x 增大 |
| 残差卡在 ~0.02 m 不动 | 死区下限，正常 | 见 §5，不是故障 |
| 车往队形"镜像"方向跑 | formation 键与车号对错 | 核对一一对应 |
| 迟迟不切入机动 | 初始误差大 / eps 太紧 | 看 console 误差；兜底 10 s 必切入 |
