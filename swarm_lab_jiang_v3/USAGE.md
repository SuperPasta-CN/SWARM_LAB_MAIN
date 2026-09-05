# swarm_lab_jiang_v3 使用说明（实操手册）

面向现场使用者：从同步代码到拿到实验数据的完整流程。
约定：开发在 Windows（`D:\ros2\bearing4cars\swarm_lab-main`），实验在 WSL
（`~/swarm_lab_jiang_v3`）里跑。v2 的对应工程是 `~/swarm_lab_jiang_v2`，
两套互不干扰，可随时切换对比。

## 0. 一句话概念

v3 的控制律：`u_i = −k·Σ a_ij·(p_i − p_j + b_ij) + Z_i·w`。
编队形状由 `formation` 里的**期望坐标**决定（含距离），整体机动由任务
速度 `w` 决定（全员统一叠加），两者解耦互不影响。

## 1. 同步代码（Windows 改动后必做）

```bash
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/ros2/bearing4cars/swarm_lab-main && \
  tar --exclude='__pycache__' --exclude='swarm_lab_jiang_v3/runs' -cf - swarm_lab_jiang_v3 | tar -xf - -C ~/"
```

同步后验证（应为 Ran 118 / OK）：

```bash
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/swarm_lab_jiang_v3 && \
  python3 -m unittest discover -s tests 2>&1 | grep -E '^(Ran|OK|FAILED)'"
```

注意：同步是**全量覆盖**——在 WSL 侧手动改过的配置会被冲掉。现场调参
建议直接改 WSL 副本，定型后再回写 Windows 端。

## 2. 开跑前（每次）

1. **动捕**：按实验室既有流程启动（`ros_setup.sh` / `vrpn_ws`），确认话题：

   ```bash
   ros2 topic echo /car6/pose --once
   ros2 topic hz /car6/pose
   ```

2. **摆车**：车距 ≥ 0.20 m；初始构型尽量接近 `formation` 期望（preflight
   会打印逐边米制误差表，大误差只警告不拒飞，但收敛要花时间）。
   车头朝向随意（`heading_target_rad=0.0` 会让各车解锁后自动转向 +x）。

3. **单车校验**（换电池/动过接线/新增车辆后）：

   ```bash
   python3 tools/check_wheels.py --address 10.1.1.86
   ```

4. **急停预案**：运行中 Ctrl-C 自动下发停车；必要时直接断电。

## 3. 运行实验

```bash
cd ~/swarm_lab_jiang_v3
python3 run_experiment.py --config v3_two
# 连跑可加 --yes 跳过回车确认（preflight 硬校验仍生效）
```

默认时序（v3_two 为例）：

- **t ∈ [0, 3) 秒**：全员静止——各车原地转向 +x，约束项做静帧纠偏；
- **t = 3 s 起**：整体以 `w=(0.10, 0)` 向 +x 巡航；
- 结束：Ctrl-C（或配置 `stop_on_converge=True` 后自动停：平均边误差
  < 0.03 m 持续 3 s）。

运行产物在 `~/swarm_lab_jiang_v3/runs/v3_two_<时间戳>/`：
`trajectory.csv`（轨迹）、`edge_errors.csv`（逐边米制误差）、
`metadata.json`、`summary.json`、`trajectory.png`。

## 4. 后处理与判读

```bash
python3 tools/postprocess.py runs/v3_two_<时间戳> --window 10
```

看 `analysis.json` 的 **`steady_state` 节**（末 N 秒），不要看全程 mean
（含瞬态，系统性偏大）。关键指标：

- `edge_global.mean_error_m / rms_error_m`：队形残差（米）。
  理论下限 ≈ `deadband/k ≈ 0.019 m`（死区内约束项冻结，属正常）；
- `vehicles.*.velocity_rmse_mps`：逐车速度跟踪；
- `initial_rms_m`：初始队形误差，用于解释收敛过程。

## 5. 改配置（常见四类需求）

都在 `configs/v3_two.py`（或新建 `configs/v3_*.py`）里改：

| 需求 | 改哪里 | 示例 |
| --- | --- | --- |
| 换队形/换间距 | `topology.formation` 坐标 | `"car5": (0.0, 1.0)` |
| 换巡航速度/方向 | `task_velocity` + `task_velocity_schedule` 末值 | 两处都要改，保持一致 |
| 调收敛快慢 | `control.k`（大=快但易饱和/抖） | `TaskDrivenConfig(k=1.2)` |
| 换车 | `vehicles` 里的车号/IP + `formation` 的键同步改 | 三处一致 |

注意事项：

- **队形坐标是世界系绝对坐标**，但实际起作用的是相对位置（整体平移
  自由），摆放位置不用迁就坐标原点，形状对就行；
- 残差到 ~0.02 m 就到底了（死区下限），再调 k 也压不下去；
- 需要 matrix 加权（非各向同性边权）时，改 `FormationSpec.weight_blocks`
  （`edge_weight_matrix` 只支持标量）；
- 任务模态目前只有整体平移（`translate_x/translate_y`）；要加内部形变
  模态，用显式 2n 向量传给 `build_task_modes`（需自行验证该模态在
  零空间内，否则会被约束项抵消）。

## 6. 故障排查

| 症状 | 可能原因 | 处理 |
| --- | --- | --- |
| preflight 卡等 mocap | VRPN 未启动 / marker 掉落 | `ros2 topic hz /carX/pose` |
| 解锁后某车不动 | 没电 / 死区偏低 / 动力链断 | console 看该车 measured 恒 0；`check_wheels --ramp` 重测死区 |
| 队形到位但原地抖 | k 偏大 / min_eff 抬升边界 | 降 k；或按车标定（`execution_override`） |
| 巡航方向跑偏 | 动捕世界系方向错 | 手推车沿 +x 确认 x 增大 |
| 残差卡在 ~0.02 m 不动 | 死区下限，正常 | 见 §4，不是故障 |
| 车往队形"镜像"方向跑 | formation 坐标顺序/车号对错 | 核对 formation 的键与车号一一对应 |

## 7. 与 v2 对比实验

同一场地、同一批车，先跑 v2（`~/swarm_lab_jiang_v2`）再跑 v3，后处理
口径注意换算：v2 的边误差是角度（deg，bearing_edges.csv），v3 是相对
位置误差（m，edge_errors.csv）。对比时建议以"队形残差收敛时间 +
稳态值 + 行进振荡"三个维度陈述。
