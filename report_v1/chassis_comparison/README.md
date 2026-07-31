# chassis_comparison —— 底盘运动学三方对比（稳态残差）

对比三种底盘运动学控制逻辑的稳态速度跟踪残差，为执行层选型提供依据：

| 模式 | 控制器 | 结构 |
| --- | --- | --- |
| `omni`（默认） | `swarm/algorithms/mecanum.py` | 麦轮全向 + 平移开环 + 航向保持 P |
| `omni_pid` | `swarm/algorithms/mecanum_pid.py` | 麦轮全向 + 车体速度前馈+PI 闭环 + 航向保持 P |
| `diff` | `swarm/algorithms/differential.py` | 差速转向 + 速度/航向双 PID（旧方案移植） |

## 工作流程

1. 在 `swarm_lab_jiang_v1/configs/chassis_step.py` 里设定阶跃 (vx, vy, duration)
   与 `execution.mode`，单车空旷场地运行：

   ```bash
   cd swarm_lab_jiang_v1
   python3 run_experiment.py --config chassis_step
   ```

   阶跃结束后自动停车退出（`stop_on_converge`）。每种模式各跑一遍
   （改 `execution.mode` 重跑），得到三个 `runs/chassis_step_<时间戳>/` 目录。

2. 对比（本目录脚本）：

   ```bash
   python3 report_v1/chassis_comparison/postprocess_chassis.py \
       swarm_lab_jiang_v1/runs/chassis_step_<t1> \
       swarm_lab_jiang_v1/runs/chassis_step_<t2> \
       swarm_lab_jiang_v1/runs/chassis_step_<t3>
   ```

   也可直接传 `swarm_lab_jiang_v1/runs/` 父目录，自动纳入全部含
   `trajectory.csv` 的运行（按目录名即时间戳排序）。

## 指标口径

- **实际速度一律由记录位置中心差分重建**（0.4 s 基线），与
  `swarm_lab_jiang_v1/tools/postprocess.py` 一致；CSV 里的 `measured_vx/vy`
  是动捕差分噪声，不直接使用。
- **稳态窗口**：默认自动取"目标速度 ≥ 峰值 50%"的样本（即阶跃激活段）的
  后 60%，排除起动瞬态与停车尾段；也可用 `--t-start/--t-end`（相对运行
  起点的秒数）手动指定。
- 逐车输出：平均目标/实际速度、残差向量、速度残差（绝对 m/s 与相对 %）、
  方向误差（deg）、‖v*−v‖ RMSE。
- 若运行目录的 `summary.json` 含 bearing 统计（编队实验），作为**编队层**
  参考一并打印；选型以底盘层残差为准（外层编队闭环会掩盖执行差异）。
- `--json PATH` 导出全部数值供进一步分析。

## 备注

- 三方对比的交集是纵向（车头方向）阶跃：`diff` 为非完整约束底盘，横向
  指令物理上不可执行；横向阶跃组是 omni vs omni_pid 的专项对比。
- 反馈信号：omni_pid 默认用位置差分 + EMA 速度（与 diff 同源，保证公平）；
  如需评估 VRPN 原生 twist，设 `runtime.require_twist=True` 并先在实机上
  验证 twist 坐标系。
- 本目录只读原始记录，不改写任何运行数据。
