# SWARM_LAB_MAIN

Bearing-only 多车编队控制：网页交互式仿真 + ROS2 实车工程。

仓库包含两条主线：先在 `stimulus` 里验证算法收敛性（理想单积分器模型），
再把控制律部署到动捕 + 麦轮小车的真实平台并处理实车工程化问题（执行器标定、
摩擦死区、起飞前安全校验）。实车工程有两代：`swarm_lab_jiang_v1`（全员平等 /
二元 leader 的静态编队）与 `swarm_lab_jiang_v2`（captain / first_mate / crew
三级角色的编队机动，Zhao & Zelazo 2015 结构）。

## stimulus/ —— 网页仿真（React + TypeScript + Vite）

Bearing 编队控制交互式可视化演示：多智能体从随机初始分布收敛到目标构型的完整动态过程。

- 控制律：`u_i = Σ P_ij(g_ij − g*_ij)`（正交投影，P = I − g·gᵀ）
- 支持：多种编队形状与拓扑、标准/饱和/事件触发控制律、尺度修正、质心对齐、误差曲线实时绘制

运行（Node.js 20+）：

```bash
cd stimulus/app
npm install
npm run dev        # http://localhost:5173
```

详见 `stimulus/app/README.md`。

## swarm_lab_jiang_v1/ —— ROS2 实车工程（Python）

面向地面麦轮小车群的 bearing-only 编队控制：ROS2/VRPN 动捕状态输入，
投影控制律输出世界系速度，麦轮全向执行器下发 UDP 轮速指令。

- **控制律**：`Σ P_ij(g−g*)` 求和 + tanh 平滑饱和 + 死区，bearing 默认 2D
- **运动模式**：`all_bearing` 全员运动（默认）/ `leader_velocity` 钉死锚点 / `leader_bias` 编队机动
- **执行器**：三种底盘运动学可选——麦轮全向开环（默认）、麦轮+车体速度 PI 闭环（`omni_pid`）、差速双 PID（`diff`）；`chassis_step` 恒速阶跃配置支持三方稳态残差对比
- **安全**：preflight 起飞前校验（锚定边 bearing 一致性、最小车距、逐边误差表 + 人工确认）
- **标定**：轮速-指令标定 + 电机静摩擦死区补偿（`tools/check_wheels.py --ramp` 实测）
- **遥测**：终端输出、实时轨迹窗、CSV 记录、结束自动保存 `trajectory.png`

环境：Ubuntu 22.04 + ROS2 Humble（rclpy、vrpn_client_ros）；Python 3.10+

```bash
cd swarm_lab_jiang_v1
python3 -m unittest discover -s tests     # 113 项单元测试，无需 ROS2/实车
python3 tools/check_wheels.py --address 10.1.1.84   # 单车硬件校验
python3 run_experiment.py --config bearing_four     # 实车实验
```

详见 `swarm_lab_jiang_v1/README.md`（含调参指南与实验室检查单）。

## swarm_lab_jiang_v2/ —— 角色化编队机动工程（Python）

v1 的机动升级版：引入 **captain / first_mate / crew** 三级角色
（Zhao & Zelazo《Bearing-Based Formation Maneuvering》2015 结构），
实现定速平移机动并预留变速接口。

- **角色**：captain 只输出固定速度（恰好 1 个，承载平移参考）；first_mate 为
  固定速度与 bearing 项的合成（weighted/additive，钉死尺度）；crew 纯 bearing 律
- **控制律**：v1 投影律 + 可选 PID（`ki>0` 消除 leader 运动下的恒定跟踪误差，
  含端到端回归验证）
- **工程改进**：按车执行端标定（弱车单独覆盖）、preflight 改为 captain 边软警告、
  后处理直接产出末 N 秒稳态统计

```bash
cd swarm_lab_jiang_v2
python3 -m unittest discover -s tests     # 139 项单元测试，无需 ROS2/实车
python3 run_experiment.py --config crew_four  # 实车机动实验
```

详见 `swarm_lab_jiang_v2/README.md`（含角色行为表、理论依据与调参指南）。

## 实验数据后处理

两层后处理，计算口径一致：速度一律由记录位置做中心差分重建（0.4 s 基线），
因为旧记录的 `measured_vx/vy` 列含 10–50 m/s 动捕尖峰，不可直接使用。

**单次运行**：`swarm_lab_jiang_v1/tools/postprocess.py`

```bash
cd swarm_lab_jiang_v1
python3 tools/postprocess.py <run_directory>   # 目录需含 trajectory.csv（可选 bearing_edges.csv）
```

在该运行目录下生成 `analysis.json` + `analysis.png`。

**跨运行对比分析**：`report_v1/analysis/`（2026-07-28 误差分析，全程只读，不改动原始记录）

```bash
cd report_v1/analysis/scripts
python3 goal1_convergence.py    # 收敛时间与稳态残差对比
python3 goal2_residual.py       # 残差来源分解（死区平衡/爬行/摩擦冻结、轮指令统计）
python3 goal3_edges_cars.py     # 逐边不对称、形状拟合、逐车速度跟踪
python3 goal4_bearing2.py       # 2 车运行形态分析
python3 goal4b_wsl_overview.py  # WSL 当日 22 次运行总览
python3 goal5_drift.py          # all_bearing 自由度漂移（旋转/尺度/质心）
```

数据源在 `common.py` 中硬编码（`report_v1/results/codeoutputs/` 下三次报告运行，
公共参数取自各目录 `metadata.json`）；产物写入 `analysis/figs/*.png` 与
`analysis/results/*.json`，报告正文为 `analysis/report.md` / `report.tex`。

**底盘运动学对比**：`report_v1/chassis_comparison/postprocess_chassis.py`

```bash
python3 report_v1/chassis_comparison/postprocess_chassis.py <run_dir_1> [<run_dir_2> ...]
# 或传入 runs/ 父目录，自动纳入所有含 trajectory.csv 的运行
```

输出稳态窗口内的速度残差（绝对/相对/方向差/RMSE）与多运行对比表；配合
`swarm_lab_jiang_v1/configs/chassis_step.py`（omni / omni_pid / diff 各跑一遍）
做三种底盘运动学的选型对比，详见 `report_v1/chassis_comparison/README.md`。

## 仓库结构

```
SWARM_LAB_MAIN/
├── stimulus/              # 网页仿真（算法验证）
│   └── app/               # Vite + React + TS 应用
├── swarm_lab_jiang_v1/    # 实车工程 v1（静态编队）
│   ├── configs/           # 两车/三车/四车/六车实验配置 + chassis_step 恒速阶跃
│   ├── swarm/             # domain / algorithms / application / infrastructure
│   ├── tools/             # 单车校验、单次运行后处理
│   └── tests/             # 单元测试 + 端到端收敛回归
├── swarm_lab_jiang_v2/    # 实车工程 v2（captain/first_mate/crew 编队机动）
│   ├── configs/           # crew_two/three/four/six（拓扑沿用 v1）
│   ├── swarm/             # 同 v1 分层，角色分派控制律 + 可选 PID
│   ├── tools/             # 单车校验、后处理（含末 N 秒稳态统计）
│   └── tests/             # 单元测试 + 机动纯 P vs PI 回归
├── report_v1/             # 2026-07-28 误差分析
│   ├── results/           # 三次报告运行的原始记录（codeoutputs/；videos/ 大二进制仅本地）
│   ├── paperwork/         # 代码逻辑变更说明
│   ├── analysis/          # 分析脚本 scripts/*.py、图 figs/、数值结果 results/、report.md/tex
│   └── chassis_comparison/ # 底盘运动学（omni/omni_pid/diff）稳态残差对比工具
└── report_v2/             # v2 机动实验报告（待实验数据填充）
```
