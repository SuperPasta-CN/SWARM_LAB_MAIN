# SWARM_LAB_MAIN

Bearing-only 多车编队控制：网页交互式仿真 + ROS2 实车工程。

仓库包含两个子项目：先在 `stimulus` 里验证算法收敛性（理想单积分器模型），
再用 `swarm_lab_jiang_v1` 把同一套控制律部署到动捕 + 麦轮小车的真实平台，
并处理实车工程化问题（执行器标定、摩擦死区、起飞前安全校验）。

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
- **执行器**：麦轮全向（世界系→车体系→X 型逆运动学 + 航向保持）为主，差速转向可配置对比
- **安全**：preflight 起飞前校验（锚定边 bearing 一致性、最小车距、逐边误差表 + 人工确认）
- **标定**：轮速-指令标定 + 电机静摩擦死区补偿（`tools/check_wheels.py --ramp` 实测）
- **遥测**：终端输出、实时轨迹窗、CSV 记录、结束自动保存 `trajectory.png`

环境：Ubuntu 22.04 + ROS2 Humble（rclpy、vrpn_client_ros）；Python 3.10+

```bash
cd swarm_lab_jiang_v1
python3 -m unittest discover -s tests     # 84 项单元测试，无需 ROS2/实车
python3 tools/check_wheels.py --address 10.1.1.84   # 单车硬件校验
python3 run_experiment.py --config bearing_four     # 实车实验
```

详见 `swarm_lab_jiang_v1/README.md`（含调参指南与实验室检查单）。

## 仓库结构

```
SWARM_LAB_MAIN/
├── stimulus/              # 网页仿真（算法验证）
│   └── app/               # Vite + React + TS 应用
└── swarm_lab_jiang_v1/    # 实车工程（ROS2 部署）
    ├── configs/           # 两车/三车/四车/六车实验配置
    ├── swarm/             # domain / algorithms / application / infrastructure
    ├── tools/             # 单车校验、后处理
    └── tests/             # 单元测试 + 端到端收敛回归
```
