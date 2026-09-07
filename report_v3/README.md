# report_v3 —— v3 矩阵加权拉普拉斯 + 任务驱动编队机动实验

工程：`swarm_lab_jiang_v3`（约束律 `u_i = −k·Σ a_ij(p_i−p_j+b_ij) + Z_i·w`，
无角色；规格见工程内 `README.md` / `MIGRATION.md` / `USAGE.md`）。

## 数据

`runs/` —— 2026-09-04 首次 v3 实车系列（自 WSL `~/swarm_lab_jiang_v3/runs/`
拷回，只读原始记录，分析不改动）：

| 运行 | 编队 | 说明 |
| --- | --- | --- |
| `v3_two_20260904_152305_491918` | 2 车（car6/car5，期望间距 0.8 m） | 当日首批 |
| `v3_two_20260904_152637_829407` | 2 车 | |
| `v3_two_20260904_153022_135638` | 2 车 | |
| `v3_three_20260904_153912_199619` | 3 车三角（car3/car5/car6，直角边 0.8 m） | |
| `v3_three_20260904_154418_357386` | 3 车三角 | |
| `v3_four_20260904_154632_535713` | 4 车正方形（car3/car2/car6/car5，边长 0.8 m） | |
| `v3_four_20260904_154751_276804` | 4 车正方形 | |

每个运行目录含 `trajectory.csv`、`edge_errors.csv`（米制逐边相对位置误差）、
`metadata.json`、`summary.json`、`trajectory.png`。

## 判读口径（与 v1/v2 报告一致）

- 实际速度一律由记录位置做中心差分重建（0.4 s 基线），不直接用 CSV 的
  `measured_vx/vy` 列（动捕差分噪声）；
- 稳态一律用末 N 秒窗口：`python3 tools/postprocess.py runs/<目录> --window 10`
  （在 `swarm_lab_jiang_v3` 工程内执行），取 `analysis.json` 的
  `steady_state` 节，禁用全程 mean；
- 边误差单位为米；理论残差下限 ≈ `deadband/k ≈ 0.019 m`；
- 核心验收指标：期望间距锁定（0.8 m）与机动段稳态边误差（目标 0.02–0.04 m）、
  巡航速度达成率（目标 w=0.10 m/s 的 95–105%）。

## 分析产物（2026-09-04 批次已完成）

`analysis/`：`scripts/`（common.py + fig1…fig6 六个分析脚本）、`figs/`（6 张图）、
`results/`（6 个 JSON）。复跑：`cd report_v3/analysis/scripts && python <脚本>`。

- fig1 收敛性：7 轮全部收敛，机动段末 5 s 稳态边误差 0.022–0.062 m
  （死区下限 0.019 m 附近），破 0.03 m 用时 2.6–4.5 s；
- fig2 尺度锁定：v3 各边距离锁定在期望值（0.5/0.7/0.8/1.13 m，按当日实际
  formation 元数据）；同队形 v2（红色虚线）尺度自由漂移——版本间本质差异；
- fig3 零空间解耦：巡航段质心速度 0.090–0.099 m/s（达成率 90–99%），
  边误差与机动互不影响；
- fig4 起步段：3 s 静止窗内完成航向自对准（最极端 car5 从 +155°/−176° 转回）
  与初步纠偏；
- fig5 v2↔v3 最终收敛对比（形状拟合残差，版本中立）：3 车 v2 26.2±8.4 mm vs
  v3 9.6–12.0 mm；4 车 v2 29.2–48.2 mm vs v3 11.5–12.3 mm；v3 拟合尺度
  1.000±0.01（锁定）、v2 0.64–1.30（自由漂移）；
- fig6 振荡（已知不足）：巡航段 yaw 摆动中位 2–8°、速度波纹中位 ~45–70%、
  轮指令换向 ~2 次/s（min_eff 极限环）；
- fig7 振荡归因：巡航段 53–85% 轮指令被钉死在 ±35、平移型翻转为主
  （45–84%）——巡航 0.10 m/s 的原始指令（≈20）低于 min_eff=35，处于结构性
  bang-bang 区（阈值等效 0.175 m/s），振荡是参数性而非控制律结构性问题。

报告：`report_v3/report.tex`（xelatex，两遍；含命题验证总表与不确定项）。

振荡专题：`oscillation_analysis.md`（初版归因）与
**`oscillation_reanalysis.md`（9-07 实车 A/B 后的修正版——PWM 有效、
affine 证伪"量化是根因"、低速死区才是本体）**。仿真：`analysis/scripts/sim_limit_cycle.py`。
