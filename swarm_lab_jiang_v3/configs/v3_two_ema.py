"""D 组变体：削减速度反馈时延（振荡治理实验，见 report_v3/oscillation_analysis.md §5）。

继承 v3_two 的全部配置，仅把速度估计的 EMA 系数 0.3 → 0.5
（更小的平滑延迟，降低 PI 环路的相位滞后）。
"""

from dataclasses import replace

from configs.v3_two import CONFIG as _BASE

CONFIG = replace(
    _BASE,
    name="v3_two_ema",
    runtime=replace(_BASE.runtime, velocity_ema_alpha=0.5),
)
