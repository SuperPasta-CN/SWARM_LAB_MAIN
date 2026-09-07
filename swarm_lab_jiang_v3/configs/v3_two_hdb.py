"""C 组变体：航向死区（振荡治理实验，见 report_v3/oscillation_analysis.md §5）。

继承 v3_two 的全部配置，仅给航向保持加 0.035 rad（≈2°）死区：
小角度偏差不纠偏，抑制航向通道经死区抬升产生的旋转极限环。
"""

from dataclasses import replace

from configs.v3_two import CONFIG as _BASE

CONFIG = replace(
    _BASE,
    name="v3_two_hdb",
    execution=replace(_BASE.execution, heading_deadband_rad=0.035),
)
