"""A 组变体：affine 死区逆（振荡治理实验，见 report_v3/oscillation_analysis.md §5）。

继承 v3_two 的全部配置，仅把死区补偿从固定抬升（lift）换成死区逆
（affine）：小指令连续映射到阈值之上，消灭 bang-bang 的量化断档。
"""

from dataclasses import replace

from configs.v3_two import CONFIG as _BASE

CONFIG = replace(
    _BASE,
    name="v3_two_affine",
    execution=replace(_BASE.execution, deadzone_mode="affine"),
)
