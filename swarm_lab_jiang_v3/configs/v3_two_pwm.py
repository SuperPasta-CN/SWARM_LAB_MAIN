"""B 组变体：pwm 占空比脉冲调制（振荡治理实验，见 report_v3/oscillation_analysis.md §5）。

继承 v3_two 的全部配置，仅把死区补偿换成占空比脉冲调制：整向量按比例
缩放后以 min_eff 脉冲输出，时间平均等于请求值，瞬时方向严格保持。
"""

from dataclasses import replace

from configs.v3_two import CONFIG as _BASE

CONFIG = replace(
    _BASE,
    name="v3_two_pwm",
    execution=replace(_BASE.execution, deadzone_mode="pwm"),
)
