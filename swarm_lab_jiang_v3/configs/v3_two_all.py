"""全开变体：pwm + 航向死区（振荡治理组合组，实车验证过的 demo 配置）。

继承 v3_two 的全部配置，叠加两项验证有效的改动：
deadzone_mode="pwm"（消灭低速 bang-bang）、heading_deadband_rad=0.035
（≈2°，压制航向微振）。EMA 保持默认 0.3（α=0.5 实车无效，不保留）。
"""

from dataclasses import replace

from configs.v3_two import CONFIG as _BASE

CONFIG = replace(
    _BASE,
    name="v3_two_all",
    execution=replace(
        _BASE.execution,
        deadzone_mode="pwm",
        heading_deadband_rad=0.035,
    ),
)
