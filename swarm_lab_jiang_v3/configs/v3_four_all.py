"""全开变体（四车）：pwm + 航向死区，继承 v3_four。"""

from dataclasses import replace

from configs.v3_four import CONFIG as _BASE

CONFIG = replace(
    _BASE,
    name="v3_four_all",
    execution=replace(
        _BASE.execution,
        deadzone_mode="pwm",
        heading_deadband_rad=0.035,
    ),
)
