"""Edge-error metrics derived without changing formation control (v3)."""

from __future__ import annotations

from math import sqrt
from typing import Optional

from swarm.domain.models import EdgeSample


def edge_error_m(sample: EdgeSample) -> Optional[float]:
    """Relative-position error norm ``||actual - desired||`` in meters.

    ``None`` when the sample is flagged invalid (an endpoint state was
    invalid).  Relative positions stay defined at zero separation, so no
    overlap special-case exists (contrast with v1/v2 bearing samples).
    """

    if not sample.valid:
        return None
    return sqrt(
        sum((sample.actual[k] - sample.desired[k]) ** 2 for k in range(3))
    )
