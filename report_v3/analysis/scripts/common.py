"""Shared data-loading helpers for the v3 task-driven formation analysis.

Two data dialects are supported side by side (for the v2-vs-v3 comparison):
- v3 runs (report_v3/runs/): edge_errors.csv in METERS (relative-position
  error), trajectory.csv without a role column;
- v2 runs (report_v2/data_0814/0814_3/runs/): bearing_edges.csv in DEGREES,
  trajectory.csv with an (empty) role column.

Velocity is always rebuilt from recorded positions via central differences
(0.4 s baseline); the recorded measured_vx/vy columns are mocap
differentiation noise.  Steady state is reported on the trailing window of
the *active* segment (task velocity actually commanded), never the
whole-run mean.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from math import atan2, hypot, isfinite, sqrt
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
ANALYSIS_DIR = SCRIPTS_DIR.parent          # report_v3/analysis/
REPORT_DIR = ANALYSIS_DIR.parent           # report_v3/
V2_0814_3 = REPORT_DIR.parent / "report_v2" / "data_0814" / "0814_3" / "runs"

FIGS_DIR = ANALYSIS_DIR / "figs"
RESULTS_DIR = ANALYSIS_DIR / "results"

V3_RUNS = {
    "two": ["v3_two_20260904_152305_491918",
            "v3_two_20260904_152637_829407",
            "v3_two_20260904_153022_135638"],
    "three": ["v3_three_20260904_153912_199619",
              "v3_three_20260904_154418_357386"],
    "four": ["v3_four_20260904_154632_535713",
             "v3_four_20260904_154751_276804"],
}
# v2 full-stack validation runs (08-14 afternoon, same 0.10 m/s cruise).
V2_RUNS = {
    "two": ["crew_two_20260814_144051_271619", "crew_two_20260814_144224_128931"],
    "three": ["crew_three_20260814_151626_079481"],
    "four": ["crew_four_20260814_152418_978476", "crew_four_20260814_152811_341988"],
}

DEADBAND_FLOOR_M = 0.015 / 0.8  # deadband_mps / k: constraint freezes below this


def v3_run_dir(name: str) -> Path:
    return REPORT_DIR / "runs" / name


def v2_run_dir(name: str) -> Path:
    return V2_0814_3 / name


def load_metadata(run_dir: Path) -> dict:
    with (run_dir / "metadata.json").open(encoding="utf-8") as src:
        return json.load(src)["experiment"]


def load_trajectory(run_dir: Path):
    """vehicle_id -> list of row dicts (floats), ordered by time."""

    vehicles = defaultdict(list)
    with (run_dir / "trajectory.csv").open("r", newline="", encoding="utf-8") as src:
        for raw in csv.DictReader(src):
            vehicle_id = raw.pop("vehicle_id")
            raw.pop("role", None)
            vehicles[vehicle_id].append(
                {key: float(value) for key, value in raw.items() if value != ""}
            )
    return dict(vehicles)


def load_v3_edges(run_dir: Path):
    """Directed v3 edge rows: (source, target) -> [{time_s, edge_error_m, ...}]."""

    edges = defaultdict(list)
    with (run_dir / "edge_errors.csv").open("r", newline="", encoding="utf-8") as src:
        for raw in csv.DictReader(src):
            source_id = raw.pop("source_id")
            target_id = raw.pop("target_id")
            edges[(source_id, target_id)].append(
                {key: float(value) if value else float("nan")
                 for key, value in raw.items()}
            )
    return dict(edges)


def load_v2_edges(run_dir: Path):
    """Directed v2 edge rows: (source, target) -> [{time_s, bearing_error_deg}]."""

    edges = defaultdict(list)
    with (run_dir / "bearing_edges.csv").open("r", newline="", encoding="utf-8") as src:
        for raw in csv.DictReader(src):
            source_id = raw.pop("source_id")
            target_id = raw.pop("target_id")
            edges[(source_id, target_id)].append(
                {key: float(value) if value else float("nan")
                 for key, value in raw.items()}
            )
    return dict(edges)


def velocity_series(rows, baseline_s: float = 0.4):
    """(time, vx, vy) from recorded positions via central differences."""

    out = []
    count = len(rows)
    for index in range(count):
        lo = index
        while lo > 0 and rows[index]["time_s"] - rows[lo]["time_s"] < baseline_s / 2:
            lo -= 1
        hi = index
        while hi < count - 1 and rows[hi]["time_s"] - rows[index]["time_s"] < baseline_s / 2:
            hi += 1
        dt = rows[hi]["time_s"] - rows[lo]["time_s"]
        if dt > 0.0:
            out.append((
                rows[index]["time_s"],
                (rows[hi]["x_m"] - rows[lo]["x_m"]) / dt,
                (rows[hi]["y_m"] - rows[lo]["y_m"]) / dt,
            ))
        else:
            out.append((rows[index]["time_s"], 0.0, 0.0))
    return out


def active_window(vehicles, cruise_mps: float):
    """(t_start, t_end): interval where the mean commanded speed is active."""

    per_time = defaultdict(list)
    for rows in vehicles.values():
        for row in rows:
            per_time[round(row["time_s"], 6)].append(
                hypot(row["target_vx_mps"], row["target_vy_mps"])
            )
    times = sorted(per_time)
    active = [t for t in times
              if sum(per_time[t]) / len(per_time[t]) > 0.5 * cruise_mps]
    return (active[0], active[-1]) if active else None


def window_values(series, t0, t1):
    return [v for t, v in series if t0 <= t <= t1]


def mean_std(values):
    values = list(values)
    if not values:
        return None, None
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / len(values)
    return mean, sqrt(var)


def rms(values):
    values = list(values)
    return sqrt(sum(v * v for v in values) / len(values)) if values else None


def percentile(values, q):
    values = sorted(values)
    if not values:
        return None
    return values[min(len(values) - 1, max(0, int(q * len(values))))]


def overall_rms(series_per_item):
    """Per-timestamp RMS across multiple (time, value) series."""

    per_time = defaultdict(list)
    for series in series_per_item:
        for t, v in series:
            if isfinite(v):
                per_time[round(t, 6)].append(v)
    times = sorted(per_time)
    return times, [rms(per_time[t]) for t in times]


def similarity_fit(reference, current):
    """2D similarity transform (complex): current ~= scale*e^{i*rot}*reference + t.

    Returns (rotation_rad, scale, translation, residual_rms_m).
    reference/current: dict vehicle_id -> complex.
    """

    ids = sorted(reference)
    n = len(ids)
    ref_mean = sum(reference[i] for i in ids) / n
    cur_mean = sum(current[i] for i in ids) / n
    num = sum(
        (current[i] - cur_mean) * (reference[i] - ref_mean).conjugate() for i in ids
    )
    den = sum(abs(reference[i] - ref_mean) ** 2 for i in ids)
    if den <= 0.0:
        return 0.0, 1.0, cur_mean - ref_mean, 0.0
    transform = num / den
    scale = abs(transform)
    rotation = atan2(transform.imag, transform.real)
    translation = cur_mean - transform * ref_mean
    residual = sqrt(
        sum(abs(current[i] - (transform * reference[i] + translation)) ** 2 for i in ids)
        / n
    )
    return rotation, scale, translation, residual


def desired_reference(exp: dict):
    """Desired formation from a v3 metadata dict as vehicle_id -> complex."""

    return {
        vid: complex(xy[0], xy[1])
        for vid, xy in exp["topology"]["formation"].items()
    }


def save_results(name: str, payload: dict):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    path = RESULTS_DIR / f"{name}.json"
    with path.open("w", encoding="utf-8") as out:
        json.dump(payload, out, indent=2, ensure_ascii=False)
    print(f"results -> {path}")


def new_figure(width=14, height=9, rows=2, cols=2):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(rows, cols, figsize=(width, height))
    return fig, axes


def save_figure(fig, filename: str):
    path = FIGS_DIR / filename
    fig.savefig(str(path), dpi=160)
    print(f"fig -> {path}")
