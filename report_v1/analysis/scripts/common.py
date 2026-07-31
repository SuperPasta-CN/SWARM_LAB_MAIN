"""Shared data-loading helpers for the 2026-07-28 error analysis.

Read-only analysis: loads trajectory.csv / bearing_edges.csv from the three
report runs under report_v1/results/codeoutputs/.  Velocity is always
derived from recorded positions via central differences (0.4 s baseline),
matching swarm_lab_jiang_v1/tools/postprocess.py::_position_velocity, because
the recorded measured_vx/vy columns contain 10-50 m/s mocap spikes.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from math import atan2, degrees, hypot, isfinite, sqrt
from pathlib import Path

ANALYSIS_DIR = Path(__file__).resolve().parent.parent  # analysis/ (scripts live one level down in scripts/)
PROJECT_ROOT = ANALYSIS_DIR.parent.parent
CODEOUTPUTS = PROJECT_ROOT / "report_v1" / "results" / "codeoutputs"
FIGS_DIR = ANALYSIS_DIR / "figs"
RESULTS_DIR = ANALYSIS_DIR / "results"

RUNS = {
    "bearing_2_jyc1": CODEOUTPUTS / "bearing_2_jyc1",
    "bearing_3_jyc10": CODEOUTPUTS / "bearing_3_jyc10",
    "bearing_4_jyc1": CODEOUTPUTS / "bearing_4_jyc1",
}

# Control parameters shared by the three report runs (from metadata.json).
KP = 0.6
DEADBAND_MPS = 0.015
COMMAND_SPEED_LIMIT_MPS = 0.25
WHEEL_MIN_EFFECTIVE = 35.0


def load_trajectory(run_dir: Path):
    """vehicle_id -> list of row dicts (floats), ordered by time."""
    vehicles = defaultdict(list)
    with (run_dir / "trajectory.csv").open("r", newline="", encoding="utf-8") as src:
        for raw in csv.DictReader(src):
            vehicle_id = raw.pop("vehicle_id")
            vehicles[vehicle_id].append(
                {key: float(value) for key, value in raw.items()}
            )
    return dict(vehicles)


def load_bearing_edges(run_dir: Path):
    """(source_id, target_id) -> list of row dicts (floats), ordered by time."""
    edges = defaultdict(list)
    path = run_dir / "bearing_edges.csv"
    if not path.is_file():
        return {}
    with path.open("r", newline="", encoding="utf-8") as src:
        for raw in csv.DictReader(src):
            source_id = raw.pop("source_id")
            target_id = raw.pop("target_id")
            edges[(source_id, target_id)].append(
                {
                    key: float(value) if value else float("nan")
                    for key, value in raw.items()
                }
            )
    return dict(edges)


def position_velocity(rows, baseline_s: float = 0.4):
    """Central-difference velocity from positions (postprocess.py copy)."""
    count = len(rows)
    velocities = []
    for index in range(count):
        lo = index
        while lo > 0 and rows[index]["time_s"] - rows[lo]["time_s"] < baseline_s / 2:
            lo -= 1
        hi = index
        while hi < count - 1 and rows[hi]["time_s"] - rows[index]["time_s"] < baseline_s / 2:
            hi += 1
        dt = rows[hi]["time_s"] - rows[lo]["time_s"]
        if dt <= 0.0:
            velocities.append((0.0, 0.0))
        else:
            velocities.append(
                (
                    (rows[hi]["x_m"] - rows[lo]["x_m"]) / dt,
                    (rows[hi]["y_m"] - rows[lo]["y_m"]) / dt,
                )
            )
    return velocities


def overall_formation_rms(edges):
    """Per-timestamp RMS of every valid directed-edge bearing error (deg)."""
    per_time = defaultdict(list)
    for rows in edges.values():
        for row in rows:
            if row.get("bearing_valid", 0.0) == 1.0 and isfinite(
                row["bearing_error_deg"]
            ):
                per_time[round(row["time_s"], 6)].append(row["bearing_error_deg"])
    times = sorted(per_time)
    rms = [
        sqrt(sum(v * v for v in per_time[t]) / len(per_time[t])) for t in times
    ]
    return times, rms


def steady_window(times, duration_s: float):
    """Indices of the last ``duration_s`` seconds of the time vector."""
    if not times:
        return []
    end = times[-1]
    return [i for i, t in enumerate(times) if t >= end - duration_s]


def mean_std(values):
    values = list(values)
    n = len(values)
    if n == 0:
        return None, None
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return mean, sqrt(var)


def rms(values):
    values = list(values)
    if not values:
        return None
    return sqrt(sum(v * v for v in values) / len(values))


def undirected_edges(edges):
    """Collapse directed pairs i->j / j->i into one entry keyed (i, j) i<j.

    The angular error of a directed edge pair is identical (g_ji = -g_ij),
    so one representative per undirected edge is enough.
    """
    result = {}
    for (src, tgt), rows in edges.items():
        key = tuple(sorted((src, tgt)))
        if key not in result:
            result[key] = rows
    return result


def configurations(vehicles):
    """Per-timestamp formation configuration.

    Returns list of (time, {vehicle_id: complex(x, y)}) using timestamps where
    every vehicle has a sample (5 Hz records are aligned across vehicles).
    """
    by_time = defaultdict(dict)
    for vehicle_id, rows in vehicles.items():
        for row in rows:
            by_time[round(row["time_s"], 6)][vehicle_id] = complex(
                row["x_m"], row["y_m"]
            )
    expected = len(vehicles)
    configs = [
        (t, pts) for t, pts in sorted(by_time.items()) if len(pts) == expected
    ]
    return configs


def similarity_fit(reference, current):
    """2D similarity transform (complex): current ~= scale*e^{i*rot}*reference + t.

    Returns (rotation_rad, scale, translation_complex, residual_rms_m).
    `reference` and `current` are dicts vehicle_id -> complex with same keys.
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
        sum(
            abs(current[i] - (transform * reference[i] + translation)) ** 2
            for i in ids
        )
        / n
    )
    return rotation, scale, translation, residual


def save_results(name: str, payload: dict):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    path = RESULTS_DIR / f"{name}.json"
    with path.open("w", encoding="utf-8") as out:
        json.dump(payload, out, indent=2, ensure_ascii=False)
    print(f"results -> {path}")


def setup_figs_dir():
    FIGS_DIR.mkdir(parents=True, exist_ok=True)


def wheel_command_summary(rows, start_time: float):
    """Wheel-command statistics over rows with time_s >= start_time.

    Returns dict with counts: zero / lifted (|cmd| == min_effective) /
    above (|cmd| > min_effective), and sign flips per second (per wheel avg).
    """
    window = [r for r in rows if r["time_s"] >= start_time]
    keys = (
        "front_left_command",
        "front_right_command",
        "rear_left_command",
        "rear_right_command",
    )
    n = len(window)
    if n == 0:
        return {}
    zero = lifted = above = 0
    flips = 0.0
    for key in keys:
        seq = [r[key] for r in window]
        for value in seq:
            if value == 0.0:
                zero += 1
            elif abs(value) <= WHEEL_MIN_EFFECTIVE + 1e-9:
                lifted += 1
            else:
                above += 1
        flips += sum(
            1
            for a, b in zip(seq, seq[1:])
            if (a > 0) != (b > 0) and a != 0.0 and b != 0.0
        )
    duration = window[-1]["time_s"] - window[0]["time_s"]
    total = 4 * n
    return {
        "samples": n,
        "duration_s": duration,
        "zero_frac": zero / total,
        "lifted_frac": lifted / total,
        "above_frac": above / total,
        "sign_flips_per_s": flips / 4 / duration if duration > 0 else 0.0,
    }


def fmt_deg(value):
    return f"{degrees(value):.3f}" if value is not None else "n/a"
