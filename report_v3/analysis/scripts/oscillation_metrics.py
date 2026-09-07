#!/usr/bin/env python3
"""Per-run oscillation metrics for the dead-zone A/B experiments.

Usage: python oscillation_metrics.py <run_directory> [--window 10]

Prints the four judgment metrics of report_v3/oscillation_analysis.md §5:
steady-state edge error (last N s), per-car wheel sign-flip rate, speed
ripple (median), yaw wobble (median), and the lifted-command fraction.
Velocity is rebuilt from positions (0.4 s central difference); the cruise
phase is the interval where the mean commanded speed exceeds half of w.
"""

from __future__ import annotations

import argparse
import sys
from math import degrees, hypot
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

WHEELS = ("front_left_command", "front_right_command",
          "rear_left_command", "rear_right_command")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--window", type=float, default=10.0)
    args = parser.parse_args()
    run_dir = args.run_directory.resolve()

    vehicles = common.load_trajectory(run_dir)
    edges = common.load_v3_edges(run_dir)
    et, er = common.overall_rms(
        [[(r["time_s"], r["edge_error_m"]) for r in rows] for rows in edges.values()]
    )
    t_end = et[-1]
    steady = [v for t, v in zip(et, er) if t >= t_end - args.window]
    mean_e, std_e = common.mean_std(steady)
    print("run | %s" % run_dir.name)
    print("  稳态边误差（末 %.0f s）: %.4f ± %.4f m   [基线 0.022-0.030, 合格线 ≤0.04]"
          % (args.window, mean_e, std_e))

    # cruise phase: commanded speed > half of w
    cruise = None
    for vid in sorted(vehicles):
        rows = vehicles[vid]
        cruise = cruise or common.active_window(vehicles, 0.10)
        t0, t1 = cruise
        act = [r for r in rows if t0 <= r["time_s"] <= t1]
        if len(act) < 10:
            continue
        dur = act[-1]["time_s"] - act[0]["time_s"]
        flips = 0
        lifted = 0
        for key in WHEELS:
            seq = [r[key] for r in act]
            flips += sum(1 for a, b in zip(seq, seq[1:])
                         if (a > 0) != (b > 0) and a != 0.0 and b != 0.0)
            lifted += sum(1 for v in seq if abs(abs(v) - 35.0) < 0.51)
        speeds = [hypot(vx, vy) for _, vx, vy in common.velocity_series(act, 0.4)]
        mean_s = sum(speeds) / len(speeds)
        ripple = [abs(s - mean_s) / mean_s for s in speeds] if mean_s > 1e-9 else [0.0]
        yaw_deg = [degrees(r["yaw_rad"]) for r in act]
        yaw_mean = sum(yaw_deg) / len(yaw_deg)
        wobble = [abs(y - yaw_mean) for y in yaw_deg]
        print("  %-5s 换向 %.1f 次/s/轮 [基线 ~2.4] | 速度波纹中位 %2.0f%% [基线 31-77] | "
              "yaw 摆动中位 %.1f° [基线 1.6-11.2] | 抬升占比 %.0f%%" % (
                  vid, flips / 4 / max(dur, 1e-9),
                  100 * common.percentile(ripple, 0.5),
                  common.percentile(wobble, 0.5),
                  100 * lifted / (4 * len(act))))


if __name__ == "__main__":
    main()
