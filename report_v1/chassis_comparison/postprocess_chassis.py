#!/usr/bin/env python3
"""Steady-state velocity-residual comparison across chassis-mode runs.

Reads one or more run directories (each with trajectory.csv as recorded by
swarm_lab_jiang_v1) and reports chassis-layer tracking quality inside a
steady-state window:

- actual velocity is rebuilt from recorded positions via central
  differences (0.4 s baseline), matching swarm_lab_jiang_v1/tools/
  postprocess.py::_position_velocity — the recorded measured_vx/vy columns
  contain mocap differentiation noise and must not be used directly;
- the default window keeps samples whose target speed is at least half
  the run's peak target speed (the active step) and takes the last 60%
  of them (steady state, transient excluded); override with
  --t-start/--t-end (seconds, relative to the run's first record);
- per vehicle: mean target vs actual velocity, residual vector, speed
  residual (absolute and %), direction error, and ||v* - v|| RMSE;
- if the run's summary.json carries bearing statistics (formation runs),
  they are printed as the formation-layer reference.

Typical use (chassis_step run once per execution.mode omni/omni_pid/diff):

    python report_v1/chassis_comparison/postprocess_chassis.py \
        swarm_lab_jiang_v1/runs/chassis_step_<t1> \
        swarm_lab_jiang_v1/runs/chassis_step_<t2> \
        swarm_lab_jiang_v1/runs/chassis_step_<t3>

A parent directory may be passed instead: every child containing a
trajectory.csv is included (sorted by name, which is timestamped).
Pure standard library; --json PATH also dumps all numbers.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from math import atan2, degrees, hypot
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare steady-state velocity residuals across run directories"
    )
    parser.add_argument(
        "paths",
        nargs="+",
        type=Path,
        help="run directories (with trajectory.csv) or parents containing them",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="steady-window start, seconds relative to the run's first record",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="steady-window end, seconds relative to the run's first record",
    )
    parser.add_argument("--json", type=Path, default=None, help="also dump metrics to this file")
    return parser.parse_args()


def _iter_run_dirs(paths: List[Path]) -> List[Path]:
    run_dirs: List[Path] = []
    for path in paths:
        if (path / "trajectory.csv").is_file():
            run_dirs.append(path)
        elif path.is_dir():
            run_dirs.extend(
                child
                for child in sorted(path.iterdir())
                if child.is_dir() and (child / "trajectory.csv").is_file()
            )
        else:
            print("warning | skipped (no trajectory.csv): %s" % path, file=sys.stderr)
    return run_dirs


def _read_rows(csv_path: Path) -> Dict[str, List[Dict[str, float]]]:
    vehicles: Dict[str, List[Dict[str, float]]] = {}
    with csv_path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            vehicle_id = raw.pop("vehicle_id")
            vehicles.setdefault(vehicle_id, []).append(
                {key: float(value) for key, value in raw.items()}
            )
    return vehicles


def _position_velocity(
    rows: List[Dict[str, float]], baseline_s: float = 0.4
) -> List[Tuple[float, float]]:
    """Central-difference velocity from positions (postprocess.py copy)."""

    count = len(rows)
    velocities: List[Tuple[float, float]] = []
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


def _steady_window(
    rows: List[Dict[str, float]],
    t_start: Optional[float],
    t_end: Optional[float],
) -> List[int]:
    """Indices of the steady-state samples within one vehicle's records."""

    if t_start is not None or t_end is not None:
        start = t_start if t_start is not None else float("-inf")
        end = t_end if t_end is not None else float("inf")
        return [i for i, row in enumerate(rows) if start <= row["time_s"] <= end]
    # Auto window: samples at >= 50% of the peak target speed (active step),
    # last 60% of them (transient excluded).
    target_speeds = [hypot(row["target_vx_mps"], row["target_vy_mps"]) for row in rows]
    peak = max(target_speeds) if target_speeds else 0.0
    if peak < 1e-9:
        return list(range(len(rows)))
    active = [i for i, speed in enumerate(target_speeds) if speed >= 0.5 * peak]
    cutoff = active[0] + int(len(active) * 0.4)
    return [i for i in active if i >= cutoff]


def _mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _vehicle_metrics(
    rows: List[Dict[str, float]],
    window: List[int],
) -> Dict[str, object]:
    actual = _position_velocity(rows)
    picked = [rows[i] for i in window]
    picked_actual = [actual[i] for i in window]
    if not picked:
        return {"samples": 0}
    mean_tx = _mean([row["target_vx_mps"] for row in picked])
    mean_ty = _mean([row["target_vy_mps"] for row in picked])
    mean_ax = _mean([vx for vx, _ in picked_actual])
    mean_ay = _mean([vy for _, vy in picked_actual])
    mean_target_speed = _mean(
        [hypot(row["target_vx_mps"], row["target_vy_mps"]) for row in picked]
    )
    mean_actual_speed = _mean([hypot(vx, vy) for vx, vy in picked_actual])
    speed_residual = mean_actual_speed - mean_target_speed
    direction_error_deg = degrees(
        atan2(mean_tx * mean_ay - mean_ty * mean_ax, mean_tx * mean_ax + mean_ty * mean_ay)
    )
    rmse = (
        _mean(
            [
                (row["target_vx_mps"] - vx) ** 2 + (row["target_vy_mps"] - vy) ** 2
                for row, (vx, vy) in zip(picked, picked_actual)
            ]
        )
        ** 0.5
    )
    return {
        "samples": len(picked),
        "window_s": [picked[0]["time_s"], picked[-1]["time_s"]],
        "mean_target_mps": [mean_tx, mean_ty],
        "mean_actual_mps": [mean_ax, mean_ay],
        "residual_mps": [mean_ax - mean_tx, mean_ay - mean_ty],
        "mean_target_speed_mps": mean_target_speed,
        "mean_actual_speed_mps": mean_actual_speed,
        "speed_residual_mps": speed_residual,
        "speed_residual_pct": (
            100.0 * speed_residual / mean_target_speed if mean_target_speed > 1e-9 else None
        ),
        "direction_error_deg": direction_error_deg,
        "velocity_rmse_mps": rmse,
    }


def _formation_summary(run_dir: Path) -> Optional[Dict[str, object]]:
    summary_path = run_dir / "summary.json"
    if not summary_path.is_file():
        return None
    with summary_path.open("r", encoding="utf-8") as source:
        summary = json.load(source)
    if summary.get("mean_bearing_error_deg") is None:
        return None
    return {
        "mean_bearing_error_deg": summary["mean_bearing_error_deg"],
        "rms_bearing_error_deg": summary["rms_bearing_error_deg"],
        "max_bearing_error_deg": summary["max_bearing_error_deg"],
    }


def _analyze_run(
    run_dir: Path,
    t_start: Optional[float],
    t_end: Optional[float],
) -> Dict[str, object]:
    vehicles = _read_rows(run_dir / "trajectory.csv")
    result: Dict[str, object] = {"run": str(run_dir), "vehicles": {}}
    vehicle_metrics: Dict[str, object] = result["vehicles"]  # type: ignore[assignment]
    for vehicle_id, rows in sorted(vehicles.items()):
        window = _steady_window(rows, t_start, t_end)
        vehicle_metrics[vehicle_id] = _vehicle_metrics(rows, window)
    formation = _formation_summary(run_dir)
    if formation is not None:
        result["formation"] = formation
    return result


def _print_run(report: Dict[str, object]) -> None:
    print("run | %s" % report["run"])
    for vehicle_id, metrics in sorted(report["vehicles"].items()):  # type: ignore[union-attr]
        if not metrics.get("samples"):
            print("  %-8s no samples in steady window" % vehicle_id)
            continue
        tx, ty = metrics["mean_target_mps"]
        ax, ay = metrics["mean_actual_mps"]
        rx, ry = metrics["residual_mps"]
        pct = metrics["speed_residual_pct"]
        print(
            "  %-8s window %.1f-%.1f s  n=%d\n"
            "      target=(%+.4f,%+.4f) m/s  actual=(%+.4f,%+.4f) m/s  residual=(%+.4f,%+.4f) m/s\n"
            "      speed %.4f -> %.4f m/s  residual %+.4f m/s (%s)  "
            "dir_err %+.2f deg  rmse %.4f m/s"
            % (
                vehicle_id,
                metrics["window_s"][0],
                metrics["window_s"][1],
                metrics["samples"],
                tx,
                ty,
                ax,
                ay,
                rx,
                ry,
                metrics["mean_target_speed_mps"],
                metrics["mean_actual_speed_mps"],
                metrics["speed_residual_mps"],
                "%+.1f%%" % pct if pct is not None else "n/a",
                metrics["direction_error_deg"],
                metrics["velocity_rmse_mps"],
            )
        )
    formation = report.get("formation")
    if formation is not None:
        print(
            "  formation | mean %.3f deg  rms %.3f deg  max %.3f deg"
            % (
                formation["mean_bearing_error_deg"],
                formation["rms_bearing_error_deg"],
                formation["max_bearing_error_deg"],
            )
        )


def _print_comparison(reports: List[Dict[str, object]]) -> None:
    vehicle_ids = sorted(
        {vid for report in reports for vid in report["vehicles"]}  # type: ignore[union-attr]
    )
    for vehicle_id in vehicle_ids:
        print("compare | %s" % vehicle_id)
        header = "  %-44s %12s %10s %10s %12s" % (
            "run",
            "residual",
            "rel.",
            "dir_err",
            "rmse",
        )
        print(header)
        for report in reports:
            metrics = report["vehicles"].get(vehicle_id)  # type: ignore[union-attr]
            if not metrics or not metrics.get("samples"):
                continue
            pct = metrics["speed_residual_pct"]
            print(
                "  %-44s %+10.4f m/s %9s %+8.2f deg %10.4f m/s"
                % (
                    Path(str(report["run"])).name,
                    metrics["speed_residual_mps"],
                    "%+.1f%%" % pct if pct is not None else "n/a",
                    metrics["direction_error_deg"],
                    metrics["velocity_rmse_mps"],
                )
            )


def main() -> None:
    args = _parse_args()
    run_dirs = _iter_run_dirs(args.paths)
    if not run_dirs:
        raise SystemExit("no run directories with trajectory.csv found")
    reports = [_analyze_run(run_dir, args.t_start, args.t_end) for run_dir in run_dirs]
    for report in reports:
        _print_run(report)
        print()
    if len(reports) > 1:
        _print_comparison(reports)
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        with args.json.open("w", encoding="utf-8") as output:
            json.dump(reports, output, indent=2, ensure_ascii=False)
        print("json -> %s" % args.json)


if __name__ == "__main__":
    main()
