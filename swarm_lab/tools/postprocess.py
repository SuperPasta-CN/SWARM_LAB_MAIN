#!/usr/bin/env python3
"""Generate compact metrics and plots from one recorded experiment run (v3).

Edge errors are relative-position errors in meters (edge_errors.csv);
steady-state statistics always use the trailing N-second window.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from math import hypot, isfinite, sqrt
from pathlib import Path
from typing import DefaultDict, Dict, List, Optional, Tuple


EdgeKey = Tuple[str, str]
VehicleRows = DefaultDict[str, List[Dict[str, float]]]
EdgeRows = DefaultDict[EdgeKey, List[Dict[str, float]]]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Post-process vehicle trajectories and directed-edge relative-position errors"
    )
    parser.add_argument(
        "run_directory",
        type=Path,
        help="directory containing trajectory.csv and edge_errors.csv",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="steady-state window in seconds (statistics over the last N "
        "seconds of the run; default 5.0)",
    )
    return parser.parse_args()


def _read_rows(path: Path) -> VehicleRows:
    vehicles: VehicleRows = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            vehicle_id = raw.pop("vehicle_id")
            vehicles[vehicle_id].append({key: float(value) for key, value in raw.items()})
    return vehicles


def _read_edge_rows(path: Path) -> EdgeRows:
    edges: EdgeRows = defaultdict(list)
    if not path.is_file():
        return edges
    with path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            source_id = raw.pop("source_id")
            target_id = raw.pop("target_id")
            edges[(source_id, target_id)].append(
                {
                    key: float(value) if value else float("nan")
                    for key, value in raw.items()
                }
            )
    return edges


def _steady_rows(rows: List[Dict[str, float]], window_s: float) -> List[Dict[str, float]]:
    """Samples inside the trailing steady-state window (all rows if shorter)."""

    if not rows:
        return []
    t_end = max(row["time_s"] for row in rows)
    windowed = [row for row in rows if row["time_s"] >= t_end - window_s]
    return windowed or list(rows)


def _steady_error_statistics(rows: List[Dict[str, float]]) -> Dict[str, object]:
    """Mean/RMS/std of the edge error (meters) over a pre-filtered window."""

    errors_m = [
        row["edge_error_m"]
        for row in rows
        if row.get("edge_valid", 0.0) == 1.0 and isfinite(row["edge_error_m"])
    ]
    if not errors_m:
        return {
            "samples": 0,
            "mean_error_m": None,
            "rms_error_m": None,
            "std_error_m": None,
        }
    mean_m = sum(errors_m) / len(errors_m)
    variance = sum((value - mean_m) ** 2 for value in errors_m) / len(errors_m)
    return {
        "samples": len(errors_m),
        "mean_error_m": mean_m,
        "rms_error_m": sqrt(sum(value * value for value in errors_m) / len(errors_m)),
        "std_error_m": sqrt(variance),
    }


def _error_statistics(rows: List[Dict[str, float]]) -> Dict[str, object]:
    errors_m = [
        row["edge_error_m"]
        for row in rows
        if row.get("edge_valid", 0.0) == 1.0 and isfinite(row["edge_error_m"])
    ]
    if errors_m:
        mean_m: Optional[float] = sum(errors_m) / len(errors_m)
        rms_m: Optional[float] = sqrt(sum(e * e for e in errors_m) / len(errors_m))
        max_m: Optional[float] = max(errors_m)
    else:
        mean_m = None
        rms_m = None
        max_m = None
    return {
        "samples": len(rows),
        "valid_samples": len(errors_m),
        "invalid_samples": len(rows) - len(errors_m),
        "mean_error_m": mean_m,
        "rms_error_m": rms_m,
        "max_error_m": max_m,
    }


def _position_velocity(rows: List[Dict[str, float]], baseline_s: float = 0.4) -> List[Tuple[float, float]]:
    """Actual velocity from recorded positions via central differences.

    The recorded ``measured_vx/vy`` columns come from the high-rate mocap
    finite-difference estimator and are dominated by differentiation noise;
    positions at 5 Hz with a ~0.4 s baseline give a far cleaner estimate of
    what the vehicle really did.
    """

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


def _metrics(
    vehicles: VehicleRows,
    edges: Optional[EdgeRows] = None,
    window_s: float = 5.0,
) -> Dict[str, object]:
    result: Dict[str, object] = {
        "edge_error_metric": "directed_edge_relative_position_error_m",
        "vehicles": {},
        "edges": {},
    }
    vehicle_metrics: Dict[str, object] = result["vehicles"]  # type: ignore
    steady_vehicles: Dict[str, object] = {}
    for vehicle_id, rows in vehicles.items():
        path_length = sum(
            hypot(current["x_m"] - previous["x_m"], current["y_m"] - previous["y_m"])
            for previous, current in zip(rows, rows[1:])
        )
        # Actual velocity comes from position central differences (the
        # recorded measured_vx/vy columns are differentiation noise).
        actual_velocity = _position_velocity(rows)
        speed_error_sq = [
            (row["target_speed_mps"] - hypot(vax, vay)) ** 2
            for row, (vax, vay) in zip(rows, actual_velocity)
        ]
        # Velocity-vector tracking error: ||v_target - v_actual|| per sample.
        velocity_error_sq = [
            (row["target_vx_mps"] - vax) ** 2 + (row["target_vy_mps"] - vay) ** 2
            for row, (vax, vay) in zip(rows, actual_velocity)
        ]
        vehicle_metrics[vehicle_id] = {
            "samples": len(rows),
            "duration_s": rows[-1]["time_s"] - rows[0]["time_s"] if len(rows) > 1 else 0.0,
            "path_length_m": path_length,
            "speed_rmse_mps": sqrt(sum(speed_error_sq) / len(speed_error_sq)) if speed_error_sq else 0.0,
            "velocity_rmse_mps": sqrt(sum(velocity_error_sq) / len(velocity_error_sq)) if velocity_error_sq else 0.0,
            "send_success_rate": (
                sum(row["command_sent"] for row in rows) / len(rows) if rows else 0.0
            ),
        }
        t_end = rows[-1]["time_s"] if rows else 0.0
        steady_velocity_error_sq = [
            error
            for row, error in zip(rows, velocity_error_sq)
            if row["time_s"] >= t_end - window_s
        ] or velocity_error_sq
        steady_vehicles[vehicle_id] = {
            "samples": len(steady_velocity_error_sq),
            "velocity_rmse_mps": (
                sqrt(sum(steady_velocity_error_sq) / len(steady_velocity_error_sq))
                if steady_velocity_error_sq
                else None
            ),
        }
    edge_rows = edges or defaultdict(list)
    edge_metrics: Dict[str, object] = result["edges"]  # type: ignore
    all_rows: List[Dict[str, float]] = []
    steady_edges: Dict[str, object] = {}
    steady_all_rows: List[Dict[str, float]] = []
    for (source_id, target_id), rows in sorted(edge_rows.items()):
        edge_metrics[source_id + "->" + target_id] = _error_statistics(rows)
        all_rows.extend(rows)
        steady_edge_rows = _steady_rows(rows, window_s)
        steady_edges[source_id + "->" + target_id] = _steady_error_statistics(
            steady_edge_rows
        )
        steady_all_rows.extend(steady_edge_rows)
    result["edge_global"] = _error_statistics(all_rows)
    # Steady-state statistics: ALWAYS report the trailing window, never the
    # whole-run mean (v1 lesson: the whole-run mean mixes the transient in
    # and is systematically inflated on short runs).
    result["steady_state"] = {
        "window_s": window_s,
        "edge_global": _steady_error_statistics(steady_all_rows),
        "edges": steady_edges,
        "vehicles": steady_vehicles,
    }
    _, overall_rms = _overall_formation_error(edge_rows)
    result["initial_rms_m"] = overall_rms[0] if overall_rms else None
    return result


def _overall_formation_error(
    edges: EdgeRows,
) -> Tuple[List[float], List[float]]:
    """RMS of every valid directed-edge position error (m) at each timestamp."""

    per_time: Dict[float, List[float]] = defaultdict(list)
    for rows in edges.values():
        for row in rows:
            if row.get("edge_valid", 0.0) == 1.0 and isfinite(row["edge_error_m"]):
                per_time[round(row["time_s"], 6)].append(row["edge_error_m"])
    times = sorted(per_time)
    rms = [
        sqrt(sum(value * value for value in per_time[t]) / len(per_time[t]))
        for t in times
    ]
    return times, rms


def _plot(
    run_directory: Path,
    vehicles: VehicleRows,
    edges: Optional[EdgeRows] = None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    edge_rows = edges or defaultdict(list)
    figure, axes_grid = plt.subplots(2, 2, figsize=(14, 9))
    trajectory_axes, error_axes = axes_grid[0]
    formation_axes, velocity_axes = axes_grid[1]

    for vehicle_id, rows in vehicles.items():
        times = [row["time_s"] for row in rows]
        trajectory_axes.plot(
            [row["x_m"] for row in rows],
            [row["y_m"] for row in rows],
            label=vehicle_id,
        )
        # Per-vehicle desired-velocity tracking error ||v_target - v_actual||,
        # with v_actual derived from recorded positions (central difference).
        actual_velocity = _position_velocity(rows)
        velocity_axes.plot(
            times,
            [
                hypot(row["target_vx_mps"] - vax, row["target_vy_mps"] - vay)
                for row, (vax, vay) in zip(rows, actual_velocity)
            ],
            label=vehicle_id,
        )

    for (source_id, target_id), rows in sorted(edge_rows.items()):
        error_axes.plot(
            [row["time_s"] for row in rows],
            [row["edge_error_m"] for row in rows],
            label=source_id + "->" + target_id,
        )

    overall_t, overall_rms = _overall_formation_error(edge_rows)
    if overall_t:
        formation_axes.plot(overall_t, overall_rms, color="black", linewidth=1.5)
        formation_axes.axhline(0.03, color="gray", linestyle="--", alpha=0.6, label="converge eps 0.03 m ref")
        formation_axes.legend(loc="best")

    trajectory_axes.set_title("2D trajectory")
    trajectory_axes.set_xlabel("x (m)")
    trajectory_axes.set_ylabel("y (m)")
    trajectory_axes.set_aspect("equal", adjustable="datalim")
    error_axes.set_title("Directed-edge relative-position error")
    error_axes.set_xlabel("time (s)")
    error_axes.set_ylabel("error (m)")
    formation_axes.set_title("Overall formation edge error (RMS over edges)")
    formation_axes.set_xlabel("time (s)")
    formation_axes.set_ylabel("error (m)")
    velocity_axes.set_title("Desired-velocity tracking error ||v* - v|| (v from positions)")
    velocity_axes.set_xlabel("time (s)")
    velocity_axes.set_ylabel("error (m/s)")
    for axes_item in (trajectory_axes, error_axes, formation_axes, velocity_axes):
        axes_item.grid(True, linestyle="--", alpha=0.4)
        handles, labels = axes_item.get_legend_handles_labels()
        if handles:
            axes_item.legend(handles, labels, loc="best")
    figure.tight_layout()
    figure.savefig(str(run_directory / "analysis.png"), dpi=160)
    plt.close(figure)


def main() -> None:
    args = _parse_args()
    run_directory = args.run_directory.resolve()
    csv_path = run_directory / "trajectory.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    vehicles = _read_rows(csv_path)
    if not vehicles:
        raise ValueError("trajectory.csv contains no vehicle samples")
    edges = _read_edge_rows(run_directory / "edge_errors.csv")
    metrics = _metrics(vehicles, edges, window_s=args.window)
    with (run_directory / "analysis.json").open("w", encoding="utf-8") as output:
        json.dump(metrics, output, indent=2, ensure_ascii=False)
    _plot(run_directory, vehicles, edges)
    print("analysis | %s" % run_directory)


if __name__ == "__main__":
    main()
