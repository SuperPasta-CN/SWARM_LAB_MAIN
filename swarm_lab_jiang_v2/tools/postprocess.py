#!/usr/bin/env python3
"""Generate compact metrics and plots from one recorded experiment run."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from math import degrees, hypot, isfinite, sqrt
from pathlib import Path
from typing import DefaultDict, Dict, List, Optional, Tuple


EdgeKey = Tuple[str, str]
VehicleRows = DefaultDict[str, List[Dict[str, float]]]
BearingEdgeRows = DefaultDict[EdgeKey, List[Dict[str, float]]]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Post-process vehicle trajectories and directed-edge bearing errors"
    )
    parser.add_argument(
        "run_directory",
        type=Path,
        help="directory containing trajectory.csv and bearing_edges.csv",
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


def _read_bearing_rows(path: Path) -> BearingEdgeRows:
    edges: BearingEdgeRows = defaultdict(list)
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


def _steady_angle_statistics(rows: List[Dict[str, float]]) -> Dict[str, object]:
    """Mean/RMS/std of the bearing error over a pre-filtered window."""

    errors_deg = [
        row["bearing_error_deg"]
        for row in rows
        if row.get("bearing_valid", 0.0) == 1.0 and isfinite(row["bearing_error_deg"])
    ]
    if not errors_deg:
        return {
            "samples": 0,
            "mean_error_deg": None,
            "rms_error_deg": None,
            "std_error_deg": None,
        }
    mean_deg = sum(errors_deg) / len(errors_deg)
    variance = sum((value - mean_deg) ** 2 for value in errors_deg) / len(errors_deg)
    return {
        "samples": len(errors_deg),
        "mean_error_deg": mean_deg,
        "rms_error_deg": sqrt(sum(value * value for value in errors_deg) / len(errors_deg)),
        "std_error_deg": sqrt(variance),
    }


def _angle_statistics(rows: List[Dict[str, float]]) -> Dict[str, object]:
    angles_rad = [
        row["bearing_error_rad"]
        for row in rows
        if row.get("bearing_valid", 0.0) == 1.0
        and isfinite(row["bearing_error_rad"])
    ]
    if angles_rad:
        mean_rad: Optional[float] = sum(angles_rad) / len(angles_rad)
        rms_rad: Optional[float] = sqrt(
            sum(angle * angle for angle in angles_rad) / len(angles_rad)
        )
        max_rad: Optional[float] = max(angles_rad)
    else:
        mean_rad = None
        rms_rad = None
        max_rad = None
    return {
        "samples": len(rows),
        "valid_samples": len(angles_rad),
        "invalid_samples": len(rows) - len(angles_rad),
        "mean_error_rad": mean_rad,
        "rms_error_rad": rms_rad,
        "max_error_rad": max_rad,
        "mean_error_deg": degrees(mean_rad) if mean_rad is not None else None,
        "rms_error_deg": degrees(rms_rad) if rms_rad is not None else None,
        "max_error_deg": degrees(max_rad) if max_rad is not None else None,
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
    bearing_edges: Optional[BearingEdgeRows] = None,
    window_s: float = 5.0,
) -> Dict[str, object]:
    result: Dict[str, object] = {
        "bearing_error_metric": "directed_edge_angular_error_3d",
        "vehicles": {},
        "bearing_edges": {},
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
    edge_rows = bearing_edges or defaultdict(list)
    edge_metrics: Dict[str, object] = result["bearing_edges"]  # type: ignore
    all_rows: List[Dict[str, float]] = []
    steady_edges: Dict[str, object] = {}
    steady_all_rows: List[Dict[str, float]] = []
    for (source_id, target_id), rows in sorted(edge_rows.items()):
        edge_metrics[source_id + "->" + target_id] = _angle_statistics(rows)
        all_rows.extend(rows)
        steady_edge_rows = _steady_rows(rows, window_s)
        steady_edges[source_id + "->" + target_id] = _steady_angle_statistics(
            steady_edge_rows
        )
        steady_all_rows.extend(steady_edge_rows)
    result["bearing_global"] = _angle_statistics(all_rows)
    # Steady-state statistics: ALWAYS report the trailing window, never the
    # whole-run mean (v1 lesson: the whole-run mean mixes the transient in
    # and is systematically inflated on short runs).
    result["steady_state"] = {
        "window_s": window_s,
        "bearing_global": _steady_angle_statistics(steady_all_rows),
        "bearing_edges": steady_edges,
        "vehicles": steady_vehicles,
    }
    _, overall_rms = _overall_formation_error(edge_rows)
    result["initial_rms_deg"] = overall_rms[0] if overall_rms else None
    return result


def _overall_formation_error(
    bearing_edges: BearingEdgeRows,
) -> Tuple[List[float], List[float]]:
    """RMS of every valid directed-edge bearing error at each timestamp.

    This is the whole-formation tracking error: how far the current
    configuration is from the desired bearing shape, as one scalar curve.
    """

    per_time: Dict[float, List[float]] = defaultdict(list)
    for rows in bearing_edges.values():
        for row in rows:
            if row.get("bearing_valid", 0.0) == 1.0 and isfinite(row["bearing_error_deg"]):
                per_time[round(row["time_s"], 6)].append(row["bearing_error_deg"])
    times = sorted(per_time)
    rms = [
        sqrt(sum(value * value for value in per_time[t]) / len(per_time[t]))
        for t in times
    ]
    return times, rms


def _plot(
    run_directory: Path,
    vehicles: VehicleRows,
    bearing_edges: Optional[BearingEdgeRows] = None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    edge_rows = bearing_edges or defaultdict(list)
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
            [row["bearing_error_deg"] for row in rows],
            label=source_id + "->" + target_id,
        )

    overall_t, overall_rms = _overall_formation_error(edge_rows)
    if overall_t:
        formation_axes.plot(overall_t, overall_rms, color="black", linewidth=1.5)
        formation_axes.axhline(5.0, color="gray", linestyle="--", alpha=0.6, label="5 deg ref")
        formation_axes.legend(loc="best")

    trajectory_axes.set_title("2D trajectory")
    trajectory_axes.set_xlabel("x (m)")
    trajectory_axes.set_ylabel("y (m)")
    trajectory_axes.set_aspect("equal", adjustable="datalim")
    error_axes.set_title("Directed-edge bearing angle error")
    error_axes.set_xlabel("time (s)")
    error_axes.set_ylabel("error (deg)")
    formation_axes.set_title("Overall formation bearing error (RMS over edges)")
    formation_axes.set_xlabel("time (s)")
    formation_axes.set_ylabel("error (deg)")
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
    bearing_edges = _read_bearing_rows(run_directory / "bearing_edges.csv")
    metrics = _metrics(vehicles, bearing_edges, window_s=args.window)
    with (run_directory / "analysis.json").open("w", encoding="utf-8") as output:
        json.dump(metrics, output, indent=2, ensure_ascii=False)
    _plot(run_directory, vehicles, bearing_edges)
    print("analysis | %s" % run_directory)


if __name__ == "__main__":
    main()
