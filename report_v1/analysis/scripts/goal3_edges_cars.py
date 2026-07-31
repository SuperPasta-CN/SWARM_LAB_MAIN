"""Goal 3: per-edge error asymmetry and per-vehicle differences (focus car1).

Analyses (steady window = last 5 s unless stated):
1. Per-undirected-edge bearing error: unsigned mean/std plus SIGNED mean
   (rotation direction of actual vs desired bearing) to expose systematic
   sub-formation rotation.
2. Best-fit desired shape: place desired bearings as a rigid shape via the
   similarity fit to the actual steady-state configuration; per-vehicle
   displacement residual shows WHO is off and WHERE.
3. Per-vehicle velocity tracking: ||v_target - v_actual|| with v_actual from
   position central differences (0.4 s baseline); steady-window RMSE and a
   responsiveness ratio mean(|v_actual|)/mean(|v_target|) over active samples.
4. Heading hold: yaw - target_heading mean/std (constant offsets and the
   min_effective-induced limit cycle).
5. Position dither amplitude sigma_pos per vehicle; the genuinely static
   vehicle (bearing_4 car1) provides the pure mocap noise floor.
"""

from __future__ import annotations

from collections import defaultdict
from math import atan2, cos, degrees, hypot, sin, sqrt

import common

STEADY_S = 5.0
TRANSIENT_S = 5.0


def signed_edge_errors(edges, t_end):
    """Signed angular error (deg, actual w.r.t. desired) per undirected edge."""
    result = {}
    for (a, b), rows in common.undirected_edges(edges).items():
        signed, unsigned = [], []
        for r in rows:
            if r["time_s"] < t_end - STEADY_S or r.get("bearing_valid", 0.0) != 1.0:
                continue
            cross = r["desired_x"] * r["actual_y"] - r["desired_y"] * r["actual_x"]
            dot = r["desired_x"] * r["actual_x"] + r["desired_y"] * r["actual_y"]
            angle = degrees(atan2(cross, dot))
            # i->j and j->i give the same signed angle only up to sign of the
            # frame; use the stored (a->b) orientation consistently.
            signed.append(angle)
            unsigned.append(abs(angle))
        smean, sstd = common.mean_std(signed)
        umean, _ = common.mean_std(unsigned)
        result[f"{a}-{b}"] = {
            "signed_mean_deg": smean,
            "signed_std_deg": sstd,
            "unsigned_mean_deg": umean,
        }
    return result


def desired_shape_fit(vehicles, edges, t_end):
    """Fit desired bearing shape to the steady-state actual configuration.

    Builds the desired shape by solving relative positions from desired unit
    bearings with edge lengths taken from the actual configuration, then
    similarity-fits it to the actual mean positions.  Per-vehicle residual
    displacement (mm) localises the geometric error.
    """
    # Mean actual positions and edge lengths in the steady window.
    mean_pos = {}
    for vehicle_id, rows in vehicles.items():
        tail = [r for r in rows if r["time_s"] >= t_end - STEADY_S]
        mean_pos[vehicle_id] = complex(
            sum(r["x_m"] for r in tail) / len(tail),
            sum(r["y_m"] for r in tail) / len(tail),
        )
    lengths = {}
    undirected = common.undirected_edges(edges)
    for (a, b) in undirected:
        lengths[(a, b)] = abs(mean_pos[a] - mean_pos[b])

    # Reconstruct desired shape: BFS from the first vehicle, walking along
    # desired bearings with actual edge lengths (least-squares free: edges in
    # these topologies form trees plus redundant constraints; use order of
    # discovery and average multiple inferences).
    desired_dir = {}
    for (a, b), rows in undirected.items():
        r = rows[-1]
        desired_dir[(a, b)] = complex(r["desired_x"], r["desired_y"])
        desired_dir[(b, a)] = complex(-r["desired_x"], -r["desired_y"])
    ids = list(mean_pos)
    origin = ids[0]
    estimates = defaultdict(list)
    estimates[origin].append(complex(0, 0))
    queue = [origin]
    while queue:
        cur = queue.pop(0)
        for nxt in ids:
            if nxt == cur or (cur, nxt) not in desired_dir:
                continue
            for base in estimates[cur]:
                estimates[nxt].append(
                    base + desired_dir[(cur, nxt)] * lengths[tuple(sorted((cur, nxt)))]
                )
            if nxt not in queue and len(estimates[nxt]) == 1:
                queue.append(nxt)
    desired = {}
    for vehicle_id in ids:
        vals = estimates[vehicle_id]
        desired[vehicle_id] = sum(vals) / len(vals)

    rotation, scale, translation, fit_rms = common.similarity_fit(desired, mean_pos)
    transform = scale * complex(cos(rotation), sin(rotation))
    per_vehicle = {}
    for vehicle_id in ids:
        fitted = transform * desired[vehicle_id] + translation
        delta = mean_pos[vehicle_id] - fitted
        per_vehicle[vehicle_id] = {
            "residual_mm": abs(delta) * 1000.0,
            "residual_x_mm": delta.real * 1000.0,
            "residual_y_mm": delta.imag * 1000.0,
            "actual": (mean_pos[vehicle_id].real, mean_pos[vehicle_id].imag),
            "desired_fitted": (fitted.real, fitted.imag),
        }
    return {
        "rotation_deg": degrees(rotation),
        "scale": scale,
        "fit_rms_mm": fit_rms * 1000.0,
        "per_vehicle": per_vehicle,
    }


def velocity_tracking(vehicles, t_end):
    """Per-vehicle velocity tracking stats; steady and transient windows."""
    stats = {}
    for vehicle_id, rows in vehicles.items():
        actual = common.position_velocity(rows)
        records = []
        for r, (vax, vay) in zip(rows, actual):
            vtx, vty = r["target_vx_mps"], r["target_vy_mps"]
            records.append(
                {
                    "t": r["time_s"],
                    "err": hypot(vtx - vax, vty - vay),
                    "target": hypot(vtx, vty),
                    "actual_speed": hypot(vax, vay),
                }
            )
        steady = [x for x in records if x["t"] >= t_end - STEADY_S]
        transient = [x for x in records if x["t"] <= records[0]["t"] + TRANSIENT_S]
        active = [x for x in records if x["target"] > 0.05]
        stats[vehicle_id] = {
            "steady_rmse_mps": common.rms(x["err"] for x in steady),
            "steady_mean_target_mps": common.mean_std(x["target"] for x in steady)[0],
            "steady_mean_actual_mps": common.mean_std(
                x["actual_speed"] for x in steady
            )[0],
            "transient_peak_err_mps": max(x["err"] for x in transient),
            "active_ratio_actual_over_target": (
                sum(x["actual_speed"] for x in active)
                / sum(x["target"] for x in active)
                if active
                else None
            ),
            "whole_run_rmse_mps": common.rms(x["err"] for x in records),
            "series_t": [x["t"] for x in records],
            "series_err": [x["err"] for x in records],
        }
    return stats


def heading_hold(vehicles, t_end):
    """Yaw minus target heading: mean offset and oscillation (limit cycle)."""
    stats = {}
    for vehicle_id, rows in vehicles.items():
        tail = [r for r in rows if r["time_s"] >= t_end - STEADY_S]
        errs = [
            atan2(
                sin(r["yaw_rad"] - r["target_heading_rad"]),
                cos(r["yaw_rad"] - r["target_heading_rad"]),
            )
            for r in tail
        ]
        mean, std = common.mean_std(errs)
        stats[vehicle_id] = {
            "mean_offset_deg": degrees(mean),
            "std_deg": degrees(std),
            "peak_to_peak_deg": degrees(max(errs) - min(errs)),
            "series_t": [r["time_s"] for r in tail],
            "series_deg": [degrees(e) for e in errs],
        }
    return stats


def dither_amplitude(vehicles, t_end):
    """sigma_pos per vehicle in the steady window (real dither + mocap)."""
    stats = {}
    for vehicle_id, rows in vehicles.items():
        tail = [r for r in rows if r["time_s"] >= t_end - STEADY_S]
        sx = common.mean_std([r["x_m"] for r in tail])[1]
        sy = common.mean_std([r["y_m"] for r in tail])[1]
        stats[vehicle_id] = {
            "sigma_x_mm": sx * 1000.0,
            "sigma_y_mm": sy * 1000.0,
            "sigma_pos_mm": sqrt((sx * sx + sy * sy) / 2) * 1000.0,
        }
    return stats


def main():
    common.setup_figs_dir()
    results = {}
    vel_series, head_series = {}, {}
    for name in common.RUNS:
        run_dir = common.RUNS[name]
        vehicles = common.load_trajectory(run_dir)
        edges = common.load_bearing_edges(run_dir)
        times, _ = common.overall_formation_rms(edges)
        t_end = times[-1]
        run = {
            "signed_edges": signed_edge_errors(edges, t_end),
            "shape_fit": desired_shape_fit(vehicles, edges, t_end),
            "velocity": velocity_tracking(vehicles, t_end),
            "heading": heading_hold(vehicles, t_end),
            "dither": dither_amplitude(vehicles, t_end),
        }
        results[name] = run
        vel_series[name] = {v: (s["series_t"], s["series_err"]) for v, s in run["velocity"].items()}
        head_series[name] = {v: (s["series_t"], s["series_deg"]) for v, s in run["heading"].items()}

        print(f"=== {name} ===")
        for edge, s in run["signed_edges"].items():
            print(
                f"  edge {edge}: unsigned={s['unsigned_mean_deg']:.2f}deg "
                f"signed={s['signed_mean_deg']:+.2f}deg std={s['signed_std_deg']:.2f}"
            )
        sf = run["shape_fit"]
        print(
            f"  shape fit: rot={sf['rotation_deg']:+.2f}deg scale={sf['scale']:.4f} "
            f"rms={sf['fit_rms_mm']:.1f}mm"
        )
        for v, s in sf["per_vehicle"].items():
            print(
                f"    {v}: residual={s['residual_mm']:.1f}mm "
                f"(dx={s['residual_x_mm']:+.1f}, dy={s['residual_y_mm']:+.1f})"
            )
        for v, s in run["velocity"].items():
            ratio = s["active_ratio_actual_over_target"]
            print(
                f"  {v}: steady_vel_rmse={s['steady_rmse_mps']:.3f} "
                f"whole_run={s['whole_run_rmse_mps']:.3f} "
                f"transient_peak={s['transient_peak_err_mps']:.3f} "
                f"ratio={f'{ratio:.2f}' if ratio is not None else 'n/a'}"
            )
        for v, s in run["heading"].items():
            print(
                f"  {v}: heading offset={s['mean_offset_deg']:+.2f}deg "
                f"std={s['std_deg']:.2f} p2p={s['peak_to_peak_deg']:.2f}"
            )
        for v, s in run["dither"].items():
            print(f"  {v}: sigma_pos={s['sigma_pos_mm']:.2f}mm")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # fig3a: per-edge steady mean error (unsigned), grouped by run.
    fig, ax = plt.subplots(figsize=(10, 5))
    runs = list(common.RUNS)
    all_edges = []
    for name in runs:
        for edge in results[name]["signed_edges"]:
            all_edges.append((name, edge))
    width = 0.8 / max(len(results[n]["signed_edges"]) for n in runs)
    colors = {"bearing_2_jyc1": "C0", "bearing_3_jyc10": "C1", "bearing_4_jyc1": "C2"}
    xticks, xlabels = [], []
    pos = 0.0
    for name in runs:
        edges = results[name]["signed_edges"]
        for i, (edge, s) in enumerate(sorted(edges.items())):
            ax.bar(
                pos,
                s["unsigned_mean_deg"],
                width * 0.9,
                color=colors[name],
                yerr=s["signed_std_deg"],
                capsize=2,
            )
            xticks.append(pos)
            xlabels.append(edge.replace("car", ""))
            pos += width
        pos += width  # gap between runs
    ax.set_xticks(xticks)
    ax.set_xticklabels(xlabels, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("steady mean |bearing error| (deg)")
    ax.set_title("Per-edge steady-state error (last 5 s, whiskers = std)")
    handles = [plt.Rectangle((0, 0), 1, 1, color=colors[n]) for n in runs]
    ax.legend(handles, runs)
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig3a_edge_asymmetry.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    # fig3b: steady configuration vs best-fit desired shape (per run).
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, name in zip(axes, runs):
        sf = results[name]["shape_fit"]
        for v, s in sf["per_vehicle"].items():
            ax.plot(*s["actual"], "o", markersize=10, label=v)
            ax.plot(*s["desired_fitted"], "x", markersize=8, color="gray")
            ax.annotate(
                "",
                xy=s["actual"],
                xytext=s["desired_fitted"],
                arrowprops=dict(arrowstyle="->", color="red", alpha=0.6),
            )
            ax.annotate(v, s["actual"], fontsize=8)
        ax.set_title(f"{name}\nfit rms={sf['fit_rms_mm']:.1f}mm")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    fig.suptitle("Steady configuration (dots) vs best-fit desired shape (x)")
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig3b_shape_fit.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    # fig3c: velocity tracking error time series.
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
    for ax, name in zip(axes, runs):
        for v, (t, err) in vel_series[name].items():
            ax.plot(t, err, label=v, linewidth=1.0)
        ax.set_title(name)
        ax.set_xlabel("time (s)")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(fontsize=8)
    axes[0].set_ylabel("||v_target - v_actual|| (m/s)")
    fig.suptitle("Velocity tracking error (actual from position differences)")
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig3c_velocity_tracking.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    # fig3d: heading-hold error in the steady window.
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
    for ax, name in zip(axes, runs):
        for v, (t, err) in head_series[name].items():
            ax.plot(t, err, label=v, linewidth=1.0)
        ax.set_title(name)
        ax.set_xlabel("time (s)")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(fontsize=8)
    axes[0].set_ylabel("yaw - target heading (deg)")
    fig.suptitle("Heading-hold error in steady window (limit cycle visible)")
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig3d_heading_hold.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    # Drop series before saving JSON.
    for name in results:
        for section in ("velocity", "heading"):
            for v in results[name][section]:
                results[name][section][v] = {
                    k: val for k, val in results[name][section][v].items()
                    if not k.startswith("series")
                }
    common.save_results("goal3_edges_cars", results)


if __name__ == "__main__":
    main()
