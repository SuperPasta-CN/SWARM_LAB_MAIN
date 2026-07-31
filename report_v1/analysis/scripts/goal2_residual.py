"""Goal 2: steady-state residual source decomposition.

Components:
1. Theoretical deadband residual: the controller zeroes the command when
   v_max*tanh(|u|/v_max) < deadband, i.e. |u| < v_max*atanh(deadband/v_max).
   For a single edge |u| = kp*sin(theta), so theta_db = asin(u_thr/kp).
   With several edges per vehicle the deadband applies to the VECTOR SUM,
   so per-edge residuals can exceed theta_db when edge errors cancel.
2. Mocap position noise: estimated from standstill samples (zero target speed
   AND zero wheel commands) via per-axis position std, cross-checked with a
   second-difference estimator sigma(DD2 p)/sqrt(6).
3. Equivalent bearing measurement noise per edge: sqrt(s_i^2 + s_j^2)/L.
4. Deadzone-compensation (min_effective=35) dithering: wheel command
   zero/lifted/above fractions and sign flips in the steady window.
5. Reconstructed per-vehicle command norm |u_i| = |kp * sum P_ij e_ij| in the
   steady window (from recorded desired/actual bearings) to verify the
   deadband-on-vector-sum equilibrium.
"""

from __future__ import annotations

from collections import defaultdict
from math import asin, atanh, degrees, hypot, radians, sqrt

import common

V_MAX = common.COMMAND_SPEED_LIMIT_MPS
KP = common.KP
DEADBAND = common.DEADBAND_MPS
U_THR = V_MAX * atanh(DEADBAND / V_MAX)  # command zeroed below this |u|
THETA_DB_DEG = degrees(asin(U_THR / KP))  # single-edge equivalent
STEADY_S = 5.0


def standstill_noise(vehicles):
    """Mocap position noise from samples with zero target and zero commands."""
    wheel_keys = (
        "front_left_command",
        "front_right_command",
        "rear_left_command",
        "rear_right_command",
    )
    noise = {}
    for vehicle_id, rows in vehicles.items():
        xs = [
            r["x_m"]
            for r in rows
            if r["target_speed_mps"] == 0.0
            and all(r[k] == 0.0 for k in wheel_keys)
        ]
        ys = [
            r["y_m"]
            for r in rows
            if r["target_speed_mps"] == 0.0
            and all(r[k] == 0.0 for k in wheel_keys)
        ]
        if len(xs) >= 20:
            sx = common.mean_std(xs)[1]
            sy = common.mean_std(ys)[1]
            noise[vehicle_id] = {
                "samples": len(xs),
                "sigma_x_m": sx,
                "sigma_y_m": sy,
                "sigma_pos_m": sqrt((sx * sx + sy * sy) / 2),
            }
        else:
            # Fallback: second-difference estimator over the last 5 s.
            tail = [r for r in rows if r["time_s"] >= rows[-1]["time_s"] - STEADY_S]
            dd2x = [
                tail[i + 1]["x_m"] - 2 * tail[i]["x_m"] + tail[i - 1]["x_m"]
                for i in range(1, len(tail) - 1)
            ]
            dd2y = [
                tail[i + 1]["y_m"] - 2 * tail[i]["y_m"] + tail[i - 1]["y_m"]
                for i in range(1, len(tail) - 1)
            ]
            sx = common.mean_std(dd2x)[1] / sqrt(6) if dd2x else None
            sy = common.mean_std(dd2y)[1] / sqrt(6) if dd2y else None
            noise[vehicle_id] = {
                "samples": len(xs),
                "method": "second_difference",
                "sigma_x_m": sx,
                "sigma_y_m": sy,
                "sigma_pos_m": sqrt((sx * sx + sy * sy) / 2) if sx and sy else None,
            }
    return noise


def edge_lengths(vehicles, edges, t_end):
    """Mean distance per undirected edge in the steady window."""
    by_time = defaultdict(dict)
    for vehicle_id, rows in vehicles.items():
        for r in rows:
            if r["time_s"] >= t_end - STEADY_S:
                by_time[round(r["time_s"], 6)][vehicle_id] = (r["x_m"], r["y_m"])
    lengths = {}
    for (a, b) in common.undirected_edges(edges):
        dists = [
            hypot(pa[0] - pb[0], pa[1] - pb[1])
            for pts in by_time.values()
            if (pa := pts.get(a)) and (pb := pts.get(b))
        ]
        lengths[f"{a}-{b}"] = {
            "mean_m": sum(dists) / len(dists) if dists else None,
            "min_m": min(dists) if dists else None,
            "max_m": max(dists) if dists else None,
        }
    return lengths


def reconstruct_command_norms(edges, t_end):
    """Per-vehicle |u_i| = |kp * sum_j P_ij (g_ij - g*_ij)| in steady window."""
    per_vehicle_time = defaultdict(lambda: defaultdict(list))
    for (src, tgt), rows in edges.items():
        for r in rows:
            if r["time_s"] < t_end - STEADY_S or r.get("bearing_valid", 0.0) != 1.0:
                continue
            g = (r["actual_x"], r["actual_y"])
            gs = (r["desired_x"], r["desired_y"])
            e = (g[0] - gs[0], g[1] - gs[1])
            dot = g[0] * e[0] + g[1] * e[1]
            proj = (e[0] - g[0] * dot, e[1] - g[1] * dot)
            per_vehicle_time[src][round(r["time_s"], 6)].append(proj)
    norms = {}
    for vehicle_id, by_time in per_vehicle_time.items():
        values = [
            KP * hypot(sum(p[0] for p in projs), sum(p[1] for p in projs))
            for _, projs in sorted(by_time.items())
        ]
        mean, std = common.mean_std(values)
        norms[vehicle_id] = {
            "mean_mps": mean,
            "std_mps": std,
            "max_mps": max(values) if values else None,
            "frac_below_deadband": sum(1 for v in values if v < U_THR) / len(values)
            if values
            else None,
            "series": values,
        }
    return norms


def analyze_run(name):
    run_dir = common.RUNS[name]
    vehicles = common.load_trajectory(run_dir)
    edges = common.load_bearing_edges(run_dir)
    times, rms_curve = common.overall_formation_rms(edges)
    t_end = times[-1]

    noise = standstill_noise(vehicles)
    lengths = edge_lengths(vehicles, edges, t_end)
    cmd_norms = reconstruct_command_norms(edges, t_end)

    # Equivalent bearing measurement noise per edge (deg).
    per_edge = {}
    for (a, b), rows in common.undirected_edges(edges).items():
        tail = [
            r["bearing_error_deg"] for r in rows if r["time_s"] >= t_end - STEADY_S
        ]
        mean, std = common.mean_std(tail)
        L = lengths[f"{a}-{b}"]["mean_m"]
        sa = noise.get(a, {}).get("sigma_pos_m")
        sb = noise.get(b, {}).get("sigma_pos_m")
        sigma_theta = (
            degrees(sqrt(sa * sa + sb * sb) / L) if sa and sb and L else None
        )
        per_edge[f"{a}-{b}"] = {
            "mean_deg": mean,
            "std_deg": std,
            "rms_deg": common.rms(tail),
            "length_m": L,
            "mocap_equiv_noise_deg": sigma_theta,
        }

    steady = [v for t, v in zip(times, rms_curve) if t >= t_end - STEADY_S]
    mean_all, std_all = common.mean_std(steady)

    # Wheel-command dithering per vehicle in the steady window.
    dither = {
        vehicle_id: common.wheel_command_summary(rows, t_end - STEADY_S)
        for vehicle_id, rows in vehicles.items()
    }

    return {
        "steady_window_s": STEADY_S,
        "steady_mean_deg": mean_all,
        "steady_std_deg": std_all,
        "steady_rms_deg": common.rms(steady),
        "mocap_noise": noise,
        "edge_lengths": lengths,
        "per_edge": per_edge,
        "command_norms": {
            k: {kk: vv for kk, vv in v.items() if kk != "series"}
            for k, v in cmd_norms.items()
        },
        "wheel_dither": dither,
    }, {k: v["series"] for k, v in cmd_norms.items()}


def main():
    common.setup_figs_dir()
    results = {}
    series_by_run = {}
    print(f"theory: u_thr={U_THR:.5f} m/s, theta_db={THETA_DB_DEG:.3f} deg (single edge)")
    for name in common.RUNS:
        result, series = analyze_run(name)
        results[name] = result
        series_by_run[name] = series
        print(
            f"{name}: steady rms={result['steady_rms_deg']:.2f}deg "
            f"mean={result['steady_mean_deg']:.2f} std={result['steady_std_deg']:.2f}"
        )
        for edge, stats in result["per_edge"].items():
            print(
                f"  edge {edge}: mean={stats['mean_deg']:.2f} std={stats['std_deg']:.2f} "
                f"L={stats['length_m']:.3f}m mocap_equiv={stats['mocap_equiv_noise_deg']}deg"
            )
        for vehicle_id, stats in result["command_norms"].items():
            print(
                f"  {vehicle_id}: |u| mean={stats['mean_mps']:.4f} max={stats['max_mps']:.4f} "
                f"frac_below_deadband={stats['frac_below_deadband']:.2f}"
            )
        for vehicle_id, stats in result["mocap_noise"].items():
            print(
                f"  {vehicle_id}: sigma_pos={stats['sigma_pos_m']*1000:.2f}mm "
                f"({stats.get('method', 'standstill')}, n={stats['samples']})"
            )
        for vehicle_id, stats in result["wheel_dither"].items():
            if stats:
                print(
                    f"  {vehicle_id}: wheel zero={stats['zero_frac']:.2f} "
                    f"lifted={stats['lifted_frac']:.2f} above={stats['above_frac']:.2f} "
                    f"flips/s={stats['sign_flips_per_s']:.2f}"
                )

    results["theory"] = {
        "u_thr_mps": U_THR,
        "theta_db_deg": THETA_DB_DEG,
        "kp": KP,
        "deadband_mps": DEADBAND,
        "v_max_mps": V_MAX,
    }

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # Figure 2a: decomposition bars per run.
    fig, ax = plt.subplots(figsize=(8, 5))
    names = list(common.RUNS)
    x = range(len(names))
    width = 0.25
    steady_rms = [results[n]["steady_rms_deg"] for n in names]
    # global mocap-equivalent: quadrature mean of per-edge equivalents
    mocap_equiv = []
    for n in names:
        vals = [
            s["mocap_equiv_noise_deg"]
            for s in results[n]["per_edge"].values()
            if s["mocap_equiv_noise_deg"]
        ]
        mocap_equiv.append(common.rms(vals) if vals else 0.0)
    explained = [
        sqrt(max(s * s - m * m, 0.0)) for s, m in zip(steady_rms, mocap_equiv)
    ]
    ax.bar([i - width for i in x], steady_rms, width, label="measured steady RMS")
    ax.bar(x, mocap_equiv, width, label="mocap-noise equivalent")
    ax.bar([i + width for i in x], explained, width, label="residual excl. mocap")
    ax.axhline(
        THETA_DB_DEG,
        color="red",
        linestyle="--",
        label=f"deadband theory {THETA_DB_DEG:.2f} deg",
    )
    ax.set_xticks(list(x))
    ax.set_xticklabels(names)
    ax.set_ylabel("deg")
    ax.set_title("Steady-state residual decomposition (last 5 s)")
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig2a_residual_decomposition.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    # Figure 2b: reconstructed |u_i| vs deadband threshold (steady window).
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
    for ax, name in zip(axes, names):
        for vehicle_id, series in series_by_run[name].items():
            ax.plot(
                [i * 0.2 for i in range(len(series))],
                series,
                label=vehicle_id,
                linewidth=1.0,
            )
        ax.axhline(U_THR, color="red", linestyle="--", alpha=0.7)
        ax.set_title(name)
        ax.set_xlabel("t in steady window (s)")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(fontsize=8)
    axes[0].set_ylabel("|u_i| (m/s)")
    fig.suptitle("Reconstructed command norm vs deadband threshold (red)")
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig2b_command_norm_deadband.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    common.save_results("goal2_residual", results)


if __name__ == "__main__":
    main()
