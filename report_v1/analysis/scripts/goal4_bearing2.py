"""Goal 4: why is bearing_2_jyc1's whole-run mean error (15.6 deg) so large?

Hypothesis: the run started ~72 deg away from the desired bearing (placement
initial condition) and lasted only 14.8 s, so the whole-run average is
dominated by the initial transient.  Checks:
1. Error-curve morphology: monotone decay vs oscillation vs stuck.
2. Convergence time and steady level (already known: 5.46 s, ~1.2 deg).
3. Transient exponential time constant per run (comparison).
4. Initial geometry: distance and bearing offset vs desired.
5. Whole-run mean decomposition: transient segment vs steady segment.
6. Car movement during the run (both cars moved; car1 slow).
"""

from __future__ import annotations

from math import atan2, degrees, exp, hypot, log, sqrt

import common


def morphology(name):
    """Monotonicity and phase structure of the RMS curve."""
    edges = common.load_bearing_edges(common.RUNS[name])
    times, rms_curve = common.overall_formation_rms(edges)
    increases = sum(
        1 for a, b in zip(rms_curve, rms_curve[1:]) if b > a + 0.05
    )
    # Slope over the last 3 s (is it still improving?).
    tail = [(t, v) for t, v in zip(times, rms_curve) if t >= times[-1] - 3.0]
    slope = (tail[-1][1] - tail[0][1]) / (tail[-1][0] - tail[0][0])
    return {
        "n_increases": increases,
        "n_samples": len(rms_curve),
        "increase_frac": increases / (len(rms_curve) - 1),
        "last3s_slope_deg_per_s": slope,
        "series": (times, rms_curve),
    }


def fit_tau(times, rms_curve, rms_ss):
    """Exponential fit rms = rms_ss + A*exp(-t/tau) via log-linear regression."""
    pts = [
        (t, v - rms_ss)
        for t, v in zip(times, rms_curve)
        if v - rms_ss > max(0.2 * rms_ss, 0.5)
    ]
    if len(pts) < 5:
        return None, None
    xs = [p[0] for p in pts]
    ys = [log(p[1]) for p in pts]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = my - slope * mx
    return -1.0 / slope, exp(intercept)


def initial_geometry(name):
    vehicles = common.load_trajectory(common.RUNS[name])
    edges = common.load_bearing_edges(common.RUNS[name])
    first = {v: rows[0] for v, rows in vehicles.items()}
    out = {}
    for (a, b), rows in common.undirected_edges(edges).items():
        pa, pb = first[a], first[b]
        dx, dy = pb["x_m"] - pa["x_m"], pb["y_m"] - pa["y_m"]
        dist = hypot(dx, dy)
        r = rows[0]
        desired = (r["desired_x"], r["desired_y"])
        actual = (dx / dist, dy / dist)
        dot = desired[0] * actual[0] + desired[1] * actual[1]
        cross = desired[0] * actual[1] - desired[1] * actual[0]
        out[f"{a}-{b}"] = {
            "initial_distance_m": dist,
            "initial_offset_deg": degrees(atan2(cross, dot)),
            "initial_error_deg": r["bearing_error_deg"],
        }
    return out


def mean_decomposition(name, split_s=5.0):
    edges = common.load_bearing_edges(common.RUNS[name])
    times, rms_curve = common.overall_formation_rms(edges)
    early = [v for t, v in zip(times, rms_curve) if t < split_s]
    late = [v for t, v in zip(times, rms_curve) if t >= split_s]
    return {
        "split_s": split_s,
        "whole_mean_deg": sum(rms_curve) / len(rms_curve),
        "first5s_mean_deg": sum(early) / len(early),
        "after5s_mean_deg": sum(late) / len(late),
    }


def movement(name):
    vehicles = common.load_trajectory(common.RUNS[name])
    out = {}
    for v, rows in vehicles.items():
        first6 = [r for r in rows if r["time_s"] <= 6.0]
        disp6 = hypot(
            first6[-1]["x_m"] - first6[0]["x_m"],
            first6[-1]["y_m"] - first6[0]["y_m"],
        )
        path = sum(
            hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"])
            for a, b in zip(rows, rows[1:])
        )
        out[v] = {"displacement_first6s_m": disp6, "path_length_m": path}
    return out


def main():
    common.setup_figs_dir()
    results = {}
    curves = {}
    for name in common.RUNS:
        morph = morphology(name)
        times, rms_curve = morph.pop("series")
        curves[name] = (times, rms_curve)
        steady = [v for t, v in zip(times, rms_curve) if t >= times[-1] - 5.0]
        rms_ss = sum(steady) / len(steady)
        tau, amp = fit_tau(times, rms_curve, rms_ss)
        results[name] = {
            "morphology": morph,
            "steady_rms_deg": rms_ss,
            "tau_s": tau,
            "amplitude_deg": amp,
            "initial_geometry": initial_geometry(name),
            "mean_decomposition": mean_decomposition(name),
            "movement": movement(name),
        }
        print(f"=== {name} ===")
        print(
            f"  morphology: {morph['n_increases']}/{morph['n_samples']-1} increases "
            f"(frac={morph['increase_frac']:.3f}), last3s slope={morph['last3s_slope_deg_per_s']:+.3f} deg/s"
        )
        print(f"  tau={tau:.2f}s amplitude={amp:.1f}deg steady={rms_ss:.2f}deg")
        for edge, g in results[name]["initial_geometry"].items():
            print(
                f"  {edge}: d0={g['initial_distance_m']:.3f}m "
                f"offset={g['initial_offset_deg']:+.1f}deg err0={g['initial_error_deg']:.1f}deg"
            )
        md = results[name]["mean_decomposition"]
        print(
            f"  mean: whole={md['whole_mean_deg']:.2f} first5s={md['first5s_mean_deg']:.2f} "
            f"after5s={md['after5s_mean_deg']:.2f}"
        )
        for v, m in results[name]["movement"].items():
            print(
                f"  {v}: disp6s={m['displacement_first6s_m']:.3f}m path={m['path_length_m']:.2f}m"
            )

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # fig4a: bearing_2 error curve with annotations.
    times, rms_curve = curves["bearing_2_jyc1"]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(times, rms_curve, color="C0", linewidth=1.6, label="overall RMS")
    ax.axhline(5.0, color="gray", linestyle="--", alpha=0.7, label="5 deg")
    ax.axhline(1.43, color="red", linestyle=":", alpha=0.8, label="deadband 1.43 deg")
    ax.axvline(5.46, color="green", linestyle="--", alpha=0.7, label="t_conv = 5.46 s")
    ax.annotate(
        "initial 72.0 deg",
        xy=(times[0], rms_curve[0]),
        xytext=(3.0, 62),
        arrowprops=dict(arrowstyle="->", color="black", alpha=0.6),
    )
    ax.annotate(
        "steady ~1.2 deg",
        xy=(13.5, 1.2),
        xytext=(8.5, 12),
        arrowprops=dict(arrowstyle="->", color="black", alpha=0.6),
    )
    ax.set_xlabel("time (s)")
    ax.set_ylabel("overall bearing RMS error (deg)")
    ax.set_title("bearing_2_jyc1: monotone convergence, stopped early (74 frames)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig4a_bearing2_morphology.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    # fig4b: tau comparison (transient speed) for the three runs.
    fig, ax = plt.subplots(figsize=(8, 5))
    for name in common.RUNS:
        times, rms_curve = curves[name]
        rms_ss = results[name]["steady_rms_deg"]
        tau = results[name]["tau_s"]
        excess = [max(v - rms_ss, 1e-3) for v in rms_curve]
        ax.semilogy(times, excess, label=f"{name} (tau={tau:.1f}s)", linewidth=1.3)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("RMS error above steady level (deg, log)")
    ax.set_title("Transient decay above the steady-state floor")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig4b_tau_compare.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    common.save_results("goal4_bearing2", results)


if __name__ == "__main__":
    main()
