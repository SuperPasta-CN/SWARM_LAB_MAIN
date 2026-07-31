"""Goal 1: convergence time and steady-state bearing residual comparison.

For each report run:
- overall formation bearing-error RMS curve (per timestamp, over edges),
- convergence time = time after which RMS stays below 5 deg for good,
- steady-state residual = mean/std/RMS over the last 5 s and last 10 s,
- initial RMS (first sample) to expose the initial-condition spread.
"""

from __future__ import annotations

from math import degrees

import common


def analyze_run(name):
    run_dir = common.RUNS[name]
    edges = common.load_bearing_edges(run_dir)
    times, rms_curve = common.overall_formation_rms(edges)
    duration = times[-1] - times[0]

    # Convergence: last time the RMS is >= 5 deg; converged right after.
    threshold = 5.0
    above = [t for t, v in zip(times, rms_curve) if v >= threshold]
    if not above:
        t_conv = times[0]
    elif above[-1] >= times[-1] - 1e-9:
        t_conv = None  # never converged
    else:
        idx = times.index(above[-1])
        t_conv = times[idx + 1]

    def window_stats(window_s):
        values = [v for t, v in zip(times, rms_curve) if t >= times[-1] - window_s]
        mean, std = common.mean_std(values)
        return {
            "window_s": window_s,
            "samples": len(values),
            "mean_deg": mean,
            "std_deg": std,
            "rms_deg": common.rms(values),
            "max_deg": max(values) if values else None,
            "final_deg": values[-1] if values else None,
        }

    # Per-edge steady stats (last 5 s), undirected edges only.
    per_edge = {}
    for (a, b), rows in common.undirected_edges(edges).items():
        tail = [r["bearing_error_deg"] for r in rows if r["time_s"] >= times[-1] - 5.0]
        mean, std = common.mean_std(tail)
        per_edge[f"{a}-{b}"] = {
            "mean_deg": mean,
            "std_deg": std,
            "rms_deg": common.rms(tail),
            "final_deg": tail[-1] if tail else None,
        }

    return {
        "duration_s": duration,
        "n_samples": len(times),
        "initial_rms_deg": rms_curve[0],
        "max_rms_deg": max(rms_curve),
        "convergence_time_s": t_conv,
        "converged": t_conv is not None,
        "steady_last5s": window_stats(5.0),
        "steady_last10s": window_stats(10.0) if duration >= 12.0 else None,
        "per_edge_last5s": per_edge,
    }, (times, rms_curve)


def main():
    common.setup_figs_dir()
    results = {}
    curves = {}
    for name in common.RUNS:
        result, curve = analyze_run(name)
        results[name] = result
        curves[name] = curve
        conv = result["convergence_time_s"]
        print(
            f"{name}: duration={result['duration_s']:.1f}s "
            f"initial_rms={result['initial_rms_deg']:.1f}deg "
            f"t_conv={conv if conv is None else round(conv, 2)}s "
            f"last5s mean={result['steady_last5s']['mean_deg']:.2f} "
            f"std={result['steady_last5s']['std_deg']:.2f} "
            f"rms={result['steady_last5s']['rms_deg']:.2f} "
            f"final={result['steady_last5s']['final_deg']:.2f}"
        )

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5))
    for name, (times, rms_curve) in curves.items():
        ax.plot(times, rms_curve, label=name, linewidth=1.4)
    ax.axhline(5.0, color="gray", linestyle="--", alpha=0.7, label="5 deg threshold")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("overall bearing RMS error (deg)")
    ax.set_title("Convergence of the three report runs (RMS over directed edges)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig1_convergence_compare.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    common.save_results("goal1_convergence", results)


if __name__ == "__main__":
    main()
