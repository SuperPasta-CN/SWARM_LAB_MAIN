"""P1 (convergence) + P2 (scale locking) for the v3 validation runs.

fig1: per-fleet-size edge-error RMS curves with the deadband floor, plus a
steady-state bar chart (active window, last 5 s).
fig2: per-edge actual distance vs desired 0.8 m (v3, solid) against the
scale-free v2 runs (dashed) — the version-defining contrast.
Outputs results/fig1_convergence.json and results/fig2_scale_lock.json.
"""

from __future__ import annotations

import common

FLEET_LABEL = {"two": "2 cars", "three": "3 cars", "four": "4 cars"}
FLEET_COLOR = {"two": "tab:blue", "three": "tab:orange", "four": "tab:green"}


def v3_rms_curve(run_dir):
    edges = common.load_v3_edges(run_dir)
    return common.overall_rms(
        [[(r["time_s"], r["edge_error_m"]) for r in rows]
         for rows in edges.values()]
    )


def v2_rms_curve(run_dir):
    edges = common.load_v2_edges(run_dir)
    return common.overall_rms(
        [[(r["time_s"], r["bearing_error_deg"]) for r in rows]
         for rows in edges.values()]
    )


def undirected_edge_distances(vehicles):
    """(i, j) i<j -> [(t, distance)] for every vehicle pair."""

    ids = sorted(vehicles)
    out = {}
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = vehicles[ids[i]], vehicles[ids[j]]
            by_t = {r["time_s"]: r for r in b}
            series = [
                (r["time_s"], ((r["x_m"] - by_t[r["time_s"]]["x_m"]) ** 2
                               + (r["y_m"] - by_t[r["time_s"]]["y_m"]) ** 2) ** 0.5)
                for r in a if r["time_s"] in by_t
            ]
            out[(ids[i], ids[j])] = series
    return out


def main() -> None:
    # ---------------- fig1: convergence
    fig, axes = common.new_figure(14, 5, 1, 2)
    ax_curve, ax_bar = axes
    results1 = {}
    bar_names, bar_means, bar_stds = [], [], []
    for size, run_names in common.V3_RUNS.items():
        for run_name in run_names:
            run_dir = common.v3_run_dir(run_name)
            times, rms = v3_rms_curve(run_dir)
            label = "%s #%s" % (FLEET_LABEL[size], run_name.split("_")[-2][-4:])
            ax_curve.plot(times, rms, color=FLEET_COLOR[size], alpha=0.75,
                          label=label)
            vehicles = common.load_trajectory(run_dir)
            window = common.active_window(vehicles, 0.10)
            t0, t1 = window if window else (times[0], times[-1])
            tail = [v for t, v in zip(times, rms) if max(t0, t1 - 5.0) <= t <= t1]
            mean, std = common.mean_std(tail)
            conv = next((t for t, v in zip(times, rms) if v < 0.03), None)
            entry = {
                "duration_s": times[-1],
                "initial_rms_m": rms[0],
                "active_window_s": [t0, t1],
                "time_below_30mm_s": conv,
                "steady_mean_m": mean,
                "steady_std_m": std,
            }
            results1[run_name] = entry
            bar_names.append(label)
            bar_means.append(mean)
            bar_stds.append(std)
    ax_curve.axhline(common.DEADBAND_FLOOR_M, color="red", linestyle="--",
                     alpha=0.7, label="deadband floor %.3f m" % common.DEADBAND_FLOOR_M)
    ax_curve.axhline(0.03, color="gray", linestyle=":", alpha=0.7, label="converge eps 0.03 m")
    ax_curve.set_title("(a) formation edge-error RMS vs time (v3, all 7 runs)")
    ax_curve.set_xlabel("time (s)")
    ax_curve.set_ylabel("edge error RMS (m)")
    ax_curve.legend(loc="best", fontsize=7)

    bars = ax_bar.bar(bar_names, bar_means, yerr=bar_stds, capsize=4,
                      color=[FLEET_COLOR[s] for s in common.V3_RUNS for _ in common.V3_RUNS[s]],
                      alpha=0.85)
    ax_bar.axhline(common.DEADBAND_FLOOR_M, color="red", linestyle="--", alpha=0.7,
                   label="deadband floor %.3f m" % common.DEADBAND_FLOOR_M)
    ax_bar.set_title("(b) steady-state edge error (active window, last 5 s)")
    ax_bar.set_ylabel("error (m)")
    ax_bar.tick_params(axis="x", labelsize=7, rotation=20)
    ax_bar.legend(loc="best", fontsize=8)
    for bar, value in zip(bars, bar_means):
        ax_bar.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    "%.3f" % value, ha="center", va="bottom", fontsize=7)
    for ax in axes:
        ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    common.save_figure(fig, "fig1_convergence.png")
    common.save_results("fig1_convergence", results1)

    # ---------------- fig2: scale locking, v3 vs v2
    fig, axes = common.new_figure(14, 5, 1, 3)
    results2 = {}
    for ax, size in zip(axes, ("two", "three", "four")):
        # v2 dashed (scale free): its distances drift
        for run_name in common.V2_RUNS[size]:
            vehicles = common.load_trajectory(common.v2_run_dir(run_name))
            for (a, b), series in undirected_edge_distances(vehicles).items():
                ax.plot([t for t, _ in series], [d for _, d in series],
                        color="tab:red", alpha=0.5, linestyle="--")
        # v3 solid vs desired
        for run_name in common.V3_RUNS[size]:
            run_dir = common.v3_run_dir(run_name)
            desired = common.desired_reference(common.load_metadata(run_dir))
            vehicles = common.load_trajectory(run_dir)
            for (a, b), series in undirected_edge_distances(vehicles).items():
                ax.plot([t for t, _ in series], [d for _, d in series],
                        color="tab:blue", alpha=0.8)
                d0 = abs(desired[a] - desired[b])
                ax.axhline(d0, color="gray", linestyle=":", alpha=0.8)
            # steady per-edge distance error for the LAST run of the size
            tail_stats = {}
            for (a, b), series in undirected_edge_distances(vehicles).items():
                t1 = series[-1][0]
                tail = [d for t, d in series if t >= t1 - 5.0]
                mean, std = common.mean_std(tail)
                tail_stats["%s-%s" % (a, b)] = {
                    "desired_m": abs(desired[a] - desired[b]),
                    "steady_mean_m": mean,
                    "steady_std_m": std,
                }
            results2[run_name] = tail_stats
        ax.set_title("%s: edge distance vs time" % FLEET_LABEL[size])
        ax.set_xlabel("time (s)")
        ax.set_ylabel("distance (m)")
        ax.grid(True, linestyle="--", alpha=0.4)
        from matplotlib.lines import Line2D
        ax.legend(handles=[
            Line2D([0], [0], color="tab:blue", label="v3 (locked)"),
            Line2D([0], [0], color="tab:red", linestyle="--", label="v2 (scale free)"),
            Line2D([0], [0], color="gray", linestyle=":", label="desired"),
        ], fontsize=7, loc="best")
    fig.tight_layout()
    common.save_figure(fig, "fig2_scale_lock.png")
    common.save_results("fig2_scale_lock", results2)


if __name__ == "__main__":
    main()
