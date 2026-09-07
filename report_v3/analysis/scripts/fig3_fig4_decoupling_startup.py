"""P3 (null-space decoupling) + P6 (startup window) for the v3 runs.

fig3: per fleet size (last run each), centroid velocity vs the task velocity
w with the edge-error RMS on a twin axis — the formation translates at w
while the constraint error decays (the task term is never fought).
fig4: startup zoom (first 6 s): per-car yaw alignment toward the heading
target (top) and edge-error RMS during the standstill window (bottom).
Outputs results/fig3_decoupling.json and results/fig4_startup.json.
"""

from __future__ import annotations

from math import degrees

import common

FLEET_LABEL = {"two": "2 cars", "three": "3 cars", "four": "4 cars"}
CRUISE = 0.10


def last_run_dir(size):
    return common.v3_run_dir(common.V3_RUNS[size][-1])


def main() -> None:
    # ---------------- fig3: decoupling
    fig, axes = common.new_figure(14, 5, 1, 3)
    results3 = {}
    for ax, size in zip(axes, ("two", "three", "four")):
        run_dir = last_run_dir(size)
        vehicles = common.load_trajectory(run_dir)
        vels = {vid: common.velocity_series(rows) for vid, rows in vehicles.items()}
        # centroid velocity x-component per timestamp (aligned 5 Hz records)
        by_id = {vid: dict((t, (vx, vy)) for t, vx, vy in series)
                 for vid, series in vels.items()}
        times = sorted(set.intersection(*[set(d) for d in by_id.values()]))
        cx = [sum(by_id[vid][t][0] for vid in by_id) / len(by_id) for t in times]
        cy = [sum(by_id[vid][t][1] for vid in by_id) / len(by_id) for t in times]
        edges = common.load_v3_edges(run_dir)
        et, er = common.overall_rms(
            [[(r["time_s"], r["edge_error_m"]) for r in rows] for rows in edges.values()]
        )
        ax.plot(times, cx, color="tab:blue", label="centroid vx")
        ax.plot(times, cy, color="tab:cyan", label="centroid vy")
        ax.axhline(CRUISE, color="gray", linestyle="--", alpha=0.7, label="w = 0.10 m/s")
        ax.set_ylabel("centroid velocity (m/s)")
        ax.set_xlabel("time (s)")
        ax.set_ylim(-0.05, 0.2)
        ax2 = ax.twinx()
        ax2.plot(et, er, color="tab:red", alpha=0.5, label="edge RMS")
        ax2.set_ylabel("edge error RMS (m)", color="tab:red")
        ax2.set_ylim(0, max(er) * 1.1)
        ax.set_title("%s: centroid velocity vs edge error" % FLEET_LABEL[size])
        ax.legend(loc="upper left", fontsize=7)
        ax.grid(True, linestyle="--", alpha=0.4)
        # steady cruise stats over the active window tail
        window = common.active_window(vehicles, CRUISE)
        t0, t1 = window
        tail_t = [t for t in times if t >= t1 - 5.0]
        tail_vx = [x for t, x in zip(times, cx) if t >= t1 - 5.0]
        tail_vy = [y for t, y in zip(times, cy) if t >= t1 - 5.0]
        mean_vx, _ = common.mean_std(tail_vx)
        mean_vy, _ = common.mean_std(tail_vy)
        results3[FLEET_LABEL[size]] = {
            "run": run_dir.name,
            "cruise_window_s": [t0, t1],
            "centroid_vx_mps": mean_vx,
            "centroid_vy_mps": mean_vy,
            "achievement_pct": 100.0 * mean_vx / CRUISE,
        }
    fig.tight_layout()
    common.save_figure(fig, "fig3_decoupling.png")
    common.save_results("fig3_decoupling", results3)

    # ---------------- fig4: startup window
    fig, axes = common.new_figure(14, 7, 2, 3)
    results4 = {}
    for col, size in enumerate(("two", "three", "four")):
        run_dir = last_run_dir(size)
        vehicles = common.load_trajectory(run_dir)
        ax_yaw = axes[0][col]
        for vid, rows in sorted(vehicles.items()):
            early = [r for r in rows if r["time_s"] <= 6.0]
            ax_yaw.plot([r["time_s"] for r in early],
                        [degrees(r["yaw_rad"]) for r in early], label=vid)
        ax_yaw.axhline(0.0, color="gray", linestyle="--", alpha=0.7)
        ax_yaw.axvspan(0, 3.0, color="tab:green", alpha=0.08)
        ax_yaw.set_title("%s: yaw alignment (target 0 deg)" % FLEET_LABEL[size])
        ax_yaw.set_xlabel("time (s)")
        ax_yaw.set_ylabel("yaw (deg)")
        ax_yaw.legend(loc="best", fontsize=7)
        ax_yaw.grid(True, linestyle="--", alpha=0.4)

        ax_rms = axes[1][col]
        edges = common.load_v3_edges(run_dir)
        et, er = common.overall_rms(
            [[(r["time_s"], r["edge_error_m"]) for r in rows] for rows in edges.values()]
        )
        mask = [t <= 6.0 for t in et]
        ax_rms.plot([t for t, m in zip(et, mask) if m],
                    [v for v, m in zip(er, mask) if m], color="tab:blue")
        ax_rms.axvspan(0, 3.0, color="tab:green", alpha=0.08, label="standstill window")
        ax_rms.set_title("%s: edge-error RMS at startup" % FLEET_LABEL[size])
        ax_rms.set_xlabel("time (s)")
        ax_rms.set_ylabel("edge error RMS (m)")
        ax_rms.legend(loc="best", fontsize=7)
        ax_rms.grid(True, linestyle="--", alpha=0.4)

        yaw0 = {vid: degrees(rows[0]["yaw_rad"]) for vid, rows in vehicles.items()}
        yaw3 = {
            vid: degrees(min(rows, key=lambda r: abs(r["time_s"] - 3.0))["yaw_rad"])
            for vid, rows in vehicles.items()
        }
        results4[FLEET_LABEL[size]] = {
            "run": run_dir.name,
            "yaw_at_t0_deg": yaw0,
            "yaw_at_t3_deg": yaw3,
            "rms_at_t0_m": er[0],
            "rms_at_t3_m": er[next((i for i, t in enumerate(et) if t >= 3.0), -1)],
        }
    fig.tight_layout()
    common.save_figure(fig, "fig4_startup.png")
    common.save_results("fig4_startup", results4)


if __name__ == "__main__":
    main()
