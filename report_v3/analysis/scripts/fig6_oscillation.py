"""P7: motion oscillation quantification for the v3 runs (known shortfall).

Panels: (a) yaw deviation from the mean heading during the cruise phase;
(b) actual speed ripple around the cruise target; (c) 2 s wheel-command
snippet (min_eff lift limit cycle); (d) per-car metrics summary.
Outputs results/fig6_oscillation.json and figs/fig6_oscillation.png.
"""

from __future__ import annotations

from math import degrees, hypot

import common

FLEET_LABEL = {"two": "2 cars", "three": "3 cars", "four": "4 cars"}
CRUISE = 0.10


def main() -> None:
    fig, axes = common.new_figure(14, 9, 2, 2)
    ax_yaw, ax_spd, ax_cmd, ax_bar = axes[0][0], axes[0][1], axes[1][0], axes[1][1]
    results = {}

    # representative run: the last (mocap-matched) 4-car run
    rep_dir = common.v3_run_dir(common.V3_RUNS["four"][-1])
    rep_vehicles = common.load_trajectory(rep_dir)
    window = common.active_window(rep_vehicles, CRUISE)
    t0, t1 = window

    for vid, rows in sorted(rep_vehicles.items()):
        act = [r for r in rows if t0 <= r["time_s"] <= t1]
        if len(act) < 10:
            continue
        yaw_deg = [degrees(r["yaw_rad"]) for r in act]
        yaw_mean = sum(yaw_deg) / len(yaw_deg)
        ax_yaw.plot([r["time_s"] for r in act], [y - yaw_mean for y in yaw_deg],
                    label=vid)
        speeds = [hypot(vx, vy) for _, vx, vy in common.velocity_series(act, 0.2)]
        ax_spd.plot([r["time_s"] for r in act], speeds, label=vid)

    ax_yaw.axhline(0.0, color="gray", linestyle="--", alpha=0.6)
    ax_yaw.set_title("(a) yaw deviation from mean heading, cruise phase (4 cars)")
    ax_yaw.set_ylabel("yaw deviation (deg)")
    ax_spd.axhline(CRUISE, color="gray", linestyle="--", alpha=0.6, label="w = 0.10 m/s")
    ax_spd.set_title("(b) actual speed ripple, cruise phase (4 cars)")
    ax_spd.set_ylabel("speed (m/s)")

    # wheel-command snippet from one car
    vid0 = sorted(rep_vehicles)[0]
    rows = [r for r in rep_vehicles[vid0] if t0 <= r["time_s"] <= t1]
    t_mid = rows[len(rows) // 2]["time_s"]
    snippet = [r for r in rows if t_mid <= r["time_s"] <= t_mid + 2.0]
    for key, name in (("front_left_command", "FL"), ("front_right_command", "FR"),
                      ("rear_left_command", "RL"), ("rear_right_command", "RR")):
        ax_cmd.plot([r["time_s"] - t_mid for r in snippet], [r[key] for r in snippet],
                    label=name)
    ax_cmd.set_title("(c) wheel commands, 2 s snippet, %s" % vid0)
    ax_cmd.set_xlabel("time (s)")
    ax_cmd.set_ylabel("command")

    # metrics over all 7 runs
    names, wobble_med, ripple_med, flips = [], [], [], []
    for size, run_names in common.V3_RUNS.items():
        for run_name in run_names:
            run_dir = common.v3_run_dir(run_name)
            vehicles = common.load_trajectory(run_dir)
            win = common.active_window(vehicles, CRUISE)
            run_res = {}
            for vid, rows in sorted(vehicles.items()):
                act = [r for r in rows if win[0] <= r["time_s"] <= win[1]]
                if len(act) < 10:
                    continue
                yaw_deg = [degrees(r["yaw_rad"]) for r in act]
                yaw_mean = sum(yaw_deg) / len(yaw_deg)
                wob = [abs(y - yaw_mean) for y in yaw_deg]
                speeds = [hypot(vx, vy) for _, vx, vy in common.velocity_series(act, 0.4)]
                mean_s = sum(speeds) / len(speeds)
                rip = [abs(s - mean_s) / mean_s for s in speeds] if mean_s > 1e-9 else [0.0]
                flip = 0
                for key in ("front_left_command", "front_right_command",
                            "rear_left_command", "rear_right_command"):
                    seq = [r[key] for r in act]
                    flip += sum(1 for a, b in zip(seq, seq[1:])
                                if (a > 0) != (b > 0) and a != 0.0 and b != 0.0)
                dur = act[-1]["time_s"] - act[0]["time_s"]
                run_res[vid] = {
                    "yaw_wobble_median_deg": common.percentile(wob, 0.5),
                    "yaw_wobble_p95_deg": common.percentile(wob, 0.95),
                    "speed_ripple_median_pct": 100.0 * common.percentile(rip, 0.5),
                    "speed_ripple_p95_pct": 100.0 * common.percentile(rip, 0.95),
                    "wheel_sign_flips_per_s_per_wheel": flip / 4.0 / max(dur, 1e-9),
                }
            results[run_name] = run_res
            wm = [v["yaw_wobble_median_deg"] for v in run_res.values()]
            rm = [v["speed_ripple_median_pct"] for v in run_res.values()]
            fl = [v["wheel_sign_flips_per_s_per_wheel"] for v in run_res.values()]
            if wm:
                names.append("%s #%s" % (FLEET_LABEL[size], run_name.split("_")[-2][-4:]))
                wobble_med.append(sum(wm) / len(wm))
                ripple_med.append(sum(rm) / len(rm))
                flips.append(sum(fl) / len(fl))

    x = range(len(names))
    ax_bar.bar([i - 0.2 for i in x], wobble_med, width=0.4, label="yaw wobble median (deg)")
    ax2 = ax_bar.twinx()
    ax2.bar([i + 0.2 for i in x], ripple_med, width=0.4, color="tab:orange",
            label="speed ripple median (%)")
    ax_bar.set_xticks(list(x))
    ax_bar.set_xticklabels(names, fontsize=7, rotation=20)
    ax_bar.set_title("(d) oscillation metrics per run (median over cars)")
    ax_bar.set_ylabel("yaw wobble (deg)")
    ax2.set_ylabel("speed ripple (%)")
    ax_bar.grid(True, linestyle="--", alpha=0.4, axis="y")

    for ax in (ax_yaw, ax_spd, ax_cmd):
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(loc="best", fontsize=7)
        ax.set_xlabel("time (s)")
    fig.tight_layout()
    common.save_figure(fig, "fig6_oscillation.png")
    common.save_results("fig6_oscillation", results)


if __name__ == "__main__":
    main()
