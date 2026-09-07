"""Fig 7: oscillation attribution for the v3 cruise phase (the known shortfall).

Quantifies the wheel-command limit cycle and classifies its pattern:
- lifted/zero/above fractions: how much of the time commands sit pinned at
  +-min_eff (35) — the dead-zone-lift signature;
- sign-flip rate per wheel (limit-cycle frequency ~= flips/2);
- pattern classification per sample: "rotation" when the diagonal pairs agree
  and the sides disagree (sign(FL)==sign(RR) != sign(FR)==sign(RL)),
  "translation" when all four agree — tells whether the oscillation is
  driven by the heading-hold channel or the translation channel;
- yaw wobble and speed ripple alongside, per run.

Outputs results/fig7_oscillation_attribution.json and
figs/fig7_oscillation_attribution.png.
"""

from __future__ import annotations

from math import degrees

import common

CRUISE = 0.10
MIN_EFF = 35.0
WHEELS = ("front_left_command", "front_right_command",
          "rear_left_command", "rear_right_command")


def classify_sample(values):
    signs = [1 if v > 0 else -1 if v < 0 else 0 for v in values]
    fl, fr, rl, rr = signs
    if 0 in signs:
        return "mixed"
    if fl == rr and fr == rl and fl != fr:
        return "rotation"
    if fl == fr == rl == rr:
        return "translation"
    return "mixed"


def main() -> None:
    fig, axes = common.new_figure(14, 9, 2, 2)
    ax_snip, ax_frac, ax_flip, ax_scatter = (
        axes[0][0], axes[0][1], axes[1][0], axes[1][1],
    )
    results = {}
    bar_names, lifted_frac, rotation_frac, flip_rates = [], [], [], []
    scatter_x, scatter_y = [], []

    for size, run_names in common.V3_RUNS.items():
        for run_name in run_names:
            run_dir = common.v3_run_dir(run_name)
            vehicles = common.load_trajectory(run_dir)
            win = common.active_window(vehicles, CRUISE)
            t0, t1 = win
            per_car = {}
            for vid, rows in sorted(vehicles.items()):
                act = [r for r in rows if t0 <= r["time_s"] <= t1]
                if len(act) < 10:
                    continue
                dur = act[-1]["time_s"] - act[0]["time_s"]
                lifted = zero = above = flips = 0
                classes = {"rotation": 0, "translation": 0, "mixed": 0}
                for r in act:
                    values = [r[k] for k in WHEELS]
                    classes[classify_sample(values)] += 1
                    for v in values:
                        if v == 0.0:
                            zero += 1
                        elif abs(abs(v) - MIN_EFF) < 0.51:
                            lifted += 1
                        else:
                            above += 1
                for key in WHEELS:
                    seq = [r[key] for r in act]
                    flips += sum(1 for a, b in zip(seq, seq[1:])
                                 if (a > 0) != (b > 0) and a != 0.0 and b != 0.0)
                total_cmds = 4 * len(act)
                samples = len(act)
                yaw_deg = [degrees(r["yaw_rad"]) for r in act]
                yaw_mean = sum(yaw_deg) / len(yaw_deg)
                wob = [abs(y - yaw_mean) for y in yaw_deg]
                flip_rate = flips / 4.0 / max(dur, 1e-9)
                wobble_med = common.percentile(wob, 0.5)
                per_car[vid] = {
                    "lifted_frac": lifted / total_cmds,
                    "zero_frac": zero / total_cmds,
                    "above_frac": above / total_cmds,
                    "rotation_pattern_frac": classes["rotation"] / samples,
                    "translation_pattern_frac": classes["translation"] / samples,
                    "wheel_sign_flips_per_s_per_wheel": flip_rate,
                    "yaw_wobble_median_deg": wobble_med,
                }
                scatter_x.append(flip_rate)
                scatter_y.append(wobble_med)
            results[run_name] = per_car
            if per_car:
                bar_names.append("%s #%s" % (size, run_name.split("_")[-2][-4:]))
                lifted_frac.append(
                    sum(v["lifted_frac"] for v in per_car.values()) / len(per_car))
                rotation_frac.append(
                    sum(v["rotation_pattern_frac"] for v in per_car.values()) / len(per_car))
                flip_rates.append(
                    sum(v["wheel_sign_flips_per_s_per_wheel"] for v in per_car.values())
                    / len(per_car))

    # (a) snippet: one car, rotation-pattern visible (diagonal anti-phase)
    rep_dir = common.v3_run_dir(common.V3_RUNS["four"][-1])
    rep_vehicles = common.load_trajectory(rep_dir)
    win = common.active_window(rep_vehicles, CRUISE)
    vid0 = sorted(rep_vehicles)[0]
    rows = [r for r in rep_vehicles[vid0] if win[0] <= r["time_s"] <= win[1]]
    t_mid = rows[len(rows) // 2]["time_s"]
    snippet = [r for r in rows if t_mid <= r["time_s"] <= t_mid + 1.5]
    for key, name in zip(WHEELS, ("FL", "FR", "RL", "RR")):
        ax_snip.plot([r["time_s"] - t_mid for r in snippet], [r[key] for r in snippet],
                     marker=".", label=name)
    ax_snip.axhline(MIN_EFF, color="gray", linestyle="--", alpha=0.5)
    ax_snip.axhline(-MIN_EFF, color="gray", linestyle="--", alpha=0.5)
    ax_snip.set_title("(a) wheel commands pinned at +-min_eff, 1.5 s snippet (%s)" % vid0)
    ax_snip.set_xlabel("time (s)")
    ax_snip.set_ylabel("command")
    ax_snip.legend(loc="best", fontsize=7, ncol=4)

    # (b) lifted vs rotation fractions per run
    x = range(len(bar_names))
    ax_frac.bar([i - 0.2 for i in x], [100 * v for v in lifted_frac], width=0.4,
                label="lifted to +-min_eff (%)")
    ax_frac.bar([i + 0.2 for i in x], [100 * v for v in rotation_frac], width=0.4,
                label="rotation pattern (%)")
    ax_frac.set_xticks(list(x))
    ax_frac.set_xticklabels(bar_names, fontsize=7, rotation=20)
    ax_frac.set_ylabel("fraction (%)")
    ax_frac.set_title("(b) command pattern fractions per run (cruise phase)")
    ax_frac.legend(loc="best", fontsize=8)

    # (c) flip rates
    ax_flip.bar(list(x), flip_rates, color="tab:red")
    ax_flip.set_xticks(list(x))
    ax_flip.set_xticklabels(bar_names, fontsize=7, rotation=20)
    ax_flip.set_ylabel("sign flips / s / wheel")
    ax_flip.set_title("(c) limit-cycle rate (frequency ~= rate/2 Hz)")

    # (d) yaw wobble vs flip rate
    ax_scatter.scatter(scatter_x, scatter_y, alpha=0.7)
    ax_scatter.set_xlabel("sign flips / s / wheel")
    ax_scatter.set_ylabel("yaw wobble median (deg)")
    ax_scatter.set_title("(d) yaw wobble vs command flip rate (per car)")

    for ax in (ax_snip, ax_frac, ax_flip, ax_scatter):
        ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    common.save_figure(fig, "fig7_oscillation_attribution.png")
    common.save_results("fig7_oscillation_attribution", results)


if __name__ == "__main__":
    main()
