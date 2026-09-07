"""Fig 9: the oscillation mitigation A/B result (09-07) and per-vehicle v_on
normalization result (09-07 afternoon), as grouped bar charts over the four
judgment metrics.  Outputs results/fig9_ab_comparison.json and
figs/fig9_ab_comparison.png.
"""

from __future__ import annotations

import statistics

import common

GROUPS = [
    ("v3_two_20260907_111125_857617_initial", "baseline lift", "tab:red"),
    ("v3_two_affine_20260907_111519_011279", "affine", "tab:orange"),
    ("v3_two_pwm_20260907_111631_055165", "pwm (no v_on)", "tab:blue"),
    ("v3_two_20260907_134254_486615", "pwm + v_on (w=0.1)", "tab:green"),
    ("v3_two_20260907_134423_445499", "pwm + v_on (w=0.2)", "tab:purple"),
]

WHEELS = ("front_left_command", "front_right_command",
          "rear_left_command", "rear_right_command")


def run_metrics(name):
    d = common.REPORT_DIR / "runs_ab" / name
    vehicles = common.load_trajectory(d)
    edges = common.load_v3_edges(d)
    et, er = common.overall_rms(
        [[(r["time_s"], r["edge_error_m"]) for r in rows] for rows in edges.values()]
    )
    t_end = et[-1]
    err, err_std = common.mean_std([v for t, v in zip(et, er) if t >= t_end - 10.0])
    flips, ripples, achievements = [], [], []
    for vid in sorted(vehicles):
        rows = vehicles[vid]
        tail = [r for r in rows if r["time_s"] >= t_end - 10.0]
        if len(tail) < 5:
            continue
        dur = tail[-1]["time_s"] - tail[0]["time_s"]
        flip = 0
        for key in WHEELS:
            seq = [r[key] for r in tail]
            flip += sum(1 for a, b in zip(seq, seq[1:])
                        if (a > 0) != (b > 0) and a != 0.0 and b != 0.0)
        flips.append(flip / 4 / max(dur, 1e-9))
        speeds = [
            (vx * vx + vy * vy) ** 0.5
            for _, vx, vy in common.velocity_series(tail, 0.4)
        ]
        mean_s = statistics.mean(speeds)
        ripples.append(
            100.0 * statistics.median([abs(s - mean_s) / mean_s for s in speeds])
        )
        tgt = statistics.mean(
            [(r["target_vx_mps"] ** 2 + r["target_vy_mps"] ** 2) ** 0.5 for r in tail]
        )
        achievements.append(100.0 * mean_s / tgt)
    return {
        "steady_error_m": err,
        "steady_error_std_m": err_std,
        "flips_per_s": statistics.mean(flips),
        "ripple_median_pct": statistics.mean(ripples),
        "achievement_pct": statistics.mean(achievements),
    }


def main() -> None:
    fig, axes = common.new_figure(14, 8, 2, 2)
    metrics = {}
    for name, label, _ in GROUPS:
        metrics[label] = run_metrics(name)
    panels = [
        ("steady_error_m", "steady edge error (m)", 1.0),
        ("flips_per_s", "wheel sign flips (1/s)", 1.0),
        ("ripple_median_pct", "speed ripple median (%)", 1.0),
        ("achievement_pct", "speed achievement (%)", 1.0),
    ]
    for ax, (key, title, scale) in zip(
        (axes[0][0], axes[0][1], axes[1][0], axes[1][1]), panels
    ):
        values = [metrics[label][key] * scale for _, label, _ in GROUPS]
        bars = ax.bar([label for _, label, _ in GROUPS], values,
                      color=[color for _, _, color in GROUPS], alpha=0.85)
        ax.set_title(title)
        ax.tick_params(axis="x", labelsize=7, rotation=15)
        ax.grid(True, linestyle="--", alpha=0.4, axis="y")
        for bar, v in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    "%.3g" % v, ha="center", va="bottom", fontsize=8)
    axes[0][0].axhline(0.04, color="gray", linestyle="--", alpha=0.6)
    axes[1][1].axhline(100.0, color="gray", linestyle="--", alpha=0.6)
    fig.tight_layout()
    common.save_figure(fig, "fig9_ab_comparison.png")
    common.save_results("fig9_ab_comparison", metrics)


if __name__ == "__main__":
    main()
