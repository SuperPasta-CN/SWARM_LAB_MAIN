"""The oscillation arc in three rounds, quantified (report figures).

fig10_waveforms.png: same-axes speed/command waveforms for four
representative runs (baseline lift / affine / pwm / pwm+v_on).
fig11_ab_matrix.png: the full 09-07 A/B matrix (6 groups x 4 metrics).
fig12_von.png: per-vehicle v_on normalization (calibration values,
pre/post achievement, edge-RMS curves at both cruise speeds).
Outputs figs + results/fig10_12_arc.json.
"""

from __future__ import annotations

import statistics

import common

WHEELS = ("front_left_command", "front_right_command",
          "rear_left_command", "rear_right_command")

R_AB = "runs_ab"


def _dir(name):
    return common.REPORT_DIR / R_AB / name


def _metrics(name):
    d = _dir(name)
    vehicles = common.load_trajectory(d)
    edges = common.load_v3_edges(d)
    et, er = common.overall_rms(
        [[(r["time_s"], r["edge_error_m"]) for r in rows] for rows in edges.values()]
    )
    t_end = et[-1]
    err, err_std = common.mean_std([v for t, v in zip(et, er) if t >= t_end - 10.0])
    out = {"steady_m": err, "steady_std_m": err_std, "cars": {}}
    for vid in sorted(vehicles):
        rows = vehicles[vid]
        tail = [r for r in rows if r["time_s"] >= t_end - 10.0]
        if len(tail) < 5:
            continue
        dur = tail[-1]["time_s"] - tail[0]["time_s"]
        flips = 0
        for key in WHEELS:
            seq = [r[key] for r in tail]
            flips += sum(1 for a, b in zip(seq, seq[1:])
                         if (a > 0) != (b > 0) and a != 0.0 and b != 0.0)
        speeds = [(vx**2 + vy**2) ** 0.5 for _, vx, vy in common.velocity_series(tail, 0.4)]
        mean_s = statistics.mean(speeds)
        tgt = statistics.mean(
            [(r["target_vx_mps"] ** 2 + r["target_vy_mps"] ** 2) ** 0.5 for r in tail])
        yaw = [r["yaw_rad"] * 57.2958 for r in tail]
        ym = statistics.mean(yaw)
        out["cars"][vid] = {
            "flips_per_s": flips / 4 / max(dur, 1e-9),
            "ripple_pct": 100.0 * statistics.median(
                [abs(s - mean_s) / mean_s for s in speeds]) if mean_s > 1e-9 else 0.0,
            "achieve_pct": 100.0 * mean_s / tgt if tgt > 1e-9 else 0.0,
            "yaw_wobble_deg": statistics.median([abs(y - ym) for y in yaw]),
        }
    return out


def fig10() -> None:
    runs = [
        ("v3_two_20260907_111125_857617_initial", "R1 baseline (lift)"),
        ("v3_two_affine_20260907_111519_011279", "R2 affine (dead-zone inverse)"),
        ("v3_two_pwm_20260907_111631_055165", "R2 pwm (duty-cycle)"),
        ("v3_two_20260907_134254_486615", "R3 pwm + per-car v_on (best)"),
    ]
    fig, axes = common.new_figure(14, 9, 2, 2)
    for ax, (name, title) in zip((axes[0][0], axes[0][1], axes[1][0], axes[1][1]), runs):
        d = _dir(name)
        vehicles = common.load_trajectory(d)
        vid = sorted(vehicles)[0]
        rows = vehicles[vid]
        t_end = rows[-1]["time_s"]
        tail = [r for r in rows if t_end - 12.0 <= r["time_s"] <= t_end]
        sp = common.velocity_series(tail, 0.2)
        ax.plot([s[0] for s in sp], [(s[1] ** 2 + s[2] ** 2) ** 0.5 for s in sp],
                color="tab:blue", label="|v|")
        ax.set_ylim(-0.05, 0.30)
        ax.set_ylabel("speed (m/s)", color="tab:blue")
        ax2 = ax.twinx()
        ax2.plot([r["time_s"] for r in tail], [r["front_left_command"] for r in tail],
                 color="tab:red", alpha=0.7, label="FL cmd")
        ax2.set_ylim(-110, 110)
        ax2.set_ylabel("wheel cmd", color="tab:red")
        ax.set_title(title)
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_xlabel("time (s)")
    fig.tight_layout()
    common.save_figure(fig, "fig10_waveforms.png")


def fig11() -> None:
    groups = [
        ("v3_two_20260907_111125_857617_initial", "baseline"),
        ("v3_two_affine_20260907_111519_011279", "affine"),
        ("v3_two_pwm_20260907_111631_055165", "pwm"),
        ("v3_two_hdb_20260907_111729_273986", "hdb"),
        ("v3_two_ema_20260907_111852_657274", "ema"),
        ("v3_two_all_20260907_112017_386743", "all"),
    ]
    mets = {label: _metrics(name) for name, label in groups}
    fig, axes = common.new_figure(14, 8, 2, 2)
    panels = [
        ("flips_per_s", "wheel sign flips (1/s)"),
        ("ripple_pct", "speed ripple median (%)"),
        ("yaw_wobble_deg", "yaw wobble median (deg)"),
        ("steady_m", "steady edge error (m)"),
    ]
    for ax, (key, title) in zip((axes[0][0], axes[0][1], axes[1][0], axes[1][1]), panels):
        if key == "steady_m":
            values = [mets[label][key] for _, label in groups]
        else:
            values = [statistics.mean(c[key] for c in mets[label]["cars"].values())
                      for _, label in groups]
        colors = ["tab:red" if label == "baseline" else
                  "tab:green" if label == "pwm" else "tab:gray" for _, label in groups]
        bars = ax.bar([label for _, label in groups], values, color=colors, alpha=0.85)
        for bar, v in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    "%.3g" % v, ha="center", va="bottom", fontsize=8)
        ax.set_title(title)
        ax.tick_params(axis="x", labelsize=8)
        ax.grid(True, linestyle="--", alpha=0.4, axis="y")
    fig.tight_layout()
    common.save_figure(fig, "fig11_ab_matrix.png")


def fig12() -> None:
    fig, axes = common.new_figure(14, 9, 2, 2)
    ax_cal, ax_ach, ax_c1, ax_c2 = axes[0][0], axes[0][1], axes[1][0], axes[1][1]

    # (a) calibration values per car (raw marks + v35 samples)
    import json
    for i, vid in enumerate(("car4", "car5")):
        rec = json.load(open(
            common.REPORT_DIR.parent / "swarm_lab_jiang_v3" / "tools"
            / "calibrations" / ("%s.json" % vid), encoding="utf-8"))
        xs = [i - 0.15 + 0.1 * k for k in range(len(rec["min_eff_raw_marks"]))]
        ax_cal.scatter(xs, rec["min_eff_raw_marks"], label="%s min_eff marks" % vid)
        ax_cal.scatter(xs, [v * 100 for v in rec["v35_samples_mps"]], marker="s",
                       label="%s v(35) x100" % vid)
    ax_cal.set_title("(a) per-car calibration: stiction cmd and v(35)x100")
    ax_cal.set_xticks([0, 1])
    ax_cal.set_xticklabels(["car4", "car5"])
    ax_cal.legend(loc="best", fontsize=8)
    ax_cal.grid(True, linestyle="--", alpha=0.4)

    # (b) achievement pre vs post per car
    pairs = [
        ("v3_two_20260907_122557_442294", "pre w0.1"),
        ("v3_two_20260907_123409_854970", "pre w0.2"),
        ("v3_two_20260907_134254_486615", "post w0.1"),
        ("v3_two_20260907_134423_445499", "post w0.2"),
    ]
    width = 0.18
    for k, (name, label) in enumerate(pairs):
        m = _metrics(name)
        xs = [0, 1]
        vals = [m["cars"].get(vid, {}).get("achieve_pct", 0.0) for vid in ("car4", "car5")]
        ax_ach.bar([x + (k - 1.5) * width for x in xs], vals, width=width, label=label)
        for x, v in zip(xs, vals):
            ax_ach.text(x + (k - 1.5) * width, v, "%.0f" % v,
                        ha="center", va="bottom", fontsize=7)
    ax_ach.axhline(100.0, color="gray", linestyle="--", alpha=0.6)
    ax_ach.set_title("(b) speed achievement per car, before/after v_on")
    ax_ach.set_xticks([0, 1])
    ax_ach.set_xticklabels(["car4", "car5"])
    ax_ach.set_ylabel("achievement (%)")
    ax_ach.legend(loc="best", fontsize=7)
    ax_ach.grid(True, linestyle="--", alpha=0.4, axis="y")

    # (c)(d) edge RMS curves at both cruise speeds, pre vs post
    for ax, pre, post, tag in (
        (ax_c1, "v3_two_20260907_122557_442294", "v3_two_20260907_134254_486615", "w=0.1"),
        (ax_c2, "v3_two_20260907_123409_854970", "v3_two_20260907_134423_445499", "w=0.2"),
    ):
        for name, color, lbl in ((pre, "tab:red", "pre (no v_on)"),
                                 (post, "tab:green", "post (v_on)")):
            d = _dir(name)
            edges = common.load_v3_edges(d)
            et, er = common.overall_rms(
                [[(r["time_s"], r["edge_error_m"]) for r in rows] for rows in edges.values()])
            ax.plot(et, er, color=color, alpha=0.75, label=lbl)
        ax.set_title("(%s) edge-error RMS, %s" % ("c" if tag == "w=0.1" else "d", tag))
        ax.set_xlabel("time (s)")
        ax.set_ylabel("edge error RMS (m)")
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    common.save_figure(fig, "fig12_von.png")


def main() -> None:
    fig10()
    fig11()
    fig12()
    results = {}
    for name, label in [
        ("v3_two_20260907_111125_857617_initial", "baseline"),
        ("v3_two_affine_20260907_111519_011279", "affine"),
        ("v3_two_pwm_20260907_111631_055165", "pwm"),
        ("v3_two_hdb_20260907_111729_273986", "hdb"),
        ("v3_two_ema_20260907_111852_657274", "ema"),
        ("v3_two_all_20260907_112017_386743", "all"),
        ("v3_two_20260907_122557_442294", "pre_von_w0.1"),
        ("v3_two_20260907_122709_054271", "pre_von_w0.2_fg"),
        ("v3_two_20260907_123409_854970", "pre_von_w0.2"),
        ("v3_two_20260907_134254_486615", "post_von_w0.1"),
        ("v3_two_20260907_134423_445499", "post_von_w0.2"),
    ]:
        results[label] = _metrics(name)
    common.save_results("fig10_12_arc", results)


if __name__ == "__main__":
    main()
