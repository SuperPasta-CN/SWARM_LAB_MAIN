"""Goal 4 supplement: whole-run mean vs initial condition across all 22 runs.

Data collected read-only from WSL ~/swarm_lab_jiang_v1/runs (initial RMS,
duration, whole-run mean, final-third mean, 5-deg convergence time).  Saved
here as analysis artifact; the scatter shows the whole-run mean bearing error
is dominated by initial placement error and run duration, not by the
steady-state behaviour.
"""

from __future__ import annotations

import json

import common

WSL_RUNS = [
    {"dir": "bearing_four_20260728_153509_218847", "duration_s": 16.6, "initial_rms_deg": 134.88, "whole_run_mean_rms_deg": 116.2, "final_third_mean_deg": 109.49, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_four_20260728_153631_858368", "duration_s": 20.05, "initial_rms_deg": 108.11, "whole_run_mean_rms_deg": 88.57, "final_third_mean_deg": 71.55, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_four_20260728_160026_771204", "duration_s": 10.94, "initial_rms_deg": 146.83, "whole_run_mean_rms_deg": 125.69, "final_third_mean_deg": 103.99, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_four_20260728_160124_386968", "duration_s": 14.37, "initial_rms_deg": 22.58, "whole_run_mean_rms_deg": 18.05, "final_third_mean_deg": 17.59, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_four_20260728_160142_287977", "duration_s": 33.26, "initial_rms_deg": 17.59, "whole_run_mean_rms_deg": 14.04, "final_third_mean_deg": 11.74, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_four_20260728_162248_612021", "duration_s": 3.85, "initial_rms_deg": 11.39, "whole_run_mean_rms_deg": 11.15, "final_third_mean_deg": 11.02, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_four_20260728_162304_742492", "duration_s": 5.87, "initial_rms_deg": 10.97, "whole_run_mean_rms_deg": 10.02, "final_third_mean_deg": 9.67, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_four_20260728_162330_485201", "duration_s": 15.77, "initial_rms_deg": 9.66, "whole_run_mean_rms_deg": 3.89, "final_third_mean_deg": 1.25, "t_conv_5deg_s": 4.65, "converged": True},
    {"dir": "bearing_four_20260728_162459_439322", "duration_s": 5.05, "initial_rms_deg": 1.26, "whole_run_mean_rms_deg": 1.21, "final_third_mean_deg": 1.5, "t_conv_5deg_s": 0.0, "converged": True},
    {"dir": "bearing_four_20260728_162713_308328", "duration_s": 56.99, "initial_rms_deg": 27.23, "whole_run_mean_rms_deg": 4.01, "final_third_mean_deg": 3.63, "t_conv_5deg_s": 3.44, "converged": True},
    {"dir": "bearing_three_20260728_153802_289323", "duration_s": 24.08, "initial_rms_deg": 49.05, "whole_run_mean_rms_deg": 16.62, "final_third_mean_deg": 9.14, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_three_20260728_153846_992269", "duration_s": 12.94, "initial_rms_deg": 28.86, "whole_run_mean_rms_deg": 12.96, "final_third_mean_deg": 9.74, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_three_20260728_155849_587082", "duration_s": 9.92, "initial_rms_deg": 27.19, "whole_run_mean_rms_deg": 10.07, "final_third_mean_deg": 2.92, "t_conv_5deg_s": 4.26, "converged": True},
    {"dir": "bearing_three_20260728_160002_755642", "duration_s": 9.11, "initial_rms_deg": 16.38, "whole_run_mean_rms_deg": 9.33, "final_third_mean_deg": 4.7, "t_conv_5deg_s": 6.07, "converged": True},
    {"dir": "bearing_three_20260728_164151_603482", "duration_s": 20.01, "initial_rms_deg": 99.12, "whole_run_mean_rms_deg": 26.37, "final_third_mean_deg": 13.46, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_three_20260728_164307_246986", "duration_s": 12.94, "initial_rms_deg": 26.3, "whole_run_mean_rms_deg": 10.75, "final_third_mean_deg": 2.52, "t_conv_5deg_s": 8.49, "converged": True},
    {"dir": "bearing_three_20260728_164557_324019", "duration_s": 22.83, "initial_rms_deg": 114.61, "whole_run_mean_rms_deg": 36.66, "final_third_mean_deg": 3.06, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_three_20260728_165900_400959", "duration_s": 28.8, "initial_rms_deg": 40.76, "whole_run_mean_rms_deg": 20.68, "final_third_mean_deg": 12.74, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_three_20260728_170047_402569", "duration_s": 8.72, "initial_rms_deg": 27.02, "whole_run_mean_rms_deg": 22.68, "final_third_mean_deg": 24.14, "t_conv_5deg_s": None, "converged": False},
    {"dir": "bearing_three_20260728_170149_685417", "duration_s": 11.92, "initial_rms_deg": 28.98, "whole_run_mean_rms_deg": 7.34, "final_third_mean_deg": 4.01, "t_conv_5deg_s": 3.84, "converged": True},
    {"dir": "bearing_three_20260728_170236_452486", "duration_s": 18.01, "initial_rms_deg": 18.91, "whole_run_mean_rms_deg": 3.83, "final_third_mean_deg": 2.7, "t_conv_5deg_s": 3.04, "converged": True},
    {"dir": "bearing_two_20260728_171106_450483", "duration_s": 14.76, "initial_rms_deg": 71.99, "whole_run_mean_rms_deg": 15.61, "final_third_mean_deg": 1.17, "t_conv_5deg_s": 5.46, "converged": True},
]

REPORT_RUNS = {
    "bearing_two_20260728_171106_450483": ("bearing_2_jyc1", (10, 8)),
    "bearing_three_20260728_170236_452486": ("bearing_3_jyc10", (10, 10)),
    "bearing_four_20260728_162713_308328": ("bearing_4_jyc1", (10, -16)),
}


def main():
    common.setup_figs_dir()
    out = {
        "source": "WSL ~/swarm_lab_jiang_v1/runs (read-only scan 2026-07-28)",
        "runs": WSL_RUNS,
    }
    path = common.RESULTS_DIR / "wsl_runs_overview.json"
    common.RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"results -> {path}")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 6))
    for r in WSL_RUNS:
        kind = r["dir"].split("_2")[0]
        color = {"bearing_two": "C0", "bearing_three": "C1", "bearing_four": "C2"}[kind]
        marker = "o" if r["converged"] else "x"
        ax.plot(
            r["initial_rms_deg"],
            r["whole_run_mean_rms_deg"],
            marker,
            color=color,
            markersize=8,
        )
        if r["dir"] in REPORT_RUNS:
            label, offset = REPORT_RUNS[r["dir"]]
            ax.annotate(
                label,
                (r["initial_rms_deg"], r["whole_run_mean_rms_deg"]),
                textcoords="offset points",
                xytext=offset,
                fontsize=9,
                weight="bold",
            )
    ax.plot([0, 150], [0, 150], color="gray", linestyle="--", alpha=0.5, label="mean = initial")
    ax.set_xlabel("initial overall RMS error (deg)")
    ax.set_ylabel("whole-run mean RMS error (deg)")
    ax.set_title(
        "Whole-run mean error is initial-condition dominated\n"
        "(o = converged <5 deg, x = not converged; 22 runs of 2026-07-28)"
    )
    ax.grid(True, linestyle="--", alpha=0.4)
    handles = [
        plt.Line2D([], [], marker="o", color="C0", linestyle="", label="two cars"),
        plt.Line2D([], [], marker="o", color="C1", linestyle="", label="three cars"),
        plt.Line2D([], [], marker="o", color="C2", linestyle="", label="four cars"),
        plt.Line2D([], [], marker="x", color="gray", linestyle="", label="not converged"),
    ]
    ax.legend(handles=handles)
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig4c_initial_vs_mean.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")


if __name__ == "__main__":
    main()
