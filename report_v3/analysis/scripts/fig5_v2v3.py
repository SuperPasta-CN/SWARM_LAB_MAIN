"""Headline comparison: v2 vs v3 final convergence quality.

Version-neutral metric: per-frame similarity-fit shape residual (mm) against
the desired formation shape — rotation/scale/translation are absorbed by the
fit, so it measures pure shape fidelity for both versions.  Two cars are
vacuous for shape (two points always fit); there the native metrics and the
scale lock (fig2) carry the comparison, and are reported in the JSON.

Reference-vertex mapping for v2 runs comes from the role slots
(captain/first_mate define the two distinguished vertices); v3 runs use
their explicit formation coordinates.  Outputs results/fig5_v2v3.json and
figs/fig5_v2v3.png.
"""

from __future__ import annotations

import common

FLEET_LABEL = {"two": "2 cars", "three": "3 cars", "four": "4 cars"}

# Unit reference shapes per fleet size, keyed by role slot.
# crew_three: right angle at the captain, first_mate at +x, crew at +y.
# crew_four (square): captain top-left, first_mate top-right, crew bottom row.
REFERENCE_BY_ROLE = {
    "three": {"captain": 0 + 0j, "first_mate": 1 + 0j, "crew": [0 + 1j]},
    "four": {"captain": 0 + 1j, "first_mate": 1 + 1j, "crew": [0 + 0j, 1 + 0j]},
}


def v2_reference(run_dir):
    """Desired shape for a v2 run as vehicle_id -> complex, via role slots."""

    meta = common.load_metadata(run_dir)
    size_key = "three" if len(meta["vehicles"]) == 3 else "four"
    slots = REFERENCE_BY_ROLE[size_key]
    reference = {}
    crews = []
    for v in meta["vehicles"]:
        role = v["role"]
        if role == "crew":
            crews.append(v["vehicle_id"])
        else:
            reference[v["vehicle_id"]] = slots[role]
    for vid, vertex in zip(sorted(crews), slots["crew"]):
        reference[vid] = vertex
    return reference


def v3_reference(run_dir):
    return common.desired_reference(common.load_metadata(run_dir))


def frame_configs(vehicles):
    """(time, {vehicle_id: complex}) at timestamps where every car has a row."""

    by_time = {}
    for vid, rows in vehicles.items():
        for r in rows:
            by_time.setdefault(round(r["time_s"], 6), {})[vid] = complex(
                r["x_m"], r["y_m"]
            )
    n = len(vehicles)
    return sorted(
        (t, pts) for t, pts in by_time.items() if len(pts) == n
    )


def shape_residual_series(reference, vehicles, steady_last_s=5.0):
    """Per-frame similarity-fit residual (m); returns the steady-window values."""

    configs = frame_configs(vehicles)
    t1 = configs[-1][0]
    steady = [pts for t, pts in configs if t >= t1 - steady_last_s]
    residuals = [common.similarity_fit(reference, pts)[3] for pts in steady]
    scales = [common.similarity_fit(reference, pts)[1] for pts in steady]
    rotations = [common.similarity_fit(reference, pts)[0] for pts in steady]
    return residuals, scales, rotations


def main() -> None:
    fig, axes = common.new_figure(14, 5, 1, 2)
    ax3, ax4 = axes
    results = {}
    for version, runs_by_size, ref_fn, color in (
        ("v2", common.V2_RUNS, v2_reference, "tab:red"),
        ("v3", common.V3_RUNS, v3_reference, "tab:blue"),
    ):
        for size in ("three", "four"):
            ax = ax3 if size == "three" else ax4
            for run_name in runs_by_size[size]:
                run_dir = (common.v3_run_dir(run_name) if version == "v3"
                           else common.v2_run_dir(run_name))
                vehicles = common.load_trajectory(run_dir)
                reference = ref_fn(run_dir)
                residuals, scales, rotations = shape_residual_series(reference, vehicles)
                mean_r, std_r = common.mean_std(residuals)
                mean_s, _ = common.mean_std(scales)
                mean_rot, _ = common.mean_std(rotations)
                key = "%s %s %s" % (version, FLEET_LABEL[size], run_name.split("_")[-2][-4:])
                results[key] = {
                    "run": run_name,
                    "shape_residual_mean_mm": 1000.0 * mean_r,
                    "shape_residual_std_mm": 1000.0 * std_r,
                    "fitted_scale_mean": mean_s,
                    "fitted_rotation_mean_deg": mean_rot * 57.2958,
                }
                # per-frame curve for the panel
                configs = frame_configs(vehicles)
                t1 = configs[-1][0]
                series = [
                    (t, common.similarity_fit(reference, pts)[3])
                    for t, pts in configs
                ]
                ax.plot([t for t, _ in series], [1000.0 * r for _, r in series],
                        color=color, alpha=0.6,
                        label="%s %s" % (version, run_name.split("_")[-2][-4:]))
    for ax, size in ((ax3, "3 cars"), (ax4, "4 cars")):
        ax.set_title("shape-fit residual vs time (%s)" % size)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("residual (mm)")
        ax.legend(loc="best", fontsize=7)
        ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    common.save_figure(fig, "fig5_v2v3.png")
    common.save_results("fig5_v2v3", results)


if __name__ == "__main__":
    main()
