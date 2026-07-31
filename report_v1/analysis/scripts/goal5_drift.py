"""Goal 5: formation-level drift under agent_mode=all_bearing.

all_bearing leaves translation / rotation / scale uncontrolled.  Quantify per
run, against the first recorded frame:
- centroid displacement |c(t) - c(0)| and drift rate (whole run + last 5 s),
- rigid rotation angle theta(t) (unwrapped) and rate,
- scale factor s(t) relative to the initial configuration,
- non-rigid residual of the similarity fit (shape deformation, mm).

Two-car run: similarity transform of a pair still well-defined (rotation of
the relative vector + distance ratio).
"""

from __future__ import annotations

from math import degrees, pi, sqrt

import common


def unwrap(phases):
    out = [phases[0]]
    for p in phases[1:]:
        prev = out[-1]
        while p - prev > pi:
            p -= 2 * pi
        while p - prev < -pi:
            p += 2 * pi
        out.append(p)
    return out


def analyze_run(name):
    vehicles = common.load_trajectory(common.RUNS[name])
    configs = common.configurations(vehicles)
    times = [t for t, _ in configs]
    reference = configs[0][1]
    rotations, scales, cx, cy, resids = [], [], [], [], []
    for _, pts in configs:
        rotation, scale, translation, residual = common.similarity_fit(
            reference, pts
        )
        rotations.append(rotation)
        scales.append(scale)
        centroid = sum(pts.values()) / len(pts)
        cx.append(centroid.real)
        cy.append(centroid.imag)
        resids.append(residual)
    rotations = unwrap(rotations)
    duration = times[-1] - times[0]

    c0 = complex(cx[0], cy[0])
    cend = complex(cx[-1], cy[-1])
    disp_total = abs(cend - c0)
    # Steady-window rates (last 5 s).
    idx5 = [i for i, t in enumerate(times) if t >= times[-1] - 5.0]
    i0 = idx5[0]
    c5disp = abs(complex(cx[-1], cy[-1]) - complex(cx[i0], cy[i0]))
    rot5 = rotations[-1] - rotations[i0]
    t5 = times[-1] - times[i0]
    # Path length of centroid (captures non-monotone wander).
    cpath = sum(
        abs(complex(bx, by) - complex(ax, ay))
        for (ax, ay), (bx, by) in zip(zip(cx, cy), zip(cx[1:], cy[1:]))
    )

    scale_dev = [abs(s - 1.0) for s in scales]
    # Reference = mean configuration of the last 5 s: pure shape jitter.
    idx5 = [i for i, t in enumerate(times) if t >= times[-1] - 5.0]
    steady_ref = {
        vehicle_id: sum(pts[vehicle_id] for _, pts in configs[i0:])
        / (len(configs) - i0)
        for vehicle_id in reference
    }
    jitter_resids = []
    for _, pts in configs[i0:]:
        _, _, _, residual = common.similarity_fit(steady_ref, pts)
        jitter_resids.append(residual)
    return {
        "duration_s": duration,
        "centroid_start": (cx[0], cy[0]),
        "centroid_end": (cx[-1], cy[-1]),
        "centroid_displacement_m": disp_total,
        "centroid_path_length_m": cpath,
        "centroid_mean_speed_mps": cpath / duration,
        "centroid_last5s_speed_mps": c5disp / t5 if t5 > 0 else 0.0,
        "rotation_total_deg": degrees(rotations[-1] - rotations[0]),
        "rotation_rate_dps": degrees(rotations[-1] - rotations[0]) / duration,
        "rotation_last5s_rate_dps": degrees(rot5) / t5 if t5 > 0 else 0.0,
        "scale_final": scales[-1],
        "scale_max_deviation": max(scale_dev),
        "scale_change_last5s": scales[-1] - scales[i0],
        "shape_jitter_rms_mm": common.rms(jitter_resids) * 1000.0,
        "shape_change_from_initial_mm": resids[-1] * 1000.0,
        "series": {
            "t": times,
            "cdisp": [
                abs(complex(x, y) - c0) for x, y in zip(cx, cy)
            ],
            "rot_deg": [degrees(r - rotations[0]) for r in rotations],
            "scale": scales,
            "resid_mm": [r * 1000.0 for r in resids],
        },
    }


def main():
    common.setup_figs_dir()
    results = {}
    for name in common.RUNS:
        r = analyze_run(name)
        results[name] = r
        print(f"=== {name} ===")
        print(
            f"  centroid: disp={r['centroid_displacement_m']:.3f}m "
            f"path={r['centroid_path_length_m']:.3f}m "
            f"mean_speed={r['centroid_mean_speed_mps']*1000:.1f}mm/s "
            f"last5s_speed={r['centroid_last5s_speed_mps']*1000:.1f}mm/s"
        )
        print(
            f"  rotation: total={r['rotation_total_deg']:+.1f}deg "
            f"rate={r['rotation_rate_dps']:+.2f}deg/s "
            f"last5s_rate={r['rotation_last5s_rate_dps']:+.2f}deg/s"
        )
        print(
            f"  scale: final={r['scale_final']:.4f} max_dev={r['scale_max_deviation']:.4f} "
            f"last5s_change={r['scale_change_last5s']:+.4f} "
            f"jitter_rms={r['shape_jitter_rms_mm']:.2f}mm "
            f"from_initial={r['shape_change_from_initial_mm']:.1f}mm"
        )

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = list(common.RUNS)
    fig, axes = plt.subplots(3, 4, figsize=(18, 10))
    for row, name in enumerate(runs):
        s = results[name]["series"]
        axes[row][0].plot(s["t"], s["cdisp"], color="C0")
        axes[row][0].set_ylabel(f"{name}\n|dc| (m)")
        axes[row][0].set_title("centroid displacement")
        axes[row][1].plot(s["t"], s["rot_deg"], color="C1")
        axes[row][1].set_title("rotation (deg)")
        axes[row][2].plot(s["t"], s["scale"], color="C2")
        axes[row][2].set_title("scale")
        axes[row][3].plot(s["t"], s["resid_mm"], color="C3")
        axes[row][3].set_title("shape residual (mm)")
        for col in range(4):
            axes[row][col].grid(True, linestyle="--", alpha=0.4)
            axes[row][col].set_xlabel("time (s)")
    fig.suptitle("all_bearing uncontrolled degrees of freedom (vs first frame)")
    fig.tight_layout()
    fig_path = common.FIGS_DIR / "fig5_formation_drift.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)
    print(f"figure -> {fig_path}")

    for name in results:
        results[name].pop("series")
    common.save_results("goal5_drift", results)


if __name__ == "__main__":
    main()
