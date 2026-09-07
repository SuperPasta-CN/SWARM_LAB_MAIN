#!/usr/bin/env python3
"""Scalar limit-cycle simulation for the cruise speed loop (v3 oscillation).

Purpose: reproduce, in a minimal model, the three observed behaviors —
lift (bang-bang), affine (continuous but still oscillating, worse), pwm
(calm) — then sweep parameters to find what actually kills the cycle.

Model (per car, cruise direction):
- Plant: first-order speed dynamics toward the command-implied steady
  speed v_ss(cmd), with a stiction dead zone:
    |cmd| < C_REL   -> thrust 0 (stopped car stays stopped; moving coasts)
    |cmd| >= 35     -> v_ss = sign(cmd) * (V_ON + (|cmd|-35) * G)
  calibrated from the 09-07 runs: V_ON = 0.12 m/s (measured speed during
  +35 phases), G = 0.005 m/s per command point (v(100) ~ 0.45).
- Velocity estimate: first-order lag TAU_EST on the true speed (stands in
  for position-difference baseline/2 + EMA; alpha is mapped via
  TAU_EST = baseline/2 + (1-alpha)/alpha * 0.01).
- Controller: feedforward + PI exactly as swarm/algorithms/mecanum_pid.py
  (kp=0.6, ki=1.2, integral_limit=0.10, output_limit=0.15, calib 0.5).
- Actuator modes: "ideal" (no dead zone), "lift", "affine", "pwm"
  (the v3 DeadzoneCompensator implementations).

Outputs: results/sim_limit_cycle.json and figs/fig8_sim_limit_cycle.png.
"""

from __future__ import annotations

import sys
from math import sqrt
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

DT = 0.02          # 50 Hz control loop
DURATION = 30.0
TARGET = 0.10      # cruise speed
CALIB = 0.5        # command 100 <=> 0.5 m/s (config calibration)
TAU_ACC = 0.55     # spin-up time constant under thrust (s), tuned to the 09-07 runs
TAU_COAST = 0.25   # friction braking time constant (s), tuned to the 09-07 runs
C_REL = 35.0       # stiction RELEASE threshold: stopped car starts here
C_HOLD = 15.0      # but a moving car keeps rolling down to here (hysteresis)
V_ON = 0.12        # sustained speed at cmd=35 (m/s), measured from 09-07 data
GAIN = 0.005       # v_ss slope above cmd=35 (m/s per command point)
EST_NOISE = 0.004  # velocity estimate noise std (m/s), mocap through estimator


def v_ss(cmd, moving):
    threshold = C_HOLD if moving else C_REL
    if abs(cmd) < threshold:
        return None  # no thrust: coast by friction
    return (V_ON + (abs(cmd) - 35.0) * GAIN) * (1.0 if cmd > 0 else -1.0)


class PID:
    def __init__(self, kp, ki, integral_limit, output_limit):
        self.kp, self.ki = kp, ki
        self.il, self.ol = integral_limit, output_limit
        self.integral = 0.0

    def update(self, target, measured, dt):
        err = target - measured
        self.integral = max(-self.il, min(self.il, self.integral + err * dt))
        return max(-self.ol, min(self.ol, self.kp * err + self.ki * self.integral))


def run(mode, kp=0.6, ki=1.2, tau_est=0.15, pwm_period=10, seed_target=TARGET, seed=7):
    import random

    rng = random.Random(seed)
    pid = PID(kp, ki, 0.10, 0.15)
    v = 0.0
    v_est = 0.0
    steps = int(DURATION / DT)
    hist = {"t": [], "v": [], "cmd": []}
    cycle = 0
    for step in range(steps):
        t = step * DT
        target = seed_target if t >= 3.0 else 0.0
        if abs(target) < 1e-9:
            pid.integral = 0.0
            u = 0.0
        else:
            u = target + pid.update(target, v_est, DT)
        raw = u / CALIB * 100.0
        # actuator
        if mode == "ideal":
            cmd = raw
        elif mode == "lift":
            cmd = raw
            if 0.0 < abs(cmd) < 35.0:
                cmd = 35.0 if cmd > 0.0 else -35.0
        elif mode == "affine":
            cmd = 0.0 if raw == 0.0 else (35.0 + min(abs(raw), 100.0) * 0.65) * (
                1.0 if raw > 0 else -1.0)
        elif mode == "pwm":
            cycle = (cycle + 1) % pwm_period
            strongest = abs(raw)
            if strongest == 0.0 or strongest >= 35.0:
                cmd = raw
            else:
                duty = strongest / 35.0
                cmd = raw * (35.0 / strongest) if cycle < duty * pwm_period else 0.0
        else:
            raise ValueError(mode)
        # plant: stiction hysteresis + asymmetric dynamics
        thrust = v_ss(cmd, moving=abs(v) > 1e-3)
        if thrust is None:
            v += (0.0 - v) * DT / TAU_COAST
        else:
            v += (thrust - v) * DT / TAU_ACC
        # estimator: first-order lag + noise
        v_est += (v - v_est) * DT / tau_est + rng.gauss(0.0, EST_NOISE) * sqrt(DT)
        hist["t"].append(t)
        hist["v"].append(v)
        hist["cmd"].append(cmd)
    return hist


def metrics(hist, tail_s=15.0):
    t, v, cmd = hist["t"], hist["v"], hist["cmd"]
    tail = [(tt, vv, cc) for tt, vv, cc in zip(t, v, cmd) if tt >= DURATION - tail_s]
    speeds = [vv for _, vv, _ in tail]
    cmds = [cc for _, _, cc in tail]
    mean_v = sum(speeds) / len(speeds)
    ripple = (
        sum(abs(x - mean_v) for x in speeds) / len(speeds) / mean_v
        if abs(mean_v) > 1e-9 else float("nan")
    )
    flips = sum(
        1 for a, b in zip(cmds, cmds[1:]) if (a > 0) != (b > 0) and a != 0 and b != 0
    )
    return {
        "mean_speed": mean_v,
        "tracking_error_pct": 100.0 * (mean_v - TARGET) / TARGET,
        "ripple_mad_pct": 100.0 * ripple,
        "flips_per_s": flips / tail_s,
    }


def main() -> None:
    results = {"baseline_modes": {}, "sweeps": {}}

    # 1) reproduce the three observed behaviors at default settings
    for mode in ("ideal", "lift", "affine", "pwm"):
        m = metrics(run(mode))
        results["baseline_modes"][mode] = m
        print("%-7s | mean %.3f (%+.0f%%) | ripple(MAD) %.0f%% | flips %.1f/s" % (
            mode, m["mean_speed"], m["tracking_error_pct"],
            m["ripple_mad_pct"], m["flips_per_s"]))

    # 2) sweeps: what actually kills the cycle?
    print("\n-- kp/ki sweep per mode (ripple % / flips per s) --")
    for mode in ("lift", "affine", "pwm"):
        grid = {}
        for kp in (0.2, 0.4, 0.6, 1.0):
            for ki in (0.0, 0.4, 0.8, 1.2):
                m = metrics(run(mode, kp=kp, ki=ki))
                grid["kp=%.1f,ki=%.1f" % (kp, ki)] = m
        results["sweeps"][mode] = grid
        for key, m in grid.items():
            if m["ripple_mad_pct"] < 20.0 and abs(m["tracking_error_pct"]) < 15.0:
                print("  GOOD %-6s %s -> mean %.3f ripple %.0f%% flips %.1f" % (
                    mode, key, m["mean_speed"], m["ripple_mad_pct"], m["flips_per_s"]))

    print("\n-- pwm period sweep (kp=0.6, ki=1.2) --")
    for period in (4, 6, 10, 20):
        m = metrics(run("pwm", pwm_period=period))
        print("  period %2d -> ripple %.0f%% flips %.1f/s" % (
            period, m["ripple_mad_pct"], m["flips_per_s"]))
        results["sweeps"].setdefault("pwm_period", {})[str(period)] = m

    print("\n-- estimator lag sweep (pwm, kp=0.6, ki=1.2) --")
    for tau in (0.05, 0.08, 0.12, 0.2):
        m = metrics(run("pwm", tau_est=tau))
        print("  tau_est %.2f s -> ripple %.0f%% flips %.1f/s" % (
            tau, m["ripple_mad_pct"], m["flips_per_s"]))
        results["sweeps"].setdefault("tau_est", {})[str(tau)] = m

    # figure: waveforms for the three observed modes
    fig, axes = common.new_figure(14, 8, 2, 2)
    ax_v, ax_c, ax_v2, ax_c2 = axes[0][0], axes[0][1], axes[1][0], axes[1][1]
    for mode, axv, axc in (("lift", ax_v, ax_c), ("affine", ax_v2, ax_c2)):
        h = run(mode)
        axv.plot(h["t"], h["v"], label="actual speed")
        axv.axhline(TARGET, color="gray", linestyle="--", label="target")
        axc.plot(h["t"], h["cmd"], color="tab:red", label="wheel command")
        axv.set_title("mode=%s: speed" % mode)
        axc.set_title("mode=%s: command" % mode)
        axv.set_ylim(-0.05, 0.3)
        axv.legend(loc="best", fontsize=8)
    h = run("pwm")
    ax_v.plot(h["t"], h["v"], color="tab:green", label="pwm speed")
    ax_c.plot(h["t"], h["cmd"], color="tab:green", alpha=0.6, label="pwm cmd")
    for ax in (ax_v, ax_c, ax_v2, ax_c2):
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_xlabel("time (s)")
    fig.tight_layout()
    common.save_figure(fig, "fig8_sim_limit_cycle.png")
    common.save_results("sim_limit_cycle", results)


if __name__ == "__main__":
    main()
