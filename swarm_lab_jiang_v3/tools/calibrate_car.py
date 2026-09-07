#!/usr/bin/env python3
"""Per-vehicle dead-zone calibration for the mecanum chassis.

Measures, per vehicle, the two numbers that bound low-speed accuracy:

1. **true min_eff** (the "just rolling" command): all-wheel command ramps
   0 -> ramp-max; the operator presses ENTER the moment the vehicle starts
   rolling.  Repeat and take the median, then add a safety margin
   (default +5) -> ``execution.wheel_command_min_effective``.
2. **v(35)** (sustained speed at the min_eff command): drive at a fixed
   command for a measured window after a spin-up; the operator measures
   the distance with a tape and enters it -> average speed.  This is the
   PWM accuracy ceiling: mean speed = duty * v(min_eff).

Outputs ``tools/calibrations/<address>.json`` and a ready-to-paste
``VehicleExecutionOverride`` snippet.

Usage:
    python tools/calibrate_car.py --address 10.1.1.84
    python tools/calibrate_car.py --address 10.1.1.84 --repeats 3 --margin 5
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

# Allow running both as ``python tools/calibrate_car.py`` and as a module.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from swarm.infrastructure.udp_chassis import ChassisSocketDriver, SocketConfig

_ZERO = {"front_left": 0.0, "front_right": 0.0, "rear_left": 0.0, "rear_right": 0.0}


def compute_min_eff(marks, margin: float) -> float:
    """Median of the marked ramp commands plus the safety margin."""

    return median(marks) + margin


def compute_speed(distance_m: float, seconds: float) -> float:
    if seconds <= 0.0:
        raise ValueError("seconds must be positive")
    return distance_m / seconds


def median(values) -> float:
    values = sorted(values)
    n = len(values)
    if n == 0:
        raise ValueError("no values")
    mid = n // 2
    if n % 2 == 1:
        return float(values[mid])
    return (values[mid - 1] + values[mid]) / 2.0


def format_override(vehicle_id: str, min_eff: float, v35: float) -> str:
    """Ready-to-paste config snippet for the calibration result."""

    return (
        "VehicleConfig(\n"
        '    "%s", "ground_vehicle", "<IP>",\n'
        "    execution_override=VehicleExecutionOverride(\n"
        "        wheel_command_min_effective=%.1f,\n"
        "    ),\n"
        ")  # v(35)=%.3f m/s (PWM accuracy ceiling; keep for reference)"
        % (vehicle_id, min_eff, v35)
    )


def _enter_watcher(stop_event: threading.Event) -> None:
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
    stop_event.set()


def _ramp_until_mark(
    driver: ChassisSocketDriver,
    command_max: float,
    seconds: float,
    hz: float = 20.0,
) -> float:
    """Ramp all wheels forward; return the command when ENTER is hit."""

    stop = threading.Event()
    watcher = threading.Thread(target=_enter_watcher, args=(stop,), daemon=True)
    watcher.start()
    print("calibrate | ramping 0 -> %.0f over %.1f s; press ENTER the instant the vehicle STARTS rolling"
          % (command_max, seconds))
    period = 1.0 / hz
    started = time.monotonic()
    marked = None
    last_print = -1.0
    while True:
        elapsed = time.monotonic() - started
        ratio = min(1.0, elapsed / seconds)
        command = command_max * ratio
        if stop.is_set() and marked is None:
            marked = command
            break
        if elapsed - last_print >= 0.5:
            last_print = elapsed
            print("  cmd = %5.1f" % command)
        driver.send(
            {
                "front_left": command,
                "front_right": command,
                "rear_left": command,
                "rear_right": command,
            }
        )
        if ratio >= 1.0:
            break
        time.sleep(period)
    driver.send(_ZERO)
    if marked is None:
        print("calibrate | ENTER never pressed; using ramp end %.0f" % command_max)
        return command_max
    return marked


def _drive_for(driver: ChassisSocketDriver, command: float, seconds: float, hz: float = 20.0) -> None:
    wheels = {
        "front_left": command,
        "front_right": command,
        "rear_left": command,
        "rear_right": command,
    }
    period = 1.0 / hz
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        driver.send(wheels)
        time.sleep(period)
    driver.send(_ZERO)


def _measure_speed_once(
    driver: ChassisSocketDriver,
    command: float,
    spinup_s: float,
    measure_s: float,
) -> float:
    print(
        "calibrate | spin-up %.1f s, then measuring %.1f s at cmd=%.0f"
        % (spinup_s, measure_s, command)
    )
    _drive_for(driver, command, spinup_s)
    print("calibrate | MEASURING now — mark the start point with tape/chalk")
    _drive_for(driver, command, measure_s)
    print("calibrate | measure window over.")
    while True:
        raw = input("calibrate | enter the measured distance in cm (r to retry): ").strip()
        if raw.lower() == "r":
            return _measure_speed_once(driver, command, spinup_s, measure_s)
        try:
            distance_m = float(raw) / 100.0
        except ValueError:
            print("calibrate | not a number, try again")
            continue
        if distance_m <= 0.0:
            print("calibrate | distance must be positive, try again")
            continue
        return compute_speed(distance_m, measure_s)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", required=True, help="vehicle IP, e.g. 10.1.1.84")
    parser.add_argument("--vehicle-id", default=None, help="id for the snippet (default: car<last octet>)")
    parser.add_argument("--port", type=int, default=12345)
    parser.add_argument("--repeats", type=int, default=3, help="repeats per measurement (default: 3)")
    parser.add_argument("--ramp-max", type=float, default=60.0, help="ramp end command (default: 60)")
    parser.add_argument("--ramp-seconds", type=float, default=25.0, help="ramp duration (default: 25 s)")
    parser.add_argument("--margin", type=float, default=5.0, help="min_eff safety margin (default: +5)")
    parser.add_argument("--speed-cmd", type=float, default=35.0, help="v() measurement command (default: 35)")
    parser.add_argument("--spinup", type=float, default=1.0, help="spin-up seconds before measuring (default: 1.0)")
    parser.add_argument("--measure-seconds", type=float, default=3.0, help="measurement window (default: 3.0 s)")
    parser.add_argument("--yes", action="store_true", help="skip the start confirmations")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    vehicle_id = args.vehicle_id or "car%s" % args.address.rsplit(".", 1)[-1].lstrip("0")
    driver = ChassisSocketDriver(args.address, SocketConfig(port=args.port))
    print("calibrate | %s (%s:%d), repeats=%d" % (vehicle_id, args.address, args.port, args.repeats))
    print("calibrate | clear ~1.5 m of floor ahead of the vehicle")
    marks = []
    speeds = []
    try:
        for round_index in range(1, args.repeats + 1):
            print("\n== round %d/%d: min_eff ramp ==" % (round_index, args.repeats))
            if not args.yes:
                input("  press ENTER to start (Ctrl-C to abort) ")
            marks.append(_ramp_until_mark(driver, args.ramp_max, args.ramp_seconds))
            print("  marked min_eff raw = %.1f" % marks[-1])
            time.sleep(1.0)

            print("== round %d/%d: v(%.0f) speed ==" % (round_index, args.repeats, args.speed_cmd))
            speeds.append(
                _measure_speed_once(driver, args.speed_cmd, args.spinup, args.measure_seconds)
            )
            print("  measured v(%.0f) = %.3f m/s" % (args.speed_cmd, speeds[-1]))
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\ncalibrate | aborted; sending stop")
    finally:
        driver.send(_ZERO)
        driver.close()

    if not marks or not speeds:
        print("calibrate | incomplete measurements, nothing written")
        return
    min_eff = compute_min_eff(marks, args.margin)
    v35 = median(speeds)
    result = {
        "vehicle_id": vehicle_id,
        "address": args.address,
        "min_eff_raw_marks": marks,
        "margin": args.margin,
        "wheel_command_min_effective": min_eff,
        "speed_cmd": args.speed_cmd,
        "v35_samples_mps": speeds,
        "v35_median_mps": v35,
    }
    out_dir = Path(__file__).resolve().parent / "calibrations"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / ("%s.json" % vehicle_id)
    with out_path.open("w", encoding="utf-8") as out:
        json.dump(result, out, indent=2, ensure_ascii=False)
    print("\ncalibrate | median min_eff(raw)=%.1f -> wheel_command_min_effective=%.1f"
          % (median(marks), min_eff))
    print("calibrate | median v(%.0f)=%.3f m/s" % (args.speed_cmd, v35))
    print("calibrate | saved %s" % out_path)
    print("\ncalibrate | paste into configs/<experiment>.py:")
    print(format_override(vehicle_id, min_eff, v35))


if __name__ == "__main__":
    main()
