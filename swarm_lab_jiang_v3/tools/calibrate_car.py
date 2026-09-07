#!/usr/bin/env python3
"""Per-vehicle dead-zone calibration for the mecanum chassis.

Measures, per vehicle, the two numbers that bound low-speed accuracy:

1. **true min_eff** (the "just rolling" command): all-wheel command ramps
   0 -> ramp-max.  With ``--mocap`` the start of rolling is detected
   automatically from the mocap position stream (displacement beyond the
   threshold, debounced); pressing ENTER marks it manually instead
   (whichever comes first).  Without ``--mocap`` the tool falls back to
   ENTER marking only.  Median + safety margin ->
   ``execution.wheel_command_min_effective``.
2. **v(35)** (sustained speed at the min_eff command): with ``--mocap``
   this is FULLY automatic — the tool drives the car and computes
   displacement / window from mocap positions.  Without ``--mocap`` the
   operator measures the distance with a tape.  This is the PWM accuracy
   ceiling: mean speed = duty * v(min_eff).

Outputs raw records to ``tools/calibrations/<vehicle>.json`` and
regenerates the importable parameter module
``configs/calibrated_overrides.py`` (aggregate of every calibrated
vehicle): use ``execution_override=CALIBRATED_OVERRIDES.get("carX")``.

Usage:
    python tools/calibrate_car.py --address 10.1.1.84 --mocap
    python tools/calibrate_car.py --address 10.1.1.84            # manual fallback
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


class RollingDetector:
    """Debounced rolling start detection on a mocap position stream."""

    def __init__(self, threshold_m: float = 0.02, debounce: int = 2) -> None:
        if threshold_m <= 0.0:
            raise ValueError("threshold_m must be positive")
        if debounce < 1:
            raise ValueError("debounce must be >= 1")
        self.threshold_m = threshold_m
        self.debounce = debounce
        self._hits = 0

    def check(self, position, origin) -> bool:
        """position/origin: (x, y) or None.  True once rolling is confirmed."""

        if position is None or origin is None:
            self._hits = 0
            return False
        dx = position[0] - origin[0]
        dy = position[1] - origin[1]
        if (dx * dx + dy * dy) ** 0.5 >= self.threshold_m:
            self._hits += 1
        else:
            self._hits = 0
        return self._hits >= self.debounce


class _MocapWatcher:
    """Thin per-vehicle pose reader over the ROS2/VRPN mocap source."""

    def __init__(self, vehicle_id: str, prefix: str = "") -> None:
        import rclpy

        from swarm.infrastructure.ros2_mocap import Ros2MocapSource

        if not rclpy.ok():
            rclpy.init()
        self._source = Ros2MocapSource([vehicle_id], prefix)
        self.vehicle_id = vehicle_id

    def start(self, timeout_s: float = 10.0) -> bool:
        self._source.start()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._source.all_valid():
                return True
            time.sleep(0.05)
        return False

    def position(self):
        state = self._source.get_snapshot().states.get(self.vehicle_id)
        if state is None or not state.valid:
            return None
        return state.x, state.y

    def stop(self) -> None:
        self._source.stop()


def write_overrides_module(calibrations_dir: Path, out_path: Path) -> dict:
    """Aggregate every per-car calibration JSON into an importable module."""

    results = {}
    for path in sorted(Path(calibrations_dir).glob("*.json")):
        with path.open(encoding="utf-8") as src:
            record = json.load(src)
        results[record["vehicle_id"]] = record
    lines = [
        '"""AUTO-GENERATED by tools/calibrate_car.py — per-vehicle calibration.',
        "",
        "Regenerated on every calibration run; edit by re-running the tool,",
        "not by hand.  Usage in experiment configs:",
        "    execution_override=CALIBRATED_OVERRIDES.get(\"carX\")",
        "",
        "Consistency rule: in each override entry, wheel_command_min_effective is",
        "the PWM ON amplitude and pwm_v_on_mps is the speed measured AT that same",
        "amplitude — never mix levels.  STICTION_MIN_EFFECTIVE keeps the",
        "ramp-measured threshold (for lift mode / lower-amplitude future work).",
        '"""',
        "",
        "from swarm.domain.config import VehicleExecutionOverride",
        "",
        "CALIBRATED_OVERRIDES = {",
    ]
    for vehicle_id, record in results.items():
        lines.append(
            '    "%s": VehicleExecutionOverride('
            "wheel_command_min_effective=%.1f, pwm_v_on_mps=%.3f),"
            % (vehicle_id, record["speed_cmd"], record["v35_median_mps"])
        )
    lines.append("}")
    lines.append("")
    lines.append("# Ramp-measured stiction thresholds + margin (lift mode / future use)")
    lines.append("STICTION_MIN_EFFECTIVE = {")
    for vehicle_id, record in results.items():
        lines.append(
            '    "%s": %.1f,' % (vehicle_id, record["wheel_command_min_effective"])
        )
    lines.append("}")
    lines.append("")
    lines.append("# Measured sustained speeds at the min_eff command (PWM accuracy reference)")
    lines.append("V35_MPS = {")
    for vehicle_id, record in results.items():
        lines.append('    "%s": %.3f,' % (vehicle_id, record["v35_median_mps"]))
    lines.append("}")
    lines.append("")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines), encoding="utf-8")
    return results


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
    watcher: "_MocapWatcher | None" = None,
    rolling_threshold_m: float = 0.02,
) -> float:
    """Ramp all wheels forward; return the command when rolling starts.

    With a mocap ``watcher`` the start of rolling is detected automatically
    (displacement beyond ``rolling_threshold_m``, debounced); pressing ENTER
    marks it manually instead — whichever comes first.
    """

    stop = threading.Event()
    watcher_thread = threading.Thread(target=_enter_watcher, args=(stop,), daemon=True)
    watcher_thread.start()
    origin = watcher.position() if watcher is not None else None
    detector = RollingDetector(rolling_threshold_m) if watcher is not None else None
    mode_text = "mocap auto-detect or ENTER" if watcher is not None else "ENTER"
    print("calibrate | ramping 0 -> %.0f over %.1f s; marking the start of rolling (%s)"
          % (command_max, seconds, mode_text))
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
            print("  marked manually at cmd = %.1f" % command)
            break
        if detector is not None and detector.check(watcher.position(), origin):
            marked = command
            print("  auto-detected rolling at cmd = %.1f" % command)
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
        print("calibrate | no mark captured; using ramp end %.0f" % command_max)
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
    watcher: "_MocapWatcher | None" = None,
) -> float:
    print(
        "calibrate | spin-up %.1f s, then measuring %.1f s at cmd=%.0f"
        % (spinup_s, measure_s, command)
    )
    _drive_for(driver, command, spinup_s)
    if watcher is not None:
        start = watcher.position()
        print("calibrate | MEASURING now (mocap-tracked, no tape needed)")
        _drive_for(driver, command, measure_s)
        end = watcher.position()
        if start is None or end is None:
            print("calibrate | mocap lost mid-measurement; retry this round")
            return _measure_speed_once(driver, command, spinup_s, measure_s, watcher)
        distance_m = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
        return compute_speed(distance_m, measure_s)
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
    parser.add_argument(
        "--mocap",
        action="store_true",
        help="use the ROS2/VRPN mocap for measurement: rolling start is "
        "auto-detected and v() is measured from positions (no tape)",
    )
    parser.add_argument("--mocap-prefix", default="", help="mocap topic prefix (default: '')")
    parser.add_argument(
        "--rolling-threshold",
        type=float,
        default=0.02,
        help="displacement (m) that counts as rolling for auto-detect (default: 0.02)",
    )
    parser.add_argument("--yes", action="store_true", help="skip the start confirmations")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    vehicle_id = args.vehicle_id or "car%s" % args.address.rsplit(".", 1)[-1].lstrip("0")
    driver = ChassisSocketDriver(args.address, SocketConfig(port=args.port))
    print("calibrate | %s (%s:%d), repeats=%d" % (vehicle_id, args.address, args.port, args.repeats))
    watcher = None
    if args.mocap:
        watcher = _MocapWatcher(vehicle_id, args.mocap_prefix)
        print("calibrate | waiting for mocap /%s%s/pose ..." % (args.mocap_prefix, vehicle_id))
        if not watcher.start():
            print("calibrate | no valid mocap within timeout — falling back to MANUAL mode")
            watcher.stop()
            watcher = None
        else:
            print("calibrate | mocap locked: auto-detect rolling + auto speed measurement")
    print("calibrate | clear ~1.5 m of floor ahead of the vehicle")
    marks = []
    speeds = []
    try:
        for round_index in range(1, args.repeats + 1):
            print("\n== round %d/%d: min_eff ramp ==" % (round_index, args.repeats))
            if not args.yes:
                input("  press ENTER to start (Ctrl-C to abort) ")
            marks.append(
                _ramp_until_mark(
                    driver,
                    args.ramp_max,
                    args.ramp_seconds,
                    watcher=watcher,
                    rolling_threshold_m=args.rolling_threshold,
                )
            )
            print("  marked min_eff raw = %.1f" % marks[-1])
            time.sleep(1.0)

            print("== round %d/%d: v(%.0f) speed ==" % (round_index, args.repeats, args.speed_cmd))
            speeds.append(
                _measure_speed_once(
                    driver, args.speed_cmd, args.spinup, args.measure_seconds, watcher
                )
            )
            print("  measured v(%.0f) = %.3f m/s" % (args.speed_cmd, speeds[-1]))
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\ncalibrate | aborted; sending stop")
    finally:
        driver.send(_ZERO)
        driver.close()
        if watcher is not None:
            watcher.stop()

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
        "mocap_assisted": watcher is not None,
    }
    out_dir = Path(__file__).resolve().parent / "calibrations"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / ("%s.json" % vehicle_id)
    with out_path.open("w", encoding="utf-8") as out:
        json.dump(result, out, indent=2, ensure_ascii=False)
    overrides_path = (
        Path(__file__).resolve().parents[1] / "configs" / "calibrated_overrides.py"
    )
    aggregated = write_overrides_module(out_dir, overrides_path)
    print("\ncalibrate | median min_eff(raw)=%.1f -> wheel_command_min_effective=%.1f"
          % (median(marks), min_eff))
    print("calibrate | median v(%.0f)=%.3f m/s" % (args.speed_cmd, v35))
    print("calibrate | saved %s" % out_path)
    print("calibrate | parameter module regenerated: %s (%d vehicles)"
          % (overrides_path, len(aggregated)))
    print("\ncalibrate | paste into configs/<experiment>.py:")
    print(format_override(vehicle_id, min_eff, v35))
    print('or simply: execution_override=CALIBRATED_OVERRIDES.get("%s")' % vehicle_id)


if __name__ == "__main__":
    main()
