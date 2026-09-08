#!/usr/bin/env python3
"""Open-loop single-vehicle hardware check for the mecanum chassis.

Run this BEFORE any formation experiment when a vehicle behaves oddly
(lateral drift on a forward command, reversed turning, one wheel dead).
Each step prints the motion the operator should observe; any mismatch
pinpoints wiring order, polarity, or roller-mounting problems directly.

Usage:
    python tools/check_wheels.py --address 10.1.1.84
    python tools/check_wheels.py --address 10.1.1.84 --yes --duration 1.0
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow running both as ``python tools/check_wheels.py`` and as a module.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from swarm.infrastructure.udp_chassis import ChassisSocketDriver, SocketConfig


Step = tuple  # (name, expectation, (FL, FR, RL, RR) signs)


def _steps(speed: float) -> list:
    return [
        (
            "all wheels forward",
            "vehicle should drive straight FORWARD (body +x)",
            (speed, speed, speed, speed),
        ),
        (
            "all wheels backward",
            "vehicle should drive straight BACKWARD (body -x)",
            (-speed, -speed, -speed, -speed),
        ),
        (
            "strafe left",
            "vehicle should translate LEFT (body +y) without turning",
            (-speed, speed, speed, -speed),
        ),
        (
            "strafe right",
            "vehicle should translate RIGHT (body -y) without turning",
            (speed, -speed, -speed, speed),
        ),
        (
            "rotate counter-clockwise",
            "vehicle should spin in place CCW (left, viewed from above)",
            (-speed, speed, -speed, speed),
        ),
        (
            "rotate clockwise",
            "vehicle should spin in place CW (right, viewed from above)",
            (speed, -speed, speed, -speed),
        ),
        (
            "front-left wheel only",
            "only the FRONT-LEFT wheel should turn forward",
            (speed, 0.0, 0.0, 0.0),
        ),
        (
            "front-right wheel only",
            "only the FRONT-RIGHT wheel should turn forward",
            (0.0, speed, 0.0, 0.0),
        ),
        (
            "rear-left wheel only",
            "only the REAR-LEFT wheel should turn forward",
            (0.0, 0.0, speed, 0.0),
        ),
        (
            "rear-right wheel only",
            "only the REAR-RIGHT wheel should turn forward",
            (0.0, 0.0, 0.0, speed),
        ),
    ]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", required=True, help="vehicle IP, e.g. 10.1.1.84")
    parser.add_argument("--port", type=int, default=12345, help="UDP port (default: 12345)")
    parser.add_argument("--speed", type=float, default=40.0, help="wheel command 0-100 (default: 40)")
    parser.add_argument("--duration", type=float, default=1.5, help="seconds per step (default: 1.5)")
    parser.add_argument("--yes", action="store_true", help="do not wait for ENTER between steps")
    parser.add_argument(
        "--ramp",
        action="store_true",
        help="dead-zone measurement: slowly ramp all wheels 0 -> --ramp-max and "
        "print the command; note the value where the vehicle starts rolling "
        "(use it for execution.wheel_command_min_effective)",
    )
    parser.add_argument("--ramp-max", type=float, default=60.0, help="ramp end command (default: 60)")
    parser.add_argument("--ramp-seconds", type=float, default=20.0, help="ramp duration (default: 20 s)")
    return parser.parse_args()


def _ramp(driver: ChassisSocketDriver, command_max: float, seconds: float, hz: float = 20.0) -> None:
    """Ramp all four wheels forward 0 -> command_max, printing the level."""

    print(
        "check_wheels | ramping all wheels forward 0 -> %.0f over %.1f s"
        % (command_max, seconds)
    )
    print("check_wheels | watch the vehicle and note the command where it STARTS to move:")
    period = 1.0 / hz
    started = time.monotonic()
    last_print = -1.0
    while True:
        elapsed = time.monotonic() - started
        ratio = min(1.0, elapsed / seconds)
        command = command_max * ratio
        if elapsed - last_print >= 0.5:
            last_print = elapsed
            print("  cmd = %5.1f" % command)
        wheels = {
            "front_left": command,
            "front_right": command,
            "rear_left": command,
            "rear_right": command,
        }
        driver.send(wheels)
        if ratio >= 1.0:
            break
        time.sleep(period)
    driver.send({"front_left": 0.0, "front_right": 0.0, "rear_left": 0.0, "rear_right": 0.0})
    print("check_wheels | ramp done; set wheel_command_min_effective slightly above the noted value")


def _send_for(driver: ChassisSocketDriver, wheels, duration: float, hz: float = 20.0) -> None:
    command = {
        "front_left": wheels[0],
        "front_right": wheels[1],
        "rear_left": wheels[2],
        "rear_right": wheels[3],
    }
    period = 1.0 / hz
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        driver.send(command)
        time.sleep(period)
    driver.send({"front_left": 0.0, "front_right": 0.0, "rear_left": 0.0, "rear_right": 0.0})


def main() -> None:
    args = _parse_args()
    driver = ChassisSocketDriver(args.address, SocketConfig(port=args.port))
    print(
        "check_wheels | target %s:%d speed=%.0f duration=%.1fs"
        % (args.address, args.port, args.speed, args.duration)
    )
    print("check_wheels | wheel order is FL, FR, RL, RR; place the vehicle on blocks or clear floor")
    try:
        if args.ramp:
            if not args.yes:
                input("check_wheels | press ENTER to start the ramp (Ctrl-C to abort) ")
            _ramp(driver, args.ramp_max, args.ramp_seconds)
        else:
            for index, (name, expectation, wheels) in enumerate(_steps(args.speed), start=1):
                print("\nstep %d: %s" % (index, name))
                print("  expect: %s" % expectation)
                if not args.yes:
                    input("  press ENTER to run (Ctrl-C to abort) ")
                _send_for(driver, wheels, args.duration)
                time.sleep(0.3)
    except KeyboardInterrupt:
        print("\ncheck_wheels | aborted")
    finally:
        driver.send({"front_left": 0.0, "front_right": 0.0, "rear_left": 0.0, "rear_right": 0.0})
        driver.close()
    print("\ncheck_wheels | done; investigate any step whose observed motion differed")


if __name__ == "__main__":
    main()
