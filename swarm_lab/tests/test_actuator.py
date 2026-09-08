"""Tests for the motor dead-zone compensation in the ground-vehicle actuator."""

from __future__ import annotations

import unittest

from swarm.domain.models import VehicleState, VelocityCommand
from swarm.infrastructure.ground_vehicle import GroundVehicleActuator, apply_min_command


class _StubController:
    def __init__(self, values):
        self._values = values

    def step(self, **kwargs) -> dict:
        return dict(self._values)

    def stop_command(self) -> dict:
        return {
            "front_left": 0.0,
            "front_right": 0.0,
            "rear_left": 0.0,
            "rear_right": 0.0,
        }


class _StubDriver:
    def __init__(self) -> None:
        self.sent = []

    def send(self, values) -> bool:
        self.sent.append(dict(values))
        return True


def _wheel_values(fl, fr, rl, rr):
    return {
        "front_left": fl,
        "front_right": fr,
        "rear_left": rl,
        "rear_right": rr,
        "target_speed": 0.1,
    }


class ApplyMinCommandTests(unittest.TestCase):
    def test_disabled_when_non_positive(self) -> None:
        values = _wheel_values(3.0, -3.0, 0.0, 50.0)
        self.assertIs(apply_min_command(values, 0.0), values)

    def test_zeros_stay_zero(self) -> None:
        lifted = apply_min_command(_wheel_values(0.0, 0.0, 0.0, 0.0), 20.0)
        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertEqual(lifted[key], 0.0)

    def test_small_commands_are_lifted_sign_aware(self) -> None:
        lifted = apply_min_command(_wheel_values(5.0, -5.0, 0.5, -0.5), 20.0)
        self.assertEqual(lifted["front_left"], 20.0)
        self.assertEqual(lifted["front_right"], -20.0)
        self.assertEqual(lifted["rear_left"], 20.0)
        self.assertEqual(lifted["rear_right"], -20.0)

    def test_large_commands_pass_through(self) -> None:
        lifted = apply_min_command(_wheel_values(60.0, -60.0, 20.0, -20.0), 20.0)
        self.assertEqual(lifted["front_left"], 60.0)
        self.assertEqual(lifted["front_right"], -60.0)
        self.assertEqual(lifted["rear_left"], 20.0)
        self.assertEqual(lifted["rear_right"], -20.0)

    def test_non_wheel_fields_untouched(self) -> None:
        lifted = apply_min_command(_wheel_values(5.0, 5.0, 5.0, 5.0), 20.0)
        self.assertEqual(lifted["target_speed"], 0.1)


class ActuatorMinCommandTests(unittest.TestCase):
    def test_execute_applies_lift_before_sending(self) -> None:
        controller = _StubController(_wheel_values(5.0, -5.0, 0.0, 60.0))
        driver = _StubDriver()
        actuator = GroundVehicleActuator("car1", controller, driver, min_command=20.0)
        actuator.execute(
            VelocityCommand(vehicle_id="car1", vx=0.1, vy=0.0, vz=0.0),
            VehicleState("car1", valid=True),
            0.02,
        )
        sent = driver.sent[0]
        self.assertEqual(sent["front_left"], 20.0)
        self.assertEqual(sent["front_right"], -20.0)
        self.assertEqual(sent["rear_left"], 0.0)
        self.assertEqual(sent["rear_right"], 60.0)

    def test_stop_stays_all_zero(self) -> None:
        controller = _StubController(_wheel_values(5.0, 5.0, 5.0, 5.0))
        driver = _StubDriver()
        actuator = GroundVehicleActuator("car1", controller, driver, min_command=20.0)
        actuator.stop()
        sent = driver.sent[0]
        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertEqual(sent[key], 0.0)


if __name__ == "__main__":
    unittest.main()
