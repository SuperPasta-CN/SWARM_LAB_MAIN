"""Tests for the configurable dead-zone compensation module."""

from __future__ import annotations

import unittest

from swarm.infrastructure.ground_vehicle import DeadzoneCompensator, apply_min_command

_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


def _values(fl=0.0, fr=0.0, rl=0.0, rr=0.0):
    return dict(zip(_WHEELS, (fl, fr, rl, rr))) | {"vx_body": 0.0}


class LiftModeTests(unittest.TestCase):
    """Legacy behaviour must stay byte-identical for comparison runs."""

    def test_lift_matches_legacy(self) -> None:
        comp = DeadzoneCompensator(mode="lift", min_effective=35.0)
        values = _values(10.0, -10.0, 60.0, 0.0)
        self.assertEqual(comp.apply(values), apply_min_command(values, 35.0))

    def test_disabled_when_zero_min_effective(self) -> None:
        comp = DeadzoneCompensator(mode="lift", min_effective=0.0)
        values = _values(1.0, 2.0, 3.0, 4.0)
        self.assertEqual(comp.apply(values), values)


class AffineModeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.comp = DeadzoneCompensator(
            mode="affine", min_effective=35.0, command_max=100.0
        )

    def test_zero_stays_zero(self) -> None:
        self.assertEqual(self.comp.apply(_values())["front_left"], 0.0)

    def test_small_command_maps_just_above_threshold(self) -> None:
        out = self.comp.apply(_values(fl=1.0))
        self.assertGreater(out["front_left"], 35.0)
        self.assertLess(out["front_left"], 36.0)

    def test_max_command_unchanged(self) -> None:
        out = self.comp.apply(_values(fl=100.0))
        self.assertAlmostEqual(out["front_left"], 100.0)

    def test_monotone_and_sign_preserving(self) -> None:
        seq = [self.comp.apply(_values(fl=c))["front_left"] for c in (5.0, 20.0, 50.0, 90.0)]
        self.assertTrue(all(a < b for a, b in zip(seq, seq[1:])))
        neg = self.comp.apply(_values(fl=-20.0))["front_left"]
        self.assertLess(neg, -35.0)
        self.assertAlmostEqual(neg, -self.comp.apply(_values(fl=20.0))["front_left"])

    def test_non_wheel_keys_untouched(self) -> None:
        out = self.comp.apply(_values(fl=10.0))
        self.assertEqual(out["vx_body"], 0.0)


class PwmModeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.comp = DeadzoneCompensator(
            mode="pwm", min_effective=35.0, command_max=100.0, pwm_period_cycles=10
        )

    def test_zero_stays_zero(self) -> None:
        for _ in range(10):
            out = self.comp.apply(_values())
            self.assertEqual([out[k] for k in _WHEELS], [0.0] * 4)

    def test_beyond_threshold_passes_through(self) -> None:
        out = self.comp.apply(_values(fl=50.0, fr=-60.0))
        self.assertEqual(out["front_left"], 50.0)
        self.assertEqual(out["front_right"], -60.0)

    def test_time_average_equals_request(self) -> None:
        # duty = 17.5/35 = 0.5 -> five ON cycles at +35 per period
        total = 0.0
        for _ in range(10):
            total += self.comp.apply(_values(fl=17.5))["front_left"]
        self.assertAlmostEqual(total / 10.0, 17.5, places=9)

    def test_on_cycle_preserves_wheel_ratios(self) -> None:
        # find an ON cycle and check the direction is preserved exactly
        seen_on = None
        for _ in range(20):
            out = self.comp.apply(_values(fl=10.0, fr=20.0, rl=-10.0, rr=-20.0))
            if out["front_left"] != 0.0:
                seen_on = out
        self.assertIsNotNone(seen_on)
        self.assertAlmostEqual(seen_on["front_right"] / seen_on["front_left"], 2.0)
        self.assertAlmostEqual(seen_on["rear_left"], -seen_on["front_left"])
        self.assertAlmostEqual(abs(seen_on["front_right"]), 35.0)


class PwmVonTests(unittest.TestCase):
    """Per-vehicle duty normalization with measured v_on."""

    def test_duty_normalized_by_v_on(self) -> None:
        # car5 calibration: v_on=0.283 m/s at cmd 35 -> duty for c=20 is
        # (20*0.5/100)/0.283 ~= 0.353 -> ~3-4 ON cycles per 10.
        comp = DeadzoneCompensator(
            mode="pwm", min_effective=35.0, v_on_mps=0.283, calib_mps=0.5,
            pwm_period_cycles=10,
        )
        on_count = sum(
            1 for _ in range(10) if comp.apply(_values(fl=20.0))["front_left"] != 0.0
        )
        self.assertIn(on_count, (3, 4))

    def test_duty_ge_one_passes_through(self) -> None:
        comp = DeadzoneCompensator(mode="pwm", min_effective=35.0, v_on_mps=0.05)
        out = comp.apply(_values(fl=20.0))
        self.assertEqual(out["front_left"], 20.0)

    def test_invalid_v_on_rejected(self) -> None:
        with self.assertRaises(ValueError):
            DeadzoneCompensator(mode="pwm", min_effective=35.0, v_on_mps=0.0)


class DeadzoneConfigValidationTests(unittest.TestCase):
    def test_invalid_mode_rejected(self) -> None:
        from dataclasses import replace

        from configs import task_two
        from swarm.domain.config import ExecutionConfig

        config = replace(
            task_two.CONFIG,
            execution=ExecutionConfig(deadzone_mode="weird"),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_negative_heading_deadband_rejected(self) -> None:
        from dataclasses import replace

        from configs import task_two
        from swarm.domain.config import ExecutionConfig

        config = replace(
            task_two.CONFIG,
            execution=ExecutionConfig(heading_deadband_rad=-0.1),
        )
        with self.assertRaises(ValueError):
            config.validate()

    def test_default_config_uses_validated_anti_oscillation_modes(self) -> None:
        from swarm.domain.config import ExecutionConfig

        execution = ExecutionConfig()
        self.assertEqual(execution.deadzone_mode, "pwm")
        self.assertEqual(execution.heading_deadband_rad, 0.03)
        self.assertEqual(execution.pwm_period_cycles, 10)


if __name__ == "__main__":
    unittest.main()
