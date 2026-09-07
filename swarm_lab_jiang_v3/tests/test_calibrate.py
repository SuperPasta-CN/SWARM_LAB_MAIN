"""Unit tests for the per-vehicle calibration helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import calibrate_car as cal


class HelperTests(unittest.TestCase):
    def test_median_odd_and_even(self) -> None:
        self.assertEqual(cal.median([3.0, 1.0, 2.0]), 2.0)
        self.assertEqual(cal.median([4.0, 1.0, 3.0, 2.0]), 2.5)

    def test_compute_min_eff_adds_margin(self) -> None:
        self.assertEqual(cal.compute_min_eff([30.0, 32.0, 31.0], 5.0), 36.0)

    def test_compute_speed(self) -> None:
        self.assertAlmostEqual(cal.compute_speed(0.45, 3.0), 0.15)
        with self.assertRaises(ValueError):
            cal.compute_speed(0.45, 0.0)

    def test_format_override_contains_both_numbers(self) -> None:
        snippet = cal.format_override("car4", 36.0, 0.145)
        self.assertIn("wheel_command_min_effective=36.0", snippet)
        self.assertIn("v(35)=0.145", snippet)
        self.assertIn("car4", snippet)


class RollingDetectorTests(unittest.TestCase):
    def test_requires_debounce_confirmations(self) -> None:
        detector = cal.RollingDetector(threshold_m=0.02, debounce=2)
        origin = (0.0, 0.0)
        self.assertFalse(detector.check((0.019, 0.0), origin))  # below threshold
        self.assertFalse(detector.check((0.025, 0.0), origin))  # first hit
        self.assertTrue(detector.check((0.025, 0.0), origin))   # second hit

    def test_reset_on_still_position(self) -> None:
        detector = cal.RollingDetector(threshold_m=0.02, debounce=2)
        origin = (0.0, 0.0)
        detector.check((0.03, 0.0), origin)
        detector.check((0.01, 0.0), origin)  # resets
        self.assertFalse(detector.check((0.03, 0.0), origin))

    def test_none_positions_never_trigger(self) -> None:
        detector = cal.RollingDetector(threshold_m=0.02, debounce=1)
        self.assertFalse(detector.check(None, (0.0, 0.0)))
        self.assertFalse(detector.check((0.5, 0.5), None))

    def test_invalid_params_rejected(self) -> None:
        with self.assertRaises(ValueError):
            cal.RollingDetector(threshold_m=0.0)
        with self.assertRaises(ValueError):
            cal.RollingDetector(threshold_m=0.02, debounce=0)


class OverridesModuleTests(unittest.TestCase):
    def test_module_aggregates_all_calibrations(self) -> None:
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            cal_dir = Path(tmp)
            for vehicle_id, min_eff, v35 in (("car4", 36.0, 0.145), ("car5", 32.5, 0.132)):
                record = {
                    "vehicle_id": vehicle_id,
                    "wheel_command_min_effective": min_eff,
                    "v35_median_mps": v35,
                }
                with (cal_dir / ("%s.json" % vehicle_id)).open("w", encoding="utf-8") as out:
                    json.dump(record, out)
            out_path = cal_dir / "calibrated_overrides.py"
            results = cal.write_overrides_module(cal_dir, out_path)
            content = out_path.read_text(encoding="utf-8")
            self.assertEqual(set(results), {"car4", "car5"})
            self.assertIn('"car4": VehicleExecutionOverride(wheel_command_min_effective=36.0)', content)
            self.assertIn('"car5": 0.132', content)
            # generated module must be valid python
            namespace = {}
            exec(content, namespace)
            self.assertIn("CALIBRATED_OVERRIDES", namespace)


if __name__ == "__main__":
    unittest.main()
