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


if __name__ == "__main__":
    unittest.main()
