"""Tests for the mocap velocity estimator's glitch rejection."""

from __future__ import annotations

import unittest

from collections import deque

from swarm.infrastructure.ros2_mocap import (
    finite_difference_velocity,
    trim_pose_history,
)


class FiniteDifferenceVelocityTests(unittest.TestCase):
    def test_normal_update_blends_ema(self) -> None:
        previous = (0.0, 0.0, 0.0, 0.0, 1.0)
        current = (0.1, 0.0, 0.0, 0.0, 1.5)  # 0.1 m in 0.5 s -> 0.2 m/s
        out = finite_difference_velocity(
            (0.0, 0.0, 0.0, 0.0), previous, current,
            alpha=0.5, max_plausible_speed_mps=1.0,
        )
        self.assertIsNotNone(out)
        vx, vy, vz, wz = out
        self.assertAlmostEqual(vx, 0.1)  # 0.5 * 0.2 m/s EMA blend

    def test_mocap_teleport_is_rejected(self) -> None:
        previous = (0.0, 0.0, 0.0, 0.0, 1.0)
        current = (5.0, 0.0, 0.0, 0.0, 1.01)  # ~500 m/s, physically impossible
        out = finite_difference_velocity(
            (0.0, 0.0, 0.0, 0.0), previous, current,
            alpha=0.5, max_plausible_speed_mps=1.0,
        )
        self.assertIsNone(out)

    def test_nonpositive_dt_is_rejected(self) -> None:
        previous = (0.0, 0.0, 0.0, 0.0, 1.0)
        current = (0.1, 0.0, 0.0, 0.0, 1.0)
        out = finite_difference_velocity(
            (0.0, 0.0, 0.0, 0.0), previous, current,
            alpha=0.5, max_plausible_speed_mps=1.0,
        )
        self.assertIsNone(out)


class TrimPoseHistoryTests(unittest.TestCase):
    def _history(self, stamps) -> deque:
        return deque((0.0, 0.0, 0.0, 0.0, stamp) for stamp in stamps)

    def test_reference_is_oldest_sample_inside_window(self) -> None:
        history = self._history([0.0, 0.125, 0.25, 0.375, 0.5])
        reference = trim_pose_history(history, current_stamp=0.5, baseline_s=0.3)
        self.assertEqual(reference[4], 0.25)  # 0.0 and 0.125 fell out of the window
        self.assertEqual([sample[4] for sample in history], [0.25, 0.375, 0.5])

    def test_exact_window_edge_is_kept(self) -> None:
        history = self._history([0.125, 0.5])
        reference = trim_pose_history(history, current_stamp=0.5, baseline_s=0.375)
        self.assertEqual(reference[4], 0.125)

    def test_sparse_stream_falls_back_to_previous_frame(self) -> None:
        history = self._history([0.0, 0.3, 0.5])  # all older than the baseline
        reference = trim_pose_history(history, current_stamp=0.5, baseline_s=0.15)
        self.assertEqual(reference[4], 0.3)  # newest older sample kept

    def test_baseline_difference_feeds_estimator(self) -> None:
        # 100 Hz poses, 0.2 m/s motion, 0.2 s baseline: reference is ~0.2 s
        # back, so the raw speed is the true 0.2 m/s and passes rejection.
        history = self._history([0.00, 0.05, 0.10, 0.15, 0.20])
        reference = trim_pose_history(history, current_stamp=0.20, baseline_s=0.2)
        self.assertEqual(reference[4], 0.0)
        current = (0.04, 0.0, 0.0, 0.0, 0.20)  # 0.04 m in 0.2 s -> 0.2 m/s
        out = finite_difference_velocity(
            (0.0, 0.0, 0.0, 0.0), reference, current,
            alpha=0.5, max_plausible_speed_mps=1.0,
        )
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out[0], 0.1)  # 0.5 * 0.2 m/s EMA blend


if __name__ == "__main__":
    unittest.main()
