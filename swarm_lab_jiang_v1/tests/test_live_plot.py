"""Tests for the live-plot worker's trajectory PNG saving."""

from __future__ import annotations

import os

# Must be pinned before matplotlib is first imported anywhere in this
# process: the backend is resolved at import time.
os.environ["MPLBACKEND"] = "Agg"

import queue
import tempfile
import unittest
from pathlib import Path

try:
    import matplotlib  # noqa: F401

    _HAVE_MATPLOTLIB = True
except ImportError:
    _HAVE_MATPLOTLIB = False

from swarm.infrastructure.telemetry.live_plot import _plot_worker


@unittest.skipUnless(_HAVE_MATPLOTLIB, "matplotlib is not installed")
class PlotWorkerSaveTests(unittest.TestCase):
    def test_worker_saves_trajectory_png_on_close(self) -> None:
        samples = queue.Queue()
        status = queue.Queue()
        samples.put(("sample", {"car1": (0.0, 0.0), "car2": (1.0, 0.0)}))
        samples.put(("sample", {"car1": (0.1, 0.2), "car2": (0.9, 0.1)}))
        samples.put(("close", None))
        with tempfile.TemporaryDirectory() as tmp:
            target = str(Path(tmp) / "nested" / "trajectory.png")
            _plot_worker(
                ("car1", "car2"), samples, status,
                plot_hz=10.0, max_points=100, save_path=target,
            )
            self.assertTrue(Path(target).is_file())
            self.assertGreater(Path(target).stat().st_size, 1000)
        reports = []
        while True:
            try:
                reports.append(status.get_nowait())
            except queue.Empty:
                break
        kinds = [kind for kind, _ in reports]
        self.assertIn("ready", kinds)
        self.assertIn("saved", kinds)

    def test_headless_without_save_path_reports_error(self) -> None:
        samples = queue.Queue()
        status = queue.Queue()
        _plot_worker(("car1",), samples, status, plot_hz=10.0, max_points=100)
        kind, _ = status.get(timeout=5.0)
        self.assertEqual(kind, "error")

    def test_keyboard_interrupt_still_saves_the_png(self) -> None:
        """A Ctrl-C landing mid-run must not skip the trajectory save."""

        class InterruptingQueue(queue.Queue):
            def __init__(self) -> None:
                super().__init__()
                self.calls = 0

            def get(self, block=True, timeout=None):  # noqa: A002 - queue API
                self.calls += 1
                if self.calls > 1:
                    raise KeyboardInterrupt  # SIGINT arrives instead of "close"
                return super().get(block, timeout)

        samples = InterruptingQueue()
        samples.put(("sample", {"car1": (0.0, 0.0)}))
        status = queue.Queue()
        with tempfile.TemporaryDirectory() as tmp:
            target = str(Path(tmp) / "trajectory.png")
            _plot_worker(
                ("car1",), samples, status,
                plot_hz=10.0, max_points=100, save_path=target,
            )
            self.assertTrue(Path(target).is_file())


if __name__ == "__main__":
    unittest.main()
