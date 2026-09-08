"""Responsive real-time trajectory plotting in an isolated process.

matplotlib is imported lazily inside the plot worker so the rest of the
project (and the unit tests) works without it.

When ``save_path`` is given, the worker saves the final trajectory figure
there (``trajectory.png`` inside the run directory) right before closing.
With a non-interactive backend (no display available) the worker skips the
live window entirely and only accumulates samples for the saved PNG.
"""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from swarm.domain.models import ControlFrame


_NON_INTERACTIVE_BACKENDS = {"agg", "pdf", "ps", "svg", "template", "cairo"}


def _trim(values: List[float], max_points: int) -> None:
    if max_points > 0 and len(values) > max_points:
        del values[: len(values) - max_points]


def _append_sample(
    histories: Dict[str, Dict[str, List[float]]],
    points: Mapping[str, Tuple[float, float]],
    max_points: int,
) -> None:
    for vehicle_id, point in points.items():
        if vehicle_id not in histories:
            continue
        histories[vehicle_id]["x"].append(point[0])
        histories[vehicle_id]["y"].append(point[1])
        _trim(histories[vehicle_id]["x"], max_points)
        _trim(histories[vehicle_id]["y"], max_points)


def _plot_worker(
    vehicle_ids: Sequence[str],
    sample_queue: Any,
    status_queue: Any,
    plot_hz: float,
    max_points: int,
    save_path: Optional[str] = None,
) -> None:
    """Own the GUI event loop and report startup failures to the control process."""

    # Ctrl-C hits the whole process group; the main process drives shutdown
    # through the "close" message, so ignore SIGINT here to keep the PNG
    # save path deterministic.  Skipped in MainProcess (unit tests).
    if mp.current_process().name != "MainProcess":
        try:
            import signal

            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except Exception:
            pass

    try:
        import matplotlib

        backend = str(matplotlib.get_backend()).lower()
        interactive = backend not in _NON_INTERACTIVE_BACKENDS
        if not interactive and save_path is None:
            raise RuntimeError("matplotlib backend '%s' cannot open a live window" % backend)
        import matplotlib.pyplot as plt

        histories = {vehicle_id: {"x": [], "y": []} for vehicle_id in vehicle_ids}
        figure, axes = plt.subplots()
        axes.set_title("Swarm 2D Trajectory")
        axes.set_xlabel("x (m)")
        axes.set_ylabel("y (m)")
        axes.set_aspect("equal", adjustable="datalim")
        axes.grid(True, linestyle="--", alpha=0.4)
        lines = {}
        for vehicle_id in vehicle_ids:
            (line,) = axes.plot([], [], marker="o", markevery=[-1], markersize=5, linewidth=1.5, label=vehicle_id)
            lines[vehicle_id] = line
        axes.legend(loc="best")
        if interactive:
            plt.ion()
            plt.show(block=False)
            figure.canvas.draw()
            figure.canvas.flush_events()
            status_queue.put_nowait(("ready", backend))
        else:
            status_queue.put_nowait(("ready", "%s (headless: png only)" % backend))

        refresh_interval = 1.0 / max(plot_hz, 1e-6)
        next_draw = time.monotonic()
        dirty = False
        running = True
        try:
            while running:
                if interactive and not plt.fignum_exists(figure.number):
                    break
                timeout = max(0.0, min(0.05, next_draw - time.monotonic())) if interactive else 0.05
                try:
                    kind, payload = sample_queue.get(timeout=timeout)
                except queue.Empty:
                    kind, payload = "idle", None
                if kind == "close":
                    running = False
                elif kind == "sample":
                    _append_sample(histories, payload, max_points)
                    dirty = True

                while running:
                    try:
                        kind, payload = sample_queue.get_nowait()
                    except queue.Empty:
                        break
                    if kind == "close":
                        running = False
                        break
                    if kind == "sample":
                        _append_sample(histories, payload, max_points)
                        dirty = True

                if interactive:
                    now = time.monotonic()
                    if now >= next_draw:
                        if dirty:
                            for vehicle_id in vehicle_ids:
                                lines[vehicle_id].set_data(histories[vehicle_id]["x"], histories[vehicle_id]["y"])
                            axes.relim()
                            axes.autoscale_view()
                            figure.canvas.draw_idle()
                            dirty = False
                        figure.canvas.flush_events()
                        plt.pause(0.001)
                        next_draw = now + refresh_interval
        except KeyboardInterrupt:
            # Belt and braces: even if a SIGINT slips through, still save.
            pass

        if save_path is not None:
            for vehicle_id in vehicle_ids:
                lines[vehicle_id].set_data(histories[vehicle_id]["x"], histories[vehicle_id]["y"])
            axes.relim()
            axes.autoscale_view()
            try:
                Path(save_path).parent.mkdir(parents=True, exist_ok=True)
                figure.savefig(save_path, dpi=150)
                status_queue.put_nowait(("saved", str(save_path)))
            except Exception as save_error:
                status_queue.put_nowait(("error", "save failed: %s: %s" % (type(save_error).__name__, save_error)))
        plt.close(figure)
    except Exception as error:
        try:
            status_queue.put_nowait(("error", "%s: %s" % (type(error).__name__, error)))
        except Exception:
            pass


class LiveTrajectoryPlotSink:
    """Non-blocking sender for a separately managed Matplotlib GUI process."""

    def __init__(
        self,
        vehicle_ids: Sequence[str],
        plot_hz: float,
        queue_size: int,
        max_points: int,
        save_path: Optional[str] = None,
    ) -> None:
        context = mp.get_context("spawn")
        self._sample_queue = context.Queue(maxsize=max(1, int(queue_size)))
        self._status_queue = context.Queue(maxsize=4)
        self._process = context.Process(
            target=_plot_worker,
            args=(tuple(vehicle_ids), self._sample_queue, self._status_queue, plot_hz, max_points, save_path),
            name="swarm-live-plot",
            daemon=False,
        )
        self._process.start()
        self._available = True
        try:
            status, detail = self._status_queue.get(timeout=5.0)
            if status == "ready":
                print("live plot | ready backend=%s" % detail)
            else:
                self._available = False
                print("live plot | unavailable: %s" % detail)
        except queue.Empty:
            self._available = False
            print("live plot | unavailable: startup timed out")

    def emit(self, frame: ControlFrame) -> None:
        if not self._available or not self._process.is_alive():
            return
        points = {
            vehicle_id: (state.x, state.y)
            for vehicle_id, state in frame.snapshot.states.items()
            if state.valid
        }
        item = ("sample", points)
        try:
            self._sample_queue.put_nowait(item)
        except queue.Full:
            try:
                self._sample_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._sample_queue.put_nowait(item)
            except queue.Full:
                pass

    def close(self) -> None:
        if self._process is None:
            return
        try:
            self._sample_queue.put_nowait(("close", None))
        except queue.Full:
            try:
                self._sample_queue.get_nowait()
                self._sample_queue.put_nowait(("close", None))
            except (queue.Empty, queue.Full):
                pass
        self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
            return
        # Drain worker reports: confirm the saved PNG, surface save errors.
        while True:
            try:
                status, detail = self._status_queue.get_nowait()
            except queue.Empty:
                break
            if status == "saved":
                print("live plot | trajectory saved: %s" % detail)
            elif status == "error":
                print("live plot | %s" % detail)
