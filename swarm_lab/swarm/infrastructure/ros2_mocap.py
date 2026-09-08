"""ROS2 adapter for VRPN pose and twist topics via vrpn_client_ros.

Expected topic pattern (from vrpn_client_ros, no prefix by default)::

    /{vehicle_id}/pose   -> geometry_msgs/msg/PoseStamped
    /{vehicle_id}/twist  -> geometry_msgs/msg/TwistStamped

``rclpy`` is an *optional* import: this module loads fine on machines
without ROS2 so unit tests can run everywhere.  Instantiating
:class:`Ros2MocapSource` is what requires ROS2.

A daemon thread runs ``rclpy.spin`` in the background.  With
``require_twist=False`` (default) a pose alone marks the state valid and
linear velocity is estimated from position finite differences over a short
baseline window (on header timestamps) smoothed by an EMA; with
``require_twist=True`` the legacy pose+twist behavior is restored.
"""

from __future__ import annotations

from collections import deque
from dataclasses import replace
from math import atan2, pi, sqrt
from threading import RLock, Thread
from typing import Deque, Dict, Optional, Sequence

try:  # ROS2 is only present on the experiment machine.
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from rclpy.node import Node
except ImportError:  # pragma: no cover - depends on the host environment
    rclpy = None  # type: ignore[assignment]
    PoseStamped = None  # type: ignore[assignment]
    TwistStamped = None  # type: ignore[assignment]
    Node = None  # type: ignore[assignment,misc]

from swarm.domain.models import MocapSnapshot, VehicleState


def _wrap_to_pi(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


def trim_pose_history(history, current_stamp: float, baseline_s: float) -> tuple:
    """Trim a pose history to the baseline window and return the reference.

    ``history`` holds ``(x, y, z, yaw, stamp)`` samples oldest-first with the
    current pose already appended.  Returns the oldest sample within
    ``[current_stamp - baseline_s, current_stamp]``; when every older sample
    lies outside the window (sparse stream), the most recent older sample is
    kept as a consecutive-frame fallback.  Differencing against this
    reference instead of the immediately previous frame shrinks the noise
    term ``position_noise / dt`` as the baseline grows, so the glitch
    rejection stops misfiring on ordinary noise at high mocap rates.
    """

    while len(history) > 2 and current_stamp - history[0][4] > baseline_s:
        history.popleft()
    return history[0]


def finite_difference_velocity(
    old: tuple,
    previous: tuple,
    current: tuple,
    alpha: float,
    max_plausible_speed_mps: float,
):
    """One EMA velocity update from two pose samples.

    ``previous``/``current`` are ``(x, y, z, yaw, stamp)`` tuples and ``old``
    is the current ``(vx, vy, vz, wz)`` EMA state.  Returns the new velocity
    tuple, or ``None`` when the raw difference is physically implausible
    (mocap position glitch) and must be rejected instead of absorbed.
    """

    px, py, pz, pyaw, pt = previous
    x, y, z, yaw, stamp = current
    dt = stamp - pt
    if dt <= 1e-6:
        return None
    raw_vx = (x - px) / dt
    raw_vy = (y - py) / dt
    raw_vz = (z - pz) / dt
    if sqrt(raw_vx * raw_vx + raw_vy * raw_vy + raw_vz * raw_vz) > max_plausible_speed_mps:
        return None
    raw_wz = _wrap_to_pi(yaw - pyaw) / dt
    old_vx, old_vy, old_vz, old_wz = old
    return (
        old_vx + alpha * (raw_vx - old_vx),
        old_vy + alpha * (raw_vy - old_vy),
        old_vz + alpha * (raw_vz - old_vz),
        old_wz + alpha * (raw_wz - old_wz),
    )


class Ros2MocapSource:
    """Merge the latest VRPN pose (and optional twist) for every vehicle."""

    def __init__(
        self,
        vehicle_ids: Sequence[str],
        topic_prefix: str,
        require_twist: bool = False,
        velocity_ema_alpha: float = 0.3,
        max_plausible_speed_mps: float = 1.0,
        velocity_diff_baseline_s: float = 0.2,
    ) -> None:
        if rclpy is None:
            raise ImportError(
                "rclpy is required to use Ros2MocapSource; "
                "install ROS2 (e.g. ros-humble-rclpy) on the experiment machine"
            )
        if not vehicle_ids:
            raise ValueError("vehicle_ids must not be empty")
        if not 0.0 < velocity_ema_alpha <= 1.0:
            raise ValueError("velocity_ema_alpha must be in (0, 1]")
        if max_plausible_speed_mps <= 0.0:
            raise ValueError("max_plausible_speed_mps must be positive")
        if velocity_diff_baseline_s <= 0.0:
            raise ValueError("velocity_diff_baseline_s must be positive")
        self.vehicle_ids = tuple(vehicle_ids)
        self.topic_prefix = topic_prefix.rstrip("/")
        self.require_twist = require_twist
        self.velocity_ema_alpha = velocity_ema_alpha
        self.max_plausible_speed_mps = max_plausible_speed_mps
        self.velocity_diff_baseline_s = velocity_diff_baseline_s
        self._lock = RLock()
        self._states = {vehicle_id: VehicleState(vehicle_id) for vehicle_id in self.vehicle_ids}
        self._pose_history: Dict[str, Deque[tuple]] = {
            vehicle_id: deque() for vehicle_id in self.vehicle_ids
        }
        self._latest_timestamp = 0.0
        self._node: Optional[Node] = None
        self._subscriptions = []
        self._spin_thread: Optional[Thread] = None

    def start(self) -> None:
        with self._lock:
            if self._node is not None:
                return
            self._node = Node("swarm_mocap_source")
            for vehicle_id in self.vehicle_ids:
                pose_topic = f"{self.topic_prefix}/{vehicle_id}/pose"
                sub_pose = self._node.create_subscription(
                    PoseStamped,
                    pose_topic,
                    lambda msg, vid=vehicle_id: self._handle_pose(vid, msg),
                    10,
                )
                self._subscriptions.append(sub_pose)
                if self.require_twist:
                    twist_topic = f"{self.topic_prefix}/{vehicle_id}/twist"
                    sub_twist = self._node.create_subscription(
                        TwistStamped,
                        twist_topic,
                        lambda msg, vid=vehicle_id: self._handle_twist(vid, msg),
                        10,
                    )
                    self._subscriptions.append(sub_twist)
            self._spin_thread = Thread(
                target=self._spin,
                name="swarm-mocap-spin",
                daemon=True,
            )
            self._spin_thread.start()

    def _spin(self) -> None:
        node = self._node
        if node is None:
            return
        try:
            rclpy.spin(node)
        except Exception:
            pass

    def stop(self) -> None:
        with self._lock:
            node = self._node
            self._node = None
            self._subscriptions = []
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        thread = self._spin_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._spin_thread = None

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return atan2(siny_cosp, cosy_cosp)

    def _estimate_velocity(self, vehicle_id: str, x: float, y: float, z: float, yaw: float, stamp: float) -> None:
        """Baseline-window finite difference + EMA velocity (pose-only mode).

        The raw difference is taken against the oldest pose inside the
        trailing ``velocity_diff_baseline_s`` window rather than the previous
        frame: at high mocap rates the per-frame ``position_noise / dt``
        easily exceeds the plausibility threshold, and the glitch rejection
        then censors mostly motion-aligned spikes, biasing the estimate low.
        A longer baseline shrinks the noise below the threshold so rejection
        only fires on real glitches (teleports).
        """

        state = self._states[vehicle_id]
        history = self._pose_history[vehicle_id]
        history.append((x, y, z, yaw, stamp))
        if len(history) < 2:
            return
        reference = trim_pose_history(history, stamp, self.velocity_diff_baseline_s)
        updated = finite_difference_velocity(
            old=(state.vx, state.vy, state.vz, state.wz),
            previous=reference,
            current=(x, y, z, yaw, stamp),
            alpha=self.velocity_ema_alpha,
            max_plausible_speed_mps=self.max_plausible_speed_mps,
        )
        if updated is None:
            return
        state.vx, state.vy, state.vz, state.wz = updated

    def _handle_pose(self, vehicle_id: str, message) -> None:
        position = message.pose.position
        with self._lock:
            state = self._states[vehicle_id]
            x = float(position.x)
            y = float(position.y)
            z = float(position.z)
            yaw = self._yaw_from_quaternion(message.pose.orientation)
            stamp = self._stamp_to_sec(message.header.stamp)
            if not self.require_twist:
                self._estimate_velocity(vehicle_id, x, y, z, yaw, stamp)
            state.x = x
            state.y = y
            state.z = z
            state.yaw = yaw
            state.frame_id = str(message.header.frame_id)
            state.timestamp = stamp
            state.pose_ready = True
            if not self.require_twist:
                state.twist_ready = False
            state.valid = state.pose_ready and (state.twist_ready or not self.require_twist)
            self._latest_timestamp = max(self._latest_timestamp, state.timestamp)

    def _handle_twist(self, vehicle_id: str, message) -> None:
        linear = message.twist.linear
        angular = message.twist.angular
        with self._lock:
            state = self._states[vehicle_id]
            state.vx = float(linear.x)
            state.vy = float(linear.y)
            state.vz = float(linear.z)
            state.wx = float(angular.x)
            state.wy = float(angular.y)
            state.wz = float(angular.z)
            state.frame_id = str(message.header.frame_id)
            state.timestamp = self._stamp_to_sec(message.header.stamp)
            state.twist_ready = True
            state.valid = state.pose_ready and state.twist_ready
            self._latest_timestamp = max(self._latest_timestamp, state.timestamp)

    def get_snapshot(self) -> MocapSnapshot:
        with self._lock:
            states = {vehicle_id: replace(state) for vehicle_id, state in self._states.items()}
            return MocapSnapshot(self._latest_timestamp, states)

    def all_valid(self) -> bool:
        with self._lock:
            return all(state.valid for state in self._states.values())
