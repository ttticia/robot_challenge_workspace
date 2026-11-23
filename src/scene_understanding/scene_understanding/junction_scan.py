"""Capture a three-panel junction view using the Nav2 Spin behavior."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
import numpy as np
from rclpy.action import ActionClient
from rclpy.time import Time

# Back at the trigger heading when within this of it.
HEADING_TOL = math.radians(2.0)
# Extra small turns allowed after the turn back, to remove Spin overshoot.
MAX_CORRECTIONS = 2


def tf_heading(node):
    """Yaw of the node's base_frame from TF (odom first: it never jumps), or None."""
    buffer = getattr(node, 'tf_buffer', None)
    if buffer is None:
        return None
    base = getattr(node, 'base_frame', 'base_link')
    for frame in (getattr(node, 'odom_frame', 'odom'),
                  getattr(node, 'map_frame', 'map')):
        try:
            q = buffer.lookup_transform(frame, base, Time()).transform.rotation
        except Exception:  # noqa: B902 - any TF failure: try the next frame
            continue
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))
    return None


class JunctionScan:
    """Turn in place, sample fresh frames, and restore the trigger heading."""

    def __init__(self, node, action_name, angle_deg):
        if not 0 < angle_deg <= 90:
            raise ValueError('junction_scan_angle_deg must be within (0, 90]')
        self.node = node
        self.angle = math.radians(angle_deg)
        self.action_name = action_name
        self.spin = ActionClient(node, Spin, action_name)
        # Measured yaw of each view from the trigger heading (deg), or None.
        self.view_offsets_deg = {}
        self.last_traveled = None
        self.unavailable_reason = f'Nav2 Spin action {action_name} unavailable'

    def _heading(self):
        """Robot yaw from TF, or None."""
        return tf_heading(self.node)

    def _wait(self, future, timeout):
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        deadline = time.monotonic() + timeout
        while self.node.context.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if ready.wait(min(remaining, 0.1)):
                return future.result()
        raise TimeoutError('Timed out or shut down while waiting for Nav2 Spin')

    def _turn(self, angle):
        if not self.node.context.ok():
            raise RuntimeError('ROS shut down during junction scan')
        goal = Spin.Goal(
            target_yaw=angle,
            time_allowance=Duration(sec=15),
        )
        traveled = None
        self.last_traveled = 0.0

        def on_feedback(message):
            nonlocal traveled
            traveled = message.feedback.angular_distance_traveled
            self.last_traveled = abs(traveled)

        pending = self.spin.send_goal_async(goal, feedback_callback=on_feedback)
        try:
            handle = self._wait(pending, 3.0)
        except TimeoutError:
            # An acceptance arriving after our deadline still needs cancellation.
            def cancel_late_goal(future):
                if not future.cancelled() and future.exception() is None:
                    late_handle = future.result()
                    if late_handle.accepted and self.node.context.ok():
                        late_handle.cancel_goal_async()

            pending.add_done_callback(cancel_late_goal)
            raise
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected junction scan turn')
        try:
            result = self._wait(handle.get_result_async(), 17.0)
        except TimeoutError:
            if self.node.context.ok():
                handle.cancel_goal_async()
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            status = {
                GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled',
            }.get(result.status, f'failed with status {result.status}')
            progress = (
                f'{math.degrees(traveled):.1f}° reported'
                if traveled is not None else 'no yaw feedback'
            )
            raise RuntimeError(
                f'Nav2 Spin {status} for {math.degrees(angle):+.1f}° '
                f'({progress}); Nav2 aborts a spin when its local costmap '
                'predicts the footprint hitting something (Collision Ahead)'
            )
        self.last_traveled = abs(angle)

    def _fresh_frame(self, settle_sec=0.0):
        # Let the robot settle and queued images from the turn clear.
        after = time.monotonic() + settle_sec
        while self.node.context.ok() and time.monotonic() < after + 2.0:
            received = self.node.latest_receive_time
            if received is not None and received > after:
                return self.node.latest_frame.copy()
            time.sleep(0.02)
        raise TimeoutError('No fresh camera image during junction scan')

    def _restore(self, center_yaw, commanded):
        """
        Turn back to the trigger heading after a view turn of `commanded`.

        With TF: turn by the measured offset, then up to MAX_CORRECTIONS
        small turns until within HEADING_TOL (Spin overshoots). Without TF:
        undo the distance Spin reported (all of it after a success, part of
        it after an abort).
        """
        heading = self._heading() if center_yaw is not None else None
        if heading is None:
            traveled = getattr(self, 'last_traveled', None)
            back = -math.copysign(
                abs(commanded) if traveled is None else traveled, commanded)
            if abs(back) > HEADING_TOL:
                self._turn(back)
            return
        for _ in range(1 + MAX_CORRECTIONS):
            error = math.remainder(center_yaw - heading, math.tau)
            if abs(error) <= HEADING_TOL:
                return
            self._turn(error)
            heading = self._heading()
            if heading is None:
                return
        error = math.remainder(center_yaw - heading, math.tau)
        if abs(error) > HEADING_TOL:
            raise RuntimeError(
                f'Could not return to the trigger heading '
                f'({math.degrees(error):+.1f}° off)')

    def _view(self, name, angle, center_yaw):
        """Turn by `angle`, take a fresh frame, always turn back."""
        try:
            self._turn(angle)
            heading = self._heading() if center_yaw is not None else None
            self.view_offsets_deg[name] = (
                round(math.degrees(math.remainder(heading - center_yaw,
                                                  math.tau)), 1)
                if heading is not None else None)
            return self._fresh_frame(0.3)
        finally:
            if self.node.context.ok():
                self._restore(center_yaw, angle)

    def capture(self, center):
        """Return a three-view strip, or None if no turn server is ready."""
        if not self.spin.server_is_ready():
            return None
        # The trigger frame defines the center heading for this scan.
        center_yaw = self._heading()
        self.view_offsets_deg = {}
        left = self._view('left', self.angle, center_yaw)
        right = self._view('right', -self.angle, center_yaw)

        if left.shape != center.shape or right.shape != center.shape:
            raise ValueError('Camera resolution changed during junction scan')
        return np.concatenate((left, center, right), axis=1)
