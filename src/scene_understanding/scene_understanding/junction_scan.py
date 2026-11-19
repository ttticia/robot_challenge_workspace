"""Capture a three-panel junction view using the Nav2 Spin behavior."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
import numpy as np
from rclpy.action import ActionClient


class JunctionScan:
    """Turn in place, sample fresh frames, and restore the trigger heading."""

    def __init__(self, node, action_name, angle_deg):
        if not 0 < angle_deg <= 90:
            raise ValueError('junction_scan_angle_deg must be within (0, 90]')
        self.node = node
        self.angle = math.radians(angle_deg)
        self.action_name = action_name
        self.spin = ActionClient(node, Spin, action_name)

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

        def on_feedback(message):
            nonlocal traveled
            traveled = message.feedback.angular_distance_traveled

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
                f'({progress})'
            )

    def _fresh_frame(self, settle_sec=0.0):
        # Let the robot settle and queued images from the turn clear.
        after = time.monotonic() + settle_sec
        while self.node.context.ok() and time.monotonic() < after + 2.0:
            received = self.node.latest_receive_time
            if received is not None and received > after:
                return self.node.latest_frame.copy()
            time.sleep(0.02)
        raise TimeoutError('No fresh camera image during junction scan')

    def capture(self, center):
        """Return a three-view strip, or None if no turn server is ready."""
        if not self.spin.server_is_ready():
            return None
        # The trigger frame defines the center heading for this scan.
        self._turn(self.angle)
        try:
            left = self._fresh_frame(0.3)
        finally:
            if self.node.context.ok():
                self._turn(-self.angle)

        self._turn(-self.angle)
        try:
            right = self._fresh_frame(0.3)
        finally:
            if self.node.context.ok():
                self._turn(self.angle)

        if left.shape != center.shape or right.shape != center.shape:
            raise ValueError('Camera resolution changed during junction scan')
        return np.concatenate((left, center, right), axis=1)
