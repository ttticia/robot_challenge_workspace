"""
Capture a three-panel junction view by turning the whole robot slowly.

Turn `angle` left, take a frame, turn back to centre, turn `angle` right,
take a frame, turn back to centre.

Why not Nav2 Spin: Spin steers by odometry, and Fast-LIO odometry on this
robot arrives ~1.2 s late. Spin keeps turning until the late reading shows
the target, so the robot overshoots (2026-11-23 04:52: asked 20 deg, reached
~73 deg). Here each turn is open loop and slow: `turn_rate` rad/s for
angle / turn_rate s, then zero; then wait until the TF heading has stopped
changing (the late odometry has caught up) and correct the remaining error
with up to `max_corrections` short turns while it exceeds `tolerance`.

Velocity goes to twist_mux's navigation input (`/nav_cmd_vel` in
lite3_launcher, priority 10), so the joystick (priority 100) overrides it.
The robot must be in auto (navigation) mode. There is NO collision check:
stand the robot with clear space around it (its corners sweep ~0.36 m).

A turn larger than `angle` + `max_error` is refused (TF jump or the robot
already far off): the scan stops with an error instead of driving the robot
round.
"""

import math
import threading
import time

from geometry_msgs.msg import Twist
import numpy as np

from scene_understanding.junction_scan import tf_heading


class TimedTurnScan:
    """Slow open-loop turns with settled-heading corrections; three views."""

    def __init__(self, node, angle_deg, cmd_topic='/nav_cmd_vel',
                 turn_rate=0.4, settle_sec=1.5, tolerance_deg=3.0,
                 max_corrections=2, max_error_deg=25.0):
        """
        Create the velocity publisher (moves nothing).

        angle_deg: turn for each side view (0, 90].
        turn_rate: rad/s while turning (the Lite3 does not turn below ~0.1).
        settle_sec: s at zero velocity before the heading is trusted; then
            it also waits until the heading is steady.
        """
        if not 0 < angle_deg <= 90:
            raise ValueError('junction_scan_angle_deg must be within (0, 90]')
        if not 0.1 <= turn_rate <= 1.0:
            raise ValueError('junction_turn_rate must be within [0.1, 1.0] rad/s')
        self.node = node
        self.angle = math.radians(angle_deg)
        self.cmd_topic = cmd_topic
        self.turn_rate = float(turn_rate)
        self.settle_sec = float(settle_sec)
        self.tolerance = math.radians(tolerance_deg)
        self.max_corrections = int(max_corrections)
        self.max_error = math.radians(max_error_deg)
        self.period = 0.05                 # 20 Hz (twist_mux timeout 0.5 s)
        self.steady_window = 0.5           # s the heading must stay within
        self.steady_tol = math.radians(0.5)
        self.max_settle = 4.0              # s, then use the heading as it is
        self.view_offsets_deg = {}
        self.unavailable_reason = None
        self.stopping = False
        self.active = False
        self._commanded = 0.0              # open-loop offset (no TF)
        self._known = None                 # settled heading since the last turn
        self._lock = threading.Lock()
        self.pub = node.create_publisher(Twist, cmd_topic, 10)

    # ------------------------------------------------------------ helpers
    def _heading(self):
        return tf_heading(self.node)

    def _running(self):
        return self.node.context.ok() and not self.stopping

    def _send(self, rate):
        msg = Twist()
        msg.angular.z = float(rate)
        self.pub.publish(msg)

    def _zero_for(self, seconds, stoppable=True):
        end = time.monotonic() + seconds
        while (self._running() if stoppable else self.node.context.ok()):
            self._send(0.0)
            if time.monotonic() >= end:
                return
            time.sleep(self.period)

    def _turn_open(self, angle):
        """Turn by `angle` rad open loop at turn_rate, then stop."""
        duration = abs(angle) / self.turn_rate
        rate = math.copysign(self.turn_rate, angle)
        end = time.monotonic() + duration
        try:
            while self._running() and time.monotonic() < end:
                self._send(rate)
                time.sleep(self.period)
        finally:
            self._zero_for(0.2, stoppable=False)
            self._known = None
        self._commanded += angle

    def _settled_heading(self):
        """Hold zero velocity until the TF heading is steady; return it."""
        self._zero_for(self.settle_sec)
        start = time.monotonic()
        samples = []
        while self._running():
            self._send(0.0)
            now = time.monotonic()
            heading = self._heading()
            if heading is None:
                return None
            samples.append((now, heading))
            recent = [h for t, h in samples if now - t <= self.steady_window]
            if now - samples[0][0] >= self.steady_window and \
                    max(abs(math.remainder(h - heading, math.tau)) for h in recent) \
                    <= self.steady_tol:
                self._known = heading
                return heading
            if now - start > self.max_settle:
                self._known = heading
                return heading
            time.sleep(self.period)
        raise RuntimeError('Turn scan stopped')

    def _turn_to(self, offset, center_yaw):
        """Turn so the heading is center_yaw + offset (rad)."""
        if center_yaw is None:                      # no TF: open loop only
            self._turn_open(offset - self._commanded)
            self._zero_for(self.settle_sec)
            return None
        heading = self._known if self._known is not None else self._settled_heading()
        for _ in range(1 + self.max_corrections):
            if heading is None:
                return None
            error = math.remainder(center_yaw + offset - heading, math.tau)
            if abs(error) <= self.tolerance:
                return heading
            if abs(error) > self.angle + self.max_error:
                raise RuntimeError(
                    f'Turn scan: heading is {math.degrees(error):+.0f} deg from '
                    f'the target, more than expected; not turning (check the '
                    'robot and TF)')
            self._turn_open(error)
            heading = self._settled_heading()
        error = math.remainder(center_yaw + offset - heading, math.tau)
        if abs(error) > self.tolerance:
            self.node.get_logger().warning(
                f'Turn scan: {math.degrees(error):+.1f} deg off the target '
                f'after {self.max_corrections} corrections')
        return heading

    def _fresh_frame(self):
        ready = time.monotonic()
        while self._running():
            self._send(0.0)
            received = getattr(self.node, 'latest_receive_time', None)
            if received is not None and received > ready:
                return self.node.latest_frame.copy()
            if time.monotonic() - ready > 2.0:
                raise TimeoutError('No fresh camera image during turn scan')
            time.sleep(self.period)
        raise RuntimeError('Turn scan stopped')

    def _view(self, name, offset, center_yaw):
        try:
            heading = self._turn_to(offset, center_yaw)
            self.view_offsets_deg[name] = (
                round(math.degrees(math.remainder(heading - center_yaw, math.tau)), 1)
                if heading is not None and center_yaw is not None else None)
            return self._fresh_frame()
        finally:
            if self.node.context.ok() and not self.stopping:
                self._turn_to(0.0, center_yaw)

    # ------------------------------------------------------------ interface
    def capture(self, center):
        """Return a three-view strip, or None if nothing would turn the robot."""
        if self.pub.get_subscription_count() == 0:
            self.unavailable_reason = (
                f'no subscriber on {self.cmd_topic} (is lite3_launcher\'s '
                'twist_mux running?)')
            return None
        self.unavailable_reason = None
        self.stopping = False
        self.active = True
        self._commanded = 0.0
        self.view_offsets_deg = {}
        center_yaw = self._heading()
        self._known = center_yaw           # standing still at the trigger
        try:
            left = self._view('left', self.angle, center_yaw)
            right = self._view('right', -self.angle, center_yaw)
        finally:
            self.active = False
            if self.node.context.ok():
                self._zero_for(0.3, stoppable=False)
        if left.shape != center.shape or right.shape != center.shape:
            raise ValueError('Camera resolution changed during turn scan')
        return np.concatenate((left, center, right), axis=1)

    def abort(self):
        """Stop turning now (node shutting down): zero velocity."""
        if self.active:
            self.stopping = True
            time.sleep(3 * self.period)
            if self.node.context.ok():
                self._zero_for(0.5, stoppable=False)
