"""
Capture a three-panel junction view by yawing only the body (Lite3 Pose mode).

The feet stay planted; the body (and the camera on it) yaws left, then right,
then back to centre. The robot is NOT turned round, so Nav2 and its collision
check are not involved.

This node sends the robot commands itself on /simple_cmd
(transfer_interfaces/MotionSimpleCMD, from ~/ros2_ws), the same sequence and
codes as challenge_ws's maze_nav cmd_vel_bridge uses for its Pose looks
(robot_info.md section 6):

  enter  [manual mode 0x21010C02 (`manual`)] -> Pose mode 0x21010D05,
         `mode_settle_sec` apart
  look   yaw axis 0x21010135, value in `size`, at 10 Hz (the robot drops the
         body back to centre ~1 s without one): value = -yaw / max_yaw *
         32767 (+ = rotate RIGHT on the robot, so the sign is flipped; dead
         zone +-9553), at most `axis_limit` of the full axis (full axis made
         the motors buzz, 2026-11-21)
  leave  axis 0 for 0.5 s -> Move mode 0x21010D06 -> [navigation (auto) mode
         0x21010C03 (`manual`)], `mode_settle_sec` apart. The gait is NOT
         re-sent.

Body yaw at full axis: about 16-17 deg in the open (measured 17.2 / 17.5 deg
on 2026-11-23 03:29; 8 deg in a tight cell on 2026-11-21). The defaults use
the full axis (yaw 16 deg, axis_limit 1.0). challenge_ws capped it at 0.8
because full axis made the motors buzz with the body against a wall: keep
the body clear of walls, or set axis_limit 0.8 (~13 deg).

view_offsets_deg: the furthest body yaw TF showed towards each side during
the whole look (incl. the return), like cmd_vel_bridge: Fast-LIO odometry
lags the body by ~1 s, so a reading at the moment of the photo can still
show the previous side.

It refuses to start (centre frame only) when another node publishes
/simple_cmd (cmd_vel_bridge, a teleop script): two senders would fight over
the robot mode. Never run it during a maze_nav run.
"""

import math
import threading
import time

import numpy as np
from rclpy.duration import Duration
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from scene_understanding.junction_scan import tf_heading

MANUAL_MODE = 0x21010C02
POSE_MODE = 0x21010D05
MOVE_MODE = 0x21010D06
AUTO_MODE = 0x21010C03
YAW_AXIS = 0x21010135      # Pose mode: + = rotate right (robot convention)
AXIS_MAX = 32767


class PoseScan:
    """Yaw the body left and right in Pose mode; stitch three views."""

    def __init__(self, node, yaw_deg, max_yaw_deg=16.0, axis_limit=1.0,
                 settle_sec=2.0, manual=True, mode_settle_sec=0.5,
                 topic='/simple_cmd'):
        """
        Create the /simple_cmd publisher (moves nothing).

        yaw_deg: body yaw for the side views (+-); limited by max_yaw_deg *
            axis_limit (about 13 deg).
        max_yaw_deg: body yaw at full axis (measured ~16 deg).
        axis_limit: fraction of the full axis never exceeded.
        settle_sec: s after each yaw command before the frame is taken.
        manual: manual mode before Pose mode and auto mode after it (the
            robot then follows /cmd_vel again, as before a teleop test).
        """
        if not 0 < yaw_deg <= 30:
            raise ValueError('junction_pose_yaw_deg must be within (0, 30]')
        if not 0 < axis_limit <= 1:
            raise ValueError('junction_pose_axis_limit must be within (0, 1]')
        if max_yaw_deg <= 0:
            raise ValueError('junction_pose_max_yaw_deg must be positive')
        self.node = node
        self.angle = math.radians(yaw_deg)
        self.max_yaw = math.radians(max_yaw_deg)
        self.axis_max = int(AXIS_MAX * axis_limit)
        self.settle_sec = float(settle_sec)
        self.manual = bool(manual)
        self.mode_settle_sec = float(mode_settle_sec)
        self.topic = topic
        self.period = 0.1
        self.view_offsets_deg = {}
        self.unavailable_reason = None
        self.active = False            # in (or entering) Pose mode
        self._center_yaw = None
        self._seen = []                # body yaw offsets (rad) seen this look
        self.stopping = False          # abort(): end the look now
        self._leave_lock = threading.Lock()
        try:
            from transfer_interfaces.msg import MotionSimpleCMD
        except ImportError:
            self.msg_type, self.pub = None, None
            return
        self.msg_type = MotionSimpleCMD
        # Same QoS as cmd_vel_bridge (robot_info.md section 6.2).
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         lifespan=Duration(seconds=1.0))
        self.pub = node.create_publisher(MotionSimpleCMD, topic, qos)

    # ------------------------------------------------------------ commands
    def _cmd(self, code, size=0):
        self.pub.publish(self.msg_type(cmd_code=code, size=int(size), type=0))

    def axis_value(self, yaw):
        """Body yaw (rad, + = left) -> yaw axis value (+ = right on the robot)."""
        v = int(round(-yaw / self.max_yaw * AXIS_MAX))
        return max(-self.axis_max, min(self.axis_max, v))

    def _running(self):
        return self.node.context.ok() and not self.stopping

    def _sample(self):
        """Record the body yaw offset from the trigger heading (TF), if known."""
        if self._center_yaw is None:
            return
        heading = self._heading()
        if heading is not None:
            self._seen.append(math.remainder(heading - self._center_yaw, math.tau))

    def _wait(self, seconds, axis=None, stoppable=True):
        """Wait `seconds`, sending the yaw axis at 10 Hz if `axis` is given."""
        end = time.monotonic() + seconds
        while self._running() if stoppable else self.node.context.ok():
            self._sample()
            if axis is not None:
                self._cmd(YAW_AXIS, axis)
            if time.monotonic() >= end:
                return
            time.sleep(self.period)

    def _heading(self):
        return tf_heading(self.node)

    def _available(self):
        """Return why a pose look can't start now, or None."""
        if self.pub is None:
            return ('transfer_interfaces not found (source '
                    '~/ros2_ws/install/setup.bash before this workspace)')
        if self.pub.get_subscription_count() == 0:
            return (f'no subscriber on {self.topic} (robot bridge not on this '
                    'ROS_DOMAIN_ID?)')
        others = self.node.count_publishers(self.topic) - 1
        if others > 0:
            return (f'{others} other publisher(s) on {self.topic} (cmd_vel_bridge '
                    'or a teleop script?); not sending Pose commands')
        return None

    # ------------------------------------------------------------ the look
    def _enter(self):
        steps = ([('manual mode', MANUAL_MODE)] if self.manual else []) \
            + [('Pose mode', POSE_MODE)]
        self.active = True
        for name, code in steps:
            if not self._running():
                raise RuntimeError('Pose look stopped while entering Pose mode')
            self._cmd(code)
            self.node.get_logger().info(f'Pose look: sent {name} (0x{code:08X})')
            self._wait(self.mode_settle_sec)

    def _view(self, name, yaw):
        """Hold body yaw `yaw`, settle, return a frame received after that."""
        axis = self.axis_value(yaw)
        start = time.monotonic()
        ready = None
        while self._running():
            now = time.monotonic()
            self._sample()
            self._cmd(YAW_AXIS, axis)     # keep it coming: the robot recentres
            if ready is None and now >= start + self.settle_sec:
                ready = now
            if ready is not None:
                received = getattr(self.node, 'latest_receive_time', None)
                if received is not None and received > ready:
                    self.node.get_logger().info(
                        f'Pose look {name}: axis {axis}, frame taken')
                    return self.node.latest_frame.copy()
                if now - ready > 2.0:
                    raise TimeoutError('No fresh camera image during pose look')
            time.sleep(self.period)
        raise RuntimeError('Pose look stopped')

    def _leave(self):
        """Body to centre, Move mode [, auto mode]. Safe to call twice."""
        with self._leave_lock:
            if not self.active:
                return
            if not self.node.context.ok():
                self.node.get_logger().error(
                    'Pose look: ROS shut down in Pose mode; switch the robot '
                    'back with the handheld controller')
                return
            try:
                self._wait(0.5, axis=0, stoppable=False)
                steps = [('Move mode', MOVE_MODE)]
                if self.manual:
                    steps.append(('navigation (auto) mode', AUTO_MODE))
                for name, code in steps:
                    self._cmd(code)
                    self.node.get_logger().info(
                        f'Pose look: sent {name} (0x{code:08X})')
                    self._wait(self.mode_settle_sec, stoppable=False)
            finally:
                self.active = False

    def _record_offsets(self):
        """Furthest body yaw seen towards each side (deg), None if not seen."""
        def furthest(sign):
            toward = [sign * v for v in self._seen if sign * v > 0]
            return round(sign * math.degrees(max(toward)), 1) if toward else None

        if self._center_yaw is None:
            self.view_offsets_deg = {'left': None, 'right': None}
            return
        self.view_offsets_deg = {'left': furthest(1), 'right': furthest(-1)}
        self.node.get_logger().info(
            f'Pose look: body yaw reached {self.view_offsets_deg} deg (TF)')

    def abort(self):
        """End a running look now (node shutting down); leave Pose mode."""
        if self.active:
            self.stopping = True
            time.sleep(3 * self.period)   # the scan thread stops sending the axis
            self._leave()

    def capture(self, center):
        """Return a three-view strip, or None if Pose looks are unavailable."""
        self.unavailable_reason = self._available()
        if self.unavailable_reason:
            return None
        self.stopping = False
        self._center_yaw = self._heading()
        self._seen = []
        self.view_offsets_deg = {}
        try:
            self._enter()
            left = self._view('left', self.angle)
            right = self._view('right', -self.angle)
        finally:
            self._leave()
            self._record_offsets()
        if left.shape != center.shape or right.shape != center.shape:
            raise ValueError('Camera resolution changed during pose look')
        return np.concatenate((left, center, right), axis=1)
