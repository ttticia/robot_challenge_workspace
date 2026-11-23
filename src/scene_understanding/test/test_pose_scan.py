"""PoseScan's /simple_cmd sequence against a fake robot (no ROS graph)."""

import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from scene_understanding import pose_scan
from scene_understanding.pose_scan import (
    AUTO_MODE, MANUAL_MODE, MOVE_MODE, POSE_MODE, PoseScan, YAW_AXIS,
)


class FakeRobot:
    """Records /simple_cmd; the body yaws while the yaw axis is non-zero."""

    def __init__(self, deg_per_full_axis=16.0):
        self.cmds = []
        self.mode = None
        self.body = 0.0
        self.scale = math.radians(deg_per_full_axis) / 32767
        self.subscribers, self.publishers = 1, 1

    def publish(self, msg):
        self.cmds.append((msg.cmd_code, msg.size))
        if msg.cmd_code == YAW_AXIS:
            if self.mode == 'pose':
                self.body = -msg.size * self.scale     # + axis = rotate right
        elif msg.cmd_code == POSE_MODE:
            self.mode = 'pose'
        elif msg.cmd_code == MOVE_MODE:
            self.mode, self.body = 'move', 0.0

    def get_subscription_count(self):
        return self.subscribers

    def codes(self):
        """Command codes in order, runs of the same code collapsed."""
        out = []
        for code, _ in self.cmds:
            if not out or out[-1] != code:
                out.append(code)
        return out


def make_scan(monkeypatch, **kwargs):
    robot = FakeRobot()
    node = SimpleNamespace(
        context=SimpleNamespace(ok=lambda: True),
        create_publisher=lambda *a, **k: robot,
        count_publishers=lambda topic: robot.publishers,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None,
                                           error=lambda *_: None),
        latest_receive_time=None,
        latest_frame=None,
    )
    fake_msg = SimpleNamespace(MotionSimpleCMD=lambda **k: SimpleNamespace(**k))
    monkeypatch.setitem(__import__('sys').modules, 'transfer_interfaces.msg',
                        fake_msg)
    monkeypatch.setitem(__import__('sys').modules, 'transfer_interfaces',
                        SimpleNamespace(msg=fake_msg))
    kwargs.setdefault('settle_sec', 0.05)
    scan = PoseScan(node, 13.0, mode_settle_sec=0.01, **kwargs)
    scan.period = 0.005
    scan._heading = lambda: robot.body
    original = robot.publish

    def publish(msg):
        original(msg)
        # Camera on the body: a frame showing the body yaw, arriving now.
        value = 128 + int(round(math.degrees(robot.body)))
        node.latest_frame = np.full((2, 3, 3), value, np.uint8)
        node.latest_receive_time = time.monotonic()

    robot.publish = publish
    return scan, robot


def test_full_sequence_and_three_panels(monkeypatch):
    scan, robot = make_scan(monkeypatch)
    strip = scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert strip.shape == (2, 9, 3)
    assert strip[0, 0, 0] > 128 > strip[0, 8, 0]          # left, right panels
    assert robot.codes() == [MANUAL_MODE, POSE_MODE, YAW_AXIS, MOVE_MODE,
                             AUTO_MODE]
    axes = [size for code, size in robot.cmds if code == YAW_AXIS]
    # Left = negative axis (robot: + rotates right); 13 of 16 deg at full axis.
    assert axes[0] == -26623 and min(axes) == -26623 and max(axes) == 26623
    assert axes[-1] == 0                                   # body recentred
    assert scan.view_offsets_deg['left'] == pytest.approx(13.0, abs=0.1)
    assert scan.view_offsets_deg['right'] == pytest.approx(-13.0, abs=0.1)
    assert not scan.active and robot.mode == 'move'


def test_without_manual_no_manual_or_auto_mode(monkeypatch):
    scan, robot = make_scan(monkeypatch, manual=False)
    scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert robot.codes() == [POSE_MODE, YAW_AXIS, MOVE_MODE]


def test_axis_scaled_and_capped(monkeypatch):
    scan, _ = make_scan(monkeypatch)
    assert scan.axis_value(math.radians(8)) == -16384     # half of 16 deg
    assert scan.axis_value(math.radians(-40)) == 32767    # default: full axis
    capped, _ = make_scan(monkeypatch, axis_limit=0.8)
    assert capped.axis_value(math.radians(-40)) == int(32767 * 0.8)


def test_lagging_odometry_still_gives_right_view_negative(monkeypatch):
    """
    Record the furthest yaw over the whole look.

    Fast-LIO lags ~1 s: at the right photo TF can still show the left side.
    """
    scan, robot = make_scan(monkeypatch, settle_sec=0.05)
    history = []

    def lagging():
        history.append(robot.body)
        return history[max(0, len(history) - 15)]     # ~15 samples behind

    scan._heading = lagging
    scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert scan.view_offsets_deg['left'] > 10
    assert scan.view_offsets_deg['right'] < -10


def test_axis_sent_continuously_while_posing(monkeypatch):
    scan, robot = make_scan(monkeypatch)
    times = []
    original = robot.publish

    def timed(msg):
        if msg.cmd_code == YAW_AXIS:
            times.append(time.monotonic())
        original(msg)

    robot.publish = timed
    scan.pub = robot
    scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert np.diff(times).max() < 0.2        # the robot recentres after ~1 s


def test_leaves_pose_mode_when_the_camera_stops(monkeypatch):
    scan, robot = make_scan(monkeypatch)
    node = scan.node
    original = robot.publish

    def no_frames(msg):
        original(msg)
        node.latest_receive_time = 0.0       # camera silent

    robot.publish = no_frames
    scan.pub = robot
    monkeypatch.setattr(pose_scan.time, 'sleep', lambda s: None)
    clock = iter(range(10**6))
    monkeypatch.setattr(pose_scan.time, 'monotonic', lambda: next(clock) * 0.01)
    with pytest.raises(TimeoutError, match='No fresh camera image'):
        scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert robot.codes()[-2:] == [MOVE_MODE, AUTO_MODE]
    assert not scan.active


@pytest.mark.parametrize('setup, reason', [
    (lambda robot, scan: setattr(robot, 'publishers', 2), 'other publisher'),
    (lambda robot, scan: setattr(robot, 'subscribers', 0), 'no subscriber'),
    (lambda robot, scan: setattr(scan, 'pub', None), 'transfer_interfaces'),
])
def test_refuses_and_sends_nothing(monkeypatch, setup, reason):
    scan, robot = make_scan(monkeypatch)
    setup(robot, scan)
    assert scan.capture(np.zeros((2, 3, 3), np.uint8)) is None
    assert reason in scan.unavailable_reason
    assert robot.cmds == []


def test_abort_mid_look_leaves_pose_mode(monkeypatch):
    scan, robot = make_scan(monkeypatch, settle_sec=30.0)   # never finishes
    errors = []

    def run():
        try:
            scan.capture(np.full((2, 3, 3), 128, np.uint8))
        except RuntimeError as e:
            errors.append(e)

    worker = threading.Thread(target=run)
    worker.start()
    deadline = time.monotonic() + 2.0
    while robot.mode != 'pose' and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.05)
    scan.abort()                              # Ctrl+C in junction_trigger
    worker.join(2.0)
    assert not worker.is_alive() and errors
    assert robot.codes()[-2:] == [MOVE_MODE, AUTO_MODE]
    axes_after_move = [s for c, s in robot.cmds[robot.cmds.index(
        next(c for c in robot.cmds if c[0] == MOVE_MODE)):] if c == YAW_AXIS]
    assert all(s == 0 for s in axes_after_move)


def test_parameter_limits(monkeypatch):
    scan, _ = make_scan(monkeypatch)
    node = scan.node
    for kwargs in ({'yaw_deg': 0.0}, {'yaw_deg': 45.0},
                   {'yaw_deg': 10.0, 'axis_limit': 1.5},
                   {'yaw_deg': 10.0, 'max_yaw_deg': 0.0}):
        with pytest.raises(ValueError):
            PoseScan(node, **kwargs)
