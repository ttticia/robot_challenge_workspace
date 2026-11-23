"""TimedTurnScan against a fake robot whose odometry arrives late."""

import collections
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from scene_understanding.timed_turn import TimedTurnScan


class LaggyRobot:
    """Turns at the commanded rate (scaled by `gain`); TF heading lags."""

    def __init__(self, gain=0.85, lag_samples=6, start=math.radians(170)):
        self.yaw = start
        self.gain = gain
        self.rate, self.last = 0.0, None
        self.lag = collections.deque([start] * lag_samples, maxlen=lag_samples)
        self.rates = []
        self.subscribers = 1

    def publish(self, msg):
        now = time.monotonic()
        if self.last is not None:
            self.yaw += self.rate * self.gain * (now - self.last)
        self.last, self.rate = now, msg.angular.z
        self.rates.append(msg.angular.z)
        self.lag.append(self.yaw)

    def heading(self):
        return math.remainder(self.lag[0], math.tau)    # late odometry

    def get_subscription_count(self):
        return self.subscribers


def make_scan(robot, **kwargs):
    node = SimpleNamespace(
        context=SimpleNamespace(ok=lambda: True),
        create_publisher=lambda *a, **k: robot,
        get_logger=lambda: SimpleNamespace(warning=lambda *_: None),
        latest_receive_time=None, latest_frame=None,
    )
    kwargs.setdefault('turn_rate', 1.0)
    kwargs.setdefault('settle_sec', 0.05)
    scan = TimedTurnScan(node, 20.0, **kwargs)
    scan.period = 0.01
    scan.steady_window = 0.08
    scan.steady_tol = math.radians(0.2)
    scan._heading = robot.heading
    original = robot.publish

    def publish(msg):
        original(msg)
        node.latest_frame = np.full(
            (2, 3, 3), 128 + int(round(math.degrees(robot.yaw - start))), np.uint8)
        node.latest_receive_time = time.monotonic()

    start = robot.yaw
    robot.publish = publish
    return scan


def test_left_center_right_and_back_despite_lag_and_slip():
    robot = LaggyRobot(gain=0.8)            # robot turns 20 % less than asked
    start = robot.yaw
    scan = make_scan(robot)
    strip = scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert strip.shape == (2, 9, 3)
    assert strip[0, 0, 0] > 128 > strip[0, 8, 0]          # left, then right
    assert scan.view_offsets_deg['left'] == pytest.approx(20, abs=3)
    assert scan.view_offsets_deg['right'] == pytest.approx(-20, abs=3)
    assert math.degrees(math.remainder(robot.yaw - start, math.tau)) == \
        pytest.approx(0, abs=3)
    assert robot.rates[-1] == 0.0
    assert max(abs(r) for r in robot.rates) == pytest.approx(1.0)


def test_turn_rate_is_the_cap():
    robot = LaggyRobot(gain=1.0)
    scan = make_scan(robot, turn_rate=0.5)
    scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert max(abs(r) for r in robot.rates) == pytest.approx(0.5)


def test_refuses_a_huge_correction():
    """A heading far off (TF jump, robot already turned) is not chased."""
    robot = LaggyRobot()
    scan = make_scan(robot)
    calls = []

    def jumpy():
        calls.append(1)
        h = robot.heading()
        return h if len(calls) == 1 else h + math.radians(73)

    scan._heading = jumpy
    with pytest.raises(RuntimeError, match='more than expected'):
        scan.capture(np.full((2, 3, 3), 128, np.uint8))
    assert robot.rates[-1] == 0.0


def test_no_twist_mux_moves_nothing():
    robot = LaggyRobot()
    robot.subscribers = 0
    scan = make_scan(robot)
    assert scan.capture(np.zeros((2, 3, 3), np.uint8)) is None
    assert 'twist_mux' in scan.unavailable_reason
    assert robot.rates == []


def test_without_tf_open_loop_returns_to_center():
    robot = LaggyRobot(gain=1.0)
    start = robot.yaw
    scan = make_scan(robot)
    scan._heading = lambda: None
    assert scan.capture(np.full((2, 3, 3), 128, np.uint8)).shape == (2, 9, 3)
    assert scan.view_offsets_deg == {'left': None, 'right': None}
    assert math.degrees(robot.yaw - start) == pytest.approx(0, abs=4)


def test_abort_stops_with_zero_velocity():
    robot = LaggyRobot()
    scan = make_scan(robot, turn_rate=0.1)   # a slow turn to interrupt
    errors = []

    def run():
        try:
            scan.capture(np.full((2, 3, 3), 128, np.uint8))
        except RuntimeError as e:
            errors.append(e)

    worker = threading.Thread(target=run)
    worker.start()
    deadline = time.monotonic() + 2.0
    while not any(robot.rates) and time.monotonic() < deadline:
        time.sleep(0.01)
    scan.abort()
    worker.join(3.0)
    assert not worker.is_alive()
    assert robot.rates[-1] == 0.0


def test_parameter_limits():
    robot = LaggyRobot()
    node = SimpleNamespace(create_publisher=lambda *a, **k: robot)
    for args, kwargs in (((0.0,), {}), ((95.0,), {}), ((20.0,), {'turn_rate': 2.0}),
                         ((20.0,), {'turn_rate': 0.05})):
        with pytest.raises(ValueError):
            TimedTurnScan(node, *args, **kwargs)
