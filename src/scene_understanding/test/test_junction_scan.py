"""Check junction scan heading recovery without commanding a robot."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from scene_understanding.junction_scan import JunctionScan


@pytest.mark.parametrize('frame_timeout', [False, True])
def test_scan_restores_center_after_spin_overshoot(monkeypatch, frame_timeout):
    center = math.radians(175)
    heading = center
    scan = JunctionScan.__new__(JunctionScan)
    scan.node = SimpleNamespace(context=SimpleNamespace(ok=lambda: True))
    scan.angle = math.radians(30)
    scan.spin = SimpleNamespace(server_is_ready=lambda: True)
    scan._heading = lambda: heading
    frame = np.zeros((2, 3, 3), dtype=np.uint8)
    scan._fresh_frame = Mock(return_value=frame)
    if frame_timeout:
        scan._fresh_frame.side_effect = TimeoutError('No fresh frame')
    monkeypatch.setattr('scene_understanding.junction_scan.time.sleep', lambda _: None)
    turns = []

    def turn(angle):
        nonlocal heading
        turns.append(math.degrees(angle))
        overshoot = 3.0 if abs(angle) > math.radians(10) else 0.0
        heading = math.remainder(heading + angle + math.radians(overshoot), math.tau)

    scan._turn = turn
    if frame_timeout:
        with pytest.raises(TimeoutError, match='No fresh frame'):
            scan.capture(frame)
        assert turns == pytest.approx([30, -33, -3])
    else:
        assert scan.capture(frame).shape == (2, 9, 3)
        assert turns == pytest.approx([30, -33, -3, -30, 27, -3])
    assert heading == pytest.approx(center)


def make_scan(heading_fn):
    scan = JunctionScan.__new__(JunctionScan)
    scan.node = SimpleNamespace(context=SimpleNamespace(ok=lambda: True))
    scan.angle = math.radians(30)
    scan.spin = SimpleNamespace(server_is_ready=lambda: True)
    scan.view_offsets_deg = {}
    scan.last_traveled = None
    scan._heading = heading_fn
    scan._fresh_frame = Mock(return_value=np.zeros((2, 3, 3), dtype=np.uint8))
    return scan


def test_aborted_turn_returns_to_center_by_measured_heading():
    """Nav2 'Collision Ahead' at 22.6 deg: turn back by what was measured."""
    heading = 0.0
    turns = []
    scan = make_scan(lambda: heading)

    def turn(angle):
        nonlocal heading
        turns.append(round(math.degrees(angle), 1))
        if not turns[1:]:                       # first turn aborts part way
            heading += math.radians(22.6)
            raise RuntimeError('Nav2 Spin aborted for +30.0° (22.6° reported)')
        heading += angle

    scan._turn = turn
    with pytest.raises(RuntimeError, match='aborted'):
        scan.capture(np.zeros((2, 3, 3), dtype=np.uint8))
    assert turns == [30.0, -22.6]
    assert heading == pytest.approx(0.0)


def test_aborted_turn_without_tf_undoes_reported_distance():
    turns = []
    scan = make_scan(lambda: None)

    def turn(angle):
        turns.append(round(math.degrees(angle), 1))
        if len(turns) == 1:
            scan.last_traveled = math.radians(22.6)
            raise RuntimeError('aborted')

    scan._turn = turn
    with pytest.raises(RuntimeError):
        scan.capture(np.zeros((2, 3, 3), dtype=np.uint8))
    assert turns == [30.0, -22.6]


def test_measured_view_offsets_recorded():
    heading = 0.0
    scan = make_scan(lambda: heading)

    def turn(angle):
        nonlocal heading
        heading += angle * 1.1 if abs(angle) > math.radians(10) else angle

    scan._turn = turn
    assert scan.capture(np.zeros((2, 3, 3), dtype=np.uint8)).shape == (2, 9, 3)
    assert scan.view_offsets_deg == {'left': 33.0, 'right': -33.0}
    assert heading == pytest.approx(0.0, abs=math.radians(2))
