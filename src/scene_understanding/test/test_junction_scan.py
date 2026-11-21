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
