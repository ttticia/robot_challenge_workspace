"""Pairing a detection with its depth frame when depth arrives late (StampBuffer)."""
import pytest

pytest.importorskip('rclpy')
from team_perception.poster_detector_node import StampBuffer  # noqa: E402


def test_match_waits_until_the_frame_arrives():
    b = StampBuffer(keep_s=2.0)
    assert b.match(10.0, 0.1) == ('wait', None)            # no depth yet: may come
    for t in (9.7, 9.8, 9.9):
        b.add(t, f'd{t}')
    state, dt = b.match(10.2, 0.1)
    assert state == 'wait' and dt == pytest.approx(0.3)
    b.add(10.2, 'd10.2')
    assert b.match(10.2, 0.1) == ('ok', 'd10.2')
    assert b.match(10.15, 0.1)[0] == 'ok'                  # 0.05 s from 10.2


def test_out_of_order_frames_are_kept_sorted():
    b = StampBuffer(keep_s=2.0)
    for t in (1.0, 1.1, 1.3, 1.4):                         # 1.2 is late ...
        b.add(t, f'd{t}')
    b.add(1.2, 'd1.2')                                     # ... and arrives after 1.4
    assert b.stamps == sorted(b.stamps)
    assert b.match(1.21, 0.05) == ('ok', 'd1.2')


def test_old_frames_are_dropped():
    b = StampBuffer(keep_s=1.0)
    for t in (0.0, 0.5, 1.0, 1.6):
        b.add(t, t)
    assert b.stamps == [1.0, 1.6]
    b.add(0.2, 0.2)                                        # very late: dropped at once
    assert b.stamps == [1.0, 1.6]
