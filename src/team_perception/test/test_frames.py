"""Frame handling: one tracking frame for announcements, non-blocking TF helper."""
import pytest

from team_perception.poster_utils import Announced
from team_perception.qr_utils import transform_first


def test_announced_once_per_position():
    a = Announced(0.5)
    assert a.add((1.0, 0.0, 0.0))
    assert not a.add((1.3, 0.1, 0.0))          # same poster, seen again
    assert a.add((2.0, 0.0, 0.0))              # another poster


tf2_ros = pytest.importorskip('tf2_ros')


def _buffer_with(*frames_xy):
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import TransformStamped
    buf = tf2_ros.Buffer()
    for parent, child, x, sec in frames_xy:
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp = TimeMsg(sec=sec)
        t.transform.translation.x = x
        t.transform.rotation.w = 1.0
        buf.set_transform(t, 'test')
    return buf


def _point(frame, sec):
    import tf2_geometry_msgs  # noqa: F401
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import PointStamped
    p = PointStamped()
    p.header.frame_id, p.header.stamp = frame, TimeMsg(sec=sec)
    return p


def test_transform_first_falls_back_to_odom_without_waiting():
    import time
    buf = _buffer_with(('odom', 'cam', 0.5, 10), ('odom', 'cam', 0.5, 11))
    t = time.perf_counter()
    out, err = transform_first(buf, _point('cam', 10), ['map', 'odom'])
    assert time.perf_counter() - t < 0.05           # no blocking wait for map
    assert out.header.frame_id == 'odom' and out.point.x == pytest.approx(0.5)
    assert err is None


def test_transform_first_prefers_map_and_reports_errors():
    buf = _buffer_with(('map', 'odom', 1.0, 10), ('map', 'odom', 1.0, 11),
                       ('odom', 'cam', 0.5, 10), ('odom', 'cam', 0.5, 11))
    out, _ = transform_first(buf, _point('cam', 10), ['map', 'odom'])
    assert out.header.frame_id == 'map' and out.point.x == pytest.approx(1.5)
    out, err = transform_first(buf, _point('cam', 30), ['map', 'odom'])   # too new
    assert out is None and set(err) == {'map', 'odom'}
