"""tf_pin: the depth camera keeps its pitched parent although /tf_static has a second one."""
import math

import pytest

tf2_ros = pytest.importorskip('tf2_ros')
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from team_perception.tf_pin import DEFAULT_PINS, TfParentPin, parse_pins  # noqa: E402

OPTICAL = (-0.5, 0.5, -0.5, 0.5)


def tf(parent, child, t, q):
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t)
    r = m.transform.rotation
    r.x, r.y, r.z, r.w = map(float, q)
    return m


# As seen live on domain 4, 2026-11-18 (robot side, via lite3_domain_bridge_4).
CAMERA_LINK = tf('base_link', 'camera_link', (0.2523, -0.0005, 0.0741), (0.0, 0.1736, 0.0, 0.9848))
DEPTH_FRAME = tf('camera_link', 'camera_depth_frame', (0, 0, 0), (0, 0, 0, 1))
GOOD = tf('camera_depth_frame', 'camera_depth_optical_frame', (0, 0, 0), OPTICAL)
BAD = tf('base_link', 'camera_depth_optical_frame', (0.25, 0, 0.05), OPTICAL)
COLOUR = tf('camera_depth_frame', 'camera_color_optical_frame', (0.0018, -0.0141, 0.0001), OPTICAL)


def degrees_down(buffer):
    """How far the depth camera's optical axis (z) points below horizontal, in base_link."""
    q = buffer.lookup_transform('base_link', 'camera_depth_optical_frame', Time()).transform.rotation
    zx = 2 * (q.x * q.z + q.w * q.y)
    zz = 1 - 2 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.atan2(-zz, zx))


class Log:
    def __init__(self):
        self.lines = []

    def warn(self, text):
        self.lines.append(text)


def test_parse_pins():
    assert parse_pins(DEFAULT_PINS) == {'camera_depth_optical_frame': 'camera_depth_frame'}
    assert parse_pins(['']) == {} and parse_pins([]) == {} and parse_pins(['nocolon']) == {}
    assert parse_pins([' /a : b ']) == {'a': 'b'}


@pytest.mark.parametrize('order', ['bad_last', 'bad_first'])
def test_pin_keeps_the_pitched_parent(order):
    b, log = tf2_ros.Buffer(), Log()
    pin = TfParentPin(b, parse_pins(DEFAULT_PINS), log)
    msgs = [[CAMERA_LINK, DEPTH_FRAME, GOOD, COLOUR], [BAD]]
    if order == 'bad_first':
        msgs.reverse()
    for transforms in msgs:
        for t in transforms:                     # what the node's TransformListener does
            b.set_transform_static(t, 'listener')
        pin.on_static(TFMessage(transforms=transforms))
    assert degrees_down(b) == pytest.approx(20.0, abs=0.1)
    assert len(log.lines) == 1 and 'base_link -> camera_depth_optical_frame' in log.lines[0]
    # The listener takes the bad one again after us: the next apply() (timer) fixes it.
    b.set_transform_static(BAD, 'listener')
    assert degrees_down(b) == pytest.approx(0.0, abs=0.1)
    pin.apply()
    assert degrees_down(b) == pytest.approx(20.0, abs=0.1)


def test_colour_and_depth_line_up_again():
    # With the bad parent, colour (pitched chain) and depth (unpitched) are 20 deg apart.
    b = tf2_ros.Buffer()
    pin = TfParentPin(b, parse_pins(DEFAULT_PINS))
    for t in (CAMERA_LINK, DEPTH_FRAME, GOOD, COLOUR, BAD):
        b.set_transform_static(t, 'listener')
    q = b.lookup_transform('camera_depth_optical_frame', 'camera_color_optical_frame',
                           Time()).transform.rotation
    assert abs(q.w) < 0.99                       # rotated relative to each other
    pin.on_static(TFMessage(transforms=[CAMERA_LINK, DEPTH_FRAME, GOOD, COLOUR, BAD]))
    q = b.lookup_transform('camera_depth_optical_frame', 'camera_color_optical_frame',
                           Time()).transform.rotation
    assert abs(q.w) == pytest.approx(1.0, abs=1e-3)   # same orientation, as the robot defines


def test_nothing_changes_without_the_wanted_parent():
    b = tf2_ros.Buffer()
    pin = TfParentPin(b, parse_pins(DEFAULT_PINS))
    b.set_transform_static(BAD, 'listener')
    pin.on_static(TFMessage(transforms=[BAD]))
    assert degrees_down(b) == pytest.approx(0.0, abs=0.1)     # left alone: nothing to pin to
