import math

from nav_straight.straight_goal import straight_line, yaw_from_quat


def test_line_ends_exactly_at_length():
    pts = straight_line(1.0, 2.0, 0.0, 1.03, step=0.05)
    assert pts[0] == (1.0, 2.0)
    assert math.isclose(pts[-1][0], 2.03) and math.isclose(pts[-1][1], 2.0)
    steps = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
    assert all(s <= 0.05 + 1e-9 for s in steps)
    assert all(s > 0 for s in steps)


def test_line_follows_heading():
    pts = straight_line(0.0, 0.0, math.pi / 2, 5.0)
    assert len(pts) == 101
    assert all(abs(x) < 1e-9 for x, _ in pts)
    assert math.isclose(pts[-1][1], 5.0)


def test_short_line_has_two_points():
    assert len(straight_line(0.0, 0.0, 0.0, 0.01)) == 2


def test_yaw_from_quat():
    a = 0.7
    assert math.isclose(yaw_from_quat(0, 0, math.sin(a / 2), math.cos(a / 2)), a)
