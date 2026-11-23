"""Terrain push (user 2026-11-23, runs/IMG_1673.MOV): at the edge of an uneven-terrain cell
cmd_vel_bridge drives straight on by itself instead of creeping at the capped speed."""
import pytest

from maze_nav.cmd_vel_bridge import push_armed, push_distance


def test_push_starts_at_the_edge_when_nav2_drives_forward():
    assert push_distance('obstacle', 0.40, True, 0.1, 0.10) == pytest.approx(0.75)
    assert push_distance('obstacle', 0.30, True, 0.1, 0.10, past=0.5) == pytest.approx(0.80)


@pytest.mark.parametrize('level, edge, armed, age, v', [
    ('obstacle', 0.90, True, 0.1, 0.10),    # edge still far: Nav2 keeps steering
    ('obstacle', None, True, 0.1, 0.10),    # no edge seen
    ('obstacle', 0.40, False, 0.1, 0.10),     # not entering: already on the terrain
    ('obstacle', 0.40, True, 5.0, 0.10),    # stale terrain message
    ('obstacle', 0.40, True, 0.1, 0.0),     # Nav2 turning on the spot / stopped
    ('rough', 0.40, True, 0.1, 0.10),
    ('drop', 0.40, True, 0.1, 0.10),        # leaving a tile: no push
    ('even', None, True, 0.1, 0.10),
])
def test_no_push(level, edge, armed, age, v):
    assert push_distance(level, edge, armed, age, v) is None


def test_armed_on_even_ground_and_kept_until_the_push():
    """The walk into the block field (stairs_lidar.terrain on the simulated maze): on_rough
    turns True just as the edge comes within reach, so arming must survive that."""
    armed = False
    for edge, on_rough in ((0.98, False), (0.70, False), (0.52, False)):
        armed = push_armed(armed, edge, on_rough)
    assert armed
    assert push_armed(armed, 0.42, True)             # at the edge: still armed -> push
    assert not push_armed(False, 0.36, True)         # on the field after the push: stays off
    assert not push_armed(False, 0.30, False)        # a flat patch inside, edge still near
    assert push_armed(False, None, False)            # back on even ground
