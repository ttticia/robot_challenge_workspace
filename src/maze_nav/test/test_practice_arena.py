"""Practice-arena changes of 2026-11-23 (maze_info.md): 0.53 m walls, toy sets taller than the
walls, pyramid stairs between railings, curtains hanging from ~0.9 m. No ROS.

    python3 -m pytest -q test/test_practice_arena.py
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from maze_nav.cell_features import CellFeatures, FeatureParams, features_to_dict, stair_entry
from maze_nav.cmd_vel_bridge import CmdVelBridge
from maze_nav.curtains import CurtainParams, detect_curtains, tall_cover
from maze_nav.scene_cells import CellGrid
from maze_nav.stair_check import StairCheck
from maze_nav.stairs import StairEstimate
from test_curtains import LIDAR_H, corridor, curtain_box, raycast

CELL = 1.02


def surface(x0, y0, z0, x1, y1, z1, step=0.02):
    """Points on the faces of an axis-aligned box (what a lidar sees of it, roughly)."""
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    zs = np.arange(z0, z1 + 1e-9, step)
    out = []
    for x in (x0, x1):
        Y, Z = np.meshgrid(ys, zs)
        out.append(np.c_[np.full(Y.size, x), Y.ravel(), Z.ravel()])
    for y in (y0, y1):
        X, Z = np.meshgrid(xs, zs)
        out.append(np.c_[X.ravel(), np.full(X.size, y), Z.ravel()])
    X, Y = np.meshgrid(xs, ys)
    out.append(np.c_[X.ravel(), Y.ravel(), np.full(X.size, z1)])
    return np.concatenate(out)


def wall(cell, d, top=0.53):
    """The 0.53 m wall on side d of cell (cell size 1.02, cell (0, 0) at the origin)."""
    cx, cy = cell[0] * CELL, cell[1] * CELL
    h = CELL / 2
    if d[0]:
        x = cx + d[0] * h
        return surface(x - 0.025, cy - h, 0.0, x + 0.025, cy + h, top)
    y = cy + d[1] * h
    return surface(cx - h, y - 0.025, 0.0, cx + h, y + 0.025, top)


def classify(pts, robot=(-3.0, -3.0)):
    f = CellFeatures(FeatureParams(max_range=20.0))
    f.add(pts, 0.0, robot, now=0.0)
    f.add(pts, 0.0, robot, now=0.1)            # two views: min_hits
    return f.classify(CellGrid(CELL))


def pyramid(cell, rails=True):
    cx, cy = cell[0] * CELL, cell[1] * CELL
    pts = []
    for k, z in enumerate((0.15, 0.30, 0.45)):     # 3 x 15 cm, 20 cm treads, up and down
        a = -0.5 + 0.2 * k
        pts.append(surface(cx + a, cy - 0.44, 0.0, cx - a, cy + 0.44, z))
    if rails:
        for y in (cy - 0.46, cy + 0.46):
            pts.append(surface(cx - 0.5, y - 0.01, 0.85, cx + 0.5, y + 0.01, 0.9))   # handrail
            for x in np.arange(cx - 0.5, cx + 0.51, 0.25):                            # posts
                pts.append(surface(x - 0.01, y - 0.01, 0.0, x + 0.01, y + 0.01, 0.9))
    return np.concatenate(pts)


# ------------------------------------------------------------------ cell features

def test_toy_set_taller_than_the_walls_is_an_object_room():
    """A 1 m toy kitchen 0.4 m deep against the back wall of room (2, 1): its upper part
    sticks into the interior above the 0.53 m walls."""
    cx, cy = 2 * CELL, 1 * CELL
    toys = surface(cx - 0.4, cy + 0.51 - 0.4, 0.0, cx + 0.4, cy + 0.51, 1.0)
    pts = np.concatenate([toys, wall((2, 1), (0, 1)), wall((2, 1), (1, 0)),
                          wall((2, 1), (-1, 0))])
    res = classify(pts)
    assert res['cells'][(2, 1)]['kind'] == 'object'
    # The walls alone (0.53 m) never make one.
    res = classify(np.concatenate([wall((2, 1), (0, 1)), wall((2, 1), (1, 0))]))
    assert all(e['kind'] is None for e in res['cells'].values())


def test_pyramid_stairs_between_rails_are_stairs_along_the_rails():
    res = classify(pyramid((3, 2)))
    e = res['cells'][(3, 2)]
    assert e['kind'] == 'stairs' and e['axis'] == 'i'
    assert e['high'] <= FeatureParams().stair_max_high
    assert res['cells'].get((3, 2))['raised'] >= FeatureParams().stair_area
    res = classify(pyramid((3, 2), rails=False))
    assert res['cells'][(3, 2)]['kind'] == 'raised'


def test_terrain_below_the_band_is_nothing():
    """Block rubble on a 10 cm tray, tops <= 0.25 m: below the 0.30 m band."""
    rng = np.random.default_rng(3)
    pts = [surface(x, y, 0.0, x + 0.1, y + 0.1, 0.10 + rng.uniform(0, 0.15))
           for x in np.arange(-0.45, 0.4, 0.1) for y in np.arange(0.57, 1.4, 0.1)]
    res = classify(np.concatenate(pts))
    assert all(e['kind'] is None for e in res['cells'].values())


def test_curtain_edge_is_tall_and_a_gapped_wall_is_not():
    h = CELL / 2
    fabric = surface(h - 0.01, -0.45, 0.15, h + 0.01, 0.45, 0.9)          # between (0,0), (1,0)
    gapped = surface(h - 0.025 + CELL, -h, 0.20, h + 0.025 + CELL, h, 0.53)  # 20 cm gap under
    res = classify(np.concatenate([fabric, gapped]))
    assert res['edges'][('i', 0, 0)]['tall']
    assert not res['edges'].get(('i', 1, 0), {'tall': False})['tall']


def test_body_points_and_forgetting():
    f = CellFeatures(FeatureParams(forget=10.0))
    toys = surface(0.6, 0.6, 0.0, 0.9, 0.9, 1.0)
    f.add(toys, 0.0, (0.75, 0.75), now=0.0)            # all within body_radius: dropped
    assert not f.bins['high']
    f.add(toys, 0.0, (-1.0, -1.0), now=0.0)
    assert f.bins['high']
    f.forget_old(now=11.0)
    assert not f.bins['high']


def test_stair_entry_faces_the_robot_side():
    cells = CellGrid(CELL)
    (ex, ey), hd = stair_entry(cells, (3, 2), 'i', robot_xy=(1.0, 2.04))   # robot at -i
    assert (ex, ey) == pytest.approx((3 * CELL - CELL / 2, 2 * CELL))
    assert hd == pytest.approx(0.0)
    (ex, ey), hd = stair_entry(cells, (3, 2), 'i', robot_xy=(5.0, 2.0))    # robot at +i
    assert ex == pytest.approx(3.5 * CELL) and abs(hd) == pytest.approx(math.pi)
    _, hd = stair_entry(cells, (3, 2), None, robot_xy=(3.06, 0.5))          # from below
    assert hd == pytest.approx(math.pi / 2)


def test_features_payload():
    res = classify(pyramid((3, 2)))
    d = features_to_dict(res, CellGrid(CELL))
    assert d['frame'] == 'map' and d['size'] == CELL
    assert {'cell': [3, 2], 'kind': 'stairs', 'axis': 'i'}.items() <= d['cells'][0].items()


# ------------------------------------------------------------------ curtains

def test_wall_with_a_bottom_gap_is_not_a_curtain():
    """0.53 m wall 1.6 m ahead hanging 10 cm off the floor: the same gap a curtain has in
    the wall band, but nothing above the wall top (CurtainParams.min_tall)."""
    gapped = corridor((1.6, -0.45, 0.10, 1.65, 0.45, 0.53))
    pts = raycast(gapped, origin=(0.0, 0.0, LIDAR_H))
    assert detect_curtains(pts, (0.0, 0.0), 0.0, 0.0) == []
    assert len(detect_curtains(pts, (0.0, 0.0), 0.0, 0.0, CurtainParams(min_tall=0.0))) == 1
    # The real curtain (fabric to 0.9 m) still is one.
    pts = raycast(corridor(curtain_box(x=1.6, gap=0.10, top=0.9)), origin=(0.0, 0.0, LIDAR_H))
    found = detect_curtains(pts, (0.0, 0.0), 0.0, 0.0)
    assert len(found) == 1
    assert tall_cover(pts, found[0].p1, found[0].p2, 0.0) > 0.8


# ------------------------------------------------------------------ stairs: check + AI walk

def test_cell_hits_count_half_and_are_capped():
    est = StairEstimate(0.0, 1.0, centre=0.0, risers=3, rise=0.15, source='lidar', width=0.9)
    chk = StairCheck(est, (0.0, 0.0), 0.0)
    chk.phase = 'HOLD'
    for _ in range(10):
        chk.add_hit('cells')
    assert chk.score == pytest.approx(2.0) and not chk.passed()
    chk.add_hit('camera')
    assert chk.passed()


class Walk:
    walk_turn = CmdVelBridge.walk_turn

    def __init__(self, yaw):
        self.odom_yaw, self.walk_gain, self.walk_max_turn = yaw, 1.0, 0.15


def test_ai_walk_holds_its_heading():
    w = {'yaw0': 0.0}
    assert Walk(0.05).walk_turn(w) == pytest.approx(-0.05)
    assert Walk(-0.5).walk_turn(w) == pytest.approx(0.15)        # clamped
    assert Walk(0.3).walk_turn({'yaw0': None}) == 0.0
    b = Walk(0.3)
    b.walk_gain = 0.0
    assert b.walk_turn(w) == 0.0


def test_ai_walk_default_reaches_the_cell_after_the_pyramid():
    """0.45 m stand-off + one 1.02 m stair cell + half a cell."""
    import inspect
    src = inspect.getsource(CmdVelBridge.__init__)
    assert "p('ai_walk_distance', 2.0)" in src
    assert 0.45 + CELL + CELL / 2 == pytest.approx(2.0, abs=0.03)
