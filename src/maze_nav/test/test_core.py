"""Unit tests for maze_nav.core (no ROS needed): python3 -m pytest -q test"""
import json
import math
import os

import numpy as np
import pytest

from maze_nav.core import (ExitDetector, Grid, choose_frontier, cloud_xyz, estimate_floor,
                           find_frontiers, geodesic_distance, wall_ranges)

HERE = os.path.dirname(os.path.abspath(__file__))
INC = math.radians(1.0)
N = 360


def ranges(points, xy=(0, 0), yaw=0.0, floor=-0.35, band=(0.30, 0.43)):
    return wall_ranges(np.array(points, float), xy, yaw, floor, band, -math.pi, INC, N, 0.25, 12.0)


def beam(angle):
    return int((angle + math.pi) / INC) % N


# ------------------------------------------------------------------ wall scan

def test_wall_in_band_is_seen_terrain_and_stairs_are_not():
    floor = -0.35
    pts = [(1.0, 0.0, floor + 0.35),           # wall, 0.35 m above floor
           (0.0, 1.5, floor + 0.15),           # ramp top
           (0.0, -1.0, floor + 0.26),          # top stair tread
           (-1.0, 0.0, floor + 0.02)]          # floor
    r = ranges(pts, floor=floor)
    assert r[beam(0.0)] == pytest.approx(1.0, abs=1e-3)
    assert np.isinf(r[beam(math.pi / 2)]) and np.isinf(r[beam(-math.pi / 2)])
    assert np.isinf(r[beam(math.pi - 0.01)])


def test_band_is_floor_referenced_not_robot_referenced():
    # Robot body 0.15 m higher (on a ramp) does not move the band: the band uses floor_z only.
    pts = [(1.0, 0.0, -0.35 + 0.40)]
    assert ranges(pts)[beam(0.0)] == pytest.approx(1.0, abs=1e-3)


def test_nearest_hit_wins_and_yaw_is_applied():
    pts = [(2.0, 0.0, 0.0), (1.2, 0.0, 0.0), (5.0, 5.0, 0.0)]
    r = ranges(pts, xy=(0.2, 0.0), yaw=math.pi / 2)
    assert r[beam(-math.pi / 2)] == pytest.approx(1.0, abs=1e-3)   # wall is to the right


def test_floor_estimate():
    rng = np.random.default_rng(0)
    xy = rng.uniform(-1.5, 1.5, (5000, 2))
    floor = np.c_[xy, -0.33 + rng.normal(0, 0.005, len(xy))]
    wall = np.c_[np.full(500, 0.9), rng.uniform(-1, 1, 500), rng.uniform(-0.33, 0.12, 500)]
    z = estimate_floor(np.r_[floor, wall], (0, 0), 0.0)
    assert z == pytest.approx(-0.33, abs=0.01)
    assert estimate_floor(wall[:50], (0, 0), 0.0) is None
    # A denser terrain mat 0.12 m above the floor in the next cell must not win.
    mat = np.c_[rng.uniform(1.2, 2.2, (12000, 2)), -0.21 + rng.normal(0, 0.005, 12000)]
    assert estimate_floor(np.r_[floor, wall, mat], (0, 0), 0.0) == pytest.approx(-0.33, abs=0.01)


def test_cloud_xyz_reads_livox_mixed_field_types():
    """Livox layout: x y z intensity float32, tag line uint8, timestamp float64 (step 26)."""
    from types import SimpleNamespace as F
    fields = [F(name='x', offset=0, datatype=7), F(name='y', offset=4, datatype=7),
              F(name='z', offset=8, datatype=7), F(name='intensity', offset=12, datatype=7),
              F(name='tag', offset=16, datatype=2), F(name='line', offset=17, datatype=2),
              F(name='timestamp', offset=18, datatype=8)]
    dt = np.dtype({'names': ['x', 'y', 'z', 'i', 'tag', 'line', 't'],
                   'formats': ['<f4'] * 4 + ['u1', 'u1', '<f8'],
                   'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})
    a = np.zeros(3, dt)
    a['x'], a['y'], a['z'], a['t'] = [1, 2, np.nan], [3, 4, 5], [6, 7, 8], 1e9
    xyz = cloud_xyz(a.tobytes(), fields, 26, 26 * 3, 3, 1)
    assert xyz.tolist() == [[1, 3, 6], [2, 4, 7]]             # NaN point dropped
    with pytest.raises(ValueError):
        cloud_xyz(a.tobytes(), fields[1:], 26, 78, 3, 1)


# ------------------------------------------------------------------ grid helpers

def corridor_grid():
    """5 m x 1 m corridor along +x, walls on both sides, unknown beyond x = 3 m."""
    res = 0.05
    data = np.full((40, 120), -1, np.int16)       # 2 m x 6 m, origin (-1, -1)
    data[10:30, 20:80] = 0                        # free: x -0..3, y -0.5..0.5
    data[9, 20:80] = 100
    data[30, 20:80] = 100
    data[9:31, 19] = 100                          # closed behind the robot
    return Grid(data, res, (-1.0, -1.0))


def test_geodesic_distance_goes_around_walls():
    p = np.ones((5, 5), bool)
    p[1:5, 2] = False
    d = geodesic_distance(p, (0, 4))
    assert np.isinf(d[4, 2])
    assert d[4, 4] == pytest.approx(4 + 4 * math.sqrt(2), abs=1e-6)   # over the top


def test_frontier_at_end_of_corridor():
    fs = find_frontiers(corridor_grid(), (0.3, 0.0))
    assert len(fs) == 1
    f = fs[0]
    assert 2.4 < f.goal[0] < 3.0 and abs(f.goal[1]) < 0.1
    assert abs(f.yaw) < 0.5
    assert f.path_length == pytest.approx(f.goal[0] - 0.3, abs=0.15)


def test_blacklisted_frontier_is_skipped():
    g = corridor_grid()
    goal = find_frontiers(g, (0.3, 0.0))[0].goal
    assert find_frontiers(g, (0.3, 0.0), blacklist=[goal]) == []


def test_choose_frontier_keeps_current_unless_clearly_worse():
    from maze_nav.core import Frontier
    a = Frontier((1, 0), 0, 2.0, 10, (1, 0))
    b = Frontier((0, 1), 0, 1.8, 10, (0, 1))
    assert choose_frontier([a, b]) is b
    assert choose_frontier([a, b], current=a) is a           # 0.2 m better: keep a
    c = Frontier((0, 2), 0, 0.5, 10, (0, 2))
    assert choose_frontier([a, c], current=a) is c           # much closer: switch


def test_exit_detector_needs_enclosed_then_open():
    g = corridor_grid()
    det = ExitDetector(dwell=1.0)
    assert not det.update(g, (0.3, 0.0), 0.0)
    assert det.seen_enclosed
    big = Grid(np.zeros((100, 100), np.int16), 0.05, (-2.5, -2.5))   # 5 x 5 m open room
    assert not det.update(big, (0.0, 0.0), 0.5)               # moved < min_displacement
    det.start = (-3.0, 0.0)
    assert not det.update(big, (0.0, 0.0), 1.0)
    assert det.update(big, (0.0, 0.0), 2.1)


# ------------------------------------------------------------------ example maze, offline

def maze_truth(res=0.05):
    """Ground-truth wall grid of the example maze inside an 10 x 8 m room (maze frame)."""
    solids = json.load(open(os.path.join(HERE, 'example_maze.json')))['solids']
    x0, y0, w, h = -7.0, -3.5, 10.0, 8.0
    occ = np.zeros((int(h / res), int(w / res)), bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True

    def box(lo, hi):
        c0, c1 = int((lo[0] - x0) / res), int(math.ceil((hi[0] - x0) / res))
        r0, r1 = int((lo[1] - y0) / res), int(math.ceil((hi[1] - y0) / res))
        occ[r0:r1, c0:c1] = True
    for s in solids:
        if s['kind'] in ('wall', 'rail'):
            box(s['min'], s['max'])
    return occ, res, (x0, y0)


def raycast(occ, res, origin, xy, known, max_range=4.0):
    """Mark what a 360 deg 2D scanner at xy sees (free along rays, walls at hits)."""
    for a in np.linspace(-math.pi, math.pi, 720, endpoint=False):
        for r in np.arange(0.0, max_range, res / 2):
            c = int((xy[0] + r * math.cos(a) - origin[0]) / res)
            rr = int((xy[1] + r * math.sin(a) - origin[1]) / res)
            if not (0 <= rr < occ.shape[0] and 0 <= c < occ.shape[1]):
                break
            if occ[rr, c]:
                known[rr, c] = 100
                break
            known[rr, c] = 0


def test_explorer_finds_the_exit_of_the_example_maze():
    occ, res, origin = maze_truth()
    known = np.full(occ.shape, -1, np.int16)
    xy, yaw = (0.0, 0.0), math.pi / 2            # start cell, facing its opening
    det = ExitDetector(dwell=0.0)
    current, visited = None, [xy]
    for step in range(30):
        raycast(occ, res, origin, xy, known)
        grid = Grid(known.copy(), res, origin)
        if det.update(grid, xy, float(step)):
            break
        fs = find_frontiers(grid, xy, blacklist=visited[1:])   # explorer blacklists done goals
        assert fs, f'ran out of frontiers at {xy} after {step} goals'
        current = choose_frontier(fs, current, yaw, xy)
        yaw = math.atan2(current.goal[1] - xy[1], current.goal[0] - xy[0])
        xy = current.goal                          # teleport: Nav2 is tested in simulation
        visited.append(xy)
    assert det.found, f'no exit; visited {visited}'
    # Out means past the stair block (x < -3.475 in the maze frame) into the room.
    assert xy[0] < -3.4, visited
    assert len(visited) <= 15, visited
