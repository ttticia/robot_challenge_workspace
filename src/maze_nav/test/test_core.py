"""Unit tests for maze_nav.core (no ROS needed): python3 -m pytest -q test"""
import json
import math
import os

import numpy as np
import pytest

from maze_nav.core import (ExitDetector, StandSettle, Grid, body_height, choose_frontier, cloud_xyz,
                           estimate_floor, posture, find_frontiers, footprint_hits, free_distance_ahead,
                           geodesic_distance, heading_towards, max_turn, plan_creep_turn,
                           plan_turn,
                           wall_ranges)

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


def test_robot_body_is_not_a_wall():
    # Legs/hips at wall-band height inside the footprint box (beyond range_min, which is only
    # a circle) are dropped; a real wall just outside the box is kept. Rotated robot too.
    floor = -0.35
    z = floor + 0.33
    for yaw in (0.0, 0.8):
        c, s = math.cos(yaw), math.sin(yaw)
        to_odom = lambda x, y: (c * x - s * y, s * x + c * y, z)  # noqa: E731
        legs = [to_odom(0.19, -0.18), to_odom(-0.35, 0.2), to_odom(0.25, 0.15)]
        wall = [to_odom(0.0, -0.45)]
        body = (0.42, 0.32, 0.28)
        r = wall_ranges(np.array(legs + wall, float), (0, 0), yaw, floor, (0.30, 0.43),
                        -math.pi, INC, N, 0.25, 12.0, body=body)
        hit = r[np.isfinite(r)]
        assert hit.min() == pytest.approx(0.45, abs=1e-3)     # only the wall
        assert ranges(legs, yaw=yaw, floor=floor)[np.isfinite(
            ranges(legs, yaw=yaw, floor=floor))].min() < 0.45  # without the box they show


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


@pytest.mark.parametrize('height, expected', [(0.35, 'standing'), (0.09, 'lying')])
def test_body_height_tells_standing_from_lying(height, expected):
    rng = np.random.default_rng(1)
    # The lidar sees the floor further out when standing (blind zone ~1.2 m ahead).
    near = 1.2 if expected == 'standing' else 0.5
    r, a = rng.uniform(near, 3.0, 6000), rng.uniform(-1.0, 1.0, 6000)
    floor = np.c_[r * np.cos(a), r * np.sin(a), rng.normal(0, 0.005, 6000)]
    wall = np.c_[np.full(800, 0.9), rng.uniform(-1, 1, 800), rng.uniform(0, 0.45, 800)]
    robot_z = height + 0.02
    h = body_height(np.r_[floor, wall], (0, 0), robot_z)
    assert h == pytest.approx(robot_z, abs=0.01)
    assert posture(h) == expected


def test_stand_settle_waits_until_fully_up():
    # Live run 2026-11-15 09:39: lying 0.08 m, floor measured at 0.27 m on the way up.
    s = StandSettle(n=5, tol=0.02)
    rising = [0.08, 0.15, 0.22, 0.27, 0.30, 0.33, 0.345]
    assert not any(s.update(h) for h in rising)
    steady = [s.update(h) for h in (0.35, 0.352, 0.349, 0.351, 0.35)]
    assert steady[-1] and not steady[0]


def test_stand_settle_restarts_when_height_is_lost_or_lying():
    s = StandSettle(n=3, tol=0.02)
    assert not s.update(0.35) and not s.update(0.35)
    assert not s.update(None)                      # no floor seen: start again
    assert not s.update(0.35) and not s.update(0.35)
    assert s.update(0.35)
    assert not s.update(0.10)                      # sat down
    assert not s.update(0.35)


def test_posture_unknown_between_and_without_floor():
    assert posture(None) is None
    assert posture(0.22) is None          # getting up or sitting down
    assert body_height(np.zeros((0, 3)), (0, 0), 0.35) is None


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


def test_frontier_found_when_robot_just_off_map_edge():
    # At startup SLAM maps nothing behind the robot (lidar blind sector), so the map can
    # start a few cm in front of base_link. That must not read as "nothing to explore".
    g = corridor_grid()
    cropped = Grid(g.data[:, 20:], g.resolution, (0.0, -1.0))
    assert not cropped.contains(cropped.cell((-0.07, 0.0)))
    fs = find_frontiers(cropped, (-0.07, 0.0))
    assert len(fs) == 1 and 2.4 < fs[0].goal[0] < 3.0


def test_heading_towards_follows_free_space_not_the_straight_line():
    # L-shaped: start cell at the bottom, corridor goes up then right. The goal is up and to
    # the right, but the straight line to it crosses the wall: head up the corridor instead.
    res = 0.05
    data = np.full((60, 60), 100, np.int16)       # 3 m x 3 m, origin (0, 0)
    data[2:58, 2:14] = 0                          # vertical corridor x 0.1..0.7 (narrow)
    data[40:58, 2:58] = 0                         # horizontal corridor y 2.0..2.9
    g = Grid(data, res, (0.0, 0.0))
    h = heading_towards(g, (0.4, 0.5), (2.5, 2.45))   # straight line: ~44 deg
    assert abs(h - math.pi / 2) < 0.35           # up, not ~45 deg into the wall
    assert heading_towards(g, (0.4, 0.5), (0.4, 1.8)) == pytest.approx(math.pi / 2, abs=0.2)


FOOTPRINT = (0.37, 0.27, 0.23)   # back, front, half width (nav2_params.yaml)


def walled_grid(wall_y=None, wall_x=None):
    """1 cm grid, origin (-2, -2), 4 x 4 m, free; a wall along y = wall_y or x = wall_x."""
    res = 0.01
    data = np.zeros((400, 400), np.int16)
    if wall_y is not None:                       # wall occupies y <= wall_y
        data[:int(round((wall_y + 2) / res)), :] = 100
    if wall_x is not None:                       # wall occupies x >= wall_x
        data[:, int(round((wall_x + 2) / res)):] = 100
    return Grid(data, res, (-2.0, -2.0))


def test_turn_limited_by_side_wall_0_15_m_from_the_body():
    # Robot side 0.15 m from a wall on its right: centre 0.38 m from it. Turning left swings
    # the rear-right corner into the wall after ~20 deg; turning right, the front-right
    # corner gets there after ~32 deg (3 cm margin).
    g = walled_grid(wall_y=-0.38)
    assert not footprint_hits(g, (0, 0), 0.0, FOOTPRINT, 0.03)
    left = math.degrees(max_turn(g, (0, 0), 0.0, FOOTPRINT, +1, math.radians(45), 0.03))
    right = math.degrees(max_turn(g, (0, 0), 0.0, FOOTPRINT, -1, math.radians(45), 0.03))
    assert 14 <= left <= 24, left
    assert 28 <= right <= 36 and right > left, right
    # centred in a 0.9 m corridor-half (wall 0.45 m away): the full 45 deg each way? Rear
    # corner reaches 0.436 m, so with a 3 cm margin it can't; without margin it can.
    g2 = walled_grid(wall_y=-0.45)
    assert max_turn(g2, (0, 0), 0.0, FOOTPRINT, +1, math.radians(45)) == pytest.approx(
        math.radians(45))


def test_creep_stops_short_of_a_wall_0_15_m_ahead():
    g = walled_grid(wall_x=0.42)                 # front edge (0.27) 0.15 m from the wall
    d = free_distance_ahead(g, (0, 0), 0.0, FOOTPRINT, 0.10, margin=0.05)
    assert 0.08 <= d <= 0.10, d
    assert free_distance_ahead(walled_grid(), (0, 0), 0.0, FOOTPRINT, 0.10, 0.05) == 0.10


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


def test_frontier_narrower_than_min_width_is_skipped():
    # Corridor open at the far end except for a 0.25 m slit: a 0.37 m minimum (80 % of the
    # 0.46 m robot) drops it; the same corridor fully open keeps its frontier.
    res = 0.05
    data = np.full((20, 80), 100, np.int16)            # 4 m x 1 m
    data[1:19, 1:70] = 0                                # free corridor, x 0.05..3.5
    data[1:19, 70:] = -1                                # unknown beyond
    data[1:19, 69] = 100                                # wall across the end ...
    data[8:13, 69] = 0                                  # ... with a 0.25 m slit
    g = Grid(data, res, (0.0, 0.0))
    assert find_frontiers(g, (0.5, 0.5), min_size=1, min_width=0.37) == []
    data[1:19, 69] = 0                                  # end fully open (0.9 m)
    fs = find_frontiers(Grid(data, res, (0.0, 0.0)), (0.5, 0.5), min_size=1, min_width=0.37)
    assert len(fs) == 1 and fs[0].width > 0.8


def test_frontier_seen_through_a_slit_is_unreachable_and_skipped():
    # Wall across the corridor with a 0.15 m slit; the lidar saw free floor beyond it, so the
    # frontier is out there (wide), but no passable path leads to it.
    res = 0.05
    data = np.full((20, 80), 100, np.int16)
    data[1:19, 1:79] = 0
    data[1:19, 60:] = -1                                # unknown far end ...
    data[1:19, 40:60] = 0                               # ... after free floor seen beyond
    data[1:19, 39] = 100                                # the wall across x 1.95..2.0
    data[9:12, 39] = 0                                  # 0.15 m slit
    g = Grid(data, res, (0.0, 0.0))
    assert find_frontiers(g, (0.5, 0.5), min_size=1) == []
    data[1:19, 39] = 0                                  # wall gone: reachable again
    assert len(find_frontiers(Grid(data, res, (0.0, 0.0)), (0.5, 0.5), min_size=1)) == 1


def test_voxel_thinning_keeps_nearest_point_per_cell():
    # Dense wall 1 m ahead (x = 1.00..1.04, every 1 cm along y) plus a farther point in the
    # same 10 cm cell as the nearest wall point: thinning must keep the wall at 1.0 m.
    ys = np.arange(-0.5, 0.5, 0.01)
    wall = [(1.0 + 0.04 * (i % 2), y, 0.35) for i, y in enumerate(ys)]
    pts = np.array(wall + [(1.08, 0.0, 0.35)], float)
    full = wall_ranges(pts, (0.0, 0.0), 0.0, 0.0, (0.30, 0.43), -math.pi, INC, N, 0.25, 12.0)
    thin = wall_ranges(pts, (0.0, 0.0), 0.0, 0.0, (0.30, 0.43), -math.pi, INC, N, 0.25, 12.0,
                       voxel=0.10)
    ahead = N // 2                                   # beam at bearing 0
    assert thin[ahead] == pytest.approx(full[ahead], abs=0.05)
    assert thin[ahead] < 1.06
    # One return per 10 cm of wall: the 1 m wall keeps ~10 hit beams (of ~53 at 1 deg).
    assert 9 <= np.isfinite(thin).sum() <= 12


def start_blind_sector_grid():
    """Map saved by the live run of 2026-11-15 09:50 (start cell, two start-scan turns cut)."""
    from PIL import Image
    a = np.array(Image.open(os.path.join(HERE, 'start_blind_sector_map.pgm')))[::-1]
    data = np.where(a == 0, 100, np.where(a == 254, 0, -1)).astype(np.int16)
    return Grid(data, 0.05, (-0.030106, -2.816108))


@pytest.mark.parametrize('xy', [(0.1, 0.0), (0.2, 0.05), (0.1, -0.1)])
def test_blacklisted_start_pose_keeps_the_rest_of_its_frontier(xy):
    # The unmapped patch behind the start pose and the real way on (the edge of what the lidar
    # saw, up to ~1 m ahead) form one frontier. Blacklisting the creep start (0, 0) used to
    # drop all of it, and the run ended as NO_EXIT right after the start scan.
    g = start_blind_sector_grid()
    fs = find_frontiers(g, xy, 0.25, 0.33, 5, blacklist=[(0.0, 0.0)], min_width=0.8 * 0.46)
    assert fs, 'no frontier left: explorer would stop with NO_EXIT'
    for f in fs:
        assert math.hypot(*f.goal) >= 0.4                  # never back to the start pose


# ------------------------------------------------------------------ plan_turn

def test_plan_turn_leaves_clear_and_small_turns_to_nav2():
    g = walled_grid()                                      # no walls
    assert plan_turn(g, (0, 0), 0.0, math.radians(90), FOOTPRINT, 0.03) == (None, 'short way clear')
    assert plan_turn(g, (0, 0), 0.0, math.radians(20), FOOTPRINT, 0.03) == (None, 'small turn')


def test_plan_turn_blocked_both_ways_next_to_a_side_wall():
    g = walled_grid(wall_y=-0.38)        # body 0.15 m from a wall on the right
    angle, why = plan_turn(g, (0, 0), 0.0, math.radians(90), FOOTPRINT, 0.03)
    assert (angle, why) == (None, 'blocked both ways')


def test_plan_turn_goes_the_long_way_round_a_post():
    # A post 0.42 m from the centre at -102 deg: only the rear corners reach it, at yaw ~46
    # and ~110 deg, both inside a 150 deg left turn; the 210 deg right turn misses it.
    g = walled_grid()
    px, py = 0.42 * math.cos(math.radians(-102)), 0.42 * math.sin(math.radians(-102))
    c = g.cell((px, py))
    g.data[c[1] - 1:c[1] + 2, c[0] - 1:c[0] + 2] = 100
    angle, why = plan_turn(g, (0, 0), 0.0, math.radians(150), FOOTPRINT, 0.03)
    assert why == 'short way blocked, long way clear'
    assert angle == pytest.approx(-math.radians(210))


def test_plan_turn_ignores_a_wall_mapped_on_the_robot():
    g = walled_grid()
    g.data[200, 200] = 100                                 # (0, 0): under the robot
    assert plan_turn(g, (0, 0), 0.0, math.radians(90), FOOTPRINT, 0.03) == \
        (None, 'map shows a wall at the robot')


# ------------------------------------------------------------------ plan_creep_turn

def corner_grid(width=0.90, dead_end=True):
    """1 cm grid, origin (-2, -2): a corridor of `width` along +x (centre line y = 0) into a
    corner cell centred at (1, 0), whose way on is a side corridor up +y. Not dead_end: the
    main corridor carries on past the corner (a T)."""
    res, n = 0.01, 400
    data = np.full((n, n), 100, np.int16)

    def free(x0, x1, y0, y1):
        data[int(round((y0 + 2) / res)):int(round((y1 + 2) / res)),
             int(round((x0 + 2) / res)):int(round((x1 + 2) / res))] = 0
    h = width / 2
    free(-2.0, 1.0 + h if dead_end else 2.0, -h, h)
    free(1.0 - h, 1.0 + h, h, 2.0)
    return Grid(data, res, (-2.0, -2.0))


def test_creep_turn_finds_room_in_the_corner_cell():
    # Stopped at the cell edge before the corner (as in the sim): the 90 deg turn up the side
    # corridor is blocked both ways here; a bit into the corner cell it fits.
    g = corner_grid()
    goal = (1.0, 1.6)
    assert plan_turn(g, (0.45, 0.0), 0.0, math.pi / 2, FOOTPRINT, 0.03) == \
        (None, 'blocked both ways')
    d, margin, why = plan_creep_turn(g, (0.45, 0.0), 0.0, goal, FOOTPRINT)
    assert d is not None, why
    assert 0.08 <= d <= 0.45, d                 # into the corner cell, short of its end wall
    assert margin == 0.01                       # 0.9 m corridors: only fits with <= 1 cm
    # and the turn really is clear there, with that margin
    p = (0.45 + d, 0.0)
    assert max_turn(g, p, 0.0, FOOTPRINT, +1, math.pi / 2, margin) >= math.pi / 2 - 1e-6 or \
        max_turn(g, p, 0.0, FOOTPRINT, -1, 1.5 * math.pi, margin) >= 1.5 * math.pi - 1e-6


def test_creep_turn_uses_the_full_margin_when_it_fits():
    d, margin, _ = plan_creep_turn(corner_grid(width=0.95), (0.45, 0.0), 0.0, (1.0, 1.6),
                                   FOOTPRINT)
    assert d is not None and margin == 0.03


def test_creep_turn_never_drives_past_the_free_distance():
    # Dead end right ahead: the corner cell's end wall limits the creep.
    g = corner_grid()
    d, _, _ = plan_creep_turn(g, (0.45, 0.0), 0.0, (1.0, 1.6), FOOTPRINT, max_creep=2.0)
    front_at_stop = 0.45 + d + FOOTPRINT[1]
    assert front_at_stop <= 1.45 - 0.05 + 0.02, front_at_stop   # creep_margin to the end wall


def test_creep_turn_none_in_a_plain_corridor():
    # No side opening anywhere ahead: the goal is behind, every spot is blocked both ways.
    g = corner_grid(dead_end=False)
    d, margin, why = plan_creep_turn(g, (-1.0, 0.0), 0.0, (-1.8, 0.0), FOOTPRINT)
    assert (d, margin) == (None, None) and why.startswith('no clear spot'), why
