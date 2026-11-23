"""The booklet-shaped sim arenas (make_challenge_mazes.py) and the sim's depth camera.

python3 -m pytest -q test/test_challenge_mazes.py   (ROS sourced: imports fake_maze_lite3)"""
import json
import math
import os
from collections import deque

import numpy as np
import pytest

import make_challenge_mazes as gen
from fake_maze_lite3 import Maze, render_depth
from maze_nav.scene_cells import CellGrid, exit_cell
from maze_nav.stairs import detect_stairs, level_transform
from stair_scene import K, R_BASE_CAM, T_BASE_CAM, raycast

HERE = os.path.dirname(os.path.abspath(__file__))
ARENAS = ['challenge_a', 'challenge_a_mirror', 'challenge_b', 'challenge_b_offset',
          'challenge_a_no_curtain', 'challenge_a_flat']
DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))
SIZE = 1.02


def load(name):
    with open(os.path.join(HERE, name + '.json')) as f:
        return json.load(f)


def j_range(g):
    return range(0, g['rows']) if g['side'] == 'left' else range(1 - g['rows'], 1)


def walls_of(d):
    return [s for s in d['solids'] if s['kind'] == 'wall']


def side_open(walls, cell, d):
    """No wall box covers the middle of cell's side d (a curtain is not a wall)."""
    x = (cell[0] + d[0] / 2) * SIZE
    y = (cell[1] + d[1] / 2) * SIZE
    return not any(s['min'][0] <= x <= s['max'][0] and s['min'][1] <= y <= s['max'][1]
                   for s in walls)


def graph(d):
    """Cell -> open neighbours (inside the grid), and the open outer sides."""
    g = d['grid']
    cells = [(i, j) for i in range(g['cols']) for j in j_range(g)]
    walls = walls_of(d)
    nbrs, outs = {c: [] for c in cells}, []
    for c in cells:
        for dd in DIRS:
            if not side_open(walls, c, dd):
                continue
            n = (c[0] + dd[0], c[1] + dd[1])
            (nbrs[c] if n in nbrs else outs).append(n if n in nbrs else (c, n))
    return nbrs, outs


def route(nbrs, a, b):
    prev, todo = {a: None}, deque([a])
    while todo:
        c = todo.popleft()
        for n in nbrs[c]:
            if n not in prev:
                prev[n] = c
                todo.append(n)
    path, c = [], b
    while c is not None:
        path.append(c)
        c = prev.get(c)
    return path[::-1]


def in_cell(s, cell, margin=0.0):
    cx, cy = cell[0] * SIZE, cell[1] * SIZE
    return (s['min'][0] >= cx - SIZE / 2 - margin and s['max'][0] <= cx + SIZE / 2 + margin
            and s['min'][1] >= cy - SIZE / 2 - margin and s['max'][1] <= cy + SIZE / 2 + margin)


# ------------------------------------------------------------------ layout

def test_files_match_the_generator():
    """Edit make_challenge_mazes.py, then rerun it; never the JSON by hand."""
    for name, m in gen.mazes().items():
        assert load(name[:-5]) == json.loads(json.dumps(m)), name


@pytest.mark.parametrize('name', ARENAS)
def test_arena_layout(name):
    """Shaped like the booklet's arena; what each feature needs to be useful in a sim run."""
    d = load(name)
    g = d['grid']
    f = d['features']
    assert (g['cols'], g['rows'], g['cell_size']) == (5, 3, SIZE)
    nbrs, outs = graph(d)
    launch = tuple(g['launch_cell'])
    # every cell reachable from the start; the only way out is from the launch cell
    assert all(route(nbrs, (0, 0), c)[0] == (0, 0) for c in nbrs)
    assert outs == [(launch, tuple(g['exit_cell']))]
    # start cell closed but for the side ahead; the arena on its side of the start
    assert nbrs[(0, 0)] == [(1, 0)]
    sign = 1 if g['side'] == 'left' else -1
    assert all(sign * s['min'][1] > -SIZE / 2 - 0.1 for s in d['solids'])
    # 4 uneven-terrain cells: <= 12 cm (wall_scan's band starts 0.30 m up; stair_detector
    # needs >= 2 risers, so a single block is terrain), the rolling pin 18 cm
    assert len(f['terrain']) == 4
    for c in f['terrain']:
        here = [s for s in d['solids'] if s['kind'] in ('block', 'mat', 'wedge', 'pin')
                and in_cell(s, c)]
        assert here and all(s['max'][2] <= (0.18 if s['kind'] == 'pin' else 0.12)
                            for s in here), c
    # stairs: a straight cell (along x) on the route, two flights of >= 2 risers
    steps = [s for s in d['solids'] if s['kind'] == 'steps']
    if 'stairs' in f:
        st = tuple(f['stairs'])
        assert sorted(nbrs[st]) == [(st[0] - 1, st[1]), (st[0] + 1, st[1])]
        assert st in route(nbrs, (0, 0), launch)
        assert [s['treads'] >= 2 for s in steps] == [True, True]
    else:
        assert not steps                                 # the _flat variant
    # curtain: in a passage, < 15 cm gap, mounted ~1 m up
    cur = [s for s in d['solids'] if s['kind'] == 'curtain']
    if 'curtain' in f:
        a, b = (tuple(c) for c in f['curtain'])
        assert b in nbrs[a] and len(cur) == 1
        assert cur[0]['min'][2] < 0.15 and cur[0]['max'][2] >= 0.9
    else:
        assert not cur                                   # _no_curtain / _flat
    # the object prop >= object_border (0.12 m) off the walls, so cell_objects finds it
    assert len([s for s in walls_of(d) if in_cell(s, f['rooms']['object'], -0.12)]) == 1
    # rolling pin (challenge_a): 18 cm, thin, wall to wall across a straight cell
    if 'rolling_pin' in f:
        cell = tuple(f['rolling_pin'])
        pins = [s for s in d['solids'] if s['kind'] == 'pin']
        assert sorted(nbrs[cell]) == [(cell[0] - 1, cell[1]), (cell[0] + 1, cell[1])]
        assert len(pins) == 1 and in_cell(pins[0], cell)
        lo, hi = pins[0]['min'], pins[0]['max']
        cy = cell[1] * SIZE
        assert hi[2] == pytest.approx(0.18) and hi[0] - lo[0] < 0.1
        assert lo[1] <= cy - SIZE / 2 + 0.03 and hi[1] >= cy + SIZE / 2 - 0.03
    # the sim's "out of the maze" line is past what the explorer calls the grid
    e, (sx, sy, syaw) = d['exit'], d['start']
    x, y = g['exit_cell'][0] * SIZE, g['exit_cell'][1] * SIZE
    if e['axis'] == 'x':
        x = e.get('above', e.get('below'))
    else:
        y = e.get('above', e.get('below'))
    j0 = 0 if g['side'] == 'left' else 1 - g['rows']
    assert exit_cell(CellGrid(SIZE, (sx, sy), syaw), (x, y), 5, 3, 0.15, j0) is not None


def test_mirror_is_the_arena_reflected():
    a, m = load('challenge_a'), load('challenge_a_mirror')
    assert len(a['solids']) == len(m['solids'])
    for s, t in zip(a['solids'], m['solids']):
        assert t['min'][1] == pytest.approx(-s['max'][1])
        assert t['max'][1] == pytest.approx(-s['min'][1])
    assert m['grid']['launch_cell'] == [4, -2] and m['grid']['side'] == 'right'


def test_sim_steps_over_the_rolling_pin():
    """fake_maze_lite3: the pin is seen (lidar points, depth box) but the body neither rides
    on it (height 0 over it) nor collides with it (not a wall box)."""
    m = Maze(os.path.join(HERE, 'challenge_a.json'))
    x, y = 2 * SIZE, 0.0
    assert m.height(x, y) == 0.0
    wb = m.wall_boxes()
    assert not ((wb[:, 0] <= x) & (wb[:, 2] >= x) & (wb[:, 1] <= y) & (wb[:, 3] >= y)).any()
    assert any(lo[0] <= x <= hi[0] and hi[2] == pytest.approx(0.18)
               for lo, hi in m.depth_boxes())
    pts, occ = m.points()
    on = (np.abs(pts[:, 0] - x) < 0.04) & (np.abs(pts[:, 1]) < 0.4) & (pts[:, 2] > 0.1)
    assert on.sum() > 50 and occ[on].all()



class _PinSim:
    """fake_maze_lite3's step() pin model on a stand-in node (no ROS)."""
    def __init__(self, maze, x, y, th, stall=1.0):
        from types import SimpleNamespace
        import fake_maze_lite3 as F
        for n in ('pin_hold', 'straddling'):
            setattr(self, n, getattr(F.FakeMazeLite3, n).__get__(self))
        self.logs = []
        log = SimpleNamespace(info=self.logs.append, warn=self.logs.append)
        self.get_logger = lambda: log
        self.pin_stall, self.pin_feet, self.dt = stall, 0.22, 0.05
        self.pins = [(s['min'][0], s['min'][1], s['max'][0], s['max'][1])
                     for s in maze.solids if s['kind'] == 'pin']
        self.pin_over, self.pin_drive = set(), 0.0
        self.x, self.y, self.th = x, y, th

    def drive(self, v, w=0.0, t=1.0):
        for _ in range(int(round(t / self.dt))):
            th = self.th + w * self.dt
            nx, ny = self.x + v * math.cos(th) * self.dt, self.y + v * math.sin(th) * self.dt
            if not self.pin_hold(nx, ny, th, v, w):
                self.x, self.y = nx, ny
            self.th = th


def r_pins(m):
    return [(s['min'][0], s['min'][1]) for s in m.solids if s['kind'] == 'pin']


def test_sim_pin_needs_steady_forward_drive():
    """pin_stall > 0 (off by default; user 2026-11-23: H-Step needs 0.5-2 s of steady forward drive): the front
    feet are held at the bar until 1 s of steady drive, a stop restarts the count, then the
    rear feet are held the same way; with steady drive the robot gets over."""
    m = Maze(os.path.join(HERE, 'challenge_a.json'))
    px = 2 * SIZE
    bar_x0 = min(p[0] for p in r_pins(m))
    front = bar_x0 - 0.03 - 0.22 - 0.001          # front feet just short of the bar
    r = _PinSim(m, front, 0.0, 0.0)
    r.drive(0.10, t=0.6)
    assert r.x == pytest.approx(front + 0.005, abs=0.006), 'front feet held at the bar'
    front = r.x
    r.drive(0.0, t=0.2)                         # Nav2 stops: the count starts again
    r.drive(0.10, t=0.6)
    assert r.x == pytest.approx(front, abs=0.01), 'still held: 0.6 s since the stop'
    r.drive(0.10, t=0.5)
    assert r.x > front + 0.005, 'front feet over after 1 s of steady drive'
    r.drive(0.10, t=8.0)
    assert r.x - 0.22 > px + 0.03, f'rear feet over too ({r.x:.2f})'
    assert any('rear feet over' in str(l) for l in r.logs), r.logs


def test_sim_pin_turning_against_the_bar_never_gets_over():
    m = Maze(os.path.join(HERE, 'challenge_a.json'))
    px = 2 * SIZE
    r = _PinSim(m, px - 0.5, 0.0, 0.0)
    r.drive(0.10, t=3.0)
    x = r.x
    r.drive(0.10, w=0.3, t=3.0)                 # forward while turning: never over
    assert (0, True) not in r.pin_over and r.x + 0.22 * math.cos(r.th) < px, (r.x, r.th)
    assert x < px


def test_sim_pin_stall_off_steps_straight_over():
    m = Maze(os.path.join(HERE, 'challenge_a.json'))
    px = 2 * SIZE
    r = _PinSim(m, px - 0.5, 0.0, 0.0, stall=0.0)
    r.drive(0.10, t=10.0)
    assert r.x == pytest.approx(px + 0.5, abs=0.01)


# ------------------------------------------------------------------ depth camera

def rays_640():
    fx, fy, cx, cy = K
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1)


def camera(maze, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    r_ob = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return (np.array([x, y, maze.height(x, y) + 0.35]) + r_ob @ T_BASE_CAM,
            r_ob @ R_BASE_CAM, r_ob)


def boxes_of(m):
    return [(np.asarray(a, float), np.asarray(b, float)) for a, b in m.depth_boxes()]


@pytest.mark.parametrize('name,pose', [('challenge_a', (2.04, 2.04, 0.0)),     # at the stairs
                                       ('challenge_b', (0.0, 2.04, -math.pi / 2))])  # curtain
def test_depth_matches_brute_force_raycast(name, pose):
    """The culled renderer gives the same image as stair_scene.raycast on every box."""
    m = Maze(os.path.join(HERE, name + '.json'))
    boxes = boxes_of(m)
    origin, r_oc, _ = camera(m, *pose)
    rays = rays_640()
    fast = render_depth(boxes, origin, r_oc, rays, K)
    ref = raycast([(tuple(a), tuple(b)) for a, b in boxes], origin,
                  rays.reshape(-1, 3) @ r_oc.T, 0.0, max_range=50.0).reshape(fast.shape)
    # render_depth skips boxes > 3.5 m away (straight-line), so compare by distance, not
    # depth along the axis (shorter at the image's edges)
    near = ref * np.linalg.norm(rays, axis=-1) < 3.4
    assert np.isfinite(fast[near]).all()
    assert np.abs(fast[near] - ref[near]).max() < 1e-3


def test_sim_depth_shows_walls_and_stairs():
    m = Maze(os.path.join(HERE, 'challenge_a.json'))
    # (4, 0) facing the outer wall: camera 0.25 m ahead of base_link, wall face 0.485 m
    # ahead, optical axis 20 deg down -> 0.23 m / cos 20 deg depth at the image centre
    origin, r_oc, _ = camera(m, 4.08, 0.0, 0.0)
    depth = render_depth(boxes_of(m), origin, r_oc, rays_640(), K)
    ahead = SIZE * 4.5 - 0.025 - origin[0]
    assert depth[240, 320] == pytest.approx(ahead / math.cos(math.radians(20)), abs=0.005)
    # each flight of steps is one depth box per tread
    for steps in [s for s in m.solids if s['kind'] == 'steps']:
        assert len([b for b in m.depth_boxes()
                    if steps['min'][0] - 1e-9 <= b[0][0] and b[1][0] <= steps['max'][0] + 1e-9
                    and steps['min'][1] - 1e-9 <= b[0][1] and b[1][1] <= steps['max'][1] + 1e-9
                    and b[0][2] == steps['min'][2]]) == steps['treads']
    # 0.9 m before the first riser, facing it (found from 0.6 to 1.2 m), stairs.detect_stairs
    # (what stair_detector runs) finds the stairs of both arenas in the sim's depth image
    for name in ('challenge_a', 'challenge_b'):
        st = load(name)['features']['stairs']
        mz = Maze(os.path.join(HERE, name + '.json'))
        origin, r_oc, r_ob = camera(mz, st[0] * SIZE - 0.45 - 0.9, st[1] * SIZE, 0.0)
        depth = render_depth(boxes_of(mz), origin, r_oc, rays_640(), K)
        depth = np.where(np.isfinite(depth), np.round(depth * 1000), 0).astype(np.uint16)
        r, t, _ = level_transform(r_ob, R_BASE_CAM, T_BASE_CAM)
        assert detect_stairs(depth, K, r, t) is not None, name
