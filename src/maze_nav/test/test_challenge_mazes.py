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
ARENAS = ['challenge_a', 'challenge_a_mirror', 'challenge_b', 'challenge_b_offset']
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
def test_every_cell_reachable_and_the_only_way_out_is_the_launch_cell(name):
    d = load(name)
    g = d['grid']
    assert (g['cols'], g['rows'], g['cell_size']) == (5, 3, SIZE)
    nbrs, outs = graph(d)
    assert set(route(nbrs, (0, 0), c)[0] for c in nbrs) == {(0, 0)}   # all reachable
    assert all(route(nbrs, (0, 0), c)[-1] == c for c in nbrs)
    assert outs == [(tuple(g['launch_cell']), tuple(g['exit_cell']))]


@pytest.mark.parametrize('name', ARENAS)
def test_start_cell_opens_only_ahead_and_the_arena_lies_on_its_side(name):
    d = load(name)
    nbrs, _ = graph(d)
    assert nbrs[(0, 0)] == [(1, 0)]
    sign = 1 if d['grid']['side'] == 'left' else -1
    assert all(sign * s['min'][1] > -SIZE / 2 - 0.1 and sign * s['max'][1] > -SIZE / 2 - 0.1
               for s in d['solids'])


@pytest.mark.parametrize('name', ARENAS)
def test_four_terrain_cells_low_enough_to_walk(name):
    """Booklet: 4 uneven-terrain cells. Nothing over 12 cm (wall_scan's band starts 0.30 m
    up, and stair_detector needs >= 2 risers, so a single block is terrain, not stairs)."""
    d = load(name)
    cells = [tuple(c) for c in d['features']['terrain']]
    assert len(cells) == 4
    for c in cells:
        here = [s for s in d['solids'] if s['kind'] in ('block', 'mat', 'wedge')
                and in_cell(s, c)]
        assert here, c
        assert max(s['max'][2] for s in here) <= 0.12


@pytest.mark.parametrize('name', ARENAS)
def test_stairs_in_a_straight_cell_on_the_route(name):
    d = load(name)
    nbrs, _ = graph(d)
    st = tuple(d['features']['stairs'])
    assert sorted(nbrs[st]) == sorted([(st[0] - 1, st[1]), (st[0] + 1, st[1])])
    assert st in route(nbrs, (0, 0), tuple(d['grid']['launch_cell']))
    steps = [s for s in d['solids'] if s['kind'] == 'steps' and in_cell(s, st)]
    assert len(steps) == 2 and all(s['treads'] >= 2 for s in steps)


@pytest.mark.parametrize('name', ARENAS)
def test_curtain_hangs_in_a_passage_mounted_about_a_metre_up(name):
    d = load(name)
    nbrs, _ = graph(d)
    a, b = (tuple(c) for c in d['features']['curtain'])
    assert b in nbrs[a]                                  # no wall behind the curtain
    cur = [s for s in d['solids'] if s['kind'] == 'curtain']
    assert len(cur) == 1
    assert cur[0]['min'][2] < 0.15 and cur[0]['max'][2] >= 0.9   # gap < 15 cm, ~1 m mount


@pytest.mark.parametrize('name', ARENAS)
def test_object_prop_stands_free_of_the_walls(name):
    """scene_cells.cell_objects only finds a prop >= object_border (0.12 m) off the walls."""
    d = load(name)
    cell = d['features']['rooms']['object']
    props = [s for s in walls_of(d) if in_cell(s, cell, margin=-0.12)]
    assert len(props) == 1


@pytest.mark.parametrize('name', ARENAS)
def test_sim_out_line_is_past_the_exit_cell_margin(name):
    """When the sim logs OUT OF THE MAZE, the explorer's grid calls it an exit cell."""
    d = load(name)
    g, e, (sx, sy, syaw) = d['grid'], d['exit'], d['start']
    cells = CellGrid(g['cell_size'], (sx, sy), syaw)
    ex, ey = g['exit_cell']
    x, y = ex * SIZE, ey * SIZE
    if e['axis'] == 'x':
        x = e.get('above', e.get('below'))
    else:
        y = e.get('above', e.get('below'))
    j0 = 0 if g['side'] == 'left' else 1 - g['rows']
    assert exit_cell(cells, (x, y), g['cols'], g['rows'], 0.15, j0) is not None


def test_mirror_is_the_arena_reflected():
    a, m = load('challenge_a'), load('challenge_a_mirror')
    assert len(a['solids']) == len(m['solids'])
    for s, t in zip(a['solids'], m['solids']):
        assert t['min'][1] == pytest.approx(-s['max'][1])
        assert t['max'][1] == pytest.approx(-s['min'][1])
    assert m['grid']['launch_cell'] == [4, -2] and m['grid']['side'] == 'right'


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


@pytest.mark.parametrize('name,pose', [('challenge_a', (0.0, 0.0, 0.0)),
                                       ('challenge_a', (2.04, 2.04, 0.0)),     # at the stairs
                                       ('challenge_b', (0.0, 2.04, -math.pi / 2)),  # curtain
                                       ('challenge_a_mirror', (1.02, -1.02, 0.0))])
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


def test_depth_sees_the_wall_ahead_at_the_right_range():
    """challenge_a, robot in (4, 0) facing the outer wall: the camera is 0.25 m ahead of
    base_link and the wall face 0.51 - 0.025 m ahead, so the optical axis (20 deg down)
    meets it at 0.23 m / cos 20 deg = ~0.25 m depth."""
    m = Maze(os.path.join(HERE, 'challenge_a.json'))
    origin, r_oc, _ = camera(m, 4.08, 0.0, 0.0)
    depth = render_depth(boxes_of(m), origin, r_oc, rays_640(), K)
    ahead = SIZE * 4.5 - 0.025 - origin[0]
    assert depth[240, 320] == pytest.approx(ahead / math.cos(math.radians(20)), abs=0.005)


@pytest.mark.parametrize('name', ['challenge_a', 'challenge_b'])
def test_stair_detector_finds_the_arena_stairs_in_the_sim_depth(name):
    """0.9 m before the first riser, facing it (detected from 0.6 to 1.2 m in both arenas):
    stairs.detect_stairs (what stair_detector runs) finds them in the sim's depth image, so
    a sim run exercises the stair handling."""
    d = load(name)
    st = d['features']['stairs']
    m = Maze(os.path.join(HERE, name + '.json'))
    x, y = st[0] * SIZE - 0.45 - 0.9, st[1] * SIZE
    origin, r_oc, r_ob = camera(m, x, y, 0.0)
    depth = render_depth(boxes_of(m), origin, r_oc, rays_640(), K)
    depth = np.where(np.isfinite(depth), np.round(depth * 1000), 0).astype(np.uint16)
    r, t, _ = level_transform(r_ob, R_BASE_CAM, T_BASE_CAM)
    assert detect_stairs(depth, K, r, t) is not None


def test_steps_become_one_box_per_tread():
    m = Maze(os.path.join(HERE, 'challenge_a.json'))
    for steps in [s for s in m.solids if s['kind'] == 'steps']:
        boxes = [b for b in m.depth_boxes()
                 if b[0][1] >= steps['min'][1] - 1e-9 and b[1][1] <= steps['max'][1] + 1e-9
                 and b[0][0] >= steps['min'][0] - 1e-9 and b[1][0] <= steps['max'][0] + 1e-9
                 and b[0][2] == steps['min'][2]]
        assert len(boxes) == int(steps['treads'])
