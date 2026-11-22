"""The challenge-shaped sim mazes (make_challenge_mazes.py) and the sim's depth camera.

python3 -m pytest -q test/test_challenge_mazes.py   (ROS sourced: imports fake_maze_lite3)"""
import json
import math
import os

import numpy as np
import pytest

import make_challenge_mazes as gen
from fake_maze_lite3 import Maze, render_depth
from maze_nav.scene_cells import CellGrid, exit_cell
from stair_scene import K, R_BASE_CAM, T_BASE_CAM, raycast

HERE = os.path.dirname(os.path.abspath(__file__))
CHALLENGE = ['challenge_exit_far', 'challenge_exit_left', 'challenge_exit_right',
             'challenge_exit_near', 'challenge_offset']
DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def load(name):
    with open(os.path.join(HERE, name + '.json')) as f:
        return json.load(f)


def side_open(walls, cell, d, size):
    """No wall box covers the middle of cell's side d."""
    x = (cell[0] + d[0] / 2) * size
    y = (cell[1] + d[1] / 2) * size
    return not any(s['min'][0] <= x <= s['max'][0] and s['min'][1] <= y <= s['max'][1]
                   for s in walls)


def test_files_match_the_generator():
    """Edit make_challenge_mazes.py, then rerun it; never the JSON by hand."""
    want = {f'challenge_exit_{n}': gen.maze(n) for n in gen.EXITS}
    want['challenge_offset'] = gen.maze('far', (0.05, -0.04, math.radians(5.0)))
    for name, m in want.items():
        assert load(name) == json.loads(json.dumps(m)), name


@pytest.mark.parametrize('name', CHALLENGE)
def test_every_cell_reachable_and_one_way_out(name):
    d = load(name)
    g = d['grid']
    assert (g['cols'], g['rows'], g['cell_size']) == (5, 3, 1.02)
    walls = [s for s in d['solids'] if s['kind'] == 'wall']
    seen, todo, outs = {(0, 0)}, [(0, 0)], []
    while todo:
        c = todo.pop()
        for dd in DIRS:
            n = (c[0] + dd[0], c[1] + dd[1])
            if not side_open(walls, c, dd, g['cell_size']):
                continue
            if 0 <= n[0] < g['cols'] and 0 <= n[1] < g['rows']:
                if n not in seen:
                    seen.add(n)
                    todo.append(n)
            else:
                outs.append((c, n))
    assert len(seen) == g['cols'] * g['rows']
    assert outs == [(tuple(g['launch_cell']), tuple(g['exit_cell']))]


@pytest.mark.parametrize('name', CHALLENGE)
def test_start_cell_opens_only_ahead(name):
    """Like the real start cell: closed but for the side it faces (+x)."""
    d = load(name)
    walls = [s for s in d['solids'] if s['kind'] == 'wall']
    opens = [dd for dd in DIRS if side_open(walls, (0, 0), dd, 1.02)]
    assert opens == [(1, 0)]


@pytest.mark.parametrize('name', CHALLENGE)
def test_sim_out_line_is_past_the_exit_cell_margin(name):
    """When the sim logs OUT OF THE MAZE, the explorer's grid already calls it an exit cell
    (start pose = explorer grid origin, as in the explorer)."""
    d = load(name)
    g, e, (sx, sy, syaw) = d['grid'], d['exit'], d['start']
    cells = CellGrid(g['cell_size'], (sx, sy), syaw)
    ex, ey = g['exit_cell']
    # a point on the out line, level with the exit cell's centre
    x, y = ex * 1.02, ey * 1.02
    if e['axis'] == 'x':
        x = e.get('above', e.get('below'))
    else:
        y = e.get('above', e.get('below'))
    assert exit_cell(cells, (x, y), g['cols'], g['rows'], margin=0.15) == tuple(g['exit_cell'])


@pytest.mark.parametrize('name,cols,rows', [('maze_curtains', 2, 3), ('maze_stairs', 2, 3)])
def test_presentation_mazes_carry_their_grid(name, cols, rows):
    g = load(name)['grid']
    assert (g['cols'], g['rows'], g['cell_size']) == (cols, rows, 1.0)


# ------------------------------------------------------------------ depth camera

def rays_640():
    fx, fy, cx, cy = K
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1)


def camera(maze, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    r_ob = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.array([x, y, maze.height(x, y) + 0.35]) + r_ob @ T_BASE_CAM, r_ob @ R_BASE_CAM


@pytest.mark.parametrize('name,pose', [('challenge_exit_far', (4.08, 0.0, 0.0)),
                                       ('example_maze', None),
                                       ('maze_stairs', (0.0, 0.6, -math.pi / 2)),
                                       ('maze_curtains', None)])
def test_depth_matches_brute_force_raycast(name, pose):
    """The culled renderer gives the same image as stair_scene.raycast on every box."""
    m = Maze(os.path.join(HERE, name + '.json'))
    boxes = [(np.asarray(a, float), np.asarray(b, float)) for a, b in m.depth_boxes()]
    origin, r_oc = camera(m, *(pose or m.start))
    rays = rays_640()
    fast = render_depth(boxes, origin, r_oc, rays, K)
    ref = raycast([(tuple(a), tuple(b)) for a, b in boxes], origin,
                  rays.reshape(-1, 3) @ r_oc.T, 0.0, max_range=50.0).reshape(fast.shape)
    near = ref < 3.5
    assert np.isfinite(fast[near]).all()
    assert np.abs(fast[near] - ref[near]).max() < 1e-3


def test_depth_sees_the_wall_ahead_at_the_right_range():
    """Robot in (4, 0) facing the far wall: the camera is 0.25 m ahead of base_link and the
    wall face 0.51 - 0.025 m ahead, so the optical axis (20 deg down) meets it at
    0.23 m / cos 20 deg = ~0.25 m depth."""
    m = Maze(os.path.join(HERE, 'challenge_exit_far.json'))
    boxes = [(np.asarray(a, float), np.asarray(b, float)) for a, b in m.depth_boxes()]
    origin, r_oc = camera(m, 4.08, 0.0, 0.0)
    depth = render_depth(boxes, origin, r_oc, rays_640(), K)
    ahead = 1.02 * 4.5 - 0.025 - origin[0]
    assert depth[240, 320] == pytest.approx(ahead / math.cos(math.radians(20)), abs=0.005)


def test_steps_become_one_box_per_tread():
    m = Maze(os.path.join(HERE, 'maze_stairs.json'))
    steps = [s for s in m.solids if s['kind'] == 'steps'][0]
    boxes = [b for b in m.depth_boxes()
             if b[0][1] >= steps['min'][1] - 1e-9 and b[1][1] <= steps['max'][1] + 1e-9
             and b[0][0] >= steps['min'][0] - 1e-9 and b[1][0] <= steps['max'][0] + 1e-9
             and b[0][2] == steps['min'][2]]
    assert len(boxes) == int(steps.get('treads', 2))
