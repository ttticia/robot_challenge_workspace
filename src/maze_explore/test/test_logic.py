import math

import numpy as np

from maze_explore.grid import Grid, fit_grid, nb
from maze_explore.plan import next_job
from maze_explore.sense import HeightMap, Occupancy, Sense

SIZE = 1.02


def paint(data, res, ox, oy, x0, y0, x1, y1, value=100):
    n = int(max(abs(x1 - x0), abs(y1 - y0)) / (res / 2)) + 1
    for t in np.linspace(0, 1, n):
        c, r = int((x0 + t * (x1 - x0) - ox) / res), int((y0 + t * (y1 - y0) - oy) / res)
        data[r, c] = value


def world(walls, curtain=None):
    """Free 6 x 6 m map with wall segments ((cell, d), ...) on a grid at the origin."""
    res, ox, oy = 0.05, -3.0, -3.0
    data = np.zeros((120, 120), np.int8)
    g = Grid(0.0, 0.0, 0.0, SIZE)
    hm = HeightMap()
    for cell, d in walls:
        x, y = g.box(cell, d, [-0.5, 0.5], [0.0])
        paint(data, res, ox, oy, x[0, 0], y[0, 0], x[1, 0], y[1, 0])
        if curtain == (cell, d):
            xs, ys = np.linspace(x[0, 0], x[1, 0], 60), np.linspace(y[0, 0], y[1, 0], 60)
            for _ in range(3):
                hm.add(xs, ys, np.full(60, 0.8))
    return g, Sense(g, Occupancy(data, res, ox, oy), hm)


def test_fit_grid_finds_an_angled_off_centre_start():
    true = Grid(0.2, -0.25, math.radians(17), SIZE)       # where the maze really is
    pts = []
    for cell, d in [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((1, 0), 1), ((1, 0), 3), ((2, 0), 0)]:
        x, y = true.box(cell, d, np.linspace(-0.5, 0.5, 21), [0.0])
        pts.append(np.column_stack([x.ravel(), y.ravel()]))
    pts = np.concatenate(pts)
    g, q = fit_grid(Grid(0.0, 0.0, 0.0, SIZE), pts[:, 0], pts[:, 1], math.radians(30), 0.40)
    assert q > 0.9
    assert abs(math.degrees(g.yaw - true.yaw)) < 1.0
    assert math.hypot(g.x0 - true.x0, g.y0 - true.y0) < 0.04


def test_kinds_wall_free_room_curtain():
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2),                 # start cell, open ahead
             ((1, 0), 3), ((1, 0), 0),                              # cell 1: wall right, CURTAIN ahead
             ((1, 1), 1), ((1, 1), 0), ((1, 1), 2)]                 # room to the left of cell 1
    g, s = world(walls, curtain=((1, 0), 0))
    assert [s.kind((0, 0), d, may_exit=False) for d in range(4)] == ['free', 'wall', 'wall', 'wall']
    assert s.kind((1, 0), 0) == 'curtain'
    assert s.kind((1, 0), 1) == 'room'
    assert s.kind((1, 0), 3) == 'wall'


def test_stairs_and_blocked():
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((1, 0), 1), ((1, 0), 3)]
    g, s = world(walls)
    x, y = g.box((0, 0), 0, np.linspace(-0.45, 0.45, 19), [0.40, 0.45])     # top riser
    for xi, yi in zip(x.ravel(), y.ravel()):
        paint(s.occ.data, 0.05, -3.0, -3.0, xi, yi, xi, yi)
    assert s.kind((0, 0), 0, may_exit=False) == 'blocked'           # no height information
    for depth, h in ((0.1, 0.15), (0.25, 0.30), (0.45, 0.45), (0.55, 0.45)):
        x, y = g.box((0, 0), 0, np.linspace(-0.4, 0.4, 30), [depth])
        s.hm.add(x.ravel(), y.ravel(), np.full(30, h))
    assert s.kind((0, 0), 0, may_exit=False) == 'stairs'


def test_stairs_starting_at_the_border_are_not_a_wall():
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((0, 0), 0)]
    g, s = world(walls)
    x, y = g.box((0, 0), 0, np.linspace(-0.4, 0.4, 30), [0.0])
    s.hm.add(x.ravel(), y.ravel(), np.full(30, 0.53))
    assert s.kind((0, 0), 0) == 'wall'                               # full-height wall
    g, s = world(walls)
    for depth, h in ((0.0, 0.30), (0.1, 0.30), (0.25, 0.30), (0.45, 0.45), (0.55, 0.45)):
        x, y = g.box((0, 0), 0, np.linspace(-0.4, 0.4, 30), [depth])
        s.hm.add(x.ravel(), y.ravel(), np.full(30, h))
    assert s.kind((0, 0), 0) == 'stairs'


def test_exit_is_an_opening_with_nothing_beyond():
    g, s = world([((0, 0), 1), ((0, 0), 3), ((0, 0), 2)])
    assert s.kind((0, 0), 0) == 'exit'


def test_nearest_first_then_curtain_then_exit():
    a, b, c = (0, 0), (1, 0), (1, 1)
    kinds = {(a, 0): 'free', (a, 1): 'wall', (a, 2): 'wall', (a, 3): 'wall',
             (b, 0): 'curtain', (b, 1): 'free', (b, 2): 'free', (b, 3): 'room'}
    args = dict(closed=set(), links={}, exit_cell=None)
    j = next_job(kinds, {a}, cur=a, facing=0, looked=set(), greeted=set(), **args)
    assert (j['job'], j['cell'], j['d']) == ('visit', a, 0)
    j = next_job(kinds, {a, b}, cur=b, facing=0, looked=set(), greeted=set(), **args)
    assert j['job'] == 'greet'                                       # face the curtain first
    j = next_job(kinds, {a, b}, cur=b, facing=0, looked=set(), greeted={(b, 0)}, **args)
    assert j['job'] in ('visit', 'look') and j['cell'] == b          # then the nearest thing
    kinds.update({(c, 0): 'wall', (c, 1): 'wall', (c, 2): 'wall', (c, 3): 'free'})
    done = dict(looked={(b, 3)}, greeted={(b, 0)})
    j = next_job(kinds, {a, b, c}, cur=c, facing=1, **done, **args)
    assert (j['job'], j['cell'], j['d'], len(j['path'])) == ('curtain', b, 0, 1)   # only now
    links = {(b, 0): ('curtain', nb(b, 0)), (nb(b, 0), 2): ('curtain', b)}
    j = next_job(kinds, {a, b, c}, closed={a}, links=links, exit_cell=b, cur=c, facing=1, **done)
    assert j['job'] == 'exit' and len(j['path']) == 1


def test_start_cell_is_not_re_entered():
    a, b = (0, 0), (1, 0)
    kinds = {(b, 0): 'wall', (b, 1): 'wall', (b, 2): 'wall', (b, 3): 'wall'}   # explorer: closed = wall
    j = next_job(kinds, {a, b}, closed={a}, links={}, cur=b, facing=0, looked=set(),
                 greeted=set(), exit_cell=None)
    assert j['job'] == 'done'


def test_cell_with_a_tall_object_is_never_walked_into():
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((1, 0), 1), ((1, 0), 3)]
    g, s = world(walls)
    assert s.kind((0, 0), 0, may_exit=False) == 'free'
    cx, cy = g.centre((1, 0))
    xs, ys = np.meshgrid(np.arange(0.875, 1.2, 0.05), np.arange(-0.125, 0.15, 0.05))   # square centres
    for _ in range(2):                                   # seen twice: not enough yet
        s.hm.add(xs.ravel(), ys.ravel(), np.full(xs.size, 0.55))
    assert s.kind((0, 0), 0, may_exit=False) == 'free'
    s.hm.add(xs.ravel(), ys.ravel(), np.full(xs.size, 0.55))
    assert s.kind((0, 0), 0, may_exit=False) == 'object'
    j = next_job({((0, 0), 0): 'object'}, {(0, 0)}, closed=set(), links={}, cur=(0, 0), facing=0,
                 looked=set(), greeted=set(), exit_cell=None, enter_rooms=True)
    assert j['job'] == 'look'                            # looked at, even with enter_rooms
    j = next_job({((0, 0), 0): 'object'}, {(0, 0)}, closed=set(), links={}, cur=(0, 0), facing=0,
                 looked={((0, 0), 0)}, greeted=set(), exit_cell=None, enter_rooms=True)
    assert j['job'] == 'done'                            # and never visited


def test_terrain_and_walls_are_not_objects():
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((1, 0), 1), ((1, 0), 3)]
    g, s = world(walls)
    cx, cy = g.centre((1, 0))
    xs, ys = np.meshgrid(cx + np.arange(-0.45, 0.46, 0.05), cy + np.arange(-0.45, 0.46, 0.05))
    for _ in range(5):                                   # rubble over the whole cell, 0.25 m
        s.hm.add(xs.ravel(), ys.ravel(), np.full(xs.size, 0.25))
    for d in (1, 3):                                     # its side walls, 0.53 m, 6 cm inside
        x, y = g.box((1, 0), d, np.linspace(-0.5, 0.5, 40), [-0.06, 0.0])
        for _ in range(5):
            s.hm.add(x.ravel(), y.ravel(), np.full(x.size, 0.53))
    assert s.kind((0, 0), 0, may_exit=False) == 'free'
