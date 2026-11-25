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


def test_uneven_cell_is_rough_and_flat_floor_is_not():
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((1, 0), 1), ((1, 0), 3)]
    g, s = world(walls)
    cx, cy = g.centre((1, 0))
    xs, ys = np.meshgrid(cx + np.arange(-0.45, 0.46, 0.05), cy + np.arange(-0.45, 0.46, 0.05))
    s.hm.add(xs.ravel(), ys.ravel(), np.full(xs.size, 0.02))          # floor with height noise
    for d in (1, 3):                                     # side walls, smeared 6 cm inwards
        x, y = g.box((1, 0), d, np.linspace(-0.5, 0.5, 40), [-0.06, 0.0])
        s.hm.add(x.ravel(), y.ravel(), np.full(x.size, 0.53))
    assert not s.rough((0, 0), 0)
    tx, ty = np.meshgrid(cx + np.arange(-0.2, 0.21, 0.05), cy + np.arange(-0.2, 0.21, 0.05))
    s.hm.add(tx.ravel(), ty.ravel(), np.full(tx.size, 0.10))          # a 10 cm tile
    assert s.rough((0, 0), 0)
    assert s.kind((0, 0), 0, may_exit=False) == 'free'                # still walked into
    assert not s.rough((0, 0), 1)                                     # nothing seen there


def test_camera_sighting_is_placed_on_the_first_closed_edge():
    from maze_explore.curtain_node import bearing, centred, decode, edge_along, watching
    # a corridor along +x: cells (0,0) (1,0) (2,0), closed at the far end of (2,0)
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((1, 0), 1), ((1, 0), 3),
             ((2, 0), 1), ((2, 0), 3), ((2, 0), 0)]
    g, s = world(walls)
    assert edge_along(g, s, (0.0, 0.0, 0.0), 0.0, max_range=3.0) == ((2, 0), 0)   # straight ahead
    assert edge_along(g, s, (1.02, 0.0, 0.0), 0.0) == ((2, 0), 0)                # from the cell before
    assert edge_along(g, s, (0.0, 0.0, 0.0), np.pi / 2) == ((0, 0), 1)           # to the left
    assert edge_along(g, s, (0.0, 0.0, 0.0), 0.0, max_range=1.0) is None         # too far to tell
    assert bearing(0.5) == 0.0 and bearing(0.0) > 0.55 and bearing(1.0) < -0.55   # + = left
    assert centred({'conf': 0.8, 'box': [0.2, 0.1, 0.7, 0.9]}, 0.5)
    assert not centred({'conf': 0.8, 'box': [0.6, 0.1, 0.9, 0.9]}, 0.5)           # off to one side
    assert not centred({'conf': 0.3, 'box': [0.2, 0.1, 0.7, 0.9]}, 0.5)
    assert watching('RUN cell=(1, 0) job=[visit cell (1, 0) side 0 via 0 cells] act=drive ')
    assert not watching('RUN cell=(1, 0) job=[look cell (1, 0) side 1 via 0 cells] act=hold ')
    assert not watching('READY cell=None job=[] act=- send /nav/start')
    # two overlapping boxes and a weak one -> the strong one, back in frame pixels
    pred = np.array([[160, 162, 40], [128, 128, 30], [100, 104, 20], [80, 84, 20],
                     [0.9, 0.6, 0.2]], np.float32)
    out = decode(pred, 0.5, 0.5, 0, 8, (480, 640, 3))
    assert len(out) == 1 and abs(out[0][1] - 0.9) < 1e-6
    assert np.allclose(out[0][0], (220, 160, 420, 320), atol=1)


def test_curtain_node_rests_while_the_explorer_turns():
    from maze_explore.curtain_node import watching
    assert not watching('RUN cell=(1, 0) job=[visit cell (1, 0) side 0 via 0 cells] act=turn ')
    assert watching('RUN cell=(1, 0) job=[visit cell (1, 0) side 0 via 0 cells] act=cv_verify ')
    # inside a look only for the explorer's own curtain check before the views
    assert watching('RUN cell=(1, 0) job=[look cell (1, 0) side 1 via 0 cells] act=cv_verify ')
    assert not watching('RUN cell=(1, 0) job=[look cell (1, 0) side 1 via 0 cells] act=stair_check ')


def test_uneven_terrain_from_dense_depth_points():
    walls = [((0, 0), 1), ((0, 0), 3), ((0, 0), 2), ((1, 0), 1), ((1, 0), 3)]
    g, s = world(walls)
    rng = np.random.default_rng(1)
    cx, cy = g.centre((1, 0))
    x, y = np.meshgrid(cx + np.arange(-0.45, 0.45, 0.01), cy + np.arange(-0.45, 0.45, 0.01))
    x, y = x.ravel(), y.ravel()
    floor = rng.normal(0.0, 0.01, x.size)                  # flat floor, 1 cm depth noise
    floor[::200] = 0.09                                    # and a few stray points
    rough, seen = s.rough_from_points((0, 0), 0, x, y, floor)
    assert rough is False and seen >= 100
    tile = floor.copy()
    tile[(np.abs(x - cx) < 0.2) & (np.abs(y - cy) < 0.2)] += 0.10   # a 10 cm tile in the middle
    assert s.rough_from_points((0, 0), 0, x, y, tile)[0] is True
    few = slice(0, 300)                                    # only a strip seen: no verdict
    assert s.rough_from_points((0, 0), 0, x[few], y[few], tile[few])[0] is None
    assert s.floor_seen((0, 0), 0) == 0                    # the lidar has seen none of it
