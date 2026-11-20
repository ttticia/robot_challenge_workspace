"""Unit tests for maze_nav.scene_understanding.scene_cells (no ROS needed): python3 -m pytest -q test"""
import math

import numpy as np
import pytest

from maze_nav.core import Grid
from maze_nav.scene_understanding.scene_cells import (CellGrid, DeadEndDetector, JunctionDetector,
                                                      NewCellTrigger, opening, run_active,
                                                      side_heading, side_walls)

RES = 0.05


def open_grid(x0=-1.5, y0=-1.5, size=3.0, value=0):
    """A square occupancy grid covering [x0, x0 + size] x [y0, y0 + size], all `value`."""
    n = int(round(size / RES))
    return Grid(np.full((n, n), value, dtype=np.int16), RES, (x0, y0))


def draw_wall_x(grid, x, y_from, y_to):
    """Occupied cells along the line x = const (a wall across the +i side of a cell)."""
    for y in np.arange(y_from, y_to, RES / 2):
        c = grid.cell((x, y))
        if grid.contains(c):
            grid.data[c[1], c[0]] = 100


# ------------------------------------------------------------------ CellGrid

def test_cell_indices_and_round_trip():
    cells = CellGrid()
    assert cells.cell((0.0, 0.0)) == (0, 0)
    assert cells.cell((0.49, -0.49)) == (0, 0)
    assert cells.cell((0.51, 0.0)) == (1, 0)
    assert cells.cell((-0.51, 1.2)) == (-1, 1)
    assert cells.centre((2, -1)) == pytest.approx((2.0, -1.0))
    assert cells.depth_inside((0.3, 0.0), (0, 0)) == pytest.approx(0.2)
    assert cells.depth_inside((0.6, 0.0), (0, 0)) == pytest.approx(-0.1)


def test_rotated_offset_grid():
    cells = CellGrid(size=1.0, origin=(0.1, -0.2), yaw=math.radians(90))
    # +i points along map +y now.
    assert cells.cell((0.1, 0.8)) == (1, 0)
    assert cells.cell((-0.9, -0.2)) == (0, 1)
    x, y = cells.centre((1, 0))
    assert (x, y) == pytest.approx((0.1, 0.8))
    assert cells.heading((1, 0)) == pytest.approx(math.radians(90))


# ------------------------------------------------------------------ opening

def test_opening_open_wall_unknown():
    cells = CellGrid()
    g = open_grid()
    assert opening(g, cells, (0, 0), (1, 0)) == 'open'
    draw_wall_x(g, 0.5, -0.5, 0.5)
    assert opening(g, cells, (0, 0), (1, 0)) == 'wall'
    assert opening(g, cells, (0, 0), (0, 1)) == 'open'
    assert opening(open_grid(value=-1), cells, (0, 0), (1, 0)) == 'unknown'
    # Off the map entirely.
    assert opening(g, cells, (5, 5), (1, 0)) == 'unknown'


def test_corner_posts_alone_are_not_a_wall():
    cells = CellGrid()
    g = open_grid()
    draw_wall_x(g, 0.5, -0.5, -0.42)     # stubs at the two ends of the edge
    draw_wall_x(g, 0.5, 0.42, 0.5)
    assert opening(g, cells, (0, 0), (1, 0)) == 'open'


# ------------------------------------------------------------------ NewCellTrigger

def run(trigger, poses):
    """Feed (x, y, yaw_deg) poses; return the events (None filtered out)."""
    out = []
    for x, y, yaw in poses:
        e = trigger.update((x, y), math.radians(yaw))
        if e is not None:
            out.append(e)
    return out


def line(x0, y0, x1, y1, yaw, n=50):
    return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, yaw) for k in range(n + 1)]


def test_start_cell_fires_once():
    tr = NewCellTrigger(CellGrid())
    events = run(tr, [(0.0, 0.0, 0.0)] * 10)
    assert len(events) == 1
    e = events[0]
    assert (e.cell, e.from_cell, e.direction) == ((0, 0), None, '')
    assert run(NewCellTrigger(CellGrid(), trigger_at_start=False), [(0, 0, 0)] * 5) == []


def test_fires_on_entering_not_before():
    tr = NewCellTrigger(CellGrid(), enter_margin=0.15)
    run(tr, [(0.0, 0.0, 0.0)])
    # Facing (1, 0) and walking up to and over the edge: not entered until 0.65 m.
    assert run(tr, line(0.0, 0.0, 0.64, 0.0, 0.0)) == []
    events = run(tr, [(0.66, 0.0, 0.0)])
    assert [(e.cell, e.from_cell, e.direction) for e in events] == [((1, 0), (0, 0), '+i')]
    assert events[0].yaw == pytest.approx(0.0)


def test_straddling_an_edge_does_not_flip_or_refire():
    tr = NewCellTrigger(CellGrid())
    run(tr, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])          # start, then (1, 0)
    wobble = [(0.5 + 0.08 * (-1) ** k, 0.0, 180.0) for k in range(20)]
    assert run(tr, wobble) == []
    assert tr.current == (1, 0)


def test_corridor_corner_and_return():
    tr = NewCellTrigger(CellGrid())
    path = (line(0, 0, 2, 0, 0) + line(2, 0, 2, 2, 90)    # along +i, turn left, along +j
            + line(2, 2, 2, 0, -90) + line(2, 0, 0, 0, 180))   # and all the way back
    events = run(tr, path)
    assert [e.cell for e in events] == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
    assert [e.direction for e in events] == ['', '+i', '+i', '+j', '+j']
    assert tr.visited == {(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)}


def test_jump_has_no_direction():
    tr = NewCellTrigger(CellGrid())
    run(tr, [(0.0, 0.0, 0.0)])
    e = run(tr, [(2.0, 1.0, 0.0)])[0]            # relocalised two cells away
    assert (e.cell, e.from_cell, e.direction) == ((2, 1), (0, 0), '')


def test_rotated_grid_directions():
    tr = NewCellTrigger(CellGrid(yaw=math.radians(90)))      # +i along map +y
    events = run(tr, line(0, 0, 0, 1, 90) + line(0, 1, -1, 1, 180))
    assert [(e.cell, e.direction) for e in events] == [((0, 0), ''), ((1, 0), '+i'),
                                                       ((1, 1), '+j')]


# ------------------------------------------------------------------ run_active

def test_run_active_follows_explorer_status():
    for status in ('CREEPING: 10 cm straight ahead, no steering', 'SCANNING: turning',
                   'EXPLORING: go', 'EXPLORING', 'TURNING: x', 'LOOKING: QR room',
                   'FACING_QR: unreadable code', 'JUNCTION: cell [1, 0]',
                   'DEAD_END: dead end at cell [2, 0]'):
        assert run_active(status), status
    for status in ('', None, 'STARTING: waiting for /map', 'PAUSED: ready; holding still',
                   'EXITED: open space', 'NO_EXIT: all frontiers mapped', 'STOPPED: Ctrl+C',
                   'EXPLORINGX'):
        assert not run_active(status), status
    assert run_active('PAUSED: x', active_states=('PAUSED',))


# ------------------------------------------------------------------ junctions and dead ends

def draw_wall_y(grid, y, x_from, x_to):
    """Occupied cells along the line y = const (a wall across a +j / -j side)."""
    for x in np.arange(x_from, x_to, RES / 2):
        c = grid.cell((x, y))
        if grid.contains(c):
            grid.data[c[1], c[0]] = 100


def maze_grid():
    """Free floor over cells i = -1..3, j = -1..1 with the walls drawn by the tests."""
    return Grid(np.zeros((int(3.0 / RES), int(5.0 / RES)), dtype=np.int16), RES, (-1.5, -1.5))


def openings_on(grid, cells):
    return lambda cell, d: opening(grid, cells, cell, d)


def test_junction_t_from_the_corridor():
    # Corridor (0,0) -> (1,0); (1,0) is a T: walls ahead (+i), open to +j and -j.
    cells = CellGrid()
    g = maze_grid()
    draw_wall_x(g, 1.5, -0.5, 0.5)
    draw_wall_y(g, 0.5, -0.5, 0.5)           # (0,0) sides closed: a corridor
    draw_wall_y(g, -0.5, -0.5, 0.5)
    jd = JunctionDetector(cells, centre_radius=0.25)
    op = openings_on(g, cells)
    assert jd.update((1.3, 0.0), (1, 0), (0, 0), op) is None     # 0.3 m from centre: not yet
    assert jd.update((1.2, 0.0), (1, 0), (0, 0), op) == ['+j', '-j']
    assert jd.update((1.0, 0.0), (1, 0), (0, 0), op) is None     # once per cell
    # The corridor cell itself: one way on (+i) -> not a junction.
    assert JunctionDetector(cells).update((0.0, 0.0), (0, 0), (-1, 0), op) is None
    # No way in known (start / jump): skipped.
    assert JunctionDetector(cells).update((1.0, 0.0), (1, 0), None, op) is None


def test_unknown_sides_are_not_ways_on():
    cells = CellGrid()
    g = maze_grid()
    g.data[:, :] = -1
    jd = JunctionDetector(cells)
    assert jd.update((1.0, 0.0), (1, 0), (0, 0), openings_on(g, cells)) is None


def dead_end_grid():
    """(0,0) -> (1,0) -> (2,0): (2,0) closed on +i, +j, -j; (1,0) open to +j too."""
    g = maze_grid()
    draw_wall_x(g, 2.5, -0.5, 0.5)
    draw_wall_y(g, 0.5, 1.5, 2.5)
    draw_wall_y(g, -0.5, 1.5, 2.5)
    return g


def test_dead_end_fires_from_the_cell_before_while_facing_it():
    cells = CellGrid()
    g = dead_end_grid()
    op = openings_on(g, cells)
    de = DeadEndDetector(cells, heading_tol=math.radians(25))
    assert de.update((1.0, 0.0), math.radians(90), (1, 0), op) is None    # facing +j: open
    assert de.update((1.0, 0.0), math.radians(40), (1, 0), op) is None    # not facing anything
    hit = de.update((1.0, 0.0), math.radians(-20), (1, 0), op)
    assert hit is not None
    cell, d, sides = hit
    assert (cell, d) == ((2, 0), '+i')
    assert sides == {'+i': 'wall', '+j': 'wall', '-i': 'open', '-j': 'wall'}
    assert de.update((1.0, 0.0), 0.0, (1, 0), op) is None                 # once per cell


def test_dead_end_not_from_inside_nor_visited_nor_half_seen():
    cells = CellGrid()
    g = dead_end_grid()
    op = openings_on(g, cells)
    # base_link already over the edge in the dead end (cell hysteresis still says (1, 0)).
    assert DeadEndDetector(cells).update((1.55, 0.0), 0.0, (1, 0), op) is None
    assert DeadEndDetector(cells).update((1.0, 0.0), 0.0, (1, 0), op, skip={(2, 0)}) is None
    # Far side unknown: not a dead end yet.
    g2 = dead_end_grid()
    for y in np.arange(-0.5, 0.5, RES / 2):
        for x in np.arange(2.4, 2.6, RES / 2):
            c = g2.cell((x, y))
            g2.data[c[1], c[0]] = -1
    assert DeadEndDetector(cells).update((1.0, 0.0), 0.0, (1, 0), openings_on(g2, cells)) is None
    # The side between robot cell and dead end walled: nothing to face.
    g3 = dead_end_grid()
    draw_wall_x(g3, 1.5, -0.5, 0.5)
    assert DeadEndDetector(cells).update((1.0, 0.0), 0.0, (1, 0), openings_on(g3, cells)) is None


def test_new_cell_trigger_tracks_previous():
    tr = NewCellTrigger(CellGrid())
    run(tr, line(0, 0, 1, 0, 0) + line(1, 0, 0, 0, 180))
    assert (tr.current, tr.previous) == ((0, 0), (1, 0))


def test_side_walls_left_and_right_seen_from_the_way_in():
    cells = CellGrid()
    w = side_walls(cells, (2, 0), (1, 0))          # dead end (2, 0), entered along +i
    assert w['left'] == pytest.approx((2.0, 0.5))
    assert w['right'] == pytest.approx((2.0, -0.5))
    w = side_walls(cells, (0, -2), (0, -1))        # along -j: left is +i
    assert w['left'] == pytest.approx((0.5, -2.0))
    assert w['right'] == pytest.approx((-0.5, -2.0))
    rot = CellGrid(1.0, (0.3, -0.2), math.radians(90))   # rotated, offset grid
    w = side_walls(rot, (1, 0), (1, 0))            # +i = map +y here
    assert w['left'] == pytest.approx((0.3 - 0.5, -0.2 + 1.0))
    assert w['right'] == pytest.approx((0.3 + 0.5, -0.2 + 1.0))


def test_side_heading_aims_at_the_wall_middle_and_is_capped():
    # From the centre of the cell before: the wall middles are 1 m on, 0.5 m aside.
    assert side_heading((1.0, 0.0), (2.0, 0.5), 0.0) == pytest.approx(math.atan2(0.5, 1.0))
    assert side_heading((1.0, 0.0), (2.0, -0.5), 0.0) == pytest.approx(-math.atan2(0.5, 1.0))
    # Standing close to the dead end and off to the right: the left wall would be far round.
    assert side_heading((1.45, -0.3), (2.0, 0.5), 0.0, math.radians(40)) == \
        pytest.approx(math.radians(40))
    assert side_heading((1.0, 0.0), (2.0, 0.5)) == pytest.approx(math.atan2(0.5, 1.0))
    # Facing -i (yaw pi): the cap wraps round correctly.
    h = side_heading((-1.0, 0.0), (-2.0, -0.5), math.pi)
    assert h == pytest.approx(math.atan2(-0.5, -1.0))


def _maze_openings(open_edges, known=None):
    """openings() for a toy maze: open_edges = set of frozenset({cell_a, cell_b}); every
    other side is a wall, or 'unknown' for cells not in `known` (when given)."""
    def openings(cell, d):
        n = (cell[0] + d[0], cell[1] + d[1])
        if known is not None and (cell not in known or n not in known):
            return 'unknown'
        return 'open' if frozenset((cell, n)) in open_edges else 'wall'
    return openings


def _edges(*pairs):
    return {frozenset(p) for p in pairs}


def test_trace_way_dead_end_next_door():
    from maze_nav.scene_understanding.scene_cells import trace_way
    op = _maze_openings(_edges(((0, 0), (1, 0))))
    t = trace_way(op, (0, 0), (1, 0))
    assert t['kind'] == 'dead_end'
    assert t['cell'] == (1, 0) and t['before'] == (0, 0) and t['dir'] == (1, 0)
    assert t['length'] == 1


def test_trace_way_dead_end_round_a_bend():
    from maze_nav.scene_understanding.scene_cells import trace_way
    # (0,0) -> (1,0) -> (1,1) -> (1,2) dead end
    op = _maze_openings(_edges(((0, 0), (1, 0)), ((1, 0), (1, 1)), ((1, 1), (1, 2))))
    t = trace_way(op, (0, 0), (1, 0))
    assert t['kind'] == 'dead_end'
    assert t['cell'] == (1, 2) and t['before'] == (1, 1) and t['dir'] == (0, 1)
    assert t['length'] == 3


def test_trace_way_junction_unknown_and_long():
    from maze_nav.scene_understanding.scene_cells import trace_way
    # a T at (1,0): not a dead end
    op = _maze_openings(_edges(((0, 0), (1, 0)), ((1, 0), (1, 1)), ((1, 0), (1, -1))))
    assert trace_way(op, (0, 0), (1, 0))['kind'] == 'junction'
    # not seen yet beyond (1,0): unknown, never a dead end
    op = _maze_openings(_edges(((0, 0), (1, 0))), known={(0, 0), (1, 0)})
    t = trace_way(op, (0, 0), (1, 0))
    assert t['kind'] == 'unknown' and t['cell'] == (1, 0)
    # a long straight corridor: open after max_cells
    op = _maze_openings(_edges(*[((k, 0), (k + 1, 0)) for k in range(20)]))
    t = trace_way(op, (0, 0), (1, 0), max_cells=5)
    assert t['kind'] == 'open' and t['length'] == 5
