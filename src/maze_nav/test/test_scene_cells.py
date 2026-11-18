"""Unit tests for maze_nav.scene_understanding.scene_cells (no ROS needed): python3 -m pytest -q test"""
import math

import numpy as np
import pytest

from maze_nav.core import Grid
from maze_nav.scene_understanding.scene_cells import CellGrid, NewCellTrigger, opening, run_active

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
                   'FACING_QR: unreadable code'):
        assert run_active(status), status
    for status in ('', None, 'STARTING: waiting for /map', 'PAUSED: ready; holding still',
                   'EXITED: open space', 'NO_EXIT: all frontiers mapped', 'STOPPED: Ctrl+C',
                   'EXPLORINGX'):
        assert not run_active(status), status
    assert run_active('PAUSED: x', active_states=('PAUSED',))
