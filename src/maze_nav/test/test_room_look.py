"""Room look-in (an object cell is looked into from a neighbour, never entered) and the Nav2
keepout mask of the object cells, explorer side, no ROS.

python3 -m pytest -q test/test_room_look.py"""
import math

import numpy as np

import maze_nav.explorer as explorer_mod
from maze_nav.core import Grid
from maze_nav.explorer import Explorer, keepout_mask, path_field, room_look_spot
from maze_nav.scene_cells import CellGrid

from test_junction_dead_end_look import Fake, run


def room_maze():
    """Free floor 4 x 3 cells (x -0.5..3.5, y -0.5..2.5), outer walls; room (2, 1) has walls
    on every side but -i (towards cell (1, 1))."""
    res = 0.05
    g = np.zeros((60, 80), np.int16)
    g[:2, :] = g[-2:, :] = 100
    g[:, :2] = g[:, -2:] = 100

    def wall(x0, y0, x1, y1):
        c0, c1 = int((x0 + 0.5) / res), int((x1 + 0.5) / res)
        r0, r1 = int((y0 + 0.5) / res), int((y1 + 0.5) / res)
        g[r0:r1 + 1, c0:c1 + 1] = 100
    wall(1.5, 1.5, 2.5, 1.5)      # +j side
    wall(1.5, 0.5, 2.5, 0.5)      # -j side
    wall(2.5, 0.5, 2.5, 1.5)      # +i side
    return Grid(g, res, (-0.5, -0.5))


def test_spot_is_the_neighbour_through_the_open_side():
    grid = room_maze()
    field = path_field(grid, (0.0, 0.0))
    n, d, m = room_look_spot(grid, grid, CellGrid(), (2, 1), field, radius=5.0)
    assert (n, d) == ((1, 1), (-1, 0)) and 1.3 < m < 1.6


def test_no_spot_out_of_reach_or_skipped():
    grid = room_maze()
    field = path_field(grid, (0.0, 0.0))
    assert room_look_spot(grid, grid, CellGrid(), (2, 1), field, radius=0.5) is None
    assert room_look_spot(grid, grid, CellGrid(), (2, 1), field, skip={(1, 1)},
                          radius=5.0) is None


def test_spot_behind_a_wall_is_not_reachable():
    """Robot 2026-11-21 10:28: the neighbour was 2 m away in a straight line, behind a wall."""
    grid = room_maze()
    g = grid.data
    g[:, 28:31] = 100                          # wall at x ~ 0.95 across the whole maze
    field = path_field(grid, (0.0, 0.0))
    assert room_look_spot(grid, grid, CellGrid(), (2, 1), field, radius=5.0) is None


def test_keepout_mask_is_lethal_in_the_cell_only():
    m = keepout_mask(CellGrid(), {(2, 1)})
    inside = m.cell((2.0, 1.0))
    outside = m.cell((1.45, 1.0))
    assert m.data[inside[1], inside[0]] == 100
    assert m.data[outside[1], outside[0]] == 0
    empty = keepout_mask(CellGrid(), set())
    assert empty.data.shape == (2, 2) and not empty.data.any()


class RoomFake(Fake):
    new_look = staticmethod(Explorer.new_look)
    for _n in ('queue_room_look', 'start_room_look', 'side_steps'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self):
        super().__init__()
        self.cfg.update({'room_looks': True, 'room_look_hold': 0.0, 'room_look_max': 6})
        self.room_looks, self.rooms_done, self.room_looks_started = {}, set(), 0
        self.scanned_cells, self.explored_cells = set(), set()
        self.goals = []

    def halt_for_look(self):
        pass


def test_room_look_never_goes_in_and_scans_its_walls_with_perception_on():
    e = RoomFake()
    e.xy, e.yaw = (1.0, 1.0), 0.0                     # already in the neighbour, facing in
    e.queue_room_look((2, 1), (2.0, 1.0))
    e.start_room_look((2, 1), (1, 1), (-1, 0))
    todo = e.junction['todo']
    assert [a['do'] for a in todo] == ['goto', 'align', 'hold', 'look']
    assert todo[0]['xy'] == (1.0, 1.0)                # the neighbour's centre, not the room
    assert abs(todo[1]['heading']) < 1e-9             # facing +x, into the room
    assert e.at_dead_end() is False                   # driving there: perception off
    todo.pop(0)
    todo.pop(0)                                       # (goto / align covered elsewhere)
    on = []
    for _ in range(40):
        on.append(e.at_dead_end())
        e.look_tick()
        if e.pose is not None:
            e.pose['since'] = -1e9
            e.pose_status = 'POSING'
            e.pose_tick()
        e.spin_busy = False
        if not e.junction.get('todo'):
            break
    assert any(on)                                    # QR / poster checks while looking
    assert [(c['cell'], c['view'], c['kind']) for c in e.captures] == [
        ([2, 1], 'centre', 'room'), ([2, 1], 'left', 'room'), ([2, 1], 'right', 'room')]
    assert e.captures[-1]['last']
    assert (2, 1) in e.rooms_done and e.room_looks_started == 1


def test_a_scanned_dead_end_gets_no_room_look():
    e = RoomFake()
    e.dead_end_cells = {(2, 1)}
    e.queue_room_look((2, 1), (2.0, 1.0))
    assert e.room_looks == {}

