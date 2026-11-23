"""Queued stairs (robot run 2026-11-23 05:43, stairs on the left of the 2nd cell): checked as
soon as the robot stands in the cell in front of their first riser, from that cell's centre
when stairs_approach in front lies behind a wall; a held stair cell the robot has stood in
is never walled. Explorer side, no ROS.

python3 -m pytest -q test/test_stairs_near.py"""
import math
from types import SimpleNamespace

import numpy as np

from maze_nav.core import Grid
from maze_nav.explorer import Explorer
from maze_nav.scene_cells import CellGrid


class Fake:
    for _n in ('workable', 'held_stair_cells', 'stairs_front_cell', 'near_stairs_queue',
               'candidate_spot'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self):
        self.cfg = {'stairs_check_near': True, 'stairs_approach': 1.2,
                    'curtain_check_approach': 0.8}
        self.cells = CellGrid(1.02, (0.0, 0.0), 0.0)
        self.cand_queue, self.queue_active, self.queue_final = [], False, False
        self.visited, self.map, self.started = set(), None, []
        self.xy = (0.0, 0.0)

    def grid_now(self):                # corridor y = -0.5..0.5, x = -0.5..2.5; walls around
        d = np.full((60, 80), 100, np.int16)
        d[10:30, 0:60] = 0
        return Grid(d, 0.05, (-0.5, -1.0))

    def robot_pose(self):
        return (self.xy, 0.0)

    def start_queue(self, why, final=False):
        self.started.append(why)
        self.queue_active = bool(self.workable())
        return self.queue_active


def stairs(x=1.31, y=0.60, heading=math.pi / 2, cid=1):
    return {'kind': 'stairs', 'id': cid, 'target': (x, y), 'heading': heading}


def front(x, y):
    return SimpleNamespace(goal=(x, y))


def test_stairs_wait_until_the_robot_is_in_the_cell_in_front():
    e = Fake()
    e.cand_queue.append(stairs())
    assert e.stairs_front_cell(e.cand_queue[0]) == (1, 0)
    assert not e.near_stairs_queue((0.2, 0.0)) and e.workable() == []     # still in cell (0, 0)
    assert e.near_stairs_queue((0.9, 0.0))
    assert e.workable() == e.cand_queue and e.queue_active


def test_frontiers_in_the_cells_around_come_first():
    e = Fake()
    e.cand_queue.append(stairs())
    assert not e.near_stairs_queue((0.9, 0.0), [front(2.0, 0.0)])         # way on in cell (2, 0)
    assert e.workable() == []
    # the robot explored on; a frontier two cells away doesn't hold the stairs back
    assert e.near_stairs_queue((2.0, 0.0), [front(3.1, 0.0)])
    assert e.workable() == e.cand_queue


def test_switch_off_keeps_stairs_for_the_end():
    e = Fake()
    e.cfg['stairs_check_near'] = False
    e.cand_queue.append(stairs())
    assert not e.near_stairs_queue((0.9, 0.0)) and e.workable() == []


def test_spot_is_half_a_cell_in_front_when_the_approach_is_behind_a_wall():
    e = Fake()
    assert e.candidate_spot(stairs()) == (1.31, 0.60 - 1.2)       # no map yet: as before
    e.map = object()                 # 1.2 m in front is behind the corridor's far wall
    x, y = e.candidate_spot(stairs())
    assert (round(x, 2), round(y, 2)) == (1.31, 0.09) and e.cells.cell((x, y)) == (1, 0)


def test_a_cell_the_robot_stood_in_is_never_walled():
    e = Fake()
    e.cand_queue.append(stairs(x=0.51, y=0.0, heading=0.0, cid=2))      # false: in the corridor
    e.xy = (-0.4, 0.0)
    assert e.held_stair_cells() == {(1, 0)}
    e.visited.add((1, 0))
    assert e.held_stair_cells() == set()
