"""backup_nav: an empty room is entered 0.5 m and its two side edges get a depth stair
check from in there. Driven without ROS on a stand-in."""
import importlib
import math
from collections import deque

import pytest

from maze_explore.grid import Grid, nb, opp


class Stand:
    def __init__(self):
        self.grid = Grid(0.0, 0.0, 0.0, 1.02)
        self.cell_size = 1.02
        self.pose = [0.0, 0.0, 0.0]
        self.queue, self.events = deque(), []
        self.kinds, self.looked, self.stairs_checked = {}, set(), {}
        self.visited, self.closed, self.walked = {(0, 0), (1, 0)}, set(), set()
        self.empty_skip, self.empty_squares, self.look_hold = True, 4, 1.0
        self.stair_check_on, self.qr_approach, self.qr_read_t = True, False, -1.0
        self.room_enter, self.room_enter_min = 0.5, 0.35
        self.arrived = []

    def send(self, *a): pass
    def update_kinds(self): pass
    def event(self, text): self.events.append(text)
    def room_empty(self, c, d): return True, 0
    def body_centre(self): return self.pose[0], self.pose[1]
    def arrive(self, cell): self.arrived.append(cell)


@pytest.fixture
def Robot():
    try:
        E = importlib.import_module('maze_explore.explorer').Explorer
    except ImportError as e:
        pytest.skip(str(e))
    return type('Robot', (Stand,), dict(do_look_last=E.do_look_last,
                                        room_side_stairs=E.room_side_stairs, kind=E.kind))


C, D = (1, 0), 0                       # looking from (1, 0) into the room (2, 0)
ROOM = nb(C, D)


def run_calls(r):
    """Run the queue's 'call' actions in order; drop the motion actions, record them."""
    done = []
    while r.queue:
        a = r.queue.popleft()
        done.append(a)
        if a[0] == 'call':
            a[1]()
    return done


def test_empty_room_is_entered_half_a_metre_and_both_sides_checked(Robot):
    r = Robot()
    r.kinds[(C, D)] = 'room'
    r.do_look_last(C, D, 'room')
    drive = next(a for a in r.queue if a[0] == 'drive')
    edge_x = r.grid.centre(C)[0] + 0.51
    assert drive[1] == pytest.approx(edge_x + 0.5)            # base_link 0.5 m past the edge
    r.pose = [drive[1], drive[2], 0.0]                        # got there
    acts = run_calls(r)
    checks = [a for a in acts if a[0] == 'stair_check']
    assert [(a[1], a[2]) for a in checks] == [(ROOM, 1), (ROOM, 3)]
    assert (C, D) in r.looked
    # nothing found: turned round and walked back to the centre of C
    assert acts[-1][0] == 'drive' and acts[-1][1:3] == pytest.approx(r.grid.centre(C))
    assert not r.arrived


def test_stairs_on_a_room_side_keep_the_robot_in_the_room(Robot):
    r = Robot()
    r.kinds[(C, D)] = 'room'
    r.do_look_last(C, D, 'room')
    drive = next(a for a in r.queue if a[0] == 'drive')
    r.pose = [drive[1], drive[2], 0.0]
    while r.queue:
        a = r.queue.popleft()
        if a[0] == 'stair_check' and a[2] == 3:
            r.stairs_checked[(a[1], a[2])] = True             # the depth camera: STAIRS
        if a[0] == 'call':
            a[1]()
    assert r.arrived == [ROOM] and (C, D) in r.walked and (ROOM, opp(D)) in r.walked


def test_stopped_short_no_turns_in_the_room(Robot):
    r = Robot()
    r.kinds[(C, D)] = 'room'
    r.do_look_last(C, D, 'room')
    r.pose = [r.grid.centre(C)[0] + 0.51 + 0.1, 0.0, 0.0]     # only 0.10 m in
    acts = run_calls(r)
    assert not any(a[0] == 'stair_check' for a in acts)
    assert any('no side checks' in e for e in r.events)


def test_off_with_room_enter_zero(Robot):
    r = Robot()
    r.room_enter = 0.0
    r.kinds[(C, D)] = 'room'
    r.do_look_last(C, D, 'room')
    assert not any(a[0] == 'drive' for a in r.queue)
