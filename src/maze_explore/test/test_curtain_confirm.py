import inspect
import math
"""Lidar curtain = candidate only; the camera must confirm before a push (user, 2026-11-25).
Driven without ROS: the explorer's own kind / go / do_cv_verify run on a stand-in. Run for
both explorers: maze_explore's and the stairs copy (stairs_crossing/explorer_stairs.py)."""
import importlib
from collections import deque

import pytest

from maze_explore.grid import Grid, nb, opp

MODULES = ('maze_explore.explorer', 'stairs_crossing.explorer_stairs')


class LidarSaysCurtain:
    def __init__(self, edge):
        self.edge = edge

    def kind(self, cell, d, may_exit=True):
        return 'curtain' if (cell, d) == self.edge else 'wall'


class Stand:
    class Log:
        def info(self, *a, **k): pass
        warn = info

    def __init__(self, cv_on=True):
        self.grid = Grid(0.0, 0.0, 0.0, 1.02)
        self.pose = [0.0, 0.0, 0.0]
        self.t, self.act_t0 = 100.0, 100.0
        self.queue, self.events = deque(), []
        self.start = (0, 0)
        self.walked, self.forced_wall, self.closed = set(), set(), set()
        self.cv_curtains, self.cv_rejected, self.cv_pending = set(), set(), set()
        self.curtain_latched, self.links = set(), {}
        self.curtains_on, self.cv_on, self.stairs_on, self.stair_check_on = True, cv_on, True, False
        self.terrain_on, self.stairs_checked, self.exit_cell = False, {}, None
        self.exit_d, self.exit_weak, self.exit_open = None, {}, set()
        self.cv_verify_time, self.cv_verify_hits, self.cv_frames = 3.0, 3, deque()
        self.gait = 'hstep'
        self.cv_first = False          # these tests are about the checks after a candidate exists
        self.maze_rows, self.gait_switch, self.cur_gait, self.rough_cells = 0, False, 'hstep', set()
        self.curtain_right, self.curtain_left = math.pi, math.radians(50.0)
        self.curtain_turns, self.curtain_clear, self.clouds = False, True, deque()
        self.gait_for_step = lambda a, b, rough: None
        self.low = False               # Sense.low_wall for every edge

    def now(self): return self.t
    def elapsed(self): return self.t - self.act_t0
    def get_logger(self): return self.Log()
    def event(self, text): self.events.append(text)
    def send(self, *a): pass

    def sense(self):
        return type('S', (), {'low_wall': lambda s, cell, d: self.low})()

    def verify(self, edge, hits, frames=9, final=True):
        self.act_t0 = self.t
        self.cv_frames = deque((self.t + 0.3 * i, i < hits) for i in range(frames))
        self.t += 3.1
        return self.do_cv_verify(*edge, final)


@pytest.fixture(params=MODULES)
def Robot(request):
    try:
        E = importlib.import_module(request.param).Explorer    # needs the ROS environment
    except ImportError as e:
        pytest.skip(f'{request.param}: {e}')
    return type('Robot', (Stand,), dict(kind=E.kind, go=E.go, do_cv_verify=E.do_cv_verify))


A = (1, 0)
EDGE = (A, 0)
BACK = (nb(A, 0), opp(0))


def test_lidar_curtain_is_verified_before_the_push(Robot):
    r = Robot()
    assert r.kind(LidarSaysCurtain(EDGE), *EDGE) == 'curtain'        # a candidate
    r.go(A, 0, 'curtain')
    assert [a[0] for a in r.queue][-1] == 'cv_verify'                # no strafe / push queued
    assert not any(a[0] == 'strafe' for a in r.queue)


def test_camera_confirms_then_push(Robot):
    r = Robot()
    r.kind(LidarSaysCurtain(EDGE), *EDGE)
    assert r.verify(EDGE, hits=4)
    assert EDGE in r.cv_curtains and BACK in r.cv_curtains
    r.go(A, 0, 'curtain')
    assert any(a[0] == 'strafe' and a[-1] == 'push' for a in r.queue)


def test_camera_rejects_lidar_curtain_becomes_wall(Robot):
    r = Robot()
    s = LidarSaysCurtain(EDGE)
    r.kind(s, *EDGE)
    assert r.verify(EDGE, hits=1)
    assert EDGE in r.cv_rejected and BACK in r.cv_rejected
    assert EDGE not in r.curtain_latched
    assert r.kind(s, *EDGE) == 'wall'                                # and stays one
    assert 'wall now' in r.events[-1]


def test_greet_miss_is_not_final(Robot):
    r = Robot()
    s = LidarSaysCurtain(EDGE)
    r.kind(s, *EDGE)
    assert r.verify(EDGE, hits=0, final=False)
    assert EDGE not in r.cv_rejected
    assert r.kind(s, *EDGE) == 'curtain'                             # still a candidate


def test_no_camera_frames_is_no_confirmation(Robot):
    r = Robot()
    s = LidarSaysCurtain(EDGE)
    r.kind(s, *EDGE)
    assert r.verify(EDGE, hits=0, frames=0)
    assert r.kind(s, *EDGE) == 'wall'
    assert 'no camera frames' in r.events[-1]


def test_camera_off_lidar_decides_alone(Robot):
    r = Robot(cv_on=False)
    r.kind(LidarSaysCurtain(EDGE), *EDGE)
    r.go(A, 0, 'curtain')
    assert any(a[0] == 'strafe' and a[-1] == 'push' for a in r.queue)


def test_camera_sees_a_curtain_over_an_ordinary_wall_the_wall_stays_a_wall(Robot):
    r = Robot()
    r.low = True                       # the height map: an ordinary 0.53 m wall on this edge
    assert r.verify(EDGE, hits=9)      # the camera sees the curtain behind it on every frame
    assert EDGE not in r.cv_curtains and BACK not in r.cv_curtains
    assert EDGE in r.cv_rejected
    assert 'behind it' in r.events[-1]


class LidarSays:
    def __init__(self, what):
        self.what = what

    def kind(self, cell, d, may_exit=True):
        return self.what


@pytest.mark.parametrize('lidar', ['stairs', 'blocked'])
def test_possible_stairs_that_are_not_stairs_get_a_look(Robot, lidar):
    r = Robot()
    r.stair_check_on = True
    r.raised = lambda cell, d: False
    assert r.kind(LidarSays(lidar), *EDGE) == 'stairs'          # a candidate: checked at once
    r.stairs_checked[EDGE] = False                              # the depth camera: not stairs
    assert r.kind(LidarSays(lidar), *EDGE) == 'object'          # a prop: looked at, not entered
    r.stairs_checked[EDGE] = True
    assert r.kind(LidarSays(lidar), *EDGE) == 'stairs'


def test_after_a_curtain_there_are_no_turns(Robot):
    r = Robot(cv_on=False)
    if not hasattr(r, 'curtain_turns') or 'curtain_turns' not in __import__('inspect').getsource(type(r).go):
        pytest.skip('the stairs copy keeps its own curtain ending')
    r.kind(LidarSaysCurtain(EDGE), *EDGE)
    r.go(A, 0, 'curtain')
    acts = [a[0] for a in r.queue]
    after = acts[acts.index('strafe') + 1:]
    assert 'turn' not in after and after[0] == 'curtain_clear'    # turns only if the lidar is covered
    r.queue.clear()
    r.curtain_turns = True                                   # the old 180 right / 50 left
    r.go(A, 0, 'curtain')
    acts = [a[0] for a in r.queue]
    assert acts[acts.index('strafe') + 1:].count('turn') == 3


def test_an_edge_that_stands_down_to_the_floor_is_not_a_curtain():
    # user, 2026-11-26 night: a curtain hangs from about 0.9 m and leaves ~0.15 m free under it
    import numpy as np
    from types import SimpleNamespace as NS
    from maze_explore.sense import HeightMap, Sense
    E = importlib.import_module('maze_explore.explorer').Explorer
    g = Grid(0.0, 0.0, 0.0, 1.02)
    hm = HeightMap()
    s = Sense(g, NS(at=lambda x, y: np.zeros(np.shape(x))), hm)
    x, y = g.box((0, 0), 0, np.linspace(-0.3, 0.3, 13), [0.0])
    for _ in range(3):
        hm.add(x.ravel(), y.ravel(), np.full(x.size, 0.9))        # fabric, tall
    assert s.solid_base((0, 0), 0) == 0.0                         # nothing at 0.06-0.12 m: a gap
    me = NS(base_share_max=0.5, event=lambda t: None)
    assert not E.base_veto(me, s, (0, 0), 0)
    for _ in range(3):
        hm.add(x.ravel(), y.ravel(), np.full(x.size, 0.09))       # the frame wall reaches the floor
    assert s.solid_base((0, 0), 0) == 1.0 and E.base_veto(me, s, (0, 0), 0)
    # a raised platform behind a real curtain fills the gap too: the camera's word is not
    # overruled, and its sightings are still checked (user, 2026-11-26 night)
    assert 'base_veto' not in inspect.getsource(E.do_cv_verify)
    assert 'base_veto' not in inspect.getsource(E.plan)
    assert 'not self.cv_on' in inspect.getsource(E.kind)


def test_the_camera_needs_5_frames_at_0_6():
    src = inspect.getsource(importlib.import_module('maze_explore.explorer').Explorer.__init__)
    assert "p('cv_conf', 0.6)" in src and "p('cv_verify_hits', 5)" in src and "p('cv_verify_time', 4.0)" in src
