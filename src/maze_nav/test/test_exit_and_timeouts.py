"""No-EXITED fixes from the 2026-11-21 sim (no ROS): the exit is checked in every moving state,
no looks in open space, curtain candidates behind a wall are skipped, stuck goals and stuck
inspections give up.

python3 -m pytest -q test/test_exit_and_timeouts.py"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

import maze_nav.explorer as explorer_mod
from maze_nav.core import ExitDetector, Grid
from maze_nav.curtains import wall_between
from maze_nav.explorer import Explorer

RES = 0.05


def grid(x0, y0, size, value=0):
    n = int(round(size / RES))
    return Grid(np.full((n, n), value, dtype=np.int16), RES, (x0, y0))


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(explorer_mod.time, 'monotonic', c)
    return c


class Fake:
    for _n in ('step', 'out_of_maze', 'in_open_space', 'goal_stalled', 'on_stairs'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self, state, xy, g):
        self.state, self.map, self.xy, self.g = state, object(), xy, g
        self.cfg = {'open_space_no_looks': 0.8, 'max_runtime': 900.0, 'goal_timeout': 45.0,
                    'goal_progress': 0.2, 'goal_max_time': 120.0}
        self.exit = ExitDetector(dwell=2.0)
        self.exit.start, self.exit.seen_enclosed = (-6.0, 0.0), True   # came from the maze
        self.finished, self.started = [], explorer_mod.time.monotonic()
        self.logs = []
        self.log = SimpleNamespace(info=self.logs.append, warn=self.logs.append)
        # what the per-state branches of step() touch before they return
        now = explorer_mod.time.monotonic()
        self.cfg.update(curtain_timeout=300.0, stairs_timeout=60.0, stairs_approach_timeout=60.0)
        self.curtain_since = self.stairs_since = now
        self.stairs_approaching = self.resume_requested = False
        self.spin_busy, self.turn_creep = True, None
        self.face_tick = lambda pose: None

    def robot_pose(self):
        return self.xy, 0.0

    def grid_now(self):
        return self.g

    def finish(self, state, why):
        self.finished.append((state, why))
        self.state = state

    def get_logger(self):
        return self.log


@pytest.mark.parametrize('state', ['JUNCTION', 'DEAD_END', 'CURTAIN', 'STAIRS', 'SCANNING',
                                   'FACING_QR', 'LOOKING', 'TURNING'])
def test_exit_found_in_any_moving_state(clock, state):
    """Outside the maze the junction looks ran back to back (0.1 s of EXPLORING between):
    the exit has to be seen from inside the look too, and hold exit_dwell s there."""
    f = Fake(state, (0.0, 0.0), grid(-4.0, -4.0, 8.0))     # wide open, seen free
    f.step()
    assert f.finished == []                                 # dwell not held yet
    clock.t += 1.0
    f.step()
    clock.t += 1.2
    f.step()
    assert f.finished and f.finished[0][0] == 'EXITED'
    assert f'(ended {state})' in f.finished[0][1]


def test_no_exit_in_a_corridor_nor_while_creeping(clock):
    g = grid(-4.0, -4.0, 8.0, value=100)
    g.data[:, int(3.6 / RES):int(4.4 / RES)] = 0            # 0.8 m wide corridor along y
    f = Fake('JUNCTION', (0.0, 0.0), g)
    for _ in range(4):
        f.step()
        clock.t += 1.0
    assert f.finished == [] and not f.in_open_space()
    f = Fake('CREEPING', (0.0, 0.0), grid(-4.0, -4.0, 8.0))  # start: never checked
    for _ in range(4):
        f.step()
        clock.t += 1.0
    assert f.finished == []


def test_open_space_skips_looks(clock):
    f = Fake('EXPLORING', (0.0, 0.0), grid(-4.0, -4.0, 8.0))
    f.exit.update(f.g, f.xy, clock.t)
    assert f.in_open_space()
    f.cfg['open_space_no_looks'] = 0.0                       # 0 = off
    assert not f.in_open_space()


def test_goal_stalled_on_no_progress_but_not_on_a_long_drive(clock):
    f = Fake('EXPLORING', (0.0, 0.0), None)
    f.goal_sent_at, f.goal_moved = clock.t, None
    for _ in range(40):                                     # moving 0.1 m/s for 120 s: fine
        clock.t += 3.0
        f.xy = (f.xy[0] + 0.3, 0.0)
        if clock.t - f.goal_sent_at <= 120.0:
            assert not f.goal_stalled(f.xy)
    clock.t += 1.0
    assert f.goal_stalled(f.xy) and 'after 120 s' in f.logs[-1]   # hard cap
    f.goal_sent_at, f.goal_moved = clock.t, None             # pinned on a wall
    assert not f.goal_stalled(f.xy)
    clock.t += 30.0
    f.xy = (f.xy[0] + 0.05, 0.0)                            # jiggling, not progress
    assert not f.goal_stalled(f.xy)
    clock.t += 16.0
    assert f.goal_stalled(f.xy) and 'moved < 0.20 m in 45 s' in f.logs[-1]


def test_wall_between_robot_and_curtain_candidate():
    g = grid(-1.0, -1.0, 4.0)
    for y in np.arange(-1.0, 3.0, RES / 2):                  # wall along x = 1.0
        g.data[int((y + 1.0) / RES), int(2.0 / RES)] = 100
    assert wall_between(g, (0.0, 0.0), (2.0, 0.0))          # candidate behind the wall
    assert not wall_between(g, (0.0, 0.0), (1.0, 0.0))      # the candidate IS that wall
    assert not wall_between(g, (0.0, 0.0), (0.0, 2.0))      # clear line along it
    assert not wall_between(g, (0.0, 0.0), (0.4, 0.0))      # too close to judge
    g.data[int(1.0 / RES), int(1.15 / RES)] = 100           # a speck in the robot's own cell
    assert not wall_between(g, (0.0, 0.0), (0.0, 2.0))


def test_stairs_timeout_restarts_once_approaching(clock):
    """2026-11-21 sim: checking + aligning used 34 of the 60 s and the approach was cut off.
    The approach gets its own stairs_approach_timeout from APPROACHING on."""
    f = Fake('STAIRS', (-9.0, 0.0), grid(-10.0, -1.0, 2.0, value=100))   # walls: no exit
    ended = []
    f.begin_after_stairs = ended.append
    f.stairs_since = clock.t
    clock.t += 50.0
    f.on_stairs(SimpleNamespace(data='APPROACHING: 0.71 m from the first riser'))
    assert f.stairs_approaching and f.stairs_since == clock.t
    clock.t += 50.0                                         # 100 s in STAIRS, 50 approaching
    f.step()
    assert ended == []
    clock.t += 11.0
    f.step()
    assert ended == ['stairs timeout; exploring on']
    assert 'of approach' in f.logs[-1]
