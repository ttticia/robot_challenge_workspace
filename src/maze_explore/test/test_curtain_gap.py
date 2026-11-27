"""Curtains are not told by height (user, 2026-11-26 night): a YOLO sighting is the only
candidate; the robot centres in the cell facing it, lies down and looks for the GAP under
the curtain (a curtain hangs from ~0.9 m with ~0.15 m free under it)."""
import math
from collections import deque

import numpy as np
import pytest

from maze_explore.grid import Grid, nb, opp

explorer = pytest.importorskip('maze_explore.explorer')
E = explorer.Explorer
CELL, D = (1, 0), 0               # the edge: x = 1.53, y in [-0.51, 0.51]


class Robot:
    gap_verdict = E.gap_verdict
    gap_steps = E.gap_steps
    do_gap_check = E.do_gap_check
    do_cv_verify = E.do_cv_verify

    class Log:
        def info(self, *a, **k): pass
        warn = info

    def __init__(self):
        self.grid, self.cell_size = Grid(0.0, 0.0, 0.0, 1.02), 1.02
        self.gap_low, self.gap_hang = (0.04, 0.13), (0.20, 0.60)
        self.gap_low_max, self.gap_hang_min, self.gap_floor_min = 0.25, 0.40, 10
        self.gap_time, self.gap_settle, self.cmd_gap, self.cur_gait = 3.0, 3.0, 1.5, 'normal'
        self.curtain_height, self.curtain_gap = False, True
        self.cv_curtains, self.cv_rejected, self.cv_pending = set(), set(), set()
        self.curtain_latched, self.forced_wall = set(), set()
        self.cv_verify_time, self.cv_verify_hits, self.cv_frames = 3.0, 5, deque()
        self.queue, self.events, self.act_mem = deque(), [], {}
        self.t, self.act_t0, self.pose, self.cloud, self.cloud_time = 10.0, 10.0, [1.02, 0.0, 0.0], None, None

    def now(self): return self.t
    def elapsed(self): return self.t - self.act_t0
    def send(self, *a, **k): pass
    def event(self, text): self.events.append(text)
    def get_logger(self): return self.Log()
    def sense(self): return type('S', (), {'low_wall': lambda s, c, d: True})()


def edge_points(hang=True, low=False, floor=True):
    """Map points around the edge x = 1.53: fabric 0.20-0.60 m, the gap band, floor beyond."""
    ys = np.linspace(-0.28, 0.28, 30)
    out = []
    if hang:
        for z in (0.25, 0.40, 0.55):
            out += [(1.53, y, z) for y in ys]
    if low:
        for z in (0.06, 0.10):
            out += [(1.53, y, z) for y in ys]
    if floor:
        out += [(1.53 + dv, y, 0.01) for dv in (0.3, 0.6) for y in ys]
    return np.array(out, float)


def test_a_curtain_has_a_gap_under_it():
    r = Robot()
    ok, why = r.gap_verdict(edge_points(), CELL, D)
    assert ok and 'a gap under it' in why


def test_a_wall_stands_down_to_the_floor():
    r = Robot()
    ok, why = r.gap_verdict(edge_points(low=True, floor=False), CELL, D)
    assert not ok and 'solid down to the floor' in why


def test_nothing_hanging_or_gap_not_seen_is_no_curtain():
    r = Robot()
    ok, why = r.gap_verdict(edge_points(hang=False), CELL, D)
    assert not ok and 'nothing hangs' in why
    ok, why = r.gap_verdict(edge_points(floor=False), CELL, D)
    assert not ok and 'not seen through' in why
    assert r.gap_verdict(None, CELL, D)[0] is False


def test_camera_hits_lie_the_robot_down_instead_of_confirming():
    # the camera alone confirms nothing any more: it starts the lie-down gap check
    r = Robot()
    r.cv_frames = deque((r.t + 0.3 * i, True, 0.0) for i in range(8))
    r.t += 3.1
    assert r.do_cv_verify(CELL, D, True)
    assert (CELL, D) not in r.cv_curtains and (CELL, D) not in r.cv_rejected  # not decided yet
    acts = [a[0] for a in r.queue]
    assert acts[0] == 'cmd' and r.queue[0][1] == explorer.robot.STAND_SIT     # lie down first
    assert 'gap_check' in acts
    i = acts.index('gap_check')
    assert r.queue[i + 1] == ('cmd', explorer.robot.STAND_SIT)                # then stand up
    codes = [a[1] for a in list(r.queue)[i + 2:] if a[0] == 'cmd']
    assert codes == list(explorer.robot.start_sequence('normal'))             # modes restored
    # an 'ordinary low wall' (height) no longer rejects a camera sighting
    assert not any('ordinary wall' in e for e in r.events)


def test_the_gap_check_confirms_or_rejects():
    for pts, curtain in ((edge_points(), True), (edge_points(low=True, floor=False), False)):
        r = Robot()
        r.pose = [0.0, 0.0, 0.0]                 # base_link at the origin, cloud in that frame
        r.cloud, r.cloud_time = pts, 1.0
        assert not r.do_gap_check(CELL, D, True)          # gathering
        r.t += 3.1
        assert r.do_gap_check(CELL, D, True)
        assert ((CELL, D) in r.cv_curtains) == curtain
        assert ((nb(CELL, D), opp(D)) in r.cv_curtains) == curtain
        assert ((CELL, D) in r.cv_rejected) == (not curtain)


def test_height_never_makes_a_curtain():
    import inspect
    src = inspect.getsource(E.kind)
    assert "k == 'curtain' and not getattr(self, 'curtain_height', True)" in src
    # default back to the height test since 2026-11-27 (user): YOLO proposes, lidar confirms
    assert "p('curtain_height', True)" in inspect.getsource(E.__init__)
    plan = inspect.getsource(E.plan)
    assert "('shift', cx, cy), ('turn', h, 2.0)" in plan     # centred, facing the curtain
