"""After the stairs, before the gait change: walk onto the centre of the cell after if the
robot is off it (user, 2026-11-26). The explorer's own do_ai_centre on a stand-in."""
import math

import numpy as np
import pytest

from maze_explore.grid import Grid, wrap

explorer = pytest.importorskip('maze_explore.explorer')


class Robot:
    do_ai_centre = explorer.Explorer.do_ai_centre

    class Log:
        def info(self, *a, **k): pass
        warn = info

    def __init__(self, x, y, yaw=0.0):
        self.grid, self.cell_size = Grid(0.0, 0.0, 0.0, 1.02), 1.02
        self.pose, self.t, self.act_mem = [x, y, yaw], 50.0, {}
        self.pose_time, self.pose_timeout = 50.0, 1.0
        self.ai_centre_on, self.ai_centre_tol, self.ai_centre_speed = True, 0.10, 0.20
        self.ai_walking, self.cloud, self.blocked = False, np.zeros((0, 3)), False
        self.odom_jumps, self.odom_hold_until, self.odom_grace_until = [], 0.0, 0.0
        self.events, self.cmds = [], []

    def now(self): return self.t
    def get_logger(self): return self.Log()
    def event(self, text): self.events.append(text)
    def front_blocked(self): return self.blocked

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        self.cmd = (vx, vy, wz)
        self.cmds.append(self.cmd)

    def run(self, cell, seconds=30.0, fresh=True):
        for _ in range(int(seconds / 0.1)):
            self.cmd = (0.0, 0.0, 0.0)
            if fresh:
                self.pose_time = self.t
            if self.do_ai_centre(cell, 0.0):
                return True
            x, y, yaw = self.pose
            vx, vy, wz = self.cmd
            self.pose = [x + vx * math.cos(yaw) * 0.1, y + vx * math.sin(yaw) * 0.1,
                         wrap(yaw + wz * 0.1)]
            self.t += 0.1
        return False


def test_centred_already_means_no_walk():
    r = Robot(2.04 - 0.06, 0.05)
    assert r.run((2, 0)) and 'in the middle' in r.events[0]
    assert all(c == (0.0, 0.0, 0.0) for c in r.cmds) and not r.ai_walking


def test_short_of_the_centre_it_walks_onto_it_forward_only():
    r = Robot(2.04 - 0.16, 0.08, 0.05)                  # where the AI walk ends, a bit aside
    assert r.run((2, 0)) and 'walked onto the centre' in r.events[0]
    assert math.hypot(r.pose[0] - 2.04, r.pose[1]) < 0.08
    assert all(vy == 0.0 and vx >= 0.0 and vx <= 0.20 for vx, vy, wz in r.cmds)
    assert not r.ai_walking and r.odom_grace_until > 50.0


def test_not_attempted_when_the_centre_is_behind_or_the_pose_is_no_good():
    r = Robot(2.04 + 0.30, 0.0)                         # past the centre: nothing backwards
    assert r.run((2, 0)) and 'not walked' in r.events[0] and len(r.cmds) == 1
    r = Robot(2.04 - 1.5, 0.0)                          # more than a cell off: pose not believed
    assert r.run((2, 0)) and 'further than a cell' in r.events[0]
    r = Robot(2.04 - 0.16, 0.0)
    r.pose_time = 40.0                                  # no fresh pose
    assert r.run((2, 0), fresh=False) and 'pose too old' in r.events[0]


def test_a_wall_ahead_ends_the_walk():
    r = Robot(2.04 - 0.40, 0.0)
    r.blocked = True
    assert r.run((2, 0)) and 'wall ahead' in r.events[0]
    assert r.pose[0] < 2.04 - 0.30


def test_uneven_confirmed_ends_the_floor_check_and_is_not_asked_again():
    # user, 2026-11-26: once the cell ahead is confirmed uneven, stop checking for it
    from types import SimpleNamespace as NS
    E = explorer.Explorer
    calls = []

    def robot(stairs_frames_uneven):
        r = NS(t=10.0, act_mem={}, act_t0=10.0, odom_grace_until=0.0, pose=(0.0, 0.0, 0.0),
               terrain_on=True, terrain_depth=True, terrain_confirm=3, rough_known=set(),
               rough_checked={}, stairs_checked={}, stair_check_time=4.5, stair_quick_no=8,
               stair_min_hits=5, stair_rails=False, body_height=0.30, clouds=[],
               depth_topic='d', depth_info_topic='i', odom_rot=np.eye(3), events=[],
               queue=__import__('collections').deque(), destroyed=[])
        r.now = lambda: r.t
        r.elapsed = lambda: r.t - r.act_t0
        r.send = lambda *a, **k: None
        r.event = r.events.append
        r.get_logger = lambda: NS(info=lambda *a, **k: None, warn=lambda *a, **k: None)
        r.create_subscription = lambda *a, **k: object()
        r.destroy_subscription = r.destroyed.append
        r.tf_buffer = NS(lookup_transform=lambda *a: (_ for _ in ()).throw(
            explorer.TransformException('none')))
        r.depth_image = lambda msg: np.ones((4, 4), np.uint16)
        r.terrain_from_depth = lambda *a: (calls.append(1), (stairs_frames_uneven.pop(0), 80))[1]
        r.stair_params = NS(body_height=0.3)
        return r

    def run(r, stairs):
        for i in range(80):
            if 'subs' in r.act_mem:
                r.act_mem['frame'], r.act_mem['k'] = object(), (1.0, 1.0, 1.0, 1.0)
            if E.do_stair_check(r, (0, 0), 0, stairs):
                return i
            r.t += 0.1
    del calls[:]
    r = robot([False, True, True, True] + [True] * 60)
    ticks = run(r, False)
    assert ticks <= 5 and len(calls) == 4                 # ended at the third uneven frame
    assert r.rough_known == {(1, 0)} and r.rough_checked[((0, 0), 0)] is True
    assert len(r.destroyed) == 2 and 'UNEVEN' in r.events[-1]
    # asked from another side later: uneven without a new check
    r.sense = lambda: NS(rough=lambda a, d: False)
    assert E.is_rough(r, (1, 1), 3) is True               # cell (1, 0) seen from (1, 1)
    del calls[:]
    r = robot([False] * 60)                               # flat: the usual 8 frames
    assert run(r, False) >= 7 and r.rough_known == set()


def gate_robot(frames):
    """Stand-in for do_stair_check with stairs asked for. frames: per depth frame
    (floor verdict, squares stair-high or None when the floor reading is unusable)."""
    from collections import deque
    from types import SimpleNamespace as NS
    r = NS(t=10.0, act_mem={}, act_t0=10.0, odom_grace_until=0.0, pose=(0.0, 0.0, 0.0),
           terrain_on=True, terrain_depth=True, terrain_confirm=3, rough_known=set(),
           rough_checked={}, stairs_checked={}, stair_check_time=4.5, stair_quick_no=8,
           stair_min_hits=5, stair_rails=False, body_height=0.30, clouds=[],
           stair_gate=True, stair_gate_min=0.20, stair_gate_squares=6, stair_gate_frames=3,
           depth_high=None, depth_topic='d', depth_info_topic='i', odom_rot=np.eye(3),
           events=[], queue=deque(), destroyed=[], floor_reads=0)
    r.now = lambda: r.t
    r.elapsed = lambda: r.t - r.act_t0
    r.send = lambda *a, **k: None
    r.event = r.events.append
    r.get_logger = lambda: NS(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    r.create_subscription = lambda *a, **k: object()
    r.destroy_subscription = r.destroyed.append
    r.tf_buffer = NS(lookup_transform=lambda *a: (_ for _ in ()).throw(
        explorer.TransformException('none')))
    r.depth_image = lambda msg: np.ones((4, 4), np.uint16)
    r.stair_params = NS(body_height=0.3)

    def floor(*a):
        verdict, high = frames[min(r.floor_reads, len(frames) - 1)]
        r.floor_reads += 1
        r.depth_high = high
        return (verdict, 80) if high is not None else (None, 20)
    r.terrain_from_depth = floor
    return r


def run_check(r, monkeypatch, stairs_there):
    detect_calls = []

    def detect(*a):
        detect_calls.append(1)
        return __import__('types').SimpleNamespace(
            floor_z=0.0, summary=lambda: 'stairs') if stairs_there else None
    detect.last_why = 'no treads'
    monkeypatch.setattr(explorer.stair_levels, 'detect', detect)
    monkeypatch.setattr(explorer.stair_levels, 'Confirm', lambda: __import__('types').SimpleNamespace(
        add=lambda now, seen: seen if seen is not None and len(detect_calls) >= 3 else None))
    r.act_mem.clear()
    for i in range(80):
        if 'subs' in r.act_mem:
            r.act_mem['frame'], r.act_mem['k'] = object(), (1.0, 1.0, 1.0, 1.0)
        if explorer.Explorer.do_stair_check(r, (0, 0), 0, True):
            return i, len(detect_calls)
        r.t += 0.1
    raise AssertionError('the check did not end')


def test_flat_floor_read_first_means_the_stair_detector_never_runs(monkeypatch):
    r = gate_robot([(False, 0)])
    ticks, detects = run_check(r, monkeypatch, stairs_there=False)
    assert detects == 0 and ticks <= 10                   # about 1 s, not 4.5
    assert r.stairs_checked[((0, 0), 0)] is False and 'floor read first' in r.events[0]
    assert r.rough_checked[((0, 0), 0)] is False


def test_uneven_but_low_floor_closes_depth_at_once(monkeypatch):
    r = gate_robot([(True, 2)])
    ticks, detects = run_check(r, monkeypatch, stairs_there=False)
    assert detects == 0 and ticks <= 5
    assert r.stairs_checked[((0, 0), 0)] is False and r.rough_known == {(1, 0)}


def test_something_stair_high_gets_the_stair_detector(monkeypatch):
    r = gate_robot([(True, 30)])
    ticks, detects = run_check(r, monkeypatch, stairs_there=True)
    assert detects >= 3 and r.stairs_checked[((0, 0), 0)] is True
    assert r.floor_reads == 3                             # the floor was read first, 3 frames


def test_no_usable_floor_reading_still_gets_the_stair_detector(monkeypatch):
    r = gate_robot([(None, None)])                        # close up: the stairs hide the floor
    ticks, detects = run_check(r, monkeypatch, stairs_there=True)
    assert detects >= 3 and r.stairs_checked[((0, 0), 0)] is True


def test_stairs_asked_for_but_not_there_ends_as_before(monkeypatch):
    r = gate_robot([(False, 12)])                         # a prop: high, but no treads
    ticks, detects = run_check(r, monkeypatch, stairs_there=False)
    assert detects == 8 * 3 and r.stairs_checked[((0, 0), 0)] is False   # 3 heights tried a frame
    assert 'not stairs (0 of 8 frames' in r.events[0]


def test_curtain_camera_is_off_while_depth_is_open():
    from maze_explore.curtain_node import watching
    job = 'RUN cell=(1, 0) job=[visit cell (1, 0) side 0 via 0 cells] '
    assert watching(job + 'act=drive ')
    assert not watching(job + 'act=stair_check ')
