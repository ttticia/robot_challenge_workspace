"""The QR approach of the explorer (user, 2026-11-25), driven without ROS: the explorer's
own action methods run on a stand-in that moves exactly as commanded."""
import math
from collections import deque

import numpy as np
import pytest

from maze_explore.grid import Grid, nb, wrap

explorer = pytest.importorskip('maze_explore.explorer')      # needs the ROS environment

USED = ('body_centre', 'do_qr_wait', 'do_qr_turn', 'do_qr_step', 'do_qr_hold', 'do_qr_sweep', 'do_qr_back', 'do_turn',
        'do_event', 'qr_got', 'travel_blocked', 'front_blocked', 'cur_cell', 'elapsed')


class Robot:
    """Stand-in for the Explorer node: its fields, its real action methods, perfect motion."""

    class Log:
        def info(self, *a, **k): pass
        warn = info

    def __init__(self):
        self.grid = Grid(0.0, 0.0, 0.0, 1.02)
        self.pose = [0.0, 0.0, 0.0]
        self.t = 100.0
        self.queue, self.act, self.act_t0, self.act_mem = deque(), None, 0.0, {}
        self.cloud = np.zeros((0, 3))
        self.obstacles = np.zeros((0, 2))                 # map-frame points, knee high
        self.fp_front, self.fp_rear, self.fp_half_width = 0.27, 0.42, 0.23
        self.side_offset, self.qr_speed, self.qr_hold, self.qr_wait = 0.075, 0.08, 1.5, 8.0
        self.qr_angle = math.radians(75.0)
        self.qr_depth, self.qr_sweep_rate = 0.3, 0.25
        self.w_max, self.min_spin, self.turn_ff, self.pivot_snap = 0.5, 0.2, 0.03, 0.25
        self.turn_kp, self.pivot_kp, self.pivot_vx, self.pivot_vy = 1.5, 1.5, 0.12, 0.10
        self.ai_walking, self.block_memory, self.front_seen = False, 60.0, []
        self.pivot_offset, self.pivot_slack = 0.0, 0.03
        self.scene_results, self.qr_read_t, self.qr_t0 = {}, -1e9, 100.0
        self.step = ('stale',)
        self.events, self.cmd, self.backwards = [], (0.0, 0.0, 0.0), False
        self.trail = []

    def now(self): return self.t
    def get_logger(self): return self.Log()
    def event(self, text): self.events.append(text)

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        self.backwards |= vx < -1e-9          # asked for; the real send() clamps it to 0
        self.cmd = (max(0.0, vx), vy, wz)

    def run(self, seconds, read_when=None):
        for _ in range(int(seconds / 0.1)):
            x, y, yaw = self.pose
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = self.obstacles[:, 0] - x, self.obstacles[:, 1] - y
            self.cloud = np.stack([c * dx + s * dy, -s * dx + c * dy,
                                   np.full(len(dx), 0.4)], 1)
            if read_when is not None and self.qr_read_t < 0 and read_when(self):
                self.qr_read_t = self.t
            self.cmd = (0.0, 0.0, 0.0)
            for _ in range(20):
                if self.act is None:
                    if not self.queue:
                        return True
                    self.act, self.act_t0, self.act_mem = self.queue.popleft(), self.t, {}
                if not getattr(self, 'do_' + self.act[0])(*self.act[1:]):
                    break
                self.act = None
            vx, vy, wz = self.cmd
            self.pose = [x + (vx * c - vy * s) * 0.1, y + (vx * s + vy * c) * 0.1,
                         wrap(yaw + wz * 0.1)]
            self.trail.append((self.act[0] if self.act else '-', *self.pose))
            self.t += 0.1
        return False


for name in USED:
    setattr(Robot, name, getattr(explorer.Explorer, name))


def look(robot, view, qr=True):
    """The robot has just made its three views from cell (0, 0) through side 0."""
    robot.pose[2] = math.radians(-25.0)                    # the last view: right
    robot.scene_results[(1, 0)] = {'cell': [1, 0], 'qr': qr, 'qr_view': view}
    robot.queue.append(('qr_wait', (0, 0), 0))


def test_right_view_turns_75_right_and_goes_in_sideways_then_comes_back():
    r = Robot()
    look(r, 'right')
    assert r.run(120.0)
    assert r.step is None
    step = [p for p in r.trail if p[0] == 'qr_step']
    far = max(p[1] for p in step)
    assert far == pytest.approx(0.51 + 0.3, abs=0.04)       # 0.3 m into the room, no deeper
    assert far <= 0.51 + 0.3 + 0.03
    going_in = [p for p in step if p[1] < far - 0.05][:int(far / 0.008) // 2]
    assert all(abs(math.degrees(p[3]) + 75.0) < 6.0 for p in going_in)     # body kept at -75
    assert max(abs(p[2]) for p in step) < 0.12              # on the line through the opening
    assert math.hypot(r.pose[0], r.pose[1]) < 0.12          # back in the cell it looked from
    assert abs(math.degrees(r.pose[2]) + 90.0) < 6.0        # came out square, purely sideways
    sweep = [p for p in r.trail if p[0] == 'qr_sweep']      # not read: turned to the back wall
    assert sweep and abs(math.degrees(sweep[-1][3])) < 6.0
    assert max(abs(b[3] - a[3]) for a, b in zip(sweep, sweep[1:])) <= 0.025 + 1e-6   # slowly
    # on the spot: the pivot is the room centre (0.51 in), and the stand-in does not drift
    # back in a turn as the Lite3 does, so the forward term that cancels it shows as 4 cm
    assert max(p[1] for p in sweep) <= 0.51 + 0.3 + 0.07
    assert any('not read' in e for e in r.events)


def test_left_view_mirrors_it():
    r = Robot()
    look(r, 'left')
    assert r.run(120.0)
    assert any(p[0] == 'qr_step' and abs(math.degrees(p[3]) - 75.0) < 6.0 and p[1] > 0.5
               for p in r.trail)
    assert math.hypot(r.pose[0], r.pose[1]) < 0.12


def test_a_read_code_ends_the_move_and_the_robot_goes_back():
    r = Robot()
    look(r, 'right')
    assert r.run(120.0, read_when=lambda s: s.pose[0] > 0.65)
    assert max(p[1] for p in r.trail) < 0.72                # stopped where it was read
    assert math.hypot(r.pose[0], r.pose[1]) < 0.12
    assert any('code read' in e for e in r.events)


def test_a_code_read_during_the_turn_means_no_step_at_all():
    r = Robot()
    look(r, 'right')
    assert r.run(60.0, read_when=lambda s: math.degrees(s.pose[2]) < -50.0)
    assert not any(p[0] == 'qr_step' for p in r.trail)
    assert math.hypot(r.pose[0], r.pose[1]) < 0.05


def test_an_object_in_the_cell_ends_the_step_without_touching_it():
    r = Robot()
    ys = np.linspace(-0.4, 0.4, 30)
    r.obstacles = np.stack([np.full(30, 1.10), ys], 1)      # a prop across the cell
    look(r, 'right')
    assert r.run(120.0)
    far = max(p[1] for p in r.trail)
    assert 0.3 < far < 0.80                                 # nose side stays clear of x = 1.10
    assert math.hypot(r.pose[0], r.pose[1]) < 0.15   # 0.15: turns no longer pull the robot onto the cell centre (2026-11-26)


def test_no_code_or_a_code_ahead_or_no_answer_means_no_move():
    for view, qr in (('', False), ('ahead', True)):
        r = Robot()
        look(r, view, qr)
        assert r.run(5.0) and not any(p[0] == 'qr_step' for p in r.trail)
        assert math.hypot(r.pose[0], r.pose[1]) < 0.01
    r = Robot()
    r.queue.append(('qr_wait', (0, 0), 0))                  # the VLM never answers
    assert not r.run(7.0)
    assert r.run(2.0) and math.hypot(r.pose[0], r.pose[1]) < 0.01


def test_nothing_is_ever_driven_backwards():
    for view in ('left', 'right'):
        r = Robot()
        look(r, view)
        r.run(120.0)
        steps = [(a, b) for a, b in zip(r.trail, r.trail[1:]) if b[0].startswith('qr_step')]
        for a, b in steps:
            fwd = (b[1] - a[1]) * math.cos(a[3]) + (b[2] - a[2]) * math.sin(a[3])
            assert fwd > -1e-6


def test_a_code_read_while_turning_to_the_back_wall_ends_the_turn():
    r = Robot()
    look(r, 'right')
    assert r.run(120.0, read_when=lambda s: s.act and s.act[0] == 'qr_sweep'
                 and math.degrees(s.pose[2]) > -40.0)
    sweep = [p for p in r.trail if p[0] == 'qr_sweep']
    assert sweep and min(abs(math.degrees(p[3])) for p in sweep) > 30.0   # stopped part-way
    # it comes out up to side_offset (7 cm) beside the centre line, towards its nose: getting
    # back onto it would mean stepping backwards. Turns no longer re-centre (2026-11-26):
    # plan() walks the robot back onto the centre only when it is more than `recentre` off.
    assert math.hypot(r.pose[0], r.pose[1]) < 0.16 and any('code read' in e for e in r.events)


class Looks:
    """Stand-in for skip_looks(): rooms that need no look count as looked at."""
    for name in ('skip_looks', 'missions_done', 'on_report', 'room_empty'):
        locals()[name] = getattr(explorer.Explorer, name)

    def __init__(self, empty):
        self.kinds = {((0, 0), 0): 'room', ((0, 0), 1): 'object', ((0, 0), 2): 'free'}
        self.looked, self.events = set(), []
        self.missions_stop = self.empty_skip = self.empty_early = True
        self.missions_want, self.missions_seen = {'M01', 'M02', 'M03', 'M04', 'M05'}, set()
        self.empty_squares, self.tall = 4, (0, True) if empty else (9, True)

    def event(self, text): self.events.append(text)
    def raised(self, c, d): return False

    def sense(self):
        return type('S', (), {'tall_inside': lambda s, c, d: self.tall})()


def test_an_empty_room_gets_no_look_and_a_room_with_something_tall_does():
    r = Looks(empty=True)
    r.skip_looks()
    assert r.looked == {((0, 0), 0)}                  # the object cell is still to be looked at
    r = Looks(empty=False)
    r.skip_looks()
    assert r.looked == set()
    r = Looks(empty=True)
    r.raised = lambda c, d: True                      # something stair-high in there: not empty
    r.skip_looks()
    assert r.looked == set()


def test_no_more_looks_once_all_five_missions_are_reported():
    r = Looks(empty=False)
    def msg(t, ok=True):
        return type('M', (), {'data': '{"task_id": "%s", "ok": %s}' % (t, 'true' if ok else 'false')})()
    for task in ('M01', 'M02', 'M03', 'M04', 'M04'):
        r.on_report(msg(task))
    r.on_report(msg('M05', ok=False))                         # the server did not accept it
    r.skip_looks()
    assert r.looked == set() and not r.missions_done()        # M05 is still open
    r.on_report(msg('M05'))
    assert r.missions_done()
    r.on_report(msg('M02', ok=False))                         # a later M02 was marked wrong
    r.skip_looks()
    assert r.looked == set() and not r.missions_done()
    r.on_report(msg('M02'))
    r.skip_looks()
    assert r.looked == {((0, 0), 0), ((0, 0), 1)} and 'all successful' in r.events[-3]
