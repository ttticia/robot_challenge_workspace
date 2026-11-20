"""A dead end visited by the junction follow-up scans its side walls in Pose mode (user
2026-11-20: "robot not going into pose mode at room scanning"), explorer side, no ROS.

python3 -m pytest -q test/test_junction_dead_end_look.py"""
import math
from types import SimpleNamespace

import numpy as np

import maze_nav.explorer as explorer_mod
from maze_nav.core import Grid
from maze_nav.explorer import Explorer
from maze_nav.scene_understanding.scene_cells import CellGrid


class Fake:
    for _n in ('plan_after_junction', 'after_junction_tick', 'next_after_junction',
               'look_tick', 'at_dead_end', 'pause_junction', 'pose_side_wall', 'use_pose',
               'send_pose', 'release_pose', 'set_state'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self, pose_status='READY'):
        self.cfg = {'junction_trace_cells': 4, 'junction_dead_end_hold': 0.0,
                    'dead_end_side_hold': 0.0, 'dead_end_look_angle': math.radians(60),
                    'pose_max_yaw': math.radians(20), 'turn_margin': 0.03,
                    'junction_follow': True}
        self.state, self.logs, self.states = 'JUNCTION', [], []
        self.log = SimpleNamespace(info=self.logs.append, warn=self.logs.append)
        self.status_pub = SimpleNamespace(publish=lambda m: self.states.append(m.data))
        self.poses = []
        self.pose_pub = SimpleNamespace(publish=lambda m: self.poses.append(m.data))
        self.pose_status, self.pose_live, self.pose = pose_status, False, None
        self.spins = []
        self.spin_busy, self.blacklist, self.map = False, [], object()
        self.box, self.pivot = (0.37, 0.27, 0.23), (0.0, 0.0)
        self.xy, self.yaw = (0.0, 1.0), math.pi        # cell before the dead end, facing -x
        self.junction, self.paused_junction, self.curtain_left_at = None, None, None

    def get_logger(self):
        return self.log

    def grid_now(self):                       # all free: no wall limits the looks
        return Grid(np.zeros((80, 80), np.int16), 0.05, (-2.0, -1.0))

    def robot_pose(self):
        return self.xy, self.yaw

    def back_to_start(self, p):
        return False

    def send_spin(self, angle, why=''):
        self.spins.append((angle, why))


def plan(e, monkeypatch):
    """Junction at cell (1, 1): way -i is a dead end at (0, 1), way +j unknown."""
    def trace(openings, cell, d, max_cells=10):
        if d == (-1, 0):
            return {'kind': 'dead_end', 'cell': (0, 1), 'before': (1, 1), 'dir': (-1, 0),
                    'length': 1}
        return {'kind': 'unknown', 'cell': (1, 2), 'length': 1}
    monkeypatch.setattr(explorer_mod, 'trace_way', trace)
    rec = {'cells': CellGrid(), 'cell': (1, 1), 'from_cell': (1, 0), 'half_width': 0.35,
           'paths': [('-i', math.pi), ('+j', math.pi / 2)]}
    return e.plan_after_junction(rec)


def test_plan_adds_a_side_wall_look_after_the_hold(monkeypatch):
    todo = plan(Fake(), monkeypatch)
    kinds = [a['do'] for a in todo]
    assert kinds == ['goto', 'align', 'hold', 'look', 'goto']
    look = todo[3]
    assert look['group'] == 'dead end [0, 1]'
    names = [s['name'] for s in look['steps']]
    assert names == ['dead end [0, 1] left wall', 'dead end [0, 1] right wall']
    # looking along -x into (0, 1), the left wall is on the -y side
    left, right = (s['point'] for s in look['steps'])
    assert left[1] < 1.0 < right[1]
    assert all(abs(s['facing'] - math.pi) < 1e-9 for s in look['steps'])


def test_look_runs_in_pose_mode_and_keeps_perception_on(monkeypatch):
    e = Fake()
    todo = plan(e, monkeypatch)
    look = todo[3]
    e.junction = Explorer.new_look([])
    e.junction['rec'] = {}                    # a junction look (plan already made)
    e.junction['todo'] = [look, todo[4]]
    seen_on = []
    for _ in range(20):
        e.look_tick()
        seen_on.append(e.at_dead_end())
        e.spin_busy = False
        if e.junction['todo'][0] is not look:
            break
    assert len(e.poses) >= 2                  # left, then right body yaw (radians)
    assert e.poses[0] > 0 > e.poses[1]
    assert abs(e.poses[0]) <= math.radians(20) + 1e-6
    assert e.spins == []                      # no Nav2 spin for the side walls
    assert all(seen_on[:-1])                  # perception on while looking
    assert e.junction['todo'][0] is todo[4]   # then on to the rightmost way


def test_without_pose_mode_the_side_walls_are_spins(monkeypatch):
    e = Fake(pose_status='UNAVAILABLE')
    todo = plan(e, monkeypatch)
    e.junction = Explorer.new_look([])
    e.junction['rec'] = {}                    # a junction look (plan already made)
    e.junction['todo'] = [todo[3]]
    for _ in range(20):
        e.look_tick()
        e.spin_busy = False
        if not e.junction or not e.junction.get('todo'):
            break
    assert e.poses == []
    assert len(e.spins) == 2 and e.spins[0][0] > 0 > e.spins[1][0]


def test_pause_mid_look_resumes_without_repeating_it(monkeypatch):
    e = Fake()
    todo = plan(e, monkeypatch)
    look = todo[3]
    e.junction = Explorer.new_look([])
    e.junction['rec'] = {}                    # a junction look (plan already made)
    e.junction['todo'] = [look]
    e.look_tick()                             # steps added
    e.look_tick()                             # left wall pose sent
    assert len(e.junction['steps']) == 2
    e.pause_junction()
    e.junction = e.paused_junction
    for _ in range(20):
        e.look_tick()
        e.spin_busy = False
        if not e.junction.get('todo'):
            break
    assert len(e.junction['steps']) == 2      # not added twice
    assert e.junction['todo'] == []
