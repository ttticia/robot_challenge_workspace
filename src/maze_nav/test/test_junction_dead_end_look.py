"""Dead-end / room look actions (goto, align, hold, side-wall look) scan the side walls in
Pose mode (user 2026-11-20: "robot not going into pose mode at room scanning"); after a
junction scan the robot leaves by the nearest unexplored way and dead-end ways are only
not taken; the one the scan viewed waits for scene understanding instead of being
blacklisted (user 2026-11-21 / 11-22). Explorer side, no ROS.

python3 -m pytest -q test/test_junction_dead_end_look.py"""
import json
import math
from types import SimpleNamespace

import numpy as np

import maze_nav.explorer as explorer_mod
from maze_nav.core import Grid
from maze_nav.explorer import Explorer
from maze_nav.scene_cells import CellGrid, side_walls


class Fake:
    for _n in ('plan_after_junction', 'after_junction_tick', 'next_after_junction',
               'look_tick', 'at_dead_end', 'pause_junction', 'pose_side_wall', 'use_pose',
               'send_pose', 'release_pose', 'set_state', 'capture_view', 'tilt_pitch',
               'pose_tick', 'junction_pending', 'start_queue', 'mark_explored',
               'hold_dead_end', 'blacklist_dead_end', 'launch_cell', 'scene_candidate',
               'no_go_cell'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self, pose_status='READY'):
        self.cfg = {'junction_trace_cells': 4, 'junction_dead_end_hold': 0.0,
                    'dead_end_side_hold': 0.0, 'dead_end_look_angle': math.radians(60),
                    'pose_max_yaw': math.radians(20), 'turn_margin': 0.03,
                    'pose_axis_limit': 1.0, 'pose_margin': 0.03, 'pose_overshoot': 0.0,
                    'junction_follow': True, 'tilt_comp': True,
                    'tilt_threshold': math.radians(30), 'pose_max_pitch': math.radians(20),
                    'camera_frame': 'camera_link', 'pose_turn_time': 0.0, 'pose_wait': 6.0}
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
        self.captures, self.pitches = [], []
        self.scene_capture_pub = SimpleNamespace(
            publish=lambda m: self.captures.append(json.loads(m.data)))
        self.pose_pitch_pub = None                     # no tilt compensation unless set
        self.tilt = (0.0, 0.0)
        self.dead_end_cells = set()
        self.cells = CellGrid()
        self.visited, self.explored_cells, self.blocked_cells = set(), set(), {}
        self.rooms_done, self.room_looks = set(), {}
        self.cand_queue, self.queue_active = [], False
        self.scene_pending, self.scanned_cells, self.scene_results = {}, set(), True
        self.exit_side = None

    def ros_ns(self):
        return 10 * 10**9

    def body_tilt(self):
        return self.tilt

    def tf_pitch(self, frame):
        return None

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


def plan_after(e, monkeypatch):
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


def plan(e, monkeypatch=None):
    """The look actions for dead end (0, 1) seen from cell (1, 1) facing -x (the shape a
    room look-in uses too): goto, align, hold facing in, side-wall look, then a goto on."""
    cells, dead, into = CellGrid(), (0, 1), (-1, 0)
    face, group = cells.heading(into), 'dead end [0, 1]'
    hold = {'do': 'hold', 's': float(e.cfg['junction_dead_end_hold']), 'group': group,
            'name': f'{group}: facing in', 'view': 'centre'}
    look = {'do': 'look', 'group': group, 'name': f'{group}: side walls',
            'steps': [{'name': f'{group} {k} wall', 'point': xy, 'facing': face, 'view': k,
                       'hold': e.cfg['dead_end_side_hold'], 'short': True}
                      for k, xy in side_walls(cells, dead, into).items()]}
    explorer_mod.dead_end_scene([hold] + look['steps'], dead, cells.centre(dead))
    return [{'do': 'goto', 'xy': cells.centre((1, 1)), 'yaw': face, 'group': group,
             'name': 'to cell [1, 1]'},
            {'do': 'align', 'heading': face, 'group': group, 'tries': 0,
             'name': f'{group}: squaring up facing in'},
            hold, look,
            {'do': 'goto', 'xy': cells.centre((1, 2)), 'yaw': math.pi / 2, 'group': 'on',
             'name': 'on'}]


def test_after_a_junction_scan_leave_by_the_nearest_unexplored_way(monkeypatch):
    e = Fake()
    e.xy, e.yaw = (1.0, 1.0), math.pi
    todo = plan_after(e, monkeypatch)
    assert [(a['do'], a['group']) for a in todo] == [('goto', 'leave')]
    assert todo[0]['xy'] == (1.0, 2.0)                 # way +j; -i is a dead end
    assert (0, 1) not in e.dead_end_cells              # not taken, but not blacklisted


def test_after_a_junction_scan_queued_candidates_come_first(monkeypatch):
    e = Fake()
    e.cand_queue = [{'kind': 'stairs', 'id': 1, 'target': (1.0, 3.0), 'heading': 0.0}]
    assert plan_after(e, monkeypatch) == [] and e.queue_active


def test_side_wall_look_actions(monkeypatch):
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


def run(e, todo, ticks=40):
    """Drive the follow-up's actions, the pose looks finishing at once (bridge POSING)."""
    e.junction = Explorer.new_look([])
    e.junction['rec'] = {}
    e.junction['todo'] = todo
    for _ in range(ticks):
        e.look_tick()
        if e.pose is not None:
            e.pose['since'] = -1e9                     # held long enough
            e.pose_status = 'POSING'
            e.pose_tick()
        e.spin_busy = False
        if not e.junction or not e.junction.get('todo'):
            break


def test_dead_end_views_are_captured_once_each_last_flagged(monkeypatch):
    e = Fake()
    e.cfg['dead_end_side_hold'] = 1.0
    todo = plan(e, monkeypatch)
    run(e, todo[2:4])                                 # hold facing in, then the side walls
    views = [(c['cell'], c['view'], c['last']) for c in e.captures]
    assert views == [([0, 1], 'centre', False), ([0, 1], 'left', False),
                     ([0, 1], 'right', True)]
    assert all(c['cell_centre'] == [0.0, 1.0] for c in e.captures)
    assert float(e.captures[1]['stamp']) - float(e.captures[1]['hold_start']) == 1.0
    assert e.captures[0]['pitch_cmd_deg'] is None


def test_scanned_dead_end_cell_waits_for_scene_understanding(monkeypatch):
    e = Fake()
    e.scanned_cells = {(0, 1)}                        # the junction scan viewed it
    plan_after(e, monkeypatch)
    assert (0, 1) not in e.dead_end_cells             # not blacklisted (user 2026-11-22)
    assert e.no_go_cell((0, 1))                       # but no frontier goal into it yet
    assert e.scene_pending[(0, 1)]['captured'] is not None   # result timeout running


def test_tilt_over_threshold_pitches_every_dead_end_view(monkeypatch):
    e = Fake()
    e.pose_pitch_pub = SimpleNamespace(publish=lambda m: e.pitches.append(m.data))
    e.tilt = (math.radians(-34), 0.0)                 # nose 34 deg up on a slope
    todo = plan(e, monkeypatch)
    run(e, todo[2:4])
    assert len(e.captures) == 3
    assert e.pitches and all(abs(p - math.radians(20)) < 1e-9 for p in e.pitches)
    assert all(c['pitch_cmd_deg'] == 20.0 for c in e.captures)   # pose_max_pitch limit
    assert e.spins == []


def test_tilt_under_threshold_sends_no_pitch(monkeypatch):
    e = Fake()
    e.pose_pitch_pub = SimpleNamespace(publish=lambda m: e.pitches.append(m.data))
    e.tilt = (math.radians(12), 0.0)
    todo = plan(e, monkeypatch)
    run(e, todo[2:4])
    assert e.pitches == [] and len(e.captures) == 3


def test_quat_pitch_is_nose_down_positive():
    q = SimpleNamespace(x=0.0, y=math.sin(0.1), z=0.0, w=math.cos(0.1))   # +0.2 rad about y
    pitch, roll = explorer_mod.quat_pitch_roll(q)
    assert abs(pitch - 0.2) < 1e-9 and abs(roll) < 1e-9


def dead_end_look(e):
    """A DEAD_END look as on_dead_end builds it: facing into (0, 1) from (1, 1), along -x."""
    sides = [{'name': f'dead end {k} wall', 'point': xy, 'facing': math.pi, 'view': k,
              'hold': 1.0, 'short': True} for k, xy in (('left', (0.0, 0.5)),
                                                         ('right', (0.0, 1.5)))]
    e.state = 'DEAD_END'
    e.junction = Explorer.new_look(explorer_mod.dead_end_scene(
        [{'name': 'dead end', 'hold': 0.5}] + sides, (0, 1), (0.0, 1.0)))
    for _ in range(40):
        e.look_tick()
        if e.pose is not None:
            e.pose['since'] = -1e9
            e.pose_status = 'POSING'
            e.pose_tick()
        e.spin_busy = False
        if e.junction is None or e.junction['hold_until'] is not None:
            if e.junction is not None:
                e.junction['hold_until'] = 0.0             # holds over at once
        if e.junction is None:
            break


def test_flat_dead_end_holds_plain_then_poses_side_walls():
    e = Fake()
    dead_end_look(e)
    assert [c['view'] for c in e.captures] == ['centre', 'left', 'right']
    assert len(e.poses) >= 2 and e.pitches == []


def test_tilted_dead_end_centre_view_is_a_pitched_pose_look():
    e = Fake()
    e.pose_pitch_pub = SimpleNamespace(publish=lambda m: e.pitches.append(m.data))
    e.tilt = (math.radians(33), math.radians(4))   # nose 33 deg down
    dead_end_look(e)
    assert [c['view'] for c in e.captures] == ['centre', 'left', 'right']
    assert e.poses[0] == 0.0                       # centre: no body yaw, only pitch
    assert all(abs(p + math.radians(20)) < 1e-9 for p in e.pitches)   # nose up, clamped


def test_pose_yaw_never_asks_more_than_the_axis_limit():
    """pose_axis_limit 0.8: the bridge never sends more than 80 % of the yaw axis, so the
    explorer asks for at most pose_max_yaw x 0.8 (motor buzzing at full axis, 11-21 10:56)."""
    e = Fake()
    e.cfg['pose_axis_limit'] = 0.8
    dead_end_look(e)
    yaws = [abs(p) for p in e.poses if not math.isnan(p) and p != 0.0]
    assert yaws and max(yaws) <= math.radians(16) + 1e-9


class TightFake(Fake):
    """A wall row 7 cm beside the body box at the stance (y 1.30-1.35 m, robot at y 1.0)."""

    def grid_now(self):
        data = np.zeros((80, 80), np.int16)
        data[46, :] = 100
        return Grid(data, 0.05, (-2.0, -1.0))


def test_no_pose_yaw_when_the_body_is_already_near_a_wall():
    from maze_nav.core import footprint_hits
    e = TightFake()
    e.cfg['pose_margin'] = 0.08
    g = e.grid_now()
    assert not footprint_hits(g, e.xy, e.yaw, e.box, 0.03)      # the old check let it yaw
    assert footprint_hits(g, e.xy, e.yaw, e.box, 0.08)
    dead_end_look(e)
    assert all(math.isnan(p) or p == 0.0 for p in e.poses)      # no body yaw at all
    assert any('within 8 cm of a wall' in str(m) for m in e.logs)
    assert [c['view'] for c in e.captures] == ['centre', 'left', 'right']   # views still held


def test_bridge_clamps_the_yaw_axis():
    from maze_nav.cmd_vel_bridge import AXIS_MAX, CmdVelBridge
    b = SimpleNamespace(pose_max_yaw=math.radians(16), pose_axis_max=int(AXIS_MAX * 0.8))
    axis = lambda y: CmdVelBridge.pose_axis(b, y)               # noqa: E731
    assert axis(math.radians(40)) == -int(AXIS_MAX * 0.8)       # + = left -> negative axis
    assert axis(math.radians(-40)) == int(AXIS_MAX * 0.8)
    assert axis(math.radians(8)) == -round(AXIS_MAX / 2)


class CentreFake(Fake):
    centre_tick = Explorer.centre_tick

    def __init__(self):
        super().__init__()
        self.cfg.update({'centre_tol': 0.03, 'centre_max': 0.35, 'centre_speed': 0.06,
                         'centre_timeout': 10.0})
        self.cmds = []
        self.cmd_pub = SimpleNamespace(publish=self.cmds.append)

    def stop_robot(self):
        self.cmds.append('stop')


def test_junction_centring_moves_sideways_onto_the_centre_line():
    e = CentreFake()
    e.xy, e.yaw = (1.0, 1.2), 0.0                      # facing +x, 0.2 m left of the line
    st = {'name': 'junction way +i', 'centre': (1.0, 1.0)}
    assert not e.centre_tick(st, 0.0)                  # plan the move
    assert not e.centre_tick(st, 0.1)
    assert e.cmds[-1].linear.y < 0 and e.cmds[-1].linear.x == 0.0   # to the right
    e.xy = (1.0, 1.01)
    assert e.centre_tick(st, 0.2) and st['centred'] and e.cmds[-1] == 'stop'


def test_junction_centring_skipped_when_already_centred():
    e = CentreFake()
    e.xy, e.yaw = (1.0, 1.01), math.pi / 2             # facing +y, 1 cm off: nothing to do
    st = {'name': 'junction way +j', 'centre': (1.0, 1.0)}
    assert e.centre_tick(st, 0.0) and e.cmds == []
