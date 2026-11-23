"""Dead-end / room side walls from the centre of the cell before it: 25 deg each way by
physical turns (default, user 2026-11-23), or Pose-mode yaw with side_pose (spin fallback).
Junction ways go to scene understanding only into a dead end (user 2026-11-23). No ROS.

python3 -m pytest -q test/test_side_turns.py"""
import math

import numpy as np
import pytest

import maze_nav.explorer as explorer_mod
from maze_nav.core import Grid
from maze_nav.explorer import Explorer
from test_junction_dead_end_look import Fake as LookFake

TURN = math.radians(25)


class Fake(LookFake):
    for _n in ('side_steps', 'dead_end_stance', 'dead_end_look', 'junction_side_views',
               'note_scanned', 'junction_way_dead_end'):
        locals()[_n] = getattr(Explorer, _n)
    new_look = staticmethod(Explorer.new_look)

    def __init__(self, pose_status='READY', side_pose=False):
        super().__init__(pose_status)
        self.cfg.update(side_turn=TURN, side_pose=side_pose, dead_end_stance=True,
                        dead_end_stance_tol=0.12, dead_end_hold=0.0,
                        junction_align_tol=math.radians(3), goal_timeout=60.0,
                        scene_look_grace=30.0)
        self.goals = []
        self.walls = None

    def send_spin(self, angle, why=''):           # the turn happens at once
        self.spins.append((angle, why))
        self.yaw = explorer_mod.wrap(self.yaw + angle)

    def send_junction_goal(self, act):
        self.goals.append(act['xy'])

    def cancel_junction_goal(self):
        pass

    def false_success(self, goal, xy):
        return False

    def grid_now(self):
        return self.walls if self.walls is not None else super().grid_now()


def walled_dead_end():
    """Map with cell (0, 1) walled on its -x, -y and +y sides (open towards (1, 1))."""
    g = Grid(np.zeros((80, 80), np.int16), 0.05, (-2.0, -1.0))
    for t in np.arange(-0.5, 0.5001, 0.025):
        for x, y in ((-0.5, 1.0 + t), (t, 0.5), (t, 1.5)):
            c = g.cell((x, y))
            g.data[c[1], c[0]] = 100
    return g


def run(e, n=60):
    """look_tick until the look ends; Pose looks finish at once (bridge POSING)."""
    for _ in range(n):
        if e.junction is None:
            break
        j = e.junction
        if j.get('todo') and j['todo'][0]['do'] == 'goto' and 'sent' in j['todo'][0]:
            j['todo'][0]['status'] = explorer_mod.GoalStatus.STATUS_SUCCEEDED
            e.xy = j['todo'][0]['xy']
        e.look_tick()
        if e.pose is not None:
            e.pose['since'] = -1e9                 # held long enough
            e.pose_status = 'POSING'
            e.pose_tick()


def body_yaws(e):
    return [round(math.degrees(p)) for p in e.poses if not math.isnan(p)]


def test_side_steps_pose_yaw_turn_or_fall_back_to_the_wall_points():
    e = Fake(side_pose=True)
    steps = e.side_steps('dead end', math.pi)
    assert [s['view'] for s in steps] == ['left', 'right']
    assert steps[0]['look'] == pytest.approx(explorer_mod.wrap(math.pi + TURN))
    assert steps[1]['look'] == pytest.approx(explorer_mod.wrap(math.pi - TURN))
    assert all(s['short'] and 'point' not in s and 'heading' not in s for s in steps)
    e.cfg['side_pose'] = False
    turns = e.side_steps('dead end', math.pi)
    assert turns[0]['heading'] == pytest.approx(explorer_mod.wrap(math.pi + TURN))
    assert all('look' not in s for s in turns)
    e.cfg['side_turn'] = 0.0
    old = e.side_steps('dead end', math.pi, [('left', (0.0, 1.5)), ('right', (0.0, 0.5))])
    assert [s['point'] for s in old] == [(0.0, 1.5), (0.0, 0.5)]


def dead_end_from_short_of_the_stance(e):
    """Dead end (0, 1) entered along -x from cell (1, 1); the trigger fired with the robot
    0.4 m short of that cell's centre and 10 deg off."""
    e.state = 'DEAD_END'
    centre, facing = (0.0, 1.0), math.pi
    e.xy, e.yaw = (1.4, 1.0), math.pi - math.radians(10)
    steps = explorer_mod.dead_end_scene(
        [{'name': 'dead end', 'hold': 0.0}] + e.side_steps('dead end', facing), (0, 1), centre)
    e.junction = e.dead_end_look(steps, centre, facing, 'dead end at cell [0, 1]')
    assert [a['do'] for a in e.junction['todo']] == ['goto', 'align', 'look']
    assert e.junction['todo'][0]['xy'] == pytest.approx((1.0, 1.0))      # cell (1, 1)
    run(e)
    assert e.goals == [pytest.approx((1.0, 1.0))]
    views = [c['view'] for c in e.captures]
    assert views == ['centre', 'left', 'right'] and e.captures[-1]['last']
    assert e.junction is None and e.states[-1].startswith('EXPLORING')


def test_dead_end_look_with_side_pose_yaws_the_body_left_and_right():
    e = Fake(side_pose=True)
    dead_end_from_short_of_the_stance(e)
    assert [round(math.degrees(a)) for a, _ in e.spins] == [10]   # only squaring up
    assert body_yaws(e) == [20, -20]               # 25 deg asked, Fake's Pose limit 20
    assert math.isnan(e.poses[-1])                 # Pose mode released at the end


def test_dead_end_look_without_pose_mode_spins_25_deg_each_way():
    e = Fake(pose_status='UNAVAILABLE (sim)', side_pose=True)
    dead_end_from_short_of_the_stance(e)
    assert [round(math.degrees(a)) for a, _ in e.spins] == [10, 25, -50]
    assert e.poses == []


def test_dead_end_look_goes_to_the_cell_centre_then_turns_25_deg_left_and_right():
    e = Fake()                                     # default: physical turns
    dead_end_from_short_of_the_stance(e)
    assert [round(math.degrees(a)) for a, _ in e.spins] == [10, 25, -50]
    assert body_yaws(e) == []


def test_dead_end_look_from_the_centre_already_skips_the_goto():
    e = Fake()
    e.xy, e.yaw = (1.05, 1.0), math.pi
    j = e.dead_end_look([], (0.0, 1.0), math.pi, 'dead end')
    assert [a['do'] for a in j['todo']] == ['align', 'look']
    e.cfg['dead_end_stance'] = False
    assert 'todo' not in e.dead_end_look([], (0.0, 1.0), math.pi, 'dead end')


def junction_ways():
    way = {'name': 'junction way -i', 'heading': math.pi, 'short': True, 'hold': 0.0,
           'scan': True, 'cell': (0, 1), 'centre': None,
           'scene': {'cell': [0, 1], 'cell_centre': [0.0, 1.0], 'kind': 'junction',
                     'view': '-i', 'last': True}}
    nxt = {'name': 'junction way +j', 'heading': math.pi / 2, 'short': False, 'hold': 0.0,
           'scan': True, 'cell': (1, 2), 'centre': None,
           'scene': {'cell': [1, 2], 'cell_centre': [1.0, 2.0], 'kind': 'junction',
                     'view': '+j', 'last': True}}
    return way, nxt


def junction_look(e, steps):
    e.xy, e.yaw = (1.0, 1.0), math.pi
    e.junction = e.new_look(steps)
    e.junction['rec'] = {'cell': (1, 1)}
    e.junction['todo'] = []
    e.junction['i'], e.junction['turned'] = 1, True          # holding at the first way


@pytest.mark.parametrize('side_pose', [False, True])
def test_junction_scan_facing_a_dead_end_adds_its_side_looks(side_pose):
    """Junction at cell (1, 1), robot at its centre facing way -i: scene_trigger reports
    the dead end (0, 1) there; 25 deg turns (or Pose yaws) after this way's hold, as one
    room record. Way +j is not a dead end: scanned, but no scene capture."""
    e = Fake(side_pose=side_pose)
    way, nxt = junction_ways()
    junction_look(e, [way, nxt])
    assert e.junction_side_views((0, 1), (0.0, 1.0), math.pi, (0, 1), 'dead end')
    assert way['scene']['kind'] == 'dead_end' and not way['scene']['last']
    assert way['scene']['view'] == 'centre'
    run(e)
    turns = [round(math.degrees(a)) for a, _ in e.spins]
    if side_pose:
        assert body_yaws(e) == [20, -20]           # the dead end's side walls
        assert turns == [-90]                      # then on to way +j (feet still at -i)
    else:
        assert body_yaws(e) == []
        assert turns == [25, -50, -65]             # side walls, then way +j from -i - 25
    views = [(c['cell'], c['view'], c['last']) for c in e.captures]
    assert views == [([0, 1], 'centre', False), ([0, 1], 'left', False),
                     ([0, 1], 'right', True)]
    assert e.scanned_cells == {(0, 1)}             # +j: no capture, room look-in later
    # not facing that dead end (another way's step): nothing added
    e.junction = e.new_look([nxt])
    e.junction['i'] = 1
    assert not e.junction_side_views((0, 1), (0.0, 1.0), math.pi, (0, 1), 'dead end')


def test_junction_way_into_open_cells_sends_no_scene_capture():
    e = Fake()
    way, nxt = junction_ways()
    junction_look(e, [way, nxt])
    run(e)
    assert e.captures == [] and e.scanned_cells == set()
    assert any('not a dead end: no scene capture' in str(m) for m in e.logs)


def test_junction_way_into_a_dead_end_on_the_map_gets_its_views_without_the_trigger():
    """scene_trigger stayed quiet (reported earlier / late), but the map shows (0, 1)
    walled on its other three sides: the view goes out as the dead end's centre view, its
    side walls are looked at, and the cell waits for scene understanding."""
    e = Fake()
    e.walls = walled_dead_end()
    way, nxt = junction_ways()
    junction_look(e, [way, nxt])
    run(e)
    views = [(c['cell'], c['kind'], c['view'], c['last']) for c in e.captures]
    assert views == [([0, 1], 'dead_end', 'centre', False), ([0, 1], 'dead_end', 'left', False),
                     ([0, 1], 'dead_end', 'right', True)]
    assert all(c['cell_centre'] == [0.0, 1.0] for c in e.captures)
    assert (0, 1) in e.scene_pending and e.scene_pending[(0, 1)]['captured'] is not None
    assert e.scanned_cells == {(0, 1)}
