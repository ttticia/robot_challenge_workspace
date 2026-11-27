"""22:15 run, 2026-11-26 night: after the AI leave sequence a 143 deg turn did not move
the robot by a degree in 12 s and the run was stopped by hand. The explorer's own do_turn
on a stand-in that ignores turn commands until the modes are sent again."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from maze_explore.grid import Grid, wrap

explorer = pytest.importorskip('maze_explore.explorer')
from test_qr_approach import Robot as Base                      # noqa: E402

E = explorer.Explorer


class Robot(Base):
    do_turn = E.do_turn
    do_hold = E.do_hold

    def __init__(self, deaf=True):
        super().__init__()
        self.cmd_gap, self.cur_gait, self.codes = 1.5, 'normal', []
        self.turn_kick_after, self.turn_kick_gap = 3.0, 60.0
        self.deaf = deaf                     # the robot ignores /cmd_vel until modes are resent

    def do_cmd(self, code):
        self.codes.append(code)
        self.deaf = False
        return True

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        super().send(*((0.0, 0.0, 0.0) if self.deaf else (vx, vy, wz)))


def test_a_robot_that_does_not_turn_gets_its_modes_sent_again():
    r = Robot(deaf=True)
    r.queue.append(('turn', math.radians(-143), 3.0))
    assert r.run(40.0)
    assert any('not turning after' in e for e in r.events)
    assert r.codes == list(explorer.robot.start_sequence('normal'))
    assert abs(wrap(r.pose[2] - math.radians(-143))) < math.radians(4)   # turned after all


def test_a_robot_that_turns_is_left_alone():
    r = Robot(deaf=False)
    r.queue.append(('turn', math.radians(90), 3.0))
    assert r.run(20.0)
    assert not r.codes and not any('not turning' in e for e in r.events)


def test_only_once_per_gap_so_a_robot_that_cannot_turn_still_times_out():
    r = Robot(deaf=True)
    r.do_cmd = lambda code: (r.codes.append(code), True)[1]           # stays deaf
    r.queue.append(('turn', math.radians(90), 3.0))
    assert r.run(60.0)
    assert len(r.codes) == len(explorer.robot.start_sequence('normal'))  # one resend only


def test_the_turn_clock_stands_still_during_a_pose_jump_hold():
    import inspect
    src = inspect.getsource(E.tick)
    i = src.index("self.why = 'pose jumped: standing until it is still, then realigning'")
    assert 'self.act_t0 += 0.1' in src[i:i + 300]


def test_no_curtain_sightings_while_the_pose_cannot_be_trusted():
    # 22:15 run: one steady false box at +23 deg was pinned on three different edges as
    # the pose jumped during and after the stairs
    from collections import deque
    msg = NS(data=json.dumps({'detections': [{'conf': 0.85, 'box': [0.3, 0.2, 0.7, 0.9],
                                              'bearing': 0.4}]}))
    for held in ({'ai_sequence': True}, {'pose_held': 1.0}, {'odom_hold_until': 99.0},
                 {'pose_trust_from': 101.0}):
        r = NS(cv_frames=deque(), cv_conf=0.6, state='RUN', grid=Grid(0, 0, 0, 1.02),
               pose=(0.0, 0.0, 0.0), occ=object(), act=None, ai_walking=False,
               ai_sequence=False, pose_held=None, odom_hold_until=0.0, map_blind_until=0.0,
               pose_trust_from=0.0, cv_seen={}, now=lambda: 100.0)
        for k, v in held.items():
            setattr(r, k, v)
        r.sense = lambda: (_ for _ in ()).throw(AssertionError('sighting was processed'))
        E.on_curtain_seen(r, msg)
        assert len(r.cv_frames) == 1 and not r.cv_seen                # counted, not pinned
