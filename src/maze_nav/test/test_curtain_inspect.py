"""curtain_inspect.InspectPlan: to the middle of the cell, square up, lie down / stand up,
Pose-mode and turning fallbacks, done."""
import math

import numpy as np

from maze_nav.curtain_inspect import InspectParams, InspectPlan


class Bridge:
    """cmd_vel_bridge's lie / pose status, driven by what the plan asks for."""

    def __init__(self, lie_ok=True, stands=True, pose='UNAVAILABLE: sim', down_s=2.0,
                 up_s=4.0):
        self.lie_ok, self.stands, self.pose = lie_ok, stands, pose
        self.down_s, self.up_s = down_s, up_s
        self.status = 'IDLE' if lie_ok is not None else 'UNAVAILABLE: sim'
        self.since = 0.0
        self.lay_for = 0.0

    def step(self, t, lie):
        st = self.status
        if lie is True and st == 'IDLE' and self.lie_ok:
            self.status, self.since = 'LYING_DOWN', t
        elif st == 'LYING_DOWN' and t - self.since >= self.down_s:
            self.status, self.since = 'LYING', t
        elif lie is False and st in ('LYING', 'LYING_DOWN'):
            self.status, self.since = 'STANDING_UP', t
        elif st == 'STANDING_UP' and self.stands and t - self.since >= self.up_s:
            self.status = 'IDLE'
        if self.status == 'LYING':
            self.lay_for = t - self.since


def run(plan, xy, yaw, bridge=None, dt=0.05, t_max=120.0):
    """Integrate the plan's (v, w) on a unicycle (never backwards, like cmd_vel_bridge)."""
    bridge = bridge or Bridge(lie_ok=None)
    xy, t = np.array(xy, float), 0.0
    phases, poses, measuring = [], [], 0
    step = None
    while t < t_max:
        step = plan.tick(t, xy, yaw, bridge.pose, bridge.status)
        bridge.step(t, step.lie)
        if not phases or phases[-1] != step.phase:
            phases.append(step.phase)
        if step.pose is not None:
            poses.append(step.pose)
        measuring += step.measuring
        if step.done:
            break
        assert step.v >= 0.0
        xy = xy + dt * step.v * np.array([math.cos(yaw), math.sin(yaw)])
        yaw = yaw + dt * step.w
        t += dt
    return xy, yaw, phases, poses, step, t, measuring * dt


def test_goes_to_the_middle_of_the_cell_squares_up_and_lies_down():
    # curtain on x = 3 (cell edge), robot 2.5 m before it, 0.3 m off its centre, turned 30 deg
    plan = InspectPlan((3.0, 0.0), (1.0, 0.0), 0.0)
    bridge = Bridge()
    xy, yaw, phases, poses, step, _, measured = run(plan, (0.5, 0.3), math.radians(30), bridge)
    assert step.done and step.why == 'measured lying down' and plan.lay
    assert phases[:6] == ['GOTO', 'FACE', 'SETTLE', 'LIE_DOWN', 'LYING', 'STAND_UP']
    assert np.linalg.norm(xy - (2.5, 0.0)) < 0.12           # middle of the cell in front
    assert abs(math.degrees(yaw)) < 5                       # square to the curtain
    assert abs(measured - plan.p.lie_hold) < 0.2            # measured while lying still
    assert bridge.status == 'IDLE' and poses == []          # standing again, no Pose look


def test_already_closer_than_half_a_cell_squares_up_where_it_is():
    plan = InspectPlan((1.0, 0.0), (1.0, 0.0), 0.0)
    xy, _, phases, _, step, _, _ = run(plan, (0.7, 0.0), 0.0, Bridge())
    assert step.done and abs(xy[0] - 0.7) < 1e-9            # never backs up
    assert 'LYING' in phases


def test_cannot_lie_down_stands_and_falls_back_to_the_pose_look():
    def pose_status(_):
        return 'POSING'
    bridge = Bridge(lie_ok=False, pose='POSING', down_s=99)
    plan = InspectPlan((0.5, 0.0), (1.0, 0.0), 0.0, InspectParams(lie_wait=2.0))
    _, _, phases, poses, step, _, _ = run(plan, (0.0, 0.0), 0.0, bridge)
    assert step.done and 'Pose mode' in step.why and not plan.lay
    assert 'LIE_DOWN' in phases and 'LOOK' in phases
    assert all(abs(p - plan.p.pitch) < 1e-9 for _, p in poses)


def test_no_lying_no_pose_turns_the_whole_robot():
    plan = InspectPlan((0.5, 0.0), (1.0, 0.0), 0.0)
    _, yaw, phases, poses, step, _, _ = run(plan, (0.0, 0.0), 0.0, Bridge(lie_ok=None))
    assert step.done and 'turning' in step.why
    assert 'LIE_DOWN' not in phases and poses == []
    assert abs(yaw) < math.radians(5)


def test_pose_mode_selected_explicitly():
    plan = InspectPlan((0.5, 0.0), (1.0, 0.0), 0.0, InspectParams(look_mode='pose'))
    _, _, phases, poses, step, _, _ = run(plan, (0.0, 0.0), 0.0, Bridge(pose='POSING'))
    assert step.done and 'Pose mode' in step.why and 'LIE_DOWN' not in phases and poses


def test_stand_up_not_confirmed_is_reported():
    plan = InspectPlan((0.5, 0.0), (1.0, 0.0), 0.0, InspectParams(stand_wait=5.0))
    _, _, _, _, step, _, _ = run(plan, (0.0, 0.0), 0.0, Bridge(stands=False))
    assert step.done and step.why.startswith('STAND-UP NOT CONFIRMED')


def test_times_out_before_lying_but_never_while_lying():
    plan = InspectPlan((5.0, 0.0), (1.0, 0.0), 0.0, InspectParams(timeout=3.0, speed=0.01))
    _, _, _, _, step, _, _ = run(plan, (0.0, 0.0), 0.0, Bridge())
    assert step.done and 'timed out' in step.why
    plan = InspectPlan((0.5, 0.0), (1.0, 0.0), 0.0,
                       InspectParams(timeout=2.0, lie_hold=6.0))      # lying outlasts timeout
    _, _, _, _, step, _, _ = run(plan, (0.0, 0.0), 0.0, Bridge())
    assert step.why == 'measured lying down'


def test_gives_up_when_it_makes_no_progress_towards_the_look_point():
    """2026-11-21 sim: the look point was behind a wall, the robot pinned, the inspection ran
    the full 75 s. Now: stuck_time s without getting stuck_gain m closer -> done."""
    plan = InspectPlan((3.0, 0.0), (1.0, 0.0), 0.0, InspectParams(stuck_time=5.0))
    xy, t, step = np.array([0.0, 0.0]), 0.0, None
    while t < 30.0:                                          # commanded, never moves
        step = plan.tick(t, xy, 0.0, 'UNAVAILABLE: sim', 'IDLE')
        if step.done:
            break
        t += 0.1
    assert step.done and step.why.startswith('no progress towards the look point')
    assert 5.0 < t < 6.0
    assert plan.p.timeout == 45.0 and InspectParams().timeout == 45.0
