"""A camera-confirmed curtain is pushed through with the obstacle stop off (2026-11-25
10:42: the fabric itself stopped the sideways step 3 cm in and the curtain became a wall)."""
import math
from collections import deque
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_qr_approach import Robot, explorer


def push(sure, solid_at=None, leg=True):
    """Sideways push from (0, 0) towards (1.02, 0) through fabric hanging at x = 0.40.
    solid_at: something the robot cannot pass (its centre stops there). leg: leg odometry
    arrives (it reports what the body really does)."""
    E = explorer.Explorer
    real = E.do_strafe

    def do_strafe(self, *a):
        if solid_at is not None and self.pose[0] > solid_at:
            self.pose[0] = solid_at                        # pressed against it: no further
        if leg and self.leg_sub is not None:
            self.on_leg(NS(pose=NS(pose=NS(position=NS(x=self.pose[0], y=self.pose[1])))))
        return real(self, *a)
    R = type('R', (Robot,), dict(do_strafe=do_strafe, side_blocked=E.side_blocked,
                                 leg_watch=E.leg_watch, on_leg=E.on_leg,
                                 push_stalled=E.push_stalled))
    r = R()
    r.side_speed, r.push_stop, r.push_sure, r.blind_until = 0.10, True, sure, 0.0
    r.push_stall, r.push_stall_time, r.push_stall_grace, r.push_stall_min = True, 1.5, 0.7, 0.03
    r.leg, r.leg_sub, r.leg_topic = deque(maxlen=200), None, '/leg_odom2'
    r.create_subscription = lambda *a: 'sub'
    r.destroy_subscription = lambda sub: None
    r.blind_pub = type('P', (), {'publish': lambda s, m: None})()
    gave_up = []
    r.stuck = lambda mode, along: gave_up.append((round(r.t - 100.0, 1), along)) or True
    r.pose = [0.0, 0.0, math.pi / 2]                       # body square to the way through
    ys = np.linspace(-0.45, 0.45, 40)
    r.obstacles = np.stack([np.full(40, 0.40), ys], 1)     # the fabric, hanging in the way
    r.queue.append(('strafe', 1.02, 0.0, 0.0, math.pi / 2, 'push'))
    r.run(40.0)
    return r, gave_up


def test_confirmed_curtain_is_pushed_through():
    r, gave_up = push(sure=True)
    assert not gave_up and r.pose[0] > 0.95                # at the centre of the cell behind


def test_unconfirmed_push_still_stops_at_the_fabric():
    r, gave_up = push(sure=False)
    assert gave_up and r.pose[0] < 0.2                     # as before: push_stop rules


def test_push_against_something_solid_is_given_up_within_seconds():
    r, gave_up = push(sure=True, solid_at=0.30)            # a prop behind the fabric
    assert gave_up and gave_up[0][0] < 6.5                 # 3 s to get there + ~2.2 s, not 8 s more
    assert any('stalled' in e for e in r.events)


def test_slow_but_moving_push_is_not_a_stall():
    r, gave_up = push(sure=True)
    assert not gave_up and not any('stalled' in e for e in r.events)


def test_without_leg_odometry_nothing_changes():
    r, gave_up = push(sure=True, leg=False)                # no /leg_odom2: no stall stop
    assert not gave_up and r.pose[0] > 0.95
    r, gave_up = push(sure=True, solid_at=0.30, leg=False)
    assert gave_up and gave_up[0][0] > 8.0                 # the 8 s no-progress rule alone
