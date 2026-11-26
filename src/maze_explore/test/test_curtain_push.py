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
    r.terrain_push_speed, r.push_stop, r.push_sure, r.blind_until = 0.10, True, sure, 0.0
    r.curtain_speed = 0.10
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


def test_pose_during_a_crossing_is_the_held_map_pose_with_fresh_odometry():
    # simulator, 2026-11-26: with the scans off SLAM stops renewing map -> odom, the pose
    # froze and the stale-pose rule stopped the robot for good in the middle of the push
    import math
    from types import SimpleNamespace as NS
    explorer = pytest.importorskip('maze_explore.explorer')

    def tf(x, y, yaw, sec):
        return NS(transform=NS(translation=NS(x=x, y=y, z=0.0),
                               rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
                  header=NS(stamp=NS(sec=sec, nanosec=0)))
    frames = {('map', 'odom'): tf(1.0, 2.0, math.pi / 2, 100),          # old: SLAM is not fed
              ('odom', 'base_level'): tf(0.5, 0.0, 0.1, 160)}           # fresh
    r = NS(curtain_blind=True, pose=None, pose_time=None,
           tf_buffer=NS(lookup_transform=lambda a, b, t: frames[(a, b)]))
    r.read_pose_blind = lambda: explorer.Explorer.read_pose_blind(r)
    assert explorer.Explorer.read_pose(r)
    assert r.pose == pytest.approx((1.0, 2.5, math.pi / 2 + 0.1)) and r.pose_time == 160


def cleared(sizes_after, ref=600):
    """The robot has just stepped through; the cloud sizes it then sees, one per second."""
    import numpy as np
    from test_qr_approach import Robot as Base
    E = explorer.Explorer
    R = type('R', (Base,), dict(do_curtain_clear=E.do_curtain_clear, lidar_blocked=E.lidar_blocked))
    r = R()
    r.fabric_frac, r.fabric_until, r.cloud_ref = 0.4, 0.0, ref
    sizes = list(sizes_after)
    r.clouds = deque(maxlen=12)
    send = r.send

    def tick_cloud(*a, **k):
        n = sizes[min(int(r.t - 100.0), len(sizes) - 1)]
        r.clouds.append(np.zeros((n, 3)))
        return send(*a, **k)
    r.send = tick_cloud
    r.pose = [1.02, 0.0, math.pi / 2]
    r.queue.append(('curtain_clear', math.pi / 2, 0))
    assert r.run(90.0)
    turns = [p for p in r.trail if p[0] == 'turn']
    return r, turns


def test_no_turns_after_a_curtain_when_the_lidar_is_clear():
    r, turns = cleared([580] * 60)
    assert turns == [] and r.fabric_until == 0.0 and r.t - 100.0 < 2.0


def test_fabric_on_the_lidar_is_shaken_off_with_90_right_then_90_left():
    r, turns = cleared([150] * 4 + [590] * 60)           # covered, clear once the robot has turned
    yaws = [math.degrees(p[3]) for p in turns]
    assert min(yaws) == pytest.approx(0.0, abs=4.0)       # 90 deg to the right of +90
    assert abs(math.degrees(r.pose[2]) - 90.0) < 4.0      # and back
    assert any('turning 90 right and 90 left' in e for e in r.events)
    assert any('lidar clear after 1 turn' in e for e in r.events) and r.fabric_until == 0.0


def test_still_covered_after_two_goes_means_no_mapping_until_it_clears():
    r, turns = cleared([150] * 90)
    assert sum('turning 90 right' in e for e in r.events) == 2
    assert r.fabric_until > r.t and any('nothing is added to the map' in e for e in r.events)
    src = __import__('inspect').getsource(explorer.Explorer.tick)
    assert 'elif self.now() < self.fabric_until:' in src and 'if self.lidar_blocked():' in src
