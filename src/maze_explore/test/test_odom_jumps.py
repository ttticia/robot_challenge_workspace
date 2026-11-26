"""Odometry jumps (user, 2026-11-26: allow some). A burst that settles only pauses the
robot; odometry that does not come back is a FAULT."""
import math
from types import SimpleNamespace as NS

import pytest

explorer = pytest.importorskip('maze_explore.explorer')


def robot():
    r = NS(t=100.0, last_yaw=None, last_odom=None, heights_turn_max=0.2, turned_at=-1e9,
           ai_walking=False, odom_jumps=[], odom_burst_from=(0.0, 0.0), odom_settle=3.0,
           odom_burst_max=6.0, odom_net_max=1.5, odom_grace_until=0.0, odom_hold_until=0.0,
           state='RUN', odom_rot=None, faults=[], warned=[])
    r.now = lambda: r.t
    r.feed_check = lambda stamp: None
    r.jump_report = lambda jump, dt: None
    r.halt = lambda state, why: r.faults.append(why)
    r.get_logger = lambda: NS(warn=lambda text, **k: r.warned.append(text))
    return r


def feed(r, points, dt=0.1):
    for x, y in points:
        r.t += dt
        stamp = NS(sec=int(r.t), nanosec=int((r.t % 1) * 1e9))
        msg = NS(header=NS(stamp=stamp), pose=NS(pose=NS(
            position=NS(x=x, y=y, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))
        explorer.Explorer.on_odom(r, msg)


# /odom in the 05:03 robot run (bag explore_20261126_050440), from just before the first jump
RUN_0503 = [(0.60, 0.00), (0.72, 0.30), (0.82, 0.45), (0.82, 0.07), (0.82, -0.61),
            (0.96, -0.19), (1.00, 0.11), (1.22, 0.40), (1.16, 0.10), (1.11, -0.16),
            (1.24, -0.05)]


def test_the_0503_burst_only_pauses_the_robot():
    r = robot()
    feed(r, [(0.57, 0.0)] + RUN_0503)
    assert r.faults == [] and len(r.warned) == 1          # one burst, one warning
    assert r.odom_hold_until == pytest.approx(r.t + 3.0, abs=0.35)
    feed(r, [(1.24, -0.05)] * 40)                         # steady from then on
    assert r.faults == [] and r.t > r.odom_hold_until     # and the robot may walk again


def test_a_second_burst_later_is_allowed_too():
    r = robot()
    feed(r, [(0.57, 0.0)] + RUN_0503 + [(1.24, -0.05)] * 60)
    feed(r, [(1.24, 0.30), (1.24, -0.05)] + [(1.24, -0.05)] * 40)
    assert r.faults == [] and len(r.warned) == 2


def test_jumping_that_does_not_stop_is_a_fault():
    r = robot()
    swing = [(0.0, 0.3 * (-1) ** i) for i in range(80)]   # 8 s of +-0.3 m while standing
    feed(r, [(0.0, 0.0)] + swing)
    assert r.faults and 'still jumping' in r.faults[0]


def test_running_away_or_one_huge_jump_is_a_fault():
    r = robot()
    feed(r, [(0.0, 0.0)] + [(0.4 * i, 0.0) for i in range(1, 8)])
    assert r.faults and 'from where the jumping began' in r.faults[0]
    r = robot()
    feed(r, [(0.0, 0.0), (2.5, 0.0)])
    assert r.faults and 'one jump' in r.faults[0]


def test_jumps_in_the_grace_window_or_on_the_stairs_do_not_count():
    r = robot()
    r.odom_grace_until = 1e9
    feed(r, [(0.0, 0.0)] + [(0.0, 0.3 * (-1) ** i) for i in range(80)])
    assert r.faults == []
    r = robot()
    r.ai_walking = True
    feed(r, [(0.0, 0.0)] + [(0.4 * i, 0.0) for i in range(1, 12)])
    assert r.faults == []


def test_on_uneven_floor_more_jumping_is_allowed():
    # user, 2026-11-26. 08:37 run: 0.35 m then 0.76 m in the uneven cell (1, -1), FAULT at 1.68 m
    def run(uneven):
        r = robot()
        r.odom_terrain, r.on_uneven = 2.0, (lambda: uneven)
        feed(r, [(0.0, 0.0), (0.0, 0.0), (0.25, 0.0)])                 # a 0.25 m slip
        slips = len(r.warned)
        feed(r, [(0.25, 0.0)] * 40 + [(0.60, 0.0), (1.36, 0.0), (1.93, 0.0)])   # 0.35, 0.76, 0.57 m
        return slips, r.faults
    slips, faults = run(False)
    assert slips == 1 and faults                         # plain floor: a jump, then a FAULT (1.68 m)
    slips, faults = run(True)
    assert slips == 0 and faults == []                   # uneven floor: not counted, and no FAULT
    r = robot()
    r.odom_terrain, r.on_uneven = 2.0, (lambda: True)
    feed(r, [(0.0, 0.0), (0.0, 0.0), (2.5, 0.0)])
    assert r.faults                                      # one jump over 2 m is a FAULT anywhere


def test_the_allowance_holds_when_the_jump_moves_the_pose_off_the_cell():
    # 20:53 run: standing on the log cell (2, -2), the first 0.47 m jump put the pose a
    # cell over, on_uneven() read the JUMPED pose, the allowance switched off and net
    # 1.93 m was a FAULT at the flat limit (1.5) instead of a stand at the doubled one
    # (3.0). The allowance now comes from where the robot ARRIVED (rough_stand).
    import inspect
    from maze_explore.grid import Grid
    assert 'self.rough_stand = cell in' in inspect.getsource(explorer.Explorer.arrive)
    r = robot()
    r.odom_terrain = 2.0
    r.rough_cells, r.rough_known, r.step = {(2, -2)}, set(), None
    r.grid, r.pose = Grid(0.0, 0.0, 0.0, 1.02), (3.1, -2.04, 0.0)  # pose reads a cell over
    r.rough_stand = False
    r.on_uneven = explorer.Explorer.on_uneven.__get__(r)
    r.cur_cell = explorer.Explorer.cur_cell.__get__(r)
    assert not r.on_uneven()                     # by the jumped pose alone: off the cell
    r.rough_stand = True                         # arrive() latched the log cell
    assert r.on_uneven()
    # the 20:53 jumps: 0.47, then 0.30 and 0.29 (under the doubled minimum), then 0.90 m;
    # net 1.96 m is under the doubled limit of 3.0: the robot stands, no FAULT
    feed(r, [(0.0, 0.0), (0.47, 0.0), (0.77, 0.0), (1.06, 0.0), (1.96, 0.0)])
    assert not r.faults
    r.rough_stand = False                        # the same burst on flat floor is a FAULT
    r.odom_jumps, r.last_odom = [], None
    feed(r, [(0.0, 0.0), (0.47, 0.0), (0.77, 0.0), (1.06, 0.0), (1.96, 0.0)])
    assert r.faults
