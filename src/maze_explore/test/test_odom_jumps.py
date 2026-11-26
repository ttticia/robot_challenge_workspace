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


def test_the_cell_last_arrived_in_counts_as_where_the_robot_stands():
    # 20:58 video: on a ramp cell, the pose jumped 0.52 m and read outside it; the normal
    # limits then applied and the run ended in FAULT at 1.57 m
    from maze_explore.grid import Grid
    E = explorer.Explorer
    me = NS(rough_cells={(1, -1)}, grid=Grid(0.0, 0.0, 0.0, 1.02), pose=(1.02, -0.50, 0.0),
            step=None, last_cell=(1, -1))
    me.cur_cell = lambda: me.grid.cell_of(me.pose[0], me.pose[1])
    assert me.cur_cell() == (1, 0)                  # the jumped pose: another cell
    assert E.on_uneven(me)                          # still on the ramp cell it arrived in
    me.last_cell = (1, 0)
    assert not E.on_uneven(me)
