"""Unit tests for maze_nav.odom_guard.GuardCore (no ROS): python3 -m pytest -q test/test_odom_guard.py"""
import math

import numpy as np
import pytest

from maze_nav.odom_guard import GuardCore, make_tf

DT = 0.1


def pose(x, y, yaw=0.0, z=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return make_tf(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), [x, y, z])


def run(core, fl_poses, legs, t0=100.0):
    """fl_poses: Fast-LIO base poses, one per DT; legs: body (vx, vy) per step (None = none)."""
    out = []
    for k, (t_fl, leg) in enumerate(zip(fl_poses, legs)):
        now = t0 + k * DT
        if leg is not None:
            for j in range(5):                      # leg odometry ~50 Hz in between
                core.add_leg(now - DT + (j + 1) * DT / 5, *leg)
        out.append(core.update(now, t_fl, now))
    return out


def test_agreeing_odometry_passes_through_unchanged():
    core = GuardCore()
    truth = [pose(0.15 * k * DT, 0.0) for k in range(30)]
    out = run(core, truth, [(0.15, 0.0)] * 30)
    for a, b in zip(out, truth):
        assert np.allclose(a, b)
    assert core.status.startswith('OK')


def test_slip_is_bridged_with_the_legs_and_never_jumps_back():
    core = GuardCore()
    n = 40
    truth = [pose(0.15 * k * DT, 0.0) for k in range(n)]
    fl = []
    for k in range(n):                               # Fast-LIO slips 0.5 m sideways in 0.5 s
        side = 0.0 if k < 10 else min(0.5, 0.1 * (k - 9))
        fl.append(pose(0.15 * k * DT, side))
    out = run(core, fl, [(0.15, 0.0)] * n)
    for a, b in zip(out, truth):                     # /odom follows the truth, not the slip
        assert np.linalg.norm(a[:2, 3] - b[:2, 3]) < 0.02
    steps = [np.linalg.norm(out[k][:2, 3] - out[k - 1][:2, 3]) for k in range(1, n)]
    assert max(steps) < 0.03                         # no jump on /odom
    assert core.offset == pytest.approx(0.5, abs=0.02)   # kept: /odom 0.5 m off Fast-LIO
    assert core.status.startswith('OK')              # slip over, guard still on
    assert any('slip' in e[1] for e in core.events)


def test_turning_on_the_spot_is_not_a_slip():
    core = GuardCore()
    fl = [pose(0.0, 0.0, yaw=0.5 * k * DT) for k in range(30)]    # 0.5 rad/s in place
    out = run(core, fl, [(0.0, 0.0)] * 30)
    for a, b in zip(out, fl):
        assert np.allclose(a, b)
    assert core.slip_since is None and not core.events


def test_rotation_comes_from_fastlio_during_a_slip():
    core = GuardCore()
    fl = [pose(0.0, 0.0 if k < 5 else 0.3, yaw=0.3 * k * DT) for k in range(10)]
    out = run(core, fl, [(0.0, 0.0)] * 10)
    yaw = math.atan2(out[-1][1, 0], out[-1][0, 0])
    assert yaw == pytest.approx(0.3 * 9 * DT, abs=1e-6)
    assert np.linalg.norm(out[-1][:2, 3]) < 1e-6     # the 0.3 m jump did not reach /odom


def test_divergence_is_passed_through_so_the_watchdog_sees_it():
    core = GuardCore(max_slip_time=3.0, max_offset=1.5)
    n = 60
    fl = [pose(0.0, 0.0 if k < 10 else 0.3 * (k - 9)) for k in range(n)]   # 3 m/s runaway
    out = run(core, fl, [(0.0, 0.0)] * n)
    assert core.passthrough is not None
    assert np.allclose(out[-1], fl[-1])              # unguarded: /odom = Fast-LIO again
    assert core.status.startswith('PASS-THROUGH')
    assert any(kind == 'error' for kind, _ in core.events)


def test_no_leg_odometry_means_unguarded():
    core = GuardCore()
    fl = [pose(0.0, 0.0 if k < 5 else 0.4) for k in range(10)]
    out = run(core, fl, [None] * 10)
    for a, b in zip(out, fl):
        assert np.allclose(a, b)
    assert 'no leg odometry' in core.status


def test_stale_leg_odometry_is_not_used():
    core = GuardCore(leg_timeout=0.5)
    core.add_leg(0.0, 0.0, 0.0)                      # one old sample only
    fl = [pose(0.0, 0.0), pose(0.4, 0.0)]
    out = [core.update(10.0, fl[0], 10.0), core.update(10.1, fl[1], 10.1)]
    assert np.allclose(out[1], fl[1])


def test_many_small_slips_add_up_without_giving_up():
    core = GuardCore(max_offset=1.5)  # per slip
    fl, side = [], 0.0
    for k in range(120):                             # 6 separate 0.4 m slips, 2 s apart
        if k % 20 == 10:
            side += 0.4
        fl.append(pose(0.0, side))
    out = run(core, fl, [(0.0, 0.0)] * 120)
    assert core.passthrough is None                  # 2.4 m in total, 0.4 m per slip
    assert core.offset == pytest.approx(2.4, abs=0.01)
    assert np.linalg.norm(out[-1][:2, 3]) < 1e-6


def test_one_big_jump_then_agreement_is_bridged():
    core = GuardCore()                               # bag replay: 1.9 m in 0.3 s, then steady
    fl = [pose(0.0, 0.0 if k < 10 else (0.6 * min(k - 9, 3))) for k in range(40)]
    out = run(core, fl, [(0.0, 0.0)] * 40)
    assert core.passthrough is None
    assert np.linalg.norm(out[-1][:2, 3]) < 1e-6
    assert core.offset == pytest.approx(1.8, abs=0.01)
