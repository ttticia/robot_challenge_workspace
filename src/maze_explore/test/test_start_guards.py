"""The three guards from the 03:29 run of 2026-11-26, where a wrong first grid fit put the
start cell's centre behind the robot and it was walked into the back wall: the start
centre stays near the placed spot along the heading, a blocked walk back onto the centre is
not repeated, and the obstacle stop remembers what it saw (the lidar's blind range)."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from maze_explore.grid import Grid, wrap
from test_qr_approach import Robot, explorer

E = explorer.Explorer


def anchored(g, start_pose=(0.0, 0.0), closed=(), limit=0.12):
    me = NS(start_pose=start_pose, start=(0, 0), left_start=bool(closed), start_u_max=limit,
            get_logger=lambda: NS(warn=lambda *a, **k: None))
    return E.anchor_start(me, g)


def test_a_fit_that_moves_the_start_centre_far_along_the_heading_is_not_believed_there():
    g = anchored(Grid(-0.30, 0.18, math.radians(4), 1.02))       # 0.3 m behind, 0.18 m left
    c, s = math.cos(g.yaw), math.sin(g.yaw)
    assert abs(g.x0 * c + g.y0 * s) < 1e-6                       # along the heading: at the placed spot
    assert -g.x0 * s + g.y0 * c == pytest.approx(0.18 * c + 0.30 * s, abs=1e-6)   # sideways kept
    assert math.degrees(g.yaw) == pytest.approx(4.0)             # angle kept


def test_small_shifts_and_fits_after_the_start_cell_are_left_alone():
    g = Grid(-0.08, 0.2, 0.0, 1.02)
    assert anchored(g) is g                                      # within the limit
    far = Grid(-0.30, 0.0, 0.0, 1.02)
    assert anchored(far, closed={(0, 0)}) is far                 # the robot has left the start cell
    assert anchored(far, start_pose=None) is far


def wall_ahead(r, x):
    ys = np.linspace(-0.3, 0.3, 30)
    r.obstacles = np.stack([np.full(30, x), ys], 1)


def see(r, visible=True):
    """What the lidar gives now: the obstacles, or nothing (inside its blind range)."""
    x, y, yaw = r.pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = r.obstacles[:, 0] - x, r.obstacles[:, 1] - y
    pts = np.stack([c * dx + s * dy, -s * dx + c * dy, np.full(len(dx), 0.4)], 1)
    r.cloud = pts if visible else np.zeros((0, 3))


def test_the_obstacle_stop_remembers_a_wall_it_can_no_longer_see():
    r = Robot()
    wall_ahead(r, 0.45)                        # nose 18 cm from it
    see(r)
    assert r.front_blocked()
    r.pose[0] += 0.08                          # crept closer: now in the blind range
    see(r, visible=False)
    assert r.front_blocked()                   # remembered
    r.pose[2] = math.pi                        # turned away: the way ahead is clear
    assert not r.front_blocked()
    r.pose[2] = 0.05                           # facing it again (03:29 run: the third try)
    assert r.front_blocked()
    r.t += 61.0                                # forgotten after block_memory
    assert not r.front_blocked()


def test_nothing_seen_means_nothing_remembered():
    r = Robot()
    see(r, visible=False)
    assert not r.front_blocked() and r.front_seen == []


def test_a_blocked_walk_back_onto_the_centre_is_reported_not_repeated():
    R = type('R', (Robot,), dict(do_drive=E.do_drive, stuck=E.stuck))
    r = R()
    r.v_max, r.ai_speed, r.curtain_speed, r.push_stop, r.push_sure = 0.2, 0.5, 0.1, True, False
    r.recentre_failed, r.step = False, None
    r.pose = [0.0, 0.0, math.pi]               # facing the back wall, "centre" 0.39 m beyond it
    r.obstacles = np.stack([np.full(30, -0.45), np.linspace(-0.3, 0.3, 30)], 1)
    r.queue.append(('drive', -0.39, 0.0, math.pi, 'recentre'))
    assert r.run(20.0)
    assert r.recentre_failed and r.pose[0] > -0.05          # stopped, not driven into it
    src = __import__('inspect').getsource(E.plan)
    assert 'cur not in self.no_recentre' in src and 'self.no_recentre.add(cur)' in src


@pytest.mark.parametrize('angle', [90.0, -90.0, 180.0])
def test_a_turn_is_made_about_the_spot_it_starts_on_not_the_cell_centre(angle):
    r = Robot()
    r.pose = [0.20, -0.15, 0.0]                # 0.25 m off the centre of cell (0, 0)
    r.queue.append(('turn', math.radians(angle), 2.0))
    assert r.run(40.0)
    moved = max(math.hypot(p[1] - 0.20, p[2] + 0.15) for p in r.trail)
    assert moved < 0.05                        # it used to walk to the cell centre
    assert abs(math.degrees(r.pose[2]) - angle) % 360 < 3 or abs(abs(math.degrees(r.pose[2])) - 180) < 3


def test_a_robot_that_backs_away_while_turning_is_held_within_10_cm():
    class Drifts(Robot):                       # the Lite3's backward creep: 9 cm per 90 deg
        def send(self, vx=0.0, vy=0.0, wz=0.0):
            Robot.send(self, vx, vy, wz)
            x, y, yaw = self.pose
            back = 0.09 / (math.pi / 2) * abs(wz) * 0.1
            self.pose = [x - back * math.cos(yaw), y - back * math.sin(yaw), yaw]
    r = Drifts()
    r.queue.append(('turn', math.pi / 2, 2.0))
    assert r.run(40.0)
    assert max(math.hypot(p[1], p[2]) for p in r.trail) < 0.10


def feed_robot():
    from collections import deque
    r = Robot()
    r.feed_pause, r.feed_late, r.feed_settle, r.feed_hold_max = True, 0.15, 1.0, 6.0
    r.feed_lags, r.feed_bad_at, r.feed_hold_from, r.feed_free_until = deque(maxlen=300), -1e9, None, 0.0
    r.feed_worst = 0.0
    r.feed_slow = 0.10
    r.feed_check, r.feed_hold = E.feed_check.__get__(r), E.feed_hold.__get__(r)
    return r


def feed(r, seconds, lag):
    """/odom at 10 Hz for `seconds`, each message arriving `lag` after its stamp."""
    held = []
    for _ in range(int(round(seconds * 10))):
        r.t += 0.1
        r.feed_check(r.t - lag)
        held.append(r.feed_hold())
    return held


def test_the_robot_stands_while_the_feed_is_late_and_goes_on_a_second_after():
    r = feed_robot()
    assert not any(feed(r, 5.0, 0.04))                   # the usual delay
    assert all(feed(r, 0.5, 0.39))                       # 03:45 run: 0.39 s before the jump
    back = feed(r, 2.0, 0.05)
    assert all(back[:9]) and not any(back[11:])          # held for feed_settle, then free


def test_a_clock_offset_is_not_a_late_feed():
    r = feed_robot()
    assert not any(feed(r, 10.0, 0.45))                  # always 0.45 s: that is the usual delay here
    assert any(feed(r, 0.5, 0.75))                       # 0.30 s over it is late


def test_a_feed_that_stays_late_does_not_hold_the_robot_for_good():
    r = feed_robot()
    feed(r, 5.0, 0.04)
    held = feed(r, 12.0, 0.60)
    assert all(held[:55]) and not any(held[65:110])      # 6 s at most, then 10 s without a pause



def shifter(pose):
    R = type('R', (Robot,), dict(do_shift=E.do_shift))
    r = R()
    r.shift_speed, r.pose = 0.08, list(pose)
    r.queue.append(('shift', 0.0, 0.0))
    return r


@pytest.mark.parametrize('pose', [(-0.15, 0.10, 0.0), (-0.05, -0.18, 0.4), (0.12, 0.12, -2.0)])
def test_shift_onto_the_centre_keeps_the_heading_and_never_goes_backwards(pose):
    r = shifter(pose)
    assert r.run(20.0)
    assert math.hypot(r.pose[0], r.pose[1]) < 0.06                 # on the centre
    assert abs(wrap(r.pose[2] - pose[2])) < math.radians(3)        # no turn
    assert not r.backwards


def test_shift_leaves_what_is_behind_the_robot():
    r = shifter((0.15, 0.10, 0.0))                                 # centre behind and to the right
    assert r.run(20.0)
    assert abs(r.pose[1]) < 0.05 and r.pose[0] == pytest.approx(0.15, abs=0.02)   # sideways done, not back
    assert not r.backwards


def jumper(before, after, tilt=0.0, still=True):
    """Stand-in at the moment a burst of jumps is over: it believed `before`, reads `after`."""
    from collections import deque
    r = Robot()
    r.jump_fix_min, r.jump_fix_max, r.jump_fix_wait = 0.10, 1.0, 6.0
    r.jump_from, r.pose = (r.t - 3.0, before), list(after)
    r.poses = deque(((r.t - 0.1 * k, tuple(after) if still else (after[0] + 0.03 * k, after[1], after[2]))
                     for k in range(12, 0, -1)), maxlen=40)
    c = math.cos(math.radians(tilt))
    r.odom_rot, r.odom_hold_until, r.why = [[1, 0, 0], [0, 1, 0], [0, 0, c]], r.t - 0.1, ''
    r.kinds = {((0, 0), 0): 'free', ((1, 0), 1): 'room', ((1, 0), 0): 'wall', ((1, 0), 3): 'unknown'}
    r.kept, r.reanchor_fit, r.heights, r.front_seen = {}, 0, 'old', [(0, 0, 0)]
    r.jump_settled, r.update_kept = E.jump_settled.__get__(r), E.update_kept.__get__(r)
    return r


def test_after_a_settled_jump_the_grid_moves_so_the_robot_keeps_its_cell_and_spot():
    before, after = (1.02, 0.05, 0.10), (1.32, -0.20, 0.25)        # 0.39 m and 8.6 deg off
    r = jumper(before, after)
    u0, v0 = r.grid.to_grid(before[0], before[1])
    assert r.jump_settled() and r.jump_from is None
    u1, v1 = r.grid.to_grid(after[0], after[1])
    assert float(u1) == pytest.approx(float(u0), abs=1e-6) and float(v1) == pytest.approx(float(v0), abs=1e-6)
    assert wrap(after[2] - r.grid.yaw) == pytest.approx(before[2], abs=1e-6)    # same heading in the grid
    assert r.kept == {((0, 0), 0): 'free'}         # other cells kept; this cell (1, 0) is read afresh
    assert r.reanchor_fit == 6 and r.front_seen == [] and r.heights != 'old'
    assert any('re-anchored' in e for e in r.events)


def test_a_small_offset_or_a_huge_one_leaves_the_grid_alone():
    for after in ((1.05, 0.05, 0.11), (2.60, 0.05, 0.10)):
        r = jumper((1.02, 0.05, 0.10), after)
        g = r.grid
        assert r.jump_settled() and r.grid is g and r.kept == {}


def test_odometry_that_has_not_settled_is_waited_for_then_given_up():
    r = jumper((1.02, 0.05, 0.10), (1.32, -0.20, 0.25), still=False)
    g = r.grid
    assert not r.jump_settled() and r.jump_from is not None        # stand a little longer
    r.odom_hold_until = r.t - 7.0                                   # 7 s after the burst ended
    assert r.jump_settled() and r.grid is g and r.jump_from is None
    r = jumper((1.02, 0.05, 0.10), (1.32, -0.20, 0.25), tilt=38.0)  # 03:45 run: -38 deg pitch standing
    r.odom_hold_until = r.t - 7.0
    assert r.jump_settled() and r.kept == {}


def test_the_three_views_of_a_look_are_45_degrees_apart():
    src = __import__('inspect').getsource(E.__init__)
    assert "p('look_angle', 45.0)" in src


def test_before_a_look_the_stair_check_runs_without_a_terrain_verdict():
    import inspect
    assert "p('look_depth', True)" in inspect.getsource(E.__init__)
    assert "('stair_check', c, d, True, False)" in inspect.getsource(E.look_check)
    src = inspect.getsource(E.do_stair_check)
    assert 'if terrain and self.terrain_on and self.terrain_depth:' in src      # no verdict, no event
    assert 'if terrain and sum(votes) >= self.terrain_confirm' in src           # cell not marked uneven


def test_slam_gets_no_scans_while_odometry_is_not_to_be_trusted():
    import inspect
    r = feed_robot()
    r.map_blind_until, r.map_blind_after = 0.0, 1.0
    feed(r, 5.0, 0.04)
    assert r.map_blind_until == 0.0                              # feed on time: scans go to SLAM
    feed(r, 0.3, 0.60)
    assert r.map_blind_until > r.t + 0.8                         # late feed: blind for a second more
    tick = inspect.getsource(E.tick)
    assert 'self.ai_walking or self.now() < self.map_blind_until' in tick     # stairs walk too
    assert "self.map_blind_until = max(getattr(self, 'map_blind_until', 0.0), now" in inspect.getsource(E.on_odom)


def test_grid_corrections_use_the_walls_near_the_robot():
    src = __import__('inspect').getsource(E.align_grid)
    assert 'radius = 4.0 if wide else self.grid_fit_radius' in src
    assert "p('grid_fit_radius', 2.0)" in __import__('inspect').getsource(E.__init__)


def test_a_look_is_made_from_10_cm_before_the_rooms_edge():
    import inspect
    src = inspect.getsource(E.plan)
    assert 'reach = self.cell_size / 2 - self.look_gap - self.fp_front' in src
    assert "p('look_gap', 0.10)" in inspect.getsource(E.__init__)
    R = type('R', (Robot,), dict(do_drive=E.do_drive, do_call=E.do_call))
    r = R()
    r.v_max, r.ai_speed, r.curtain_speed, r.push_stop, r.push_sure, r.step = 0.2, 0.5, 0.1, True, False, None
    r.pose = [0.03, -0.02, 0.05]                               # at the centre of cell (0, 0)
    reach = 1.02 / 2 - 0.10 - r.fp_front
    r.queue.extend([('turn', 0.0, 3.0), ('drive', reach, 0.0, 0.0, 'creep')])
    assert r.run(30.0)
    assert r.pose[0] + r.fp_front == pytest.approx(0.51 - 0.10, abs=0.04)     # nose 0.10 m before the edge
    assert not r.backwards
