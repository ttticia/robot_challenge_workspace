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


def test_the_three_views_of_a_look_are_35_degrees_apart():
    src = __import__('inspect').getsource(E.__init__)
    assert "p('look_angle', 35.0)" in src


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
    assert "p('grid_fit_radius', 4.0)" in __import__('inspect').getsource(E.__init__)   # 2 m was refused too often


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


def test_turns_are_slower_in_an_uneven_cell():
    def rate(uneven):
        r = Robot()
        if uneven:
            r.rough_cells = {(0, 0)}
        r.queue.append(('turn', math.pi / 2, 2.0))
        top = 0.0
        for _ in range(40):
            r.run(0.1)
            top = max(top, abs(r.cmd[2]))
        return top
    assert rate(False) == pytest.approx(0.5) and rate(True) == pytest.approx(0.3)


def drifted(off):
    """Walls of a 3 x 3 block of cells, mapped `off` m along x from where the grid has them."""
    xs, ys = [], []
    for k in range(4):                                   # lines at -0.51, 0.51, 1.53, 2.55
        line = np.arange(-0.51, 2.551, 0.05)
        xs += list(np.full(len(line), -0.51 + 1.02 * k) + off); ys += list(line - 1.02)
        xs += list(line + off); ys += list(np.full(len(line), -0.51 + 1.02 * k) - 1.02)
    logs = []
    me = NS(occ=NS(occupied_points=lambda: (np.array(xs), np.array(ys))), aligned=True,
            grid=Grid(0.0, 0.0, 0.0, 1.02), pose=(1.02, 0.0, 0.0), reanchor_fit=0,
            grid_fit_radius=4.0, arrive_fit_deg=8.0, arrive_fit_m=0.20, start_pose=None,
            left_start=True, start=(0, 0), start_u_max=0.12, logs=logs, event=logs.append)
    me.anchor_start = lambda g: g
    me.get_logger = lambda: NS(info=lambda text, **k: logs.append(text),
                               warn=lambda text, **k: logs.append(text))
    return me


def test_realign_on_arrival_brings_back_a_grid_the_small_correction_gives_up_on():
    me = drifted(0.15)
    E.align_grid(me)                                      # before a planning step: 0.08 m at most
    assert me.grid.x0 == 0.0 and any('not corrected' in l for l in me.logs)
    E.align_grid(me, arrived=True)                        # on arrival: up to 0.20 m
    assert me.grid.x0 == pytest.approx(0.15, abs=0.03)
    assert any('realigned on arrival' in l for l in me.logs)
    far = drifted(0.35)                                   # beyond 0.20 m: not guessed
    E.align_grid(far, arrived=True)
    assert far.grid.x0 == 0.0


def test_the_realign_waits_for_a_still_pose():
    src = __import__('inspect').getsource(E.do_resolve)
    assert 'self.align_grid(arrived=True)' in src and 'if not still and self.elapsed() < 4.0' in src


def test_a_turn_that_goes_nowhere_is_given_up_in_about_ten_seconds():
    class Stuck(Robot):                         # the robot does not respond at all
        def send(self, vx=0.0, vy=0.0, wz=0.0):
            self.cmd = (0.0, 0.0, 0.0)
    def gave_up_after(turn, uneven=False):
        r = Stuck()
        if uneven:
            r.rough_cells = {(0, 0)}
        r.queue.append(('turn', turn, 2.0))
        assert r.run(60.0)
        return r.t - 100.0
    assert 8.0 < gave_up_after(math.pi / 2) < 11.5          # 6 + 1.5 * (pi / 2) / 0.5 = 10.7 s
    assert gave_up_after(math.radians(45)) < 9.0
    assert 20.0 < gave_up_after(math.radians(179), uneven=True) < 23.0     # slower turn: longer limit


def test_a_lost_pose_is_not_called_no_exit():
    import inspect
    src = inspect.getsource(E.plan)
    lost = src.index('if off > self.lost_dist:')
    assert lost < src.index("self.event(f'no exit found") and "'STOPPED'" in src[lost:lost + 900]
    assert 'STAND_SIT' not in src[lost:src.index('# Everything reachable is explored and no exit is known')]
    assert "p('lost_dist', 0.75)" in inspect.getsource(E.__init__)


def test_the_walk_up_to_a_room_is_made_from_uneven_cells_too():
    import inspect
    assert 'self.look_gap_uneven or not (uneven and self.terrain_on)' in inspect.getsource(E.plan)
    assert "p('look_gap_uneven', True)" in inspect.getsource(E.__init__)


def through(edges, visited, kinds, rough=False):
    """edges: {(cell, side): 'wall' | 'open' | 'unknown'} for the cells not visited yet."""
    said = []
    me = NS(visited=set(visited), closed=set(), kinds=kinds, rough_cells=set(), skip_through=True,
            is_rough=lambda c, d: rough, event=said.append)
    sense = NS(edge=lambda cell, e: (edges.get((cell, e), 'wall'), False))
    return E.through_cells(me, sense), said


def test_a_path_cell_that_only_leads_to_explored_cells_is_not_entered():
    # (1, 1) is open from (1, 0); its other ways out go to (0, 1) and (2, 1), both visited
    kinds = {((1, 0), 1): 'free'}
    edges = {((1, 1), 0): 'open', ((1, 1), 2): 'open', ((1, 1), 1): 'wall'}
    skip, said = through(edges, {(1, 0), (0, 1), (2, 1)}, kinds)
    assert skip == {(1, 1)} and 'not entered' in said[0]
    skip, _ = through(edges, {(1, 0), (0, 1)}, kinds)               # (2, 1) not visited yet
    assert skip == set()
    unseen = dict(edges)
    unseen[((1, 1), 1)] = 'unknown'
    skip, _ = through(unseen, {(1, 0), (0, 1), (2, 1)}, kinds)
    assert skip == set()                                             # one side not seen: go and see
    skip, _ = through(edges, {(1, 0), (0, 1), (2, 1)}, kinds, rough=True)
    assert skip == set()                                             # uneven cells are crossed for points
    skip, _ = through({}, {(1, 0)}, kinds)                           # walled all round: a room, not this rule
    assert skip == set()


def test_the_planner_does_not_send_the_robot_into_such_a_cell():
    from maze_explore.plan import next_job
    a, b = (1, 0), (1, 1)
    kinds = {(a, 1): 'free', (a, 0): 'wall', (a, 2): 'wall', (a, 3): 'wall'}
    args = dict(closed=set(), links={}, cur=a, facing=0, looked=set(), greeted=set(), exit_cell=None)
    assert next_job(kinds, {a}, **args)['job'] == 'visit'
    assert next_job(kinds, {a} | {b}, **args)['job'] != 'visit'      # what plan() passes when b is skipped


def test_a_skipped_path_cell_is_entered_when_nothing_else_is_left():
    import inspect
    from maze_explore.plan import next_job
    src = inspect.getsource(E.plan)
    assert "if through and j['job'] not in ('visit', 'look'):" in src and 'entered after all' in src
    a, b, other = (1, 0), (1, 1), (2, 0)
    args = dict(closed=set(), links={}, cur=a, facing=0, looked=set(), greeted=set(), exit_cell=None)
    # something else is still unexplored: (2, 0) ahead. The skipped cell (1, 1) is left alone.
    kinds = {(a, 1): 'free', (a, 0): 'free', (a, 2): 'wall', (a, 3): 'wall'}
    j = next_job(kinds, {a, b}, **args)
    assert j['job'] == 'visit' and (j['cell'], j['d']) == (a, 0)
    # nothing else left: with the skip the planner has no visit; without it, it enters (1, 1)
    kinds = {(a, 1): 'free', (a, 0): 'wall', (a, 2): 'wall', (a, 3): 'wall'}
    assert next_job(kinds, {a, b}, **args)['job'] not in ('visit', 'look')
    j = next_job(kinds, {a}, **args)
    assert j['job'] == 'visit' and (j['cell'], j['d']) == (a, 1)


def test_a_junction_cell_is_skipped_the_same_way():
    # user, 2026-11-26 night: the rule holds for junction cells too. (1, 1) opens on all
    # four sides; entered from (1, 0), its other three ways go to cells already visited.
    kinds = {((1, 0), 1): 'free'}
    edges = {((1, 1), 0): 'open', ((1, 1), 1): 'open', ((1, 1), 2): 'open'}
    been = {(1, 0), (2, 1), (1, 2), (0, 1)}
    skip, said = through(edges, been, kinds)
    assert skip == {(1, 1)} and len(said) == 1
    skip, _ = through(edges, been - {(1, 2)}, kinds)          # one of its ways is new: enter it
    assert skip == set()


def test_square_up_accepts_8_cm_and_5_degrees():
    R = type('R', (Robot,), dict(do_drive=E.do_drive))
    r = R()
    r.v_max, r.ai_speed, r.curtain_speed, r.push_stop, r.push_sure, r.step = 0.2, 0.5, 0.1, True, False, None
    r.pose = [0.0, 0.06, math.radians(4)]          # 6 cm off the line, 4 deg off: good enough now
    r.queue.append(('drive', 0.0, 0.0, 0.0, 'square'))
    assert r.run(5.0) and r.t - 100.0 < 0.3 and r.cmd == (0.0, 0.0, 0.0)


def test_a_curtain_sighting_is_checked_only_where_the_lidar_sees_something_tall():
    from maze_explore.sense import HeightMap
    g = Grid(0.0, 0.0, 0.0, 1.02)
    hm = HeightMap()
    me = NS(grid=g)
    sense = NS(hm=hm)
    assert E.tall_share(me, sense, (0, 0), 1) is None              # nothing measured there yet
    x, y = g.box((0, 0), 1, np.linspace(-0.3, 0.3, 13), [0.0])
    for _ in range(3):
        hm.add(x.ravel(), y.ravel(), np.full(x.size, 0.53))          # an ordinary wall
    assert E.tall_share(me, sense, (0, 0), 1) == 0.0
    for _ in range(3):
        hm.add(x.ravel(), y.ravel(), np.full(x.size, 0.9))           # fabric up to 0.9 m
    assert E.tall_share(me, sense, (0, 0), 1) > 0.5
    src = __import__('inspect').getsource(E.plan)
    assert 'share < self.cv_tall_min' in src and 'if share is None:' in src


def test_a_pose_jump_drops_the_plan_and_waits_for_a_still_pose():
    from collections import deque
    me = NS(ai_walking=False, curtain_blind=False, pose_jump_max=0.5,
            poses=deque([(100.0, (1.0, 0.0, 0.0)), (100.1, (1.04, 0.0, 0.0))]))
    assert E.pose_jumped(me) == 0.0
    me.poses.append((100.2, (2.08, 0.05, 0.3)))                      # 10:39 run: 1.04 m in a turn
    assert E.pose_jumped(me) == pytest.approx(1.04, abs=0.01)
    me.ai_walking = True                                             # not on the stairs (expected there)
    assert E.pose_jumped(me) == 0.0
    tick = __import__('inspect').getsource(E.tick)
    assert 'self.queue.clear()' in tick and 'if self.pose_held is not None:' in tick
    assert 'self.align_grid(arrived=True)' in tick
