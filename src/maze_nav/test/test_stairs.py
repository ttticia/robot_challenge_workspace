"""stairs.py: detection on synthetic depth images (stair_scene.py), the chain rules, the
odom tracker, and the approach controller on a simulated robot."""
import math
import os

import numpy as np
import pytest

from maze_nav.stairs import (Approach, Face, StairEstimate, StairParams, StairTracker,
                             WallCheck, detect_stairs, find_chain, in_front, level_transform,
                             quat_to_matrix, refine_lock, to_odom, wrap)
from stair_scene import K, R_BASE_CAM, T_BASE_CAM, maze_boxes, render, staircase

# Example maze: stairs going up towards -x, first riser at x = -2.625, centred on y = 0.
MAZE_EDGE_X, MAZE_HEADING = -2.625, math.pi
SIDE_WALLS = [((0.0, 0.5, 0.0), (3.0, 0.55, 0.45)), ((0.0, -0.55, 0.0), (3.0, -0.5, 0.45))]


def detect(boxes, base, yaw, **kw):
    depth, r_ob = render(boxes, base, yaw, **{k: v for k, v in kw.items()
                                              if k in ('seed', 'pitch', 'roll', 'noise')})
    r, t, _ = level_transform(r_ob, R_BASE_CAM, T_BASE_CAM)
    return detect_stairs(depth, K, r, t, kw.get('params'), floor_z=kw.get('floor_z'))


@pytest.fixture(scope='module')
def maze():
    return maze_boxes()


# ---------------------------------------------------------------- detection
@pytest.mark.parametrize('dist', [0.6, 1.0, 1.5, 2.0])
def test_stairs_straight_ahead(dist):
    det = detect(staircase(dist) + SIDE_WALLS, (0, 0, 0.35), 0.0)
    assert det is not None and det.risers == 3
    assert det.distance == pytest.approx(dist, abs=0.03)
    assert abs(math.degrees(det.yaw)) < 1.5
    assert det.rise == pytest.approx(0.12, abs=0.02)
    assert det.tread == pytest.approx(0.15, abs=0.02)


@pytest.mark.parametrize('turn_deg', [-31.7, -17.6, 7.3, 24.2])
def test_stairs_at_an_angle(turn_deg):
    """Robot turned away from the stairs: yaw is the turn, the other way."""
    det = detect(staircase(1.1, width=2.5), (0, 0, 0.35), math.radians(turn_deg))
    assert det is not None
    assert math.degrees(det.yaw) == pytest.approx(-turn_deg, abs=1.5)
    assert det.distance == pytest.approx(1.1, abs=0.03)


def test_off_centre_lateral():
    det = detect(staircase(1.0, y_c=0.2), (0, 0, 0.35), 0.0)          # (side walls would cut it)
    assert det.centre_ok and det.lateral == pytest.approx(0.2, abs=0.03)


def test_other_rise_and_tread():
    det = detect(staircase(1.2, risers=4, rise=0.17, tread=0.28, width=1.0), (0, 0, 0.35), 0.0)
    assert det is not None and det.risers >= 3
    assert det.rise == pytest.approx(0.17, abs=0.025)
    assert det.tread == pytest.approx(0.28, abs=0.03)


def test_body_pitch_is_taken_out():
    """On a ramp the body pitches; the level frame uses odom's gravity, not base_link."""
    det = detect(staircase(1.2) + SIDE_WALLS, (0, 0, 0.35), 0.0, pitch=math.radians(-6))
    assert det is not None and det.distance == pytest.approx(1.2, abs=0.04)
    assert det.rise == pytest.approx(0.12, abs=0.04)


@pytest.mark.parametrize('name,boxes', [
    ('flat floor', []),
    ('wall 0.45 m', [((1.0, -1, 0), (1.05, 1, 0.45))]),
    ('mat then wall', [((0.8, -1, 0), (1.8, 1, 0.12)), ((1.8, -1, 0), (1.85, 1, 0.45))]),
    ('one step', staircase(1.0, risers=1, top=0.8)),
    ('ramp (1 cm steps)', staircase(0.8, risers=15, rise=0.01, tread=0.03, width=1.0)),
    ('walls of two heights', [((1.0, -1, 0), (1.3, 1, 0.30)), ((1.3, -1, 0), (1.35, 1, 0.75))]),
    ('narrow box stack', staircase(1.0, width=0.2)),
    ('corridor', SIDE_WALLS + [((2.0, -0.5, 0), (2.05, 0.5, 0.45))]),
])
def test_not_stairs(name, boxes):
    assert detect(boxes, (0, 0, 0.35), 0.0) is None, name


@pytest.mark.parametrize('x,y,turn', [(-1.6, 0.0, 0), (-1.75, 0.12, -14), (-1.9, -0.1, 21),
                                      (-1.65, -0.15, 9)])
def test_example_maze(maze, x, y, turn):
    """From the cell before the stairs (floor tiles 0.1 m up, a 0.1 m trench before the
    first riser, rails at the sides): edge, heading and centre in odom."""
    yaw = MAZE_HEADING + math.radians(turn)
    det = detect(maze, (x, y, 0.45), yaw)
    assert det is not None and det.risers == 3
    est = to_odom(det, (x, y), yaw)
    assert abs(math.degrees(wrap(est.heading - MAZE_HEADING))) < 1.5
    assert est.distance(np.array([x, y])) == pytest.approx(x - MAZE_EDGE_X, abs=0.03)
    if est.centre is not None:                  # facing -x, left of the centre line is -y
        assert est.lateral(np.array([x, y])) == pytest.approx(-y, abs=0.04)


@pytest.mark.parametrize('base,yaw', [((0, 0, 0.45), math.pi / 2), ((-2.0, 1.0, 0.55), -math.pi / 2),
                                      ((0, 0.4, 0.45), math.pi), ((-1.0, 1.5, 0.55), math.pi / 2),
                                      ((-1.0, 1.0, 0.55), 0.0)])
def test_example_maze_elsewhere(maze, base, yaw):
    """Mats, ramps, walls and dead ends are not stairs."""
    assert detect(maze, base, yaw) is None


# ---------------------------------------------------------------- chain rules
def face(u, lo, hi, vlo=-0.4, vhi=0.4):
    return Face(u=u, lo=lo, hi=hi, vlo=vlo, vhi=vhi, n=100, col_min=200, col_max=400)


def test_chain_rules():
    p = StairParams()
    # Face ends come out a few cm short (smoothed normals); middles 0.12 apart.
    good = [face(1.0, 0.02, 0.10), face(1.15, 0.14, 0.22), face(1.30, 0.26, 0.34)]
    assert len(find_chain(good, p)) == 3
    assert find_chain(good[:1], p) == []                                   # one riser
    wall = [face(1.0, 0.02, 0.10), face(1.15, 0.12, 0.45)]                 # 0.33 m face
    assert find_chain(wall, p) == []
    far = [face(1.0, 0.02, 0.10), face(1.70, 0.14, 0.22)]                  # 0.7 m tread
    assert find_chain(far, p) == []
    floating = [face(1.0, 0.02, 0.10), face(1.15, 0.30, 0.38)]             # doesn't connect
    assert find_chain(floating, p) == []
    apart = [face(1.0, 0.02, 0.10, -0.4, 0.0), face(1.15, 0.14, 0.22, 0.1, 0.5)]   # side by side
    assert find_chain(apart, p) == []
    uneven = good + [face(1.45, 0.42, 0.56)]                               # 4th rise 0.18 vs 0.12
    assert len(find_chain(uneven, p)) == 3
    # A weak smear (13 points) on the tread behind the bottom riser is skipped, not the end
    # of the chain (it started the stairs one tread late at 1.5 m); a strong face is not.
    smear = face(1.11, 0.01, 0.06)
    smear.n = 13
    assert [f.u for f in find_chain(good[:1] + [smear] + good[1:], p)] == [1.0, 1.15, 1.30]
    smear.n = 60
    assert [f.u for f in find_chain(good[:1] + [smear] + good[1:], p)] == [1.15, 1.30]
    # A low bottom step (10 cm under 12 cm ones) doesn't break the rise rules.
    low = [face(1.0, 0.02, 0.08), face(1.15, 0.12, 0.20), face(1.30, 0.24, 0.32),
           face(1.45, 0.36, 0.44)]
    assert len(find_chain(low, p)) == 4
    # The tread between two faces must be at the lower one's top.
    assert find_chain(good, p, lambda a, b: a.hi + 0.10) == []


# ---------------------------------------------------------------- odom bookkeeping
def est_at(edge_x, heading=math.pi, centre=0.0):
    e = StairEstimate(heading=heading, along=0.0, centre=centre, risers=3, rise=0.12)
    e.along = float(np.dot([edge_x, 0.0], e.d()))
    return e


def test_tracker_votes_and_rejects_one_tread_off():
    tr = StairTracker(window=6, need=3, pos_tol=0.08)
    tr.add(0.0, est_at(-2.625))
    tr.add(0.2, None)
    tr.add(0.4, est_at(-2.775))            # second riser taken for the first
    assert tr.estimate(0.4) is None
    tr.add(0.6, est_at(-2.630))
    tr.add(0.8, est_at(-2.620))
    got = tr.estimate(0.8)
    assert got is not None
    assert got.distance(np.array([-2.0, 0.0])) == pytest.approx(0.625, abs=0.01)
    assert tr.estimate(10.0) is None       # too old


def test_tracker_prefers_a_group_one_tread_nearer():
    """3 frames one tread late (bottom riser missed) and 2 at the bottom riser: with
    nearer_need=2 the nearer group wins; without, the larger one."""
    for need, edge in ((2, -2.625), (0, -2.825)):
        tr = StairTracker(window=6, need=3, pos_tol=0.08, nearer_need=need)
        for i, x in enumerate((-2.825, -2.625, -2.83, -2.62, -2.82)):
            tr.add(0.1 * i, est_at(x))
        got = tr.estimate(0.5)
        assert got.distance(np.array([-2.0, 0.0])) == pytest.approx(-2.0 - edge, abs=0.01)
    tr = StairTracker(window=6, need=3, pos_tol=0.08, nearer_need=2)
    for i, x in enumerate((-2.825, -2.625, -2.83, -2.82)):     # one nearer frame: not enough
        tr.add(0.1 * i, est_at(x))
    assert tr.estimate(0.4).distance(np.array([-2.0, 0.0])) == pytest.approx(0.825, abs=0.01)


def test_refine_lock_freezes_close_up():
    lock = est_at(-2.625)
    late = est_at(-2.67)
    assert refine_lock(lock, late, (-1.5, 0), 0.75, 0.08, 0.2) is late        # far: refined
    assert refine_lock(lock, late, (-2.0, 0), 0.75, 0.08, 0.2) is lock        # close: frozen
    assert refine_lock(lock, est_at(-2.775), (-1.5, 0), 0.75, 0.08, 0.2) is lock   # disagrees
    no_centre = est_at(-2.63, centre=None)
    assert refine_lock(lock, no_centre, (-1.5, 0), 0.75, 0.08, 0.2).centre == 0.0


def test_to_odom():
    """Robot at (1, 2) heading +y; stairs 1 m ahead, 10 deg to its right, 0.1 m left of it."""
    from maze_nav.stairs import Detection
    det = Detection(yaw=math.radians(-10), distance=1.0, lateral=0.1, centre_ok=True, risers=3,
                    rise=0.12, tread=0.15, width=0.8, floor_z=-0.35)
    est = to_odom(det, (1.0, 2.0), math.pi / 2)
    assert math.degrees(est.heading) == pytest.approx(80.0)
    assert est.distance(np.array([1.0, 2.0])) == pytest.approx(1.0)
    assert est.lateral(np.array([1.0, 2.0])) == pytest.approx(-0.1)


# ---------------------------------------------------------------- approach
def simulate(est, xy, yaw, latency=2, steps=1500, dt=0.05, **kw):
    """Unicycle at 20 Hz with `latency` ticks of command delay (the gait reacts late)."""
    ap = Approach(**kw)
    xy = np.array(xy, float)
    queue = [(0.0, 0.0)] * latency
    vs = []
    for i in range(steps):
        v, w = ap.update(i * dt, xy, yaw, est)
        vs.append(v)
        queue.append((v, w))
        v, w = queue.pop(0)
        yaw = wrap(yaw + w * dt)
        xy = xy + v * dt * np.array([math.cos(yaw), math.sin(yaw)])
        if ap.phase == 'DONE':
            break
    return ap, xy, yaw, vs


@pytest.mark.parametrize('xy,yaw', [((-1.4, 0.15), math.pi + 0.6), ((-1.5, -0.2), math.pi - 0.4),
                                    ((-1.2, 0.0), math.pi), ((-1.6, 0.1), math.pi + 1.2)])
def test_approach_lines_up_and_stops(xy, yaw):
    est = est_at(MAZE_EDGE_X)
    ap, end, end_yaw, vs = simulate(est, xy, yaw, stop_distance=0.45)
    assert ap.phase == 'DONE'
    assert min(vs) >= 0.0                                     # never backwards
    assert est.distance(end) == pytest.approx(0.45, abs=0.03)
    assert abs(math.degrees(wrap(end_yaw - est.heading))) < 3.0
    assert abs(est.lateral(end)) < 0.05


def test_approach_without_centre_only_faces_the_stairs():
    est = est_at(MAZE_EDGE_X, centre=None)
    ap, end, end_yaw, _ = simulate(est, (-1.5, 0.2), math.pi + 0.3)
    assert ap.phase == 'DONE'
    assert est.distance(end) == pytest.approx(0.45, abs=0.03)
    assert abs(math.degrees(wrap(end_yaw - est.heading))) < 3.0


def test_approach_too_close_only_turns():
    est = est_at(MAZE_EDGE_X)
    start = np.array([-2.25, 0.05])                           # 0.375 m from the riser
    ap, end, end_yaw, vs = simulate(est, start, math.pi + 0.5)
    assert ap.phase == 'DONE' and max(vs) == 0.0
    assert np.allclose(end, start)
    assert abs(math.degrees(wrap(end_yaw - est.heading))) < 3.0


def test_closed_loop_in_the_example_maze(maze):
    """Detection on rendered frames + tracker + lock + approach, robot starting off-centre and
    turned away, in the cell before the stairs. Ends square, centred, 0.45 m from the riser."""
    xy, yaw = np.array([-1.62, 0.12]), math.pi + math.radians(25)
    tr = StairTracker(window=6, need=3, pos_tol=0.08, yaw_tol=math.radians(8))
    lock, ap, frame = None, Approach(stop_distance=0.45), 0
    for i in range(1200):
        if i % 8 == 0 and (lock is None or ap.phase in ('ALIGN', 'APPROACH')):   # 2.5 Hz
            depth, r_ob = render(maze, (xy[0], xy[1], 0.45), yaw, seed=frame)
            frame += 1
            r, t, _ = level_transform(r_ob, R_BASE_CAM, T_BASE_CAM)
            det = detect_stairs(depth, K, r, t)
            tr.add(i * 0.05, to_odom(det, xy, yaw) if det is not None and det.front_seen
                   else None)
            found = tr.estimate(i * 0.05)
            if lock is None:
                lock = found
            elif found is not None:
                lock = refine_lock(lock, found, xy, 0.75, 0.08, math.radians(8))
        if lock is None:
            continue
        v, w = ap.update(i * 0.05, xy, yaw, lock)
        yaw = wrap(yaw + w * 0.05)
        xy = xy + v * 0.05 * np.array([math.cos(yaw), math.sin(yaw)])
        if ap.phase == 'DONE':
            break
    assert ap.phase == 'DONE'
    assert (xy[0] - MAZE_EDGE_X) == pytest.approx(0.45, abs=0.04)
    assert abs(math.degrees(wrap(yaw - MAZE_HEADING))) < 3.0
    assert abs(xy[1]) < 0.05


# ---------------------------------------------------------------- wall checks (WallCheck)
def maze_wall_grid(res=0.05):
    """The example maze's walls and rails as SLAM would map them (5 cm cells), maze frame."""
    import json
    import os
    from maze_nav.core import Grid
    ox, oy = -5.0, -3.0
    data = np.zeros((int(8 / res), int(8 / res)), np.int16)
    with open(os.path.join(os.path.dirname(__file__), 'example_maze.json')) as f:
        for s in json.load(f)['solids']:
            if s['kind'] in ('wall', 'rail'):
                (x0, y0, _), (x1, y1, _) = s['min'], s['max']
                data[int((y0 - oy) / res):int(math.ceil((y1 - oy) / res)),
                     int((x0 - ox) / res):int(math.ceil((x1 - ox) / res))] = 100
    return Grid(data, res, (ox, oy))


def line_up(est, xy, yaw, walls, latency=2, steps=2500, dt=0.05):
    """simulate() plus the deepest footprint overlap with a wall (no margin) on the way."""
    ap = Approach(stop_distance=0.45, walls=walls)
    xy, queue, worst = np.array(xy, float), [(0.0, 0.0)] * latency, 0
    for i in range(steps):
        v, w = ap.update(i * dt, xy, yaw, est)
        queue.append((v, w))
        v, w = queue.pop(0)
        yaw = wrap(yaw + w * dt)
        xy = xy + v * dt * np.array([math.cos(yaw), math.sin(yaw)])
        if walls is not None:
            worst = max(worst, walls.overlap(xy, yaw, 0.0))
        if ap.phase in ('DONE', 'BLOCKED'):
            break
    return ap, xy, yaw, worst


def test_wall_check_cuts_a_command_that_would_hit():
    from maze_nav.core import Grid
    data = np.zeros((80, 80), np.int16)
    data[:, 50:] = 100                                   # wall from x = 0.5 (origin -2, 5 cm)
    walls = WallCheck(Grid(data, 0.05, (-2.0, -2.0)), margin=0.03, min_margin=0.01)
    # Front at 0.27: 0.23 m to the wall. Creeping at 0.08 m/s for 0.5 s is fine, 0.5 m/s not.
    assert walls.safe((0.0, 0.0), 0.0, 0.08, 0.0) == (0.08, 0.0, '')
    v, w, why = walls.safe((0.0, 0.0), 0.0, 0.5, 0.0)
    assert (v, w) == (0.0, 0.0) and 'driving' in why
    v, w, why = walls.safe((0.0, 0.0), 0.0, 0.5, 0.3)    # keep the turn, drop the drive
    assert (v, w) == (0.0, 0.3) and why
    # The odom -> map offset is applied: same robot pose, odom shifted 0.2 m towards the wall.
    shifted = WallCheck(walls.grid, offset=(0.2, 0.0, 0.0))
    assert shifted.safe((0.0, 0.0), 0.0, 0.08, 0.0)[:2] == (0.0, 0.0)


def test_stop_pose_is_moved_off_a_wall():
    from maze_nav.core import Grid
    est = est_at(MAZE_EDGE_X)
    data = np.zeros((120, 120), np.int16)                # origin (-4, -3), 5 cm
    data[int((-0.28 + 3) / 0.05):int((-0.23 + 3) / 0.05), int((-2.6 + 4) / 0.05):
         int((-1.9 + 4) / 0.05)] = 100                   # a post line 0.23..0.28 m right of centre
    walls = WallCheck(Grid(data, 0.05, (-4.0, -3.0)))
    ap = Approach(stop_distance=0.45, walls=walls)
    ap.plan_stop(np.array([-1.6, 0.0]), est)
    stop = ap.stop_point(est)
    assert stop[1] > 0.0 and ap.stop_extra == 0.0        # moved to +y, away from the post
    assert abs(stop[1]) <= 0.06                          # by a few cm only
    assert walls.pose_clear(stop, est.heading, walls.min_margin)
    assert 'sideways' in ap.note


@pytest.fixture(scope='module')
def handover_poses():
    """Wall-free poses in the cell before the stairs, roughly facing them (the camera only
    sees stairs within its field of view, so a handover never starts turned away)."""
    walls = WallCheck(maze_wall_grid())
    est, rng, out = est_at(MAZE_EDGE_X), np.random.default_rng(1), []
    while len(out) < 30:
        xy = (rng.uniform(-2.45, -1.95), rng.uniform(-0.3, 0.35))
        yaw = math.pi + rng.uniform(-0.7, 0.7)
        if walls.pose_clear(xy, yaw, 0.01) and est.distance(np.array(xy)) >= 0.3:
            out.append((xy, yaw))
    return out


def test_line_up_never_touches_a_wall_in_the_example_maze(handover_poses):
    est, walls = est_at(MAZE_EDGE_X), WallCheck(maze_wall_grid())
    phases = []
    for xy, yaw in handover_poses:
        ap, end, end_yaw, worst = line_up(est, xy, yaw, walls)
        assert worst == 0, (xy, yaw, ap.phase, ap.note)             # never touches a wall
        phases.append(ap.phase)
        if ap.phase == 'BLOCKED':
            assert ap.note, (xy, yaw)                                 # says why it holds
        else:
            assert ap.phase == 'DONE'
            err = abs(math.degrees(wrap(end_yaw - est.heading)))
            assert err < 3.0 or ('only' in ap.note and err <= 8.0), (xy, yaw, err, ap.note)
    assert phases.count('DONE') >= 0.75 * len(phases), phases       # most still line up


def test_without_wall_checks_some_line_ups_scrape(handover_poses):
    # Why the checks are there: the same handovers without them touch walls or rails.
    est, walls = est_at(MAZE_EDGE_X), WallCheck(maze_wall_grid())
    touched = 0
    for xy, yaw in handover_poses:
        ap = Approach(stop_distance=0.45)
        p, y = np.array(xy, float), yaw
        for i in range(2500):
            v, w = ap.update(i * 0.05, p, y, est)
            y = wrap(y + w * 0.05)
            p = p + v * 0.05 * np.array([math.cos(y), math.sin(y)])
            if walls.overlap(p, y, 0.0):
                touched += 1
                break
            if ap.phase == 'DONE':
                break
    assert touched >= 1


def test_blocked_squaring_holds_instead_of_grinding():
    # Already inside stop_distance (can't drive), 40 deg off, rear corner near the east...
    # nearer the rails: squaring up would swing the body into a rail -> BLOCKED, no contact.
    est, walls = est_at(MAZE_EDGE_X), WallCheck(maze_wall_grid())
    xy, yaw = (-2.27, 0.17), math.pi - math.radians(40)
    assert walls.pose_clear(xy, yaw, 0.01)
    ap, end, end_yaw, worst = line_up(est, xy, yaw, walls)
    assert worst == 0
    assert ap.phase in ('BLOCKED', 'DONE')
    if ap.phase == 'BLOCKED':
        assert 'squaring' in ap.note or 'turn' in ap.note or 'hit' in ap.note


# ---------------------------------------------------------------- hand-over gate
def test_low_risers_are_not_stairs():
    """Steps count from 8 cm (user 2026-11-21): 6.5 cm chains (robot clutter while standing
    up, real lab depth) are not stairs; 9 cm stairs still start at their first riser (one
    faint step must not shift the stairs a tread late)."""
    assert detect(staircase(1.2, rise=0.065, tread=0.25, width=1.0), (0, 0, 0.35), 0.0) is None
    det = detect(staircase(1.2, rise=0.09, tread=0.30, width=1.0), (0, 0, 0.35), 0.0)
    assert det is not None and det.distance == pytest.approx(1.2, abs=0.03)


LOW_FIRST = dict(risers=4, rise=0.13, tread=0.20, first_rise=0.10)   # the challenge stairs


@pytest.mark.parametrize('seed', [0, 1, 2])
@pytest.mark.parametrize('dist', [0.55, 0.9, 1.2, 1.5, 1.8])
@pytest.mark.parametrize('rise', [0.10, 0.13, 0.17])
def test_low_first_step_starts_at_the_bottom_riser(dist, rise, seed):
    """10 cm bottom step under 10-17 cm steps: the stairs start at the bottom riser, never a
    tread late (1.5 m: a smear behind the bottom riser cut the chain; 1.2 m: a 4-face ghost
    direction beat the real stairs). The rise is the upper steps', first_rise the bottom's
    (info only; a floor that reads high never adds a step)."""
    det = detect(staircase(dist, **dict(LOW_FIRST, rise=rise)) + SIDE_WALLS, (0, 0, 0.35), 0.0,
                 seed=seed)
    assert det is not None and not det.inferred_bottom
    assert det.distance == pytest.approx(dist, abs=0.03)
    assert abs(math.degrees(det.yaw)) < 1.5
    assert det.rise == pytest.approx(rise, abs=0.015)
    if dist >= 0.9:     # nearer, the camera's floor is the 10 cm tread (floor_band): reads ~0
        assert det.first_rise == pytest.approx(0.10, abs=0.025)


@pytest.mark.parametrize('dist', [0.35, 0.40, 0.45])
@pytest.mark.parametrize('floor_err', [-0.02, 0.0, 0.02])
def test_bottom_riser_below_the_view_is_inferred_on_a_measured_floor(dist, floor_err):
    """Close up the bottom riser is below the view. With the floor measured (lidar map) and
    the 10 cm tread seen in front of the 2nd riser, the bottom riser is put one tread
    nearer; without a measured floor it isn't (the first face found is the 2nd riser)."""
    boxes = staircase(dist, **LOW_FIRST) + SIDE_WALLS
    det = detect(boxes, (0, 0, 0.35), 0.0, floor_z=-0.35 + floor_err)
    assert det is not None and det.inferred_bottom
    assert det.distance == pytest.approx(dist, abs=0.03)
    blind = detect(boxes, (0, 0, 0.35), 0.0)
    assert not blind.inferred_bottom and blind.distance == pytest.approx(dist + 0.20, abs=0.03)


@pytest.mark.parametrize('dist', [0.6, 1.0, 1.5])
def test_no_bottom_riser_inferred_when_it_is_in_view(dist):
    det = detect(staircase(dist, **LOW_FIRST) + SIDE_WALLS, (0, 0, 0.35), 0.0, floor_z=-0.35)
    assert not det.inferred_bottom and det.distance == pytest.approx(dist, abs=0.03)


def test_in_front_with_centre():
    est = est_at(MAZE_EDGE_X)                                 # centre line y = 0
    assert in_front(est, (-2.0, 0.10))[0]
    assert in_front(est, (-2.0, -0.30))[0]
    ok, off = in_front(est, (-2.0, 0.80))                     # studded-mat cell, beside them
    assert not ok and off == pytest.approx(0.80)


def test_in_front_without_centre_uses_the_riser_span():
    from maze_nav.stairs import Detection
    est = est_at(MAZE_EDGE_X, centre=None)
    det = Detection(yaw=0.0, distance=0.6, lateral=0.0, centre_ok=False, risers=3, rise=0.12,
                    tread=0.15, width=0.5, floor_z=-0.35, span_lo=-0.2, span_hi=0.3)
    assert in_front(est, (-2.0, 0.0), det)[0]
    det.span_lo, det.span_hi = 0.3, 0.9                       # risers all to the left
    assert not in_front(est, (-2.0, 0.0), det)[0]
    assert not in_front(est, (-2.0, 0.0), None)[0]            # nothing to go on: wait


@pytest.mark.parametrize('x,y,turn,expect', [
    (-1.62, 0.12, 25, True), (-2.0, -0.08, -20, True), (-1.8, 0.25, 15, True),   # in front
    (-2.0, 0.8, 45, False), (-2.15, 0.93, 65, False),         # from the studded mat, beside
])
def test_example_maze_hand_over_gate(maze, x, y, turn, expect):
    """Detection + in_front on rendered frames: from the studded-mat cell north of the stairs
    they are seen close along their direction, but the robot is 0.8-0.9 m beside them."""
    yaw = MAZE_HEADING + math.radians(turn)
    det = detect(maze, (x, y, 0.45 if expect else 0.55), yaw)
    assert det is not None
    assert in_front(to_odom(det, (x, y), yaw), (x, y), det)[0] is expect


# ---------------------------------------------------------------- real stairs, live robot
def test_camera_from_link_matches_the_orbbec_chain():
    from maze_nav.stairs import camera_from_link, mount_disagreement
    r, t = camera_from_link(quat_to_matrix(0.0, 0.1736, 0.0, 0.9848), [0.2523, -0.0005, 0.0741])
    assert np.allclose(r, R_BASE_CAM, atol=1e-6) and np.allclose(t, T_BASE_CAM)
    # The unpitched second parent seen on the robot's /tf_static on 2026-11-18.
    ang, _ = mount_disagreement((r, t), (quat_to_matrix(-0.5, 0.5, -0.5, 0.5),
                                         [0.25, 0.0, 0.05]))
    assert ang == pytest.approx(20.0, abs=0.1)


def test_live_stairs_close_up():
    """Real depth (2026-11-18), robot standing ~0.5 m from real stairs (13 cm rise, ~20 cm
    tread), turned ~6 deg, no odom (body level within ~2 deg). Close up the floor is out of
    view; the bottom riser's lower part is cut off, but it is the first face found."""
    z = np.load(os.path.join(os.path.dirname(__file__), 'data', 'live_stairs_close.npz'))
    k = tuple(z['K'])
    r, t, _ = level_transform(np.eye(3), R_BASE_CAM, T_BASE_CAM)
    for depth in z['depth']:
        det = detect_stairs(depth, k, r, t)
        assert det is not None and det.risers >= 3
        assert det.distance == pytest.approx(0.51, abs=0.03)
        assert math.degrees(det.yaw) == pytest.approx(6.0, abs=1.5)
        assert det.rise == pytest.approx(0.125, abs=0.03)
        assert in_front(to_odom(det, (0.0, 0.0), 0.0), (0.0, 0.0), det)[0]
