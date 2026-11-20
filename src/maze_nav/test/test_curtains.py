"""Unit tests for maze_nav.curtains (no ROS): python3 -m pytest -q test/test_curtains.py"""
import math

import numpy as np
import pytest

from extru_wall import extru_boxes, windows

from maze_nav.core import Grid
from maze_nav.curtains import (BlockageMonitor, Curtain, CurtainTracker,
                               classify_bins, clear_curtains, curtains_from_json,
                               curtains_to_json, depth_blockage, depth_points,
                               detect_curtains, in_curtains, lidar_blockage)

LIDAR_H = 0.475          # lidar above the floor (Lite3 standing)


def raycast(boxes, origin=(0.0, 0.0, LIDAR_H), yaw=0.0, az_step=0.4, el_step=0.5,
            el_range=(-22.0, 37.0), az_range=(-70.0, 70.0), max_r=8.0):
    """First hits of lidar rays on axis-aligned boxes and the floor (z = 0): like the
    Mid-360, with occlusion. boxes: [(x0, y0, z0, x1, y1, z1)]."""
    az = np.radians(np.arange(az_range[0], az_range[1], az_step)) + yaw
    el = np.radians(np.arange(el_range[0], el_range[1], el_step))
    A, E = np.meshgrid(az, el)
    d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    o = np.asarray(origin, float)
    t_best = np.full(len(d), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        tf = np.where(d[:, 2] < 0, -o[2] / d[:, 2], np.inf)
        t_best = np.minimum(t_best, tf)
        for b in boxes:
            lo, hi = np.array(b[:3]), np.array(b[3:])
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin = np.nanmax(np.minimum(t1, t2), axis=1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tmax >= tmin) & (tmax > 0)
            t = np.where(hit, np.maximum(tmin, 0), np.inf)
            t_best = np.minimum(t_best, t)
    ok = np.isfinite(t_best) & (t_best < max_r)
    return o + d[ok] * t_best[ok, None]


def corridor(front=None, length=5.0, half=0.45):
    """Corridor along +x, walls 0.45 m tall; `front` = box across it."""
    boxes = [(-0.5, half, 0, length, half + 0.05, 0.45),
             (-0.5, -half - 0.05, 0, length, -half, 0.45)]
    if front is not None:
        boxes.append(front)
    return boxes


def curtain_box(x=1.6, gap=0.08, half=0.45, top=0.8):
    return (x, -half, gap, x + 0.02, half, top)


def detect(boxes, xy=(0.0, 0.0), yaw=0.0):
    pts = raycast(boxes, origin=(xy[0], xy[1], LIDAR_H), yaw=yaw)
    return detect_curtains(pts, xy, yaw, 0.0)


# ------------------------------------------------------------------ detection

def test_curtain_with_low_gap_is_found():
    found = detect(corridor(curtain_box(gap=0.08)))
    assert len(found) == 1
    c = found[0]
    assert c.width == pytest.approx(0.9, abs=0.12)
    assert c.width > 0.7
    assert c.gap == pytest.approx(0.08, abs=0.04)
    assert c.centre[0] == pytest.approx(1.6, abs=0.05)
    assert c.centre[1] == pytest.approx(0.0, abs=0.08)
    assert c.n[0] > 0.99                     # points away from the robot
    assert c.signed((0.0, 0.0)) < 0


def test_curtain_found_from_off_centre_and_at_an_angle():
    found = detect(corridor(curtain_box(x=1.8)), xy=(0.0, 0.15), yaw=math.radians(15))
    assert len(found) == 1
    assert found[0].centre[0] == pytest.approx(1.8, abs=0.06)


def test_plain_wall_is_not_a_curtain():
    assert detect(corridor((1.6, -0.45, 0.0, 1.65, 0.45, 0.45))) == []
    cells = classify_bins(raycast(corridor((1.6, -0.45, 0.0, 1.65, 0.45, 0.45))),
                          (0, 0), 0.0, 0.0)
    kinds = {c['kind'] for c in cells if c['r'] is not None and abs(c['bearing']) < 0.2}
    assert kinds == {'wall'}


def test_gap_of_15_cm_or_more_is_not_a_curtain():
    assert detect(corridor(curtain_box(gap=0.20))) == []
    cells = classify_bins(raycast(corridor(curtain_box(gap=0.20))), (0, 0), 0.0, 0.0)
    assert 'high' in {c['kind'] for c in cells}


def test_curtain_narrower_than_70_cm_is_not_a_curtain():
    # 0.5 m of curtain, wall on either side of it
    boxes = corridor() + [(1.6, -0.45, 0.0, 1.65, -0.25, 0.45), (1.6, 0.25, 0.0, 1.65, 0.45, 0.45),
                          (1.6, -0.25, 0.08, 1.62, 0.25, 0.8)]
    assert detect(boxes) == []


def test_curtain_beyond_range_or_too_close_is_not_seen():
    assert detect(corridor(curtain_box(x=3.5))) == []
    # 0.7 m away the lidar can't see the floor at the curtain: no claim either way
    assert detect(corridor(curtain_box(x=0.7))) == []


def test_curtain_on_raised_ground_uses_the_local_floor():
    # a 0.12 m mat under and around the curtain; gap 0.08 above the mat
    mat = (0.9, -0.45, 0.0, 3.0, 0.45, 0.12)
    found = detect(corridor() + [mat, curtain_box(gap=0.20)])
    assert len(found) == 1
    assert found[0].gap == pytest.approx(0.08, abs=0.05)


# ------------------------------------------------------------------ tracking, map, json

def test_tracker_confirms_after_votes_and_keeps_it():
    tr = CurtainTracker(window=3.0, need=3)
    c = detect(corridor(curtain_box()))[0]
    assert tr.add(0.0, [c]) == []
    assert tr.add(0.5, [c]) == []
    new = tr.add(1.0, [c])
    assert len(new) == 1 and new[0].id == 1
    assert tr.add(20.0, []) == []
    assert len(tr.confirmed) == 1           # never forgotten


def test_tracker_drops_one_off_detections():
    tr = CurtainTracker(window=3.0, need=3)
    c = detect(corridor(curtain_box()))[0]
    tr.add(0.0, [c])
    tr.add(10.0, [])
    assert tr.tracks == []


def test_in_curtains_and_clear_curtains():
    c = Curtain(np.array([1.6, -0.45]), np.array([1.6, 0.45]), np.array([1.0, 0.0]))
    pts = np.array([[1.6, 0.0], [1.7, 0.3], [1.6, 0.6], [2.0, 0.0]])
    assert in_curtains(pts, [c]).tolist() == [True, True, False, False]
    data = np.zeros((40, 60), np.int16)
    data[:, 32] = 100                         # wall at x = 1.6..1.65 (res 0.05, origin 0, -1)
    g = Grid(data, 0.05, (0.0, -1.0))
    out = clear_curtains(g, [c])
    col = out.data[:, 32]
    rows = np.arange(40)
    y = -1.0 + (rows + 0.5) * 0.05
    assert (col[np.abs(y) < 0.4] == 0).all()
    assert (col[np.abs(y) > 0.55] == 100).all()
    assert (g.data[:, 32] == 100).all()      # original untouched


def test_json_round_trip():
    c = Curtain(np.array([1.0, 2.0]), np.array([1.0, 3.0]), np.array([-1.0, 0.0]), 0.07, 4,
                [1])
    frame, out = curtains_from_json(curtains_to_json([c], 'odom'))
    assert frame == 'odom' and len(out) == 1
    assert out[0].id == 4 and out[0].passed == [1]
    assert np.allclose(out[0].p2, [1.0, 3.0])
    assert curtains_from_json('nonsense') == ('', [])


def test_transformed_and_side():
    c = Curtain(np.array([1.0, -0.5]), np.array([1.0, 0.5]), np.array([1.0, 0.0]))
    assert c.side_of((0.0, 0.0)) == 1 and c.side_of((2.0, 0.0)) == -1
    t = c.transformed(0.0, 0.0, math.pi / 2)
    assert np.allclose(t.centre, [0.0, 1.0])
    assert np.allclose(t.n, [0.0, 1.0])


# ------------------------------------------------------------------ blockage

def test_lidar_blockage_counts():
    pts = raycast(corridor(curtain_box()))
    base_xy, lid = (-0.19, 0.0), (0.0, 0.0, LIDAR_H)
    near, far = lidar_blockage(pts, lid, base_xy, 0.0)
    assert far > 1000 and near < 0.05 * far
    # fabric 0.15 m in front of the lidar over the whole front
    draped = raycast([(0.15, -1, 0, 0.17, 1, 1)])
    near, far = lidar_blockage(draped, lid, base_xy, 0.0)
    assert near > far


def test_blockage_monitor():
    m = BlockageMonitor()
    for _ in range(10):
        assert m.update(10, 5000, learn=True)[0] is False
    assert m.update(4000, 1000, learn=False)[0] is True       # near returns
    assert m.update(0, 1000, learn=False)[0] is True          # returns missing
    assert m.update(10, 4500, learn=False)[0] is False


def test_depth_blockage():
    clear = np.full((480, 640), 1.5, np.float32)
    assert depth_blockage(clear)[0] is False
    covered = np.zeros((480, 640), np.float32)
    covered[:50] = 1.5
    assert depth_blockage(covered)[0] is True
    assert depth_blockage(np.full((480, 640), 0.1, np.float32))[0] is True


def test_wall_with_floor_seen_behind_it_is_still_a_wall():
    # no occlusion (like the plain simulator): floor behind a wall that stands on the floor
    pts = np.concatenate([raycast(corridor((1.6, -0.45, 0.0, 1.65, 0.45, 0.45))),
                          raycast(corridor())])
    assert detect_curtains(pts, (0, 0), 0.0, 0.0) == []


# ------------------------------------------------------------------ depth camera, lidar + depth
# Robot base_link at the origin facing +x. Lite3 mounts (bag curtain_test1, /tf_static):
# lidar 0.187 m ahead, 0.475 m up; Orbbec camera_link 0.252 m ahead, 0.424 m up, pitched 20 deg
# down. Depth rendered at 160 x 120 with the 640 x 480 intrinsics / 4.
LIDAR_X = 0.187
CAM_X, CAM_H, CAM_PITCH = 0.252, 0.424, math.radians(20)
K4 = (372.79 / 4, 372.79 / 4, 319.4 / 4, 241.5 / 4)


def camera_pose(base=(0.0, 0.0), yaw=0.0):
    """(R, t): depth optical frame -> world."""
    r_lo = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)      # optical -> camera_link
    cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    r_p = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    c, s = math.cos(yaw), math.sin(yaw)
    r_y = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return r_y @ r_p @ r_lo, np.array([base[0] + c * CAM_X, base[1] + s * CAM_X, CAM_H])


def render_depth(boxes, base=(0.0, 0.0), yaw=0.0, w=160, h=120, k=K4):
    """Depth image (optical z, m; 0 = nothing) of boxes and the floor."""
    fx, fy, cx, cy = k
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    r, o = camera_pose(base, yaw)
    d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones((h, w))], -1).reshape(-1, 3) @ r.T
    with np.errstate(divide='ignore', invalid='ignore'):
        t_best = np.where(d[:, 2] < 0, -o[2] / d[:, 2], np.inf)
        for b in boxes:
            lo, hi = np.array(b[:3]), np.array(b[3:])
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin = np.nanmax(np.minimum(t1, t2), axis=1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=1)
            t_best = np.minimum(t_best, np.where((tmax >= tmin) & (tmax > 0),
                                                 np.maximum(tmin, 0), np.inf))
    z = np.where(np.isfinite(t_best) & (t_best < 6), t_best, 0.0)    # optical z = t (z = 1)
    return z.reshape(h, w).astype(np.float32)


def sense(boxes, mode, dist_base=None, base=(0.0, 0.0), yaw=0.0):
    """detect_curtains on lidar, depth or both from a robot at base (base_link)."""
    lid = (base[0] + math.cos(yaw) * LIDAR_X, base[1] + math.sin(yaw) * LIDAR_X, LIDAR_H)
    r, t = camera_pose(base, yaw)
    parts = []
    if mode in ('lidar', 'both'):
        parts.append(raycast(boxes, origin=lid, yaw=yaw))
    if mode in ('depth', 'both'):
        parts.append(depth_points(render_depth(boxes, base, yaw), K4, r, t, stride=1))
    origin = lid if mode != 'depth' else t
    return detect_curtains(np.concatenate(parts), origin[:2], yaw, 0.0)


def curtain_at(d, gap=0.08):
    """Corridor with a curtain d m ahead of base_link."""
    return corridor(curtain_box(x=d, gap=gap))


def test_depth_points_land_on_the_floor_and_the_face():
    boxes = curtain_at(1.0)
    r, t = camera_pose()
    pts = depth_points(render_depth(boxes), K4, r, t, stride=1)
    floor = pts[(pts[:, 0] < 0.95) & (np.abs(pts[:, 1]) < 0.4)]     # between the side walls
    assert len(floor) > 100
    assert np.abs(floor[:, 2]).max() < 0.02           # 20 deg pitch: the floor comes out flat
    face = pts[(pts[:, 2] > 0.1) & (np.abs(pts[:, 1]) < 0.3)]
    assert np.median(face[:, 0]) == pytest.approx(1.0, abs=0.02)
    assert depth_points(np.zeros((120, 160), np.float32), K4, r, t).shape == (0, 3)


@pytest.mark.parametrize('d', [0.6, 0.8, 1.0, 1.5])
def test_lidar_and_depth_find_the_curtain_from_0_6_to_1_5_m(d):
    found = sense(curtain_at(d), 'both')
    assert len(found) == 1
    assert found[0].centre[0] == pytest.approx(d, abs=0.05)
    assert found[0].gap == pytest.approx(0.08, abs=0.03)
    assert found[0].width > 0.7


@pytest.mark.parametrize('d', [0.6, 0.8, 1.0])
def test_lidar_alone_cant_see_under_a_curtain_closer_than_1_2_m(d):
    assert sense(curtain_at(d), 'lidar') == []


def test_depth_alone_works_from_0_6_m_when_it_sees_the_whole_width():
    assert len(sense(curtain_at(0.8), 'depth')) == 1
    # min_width 0.60 (user 2026-11-20): at 0.6 m its +-40 deg sees ~0.62 m - just enough;
    # at 0.55 m it sees less than min_width: width not proven
    assert len(sense(curtain_at(0.6), 'depth')) == 1
    assert sense(curtain_at(0.55), 'depth') == []


def test_at_0_5_m_nothing_can_tell_a_curtain_from_a_wall():
    # the camera's lowest ray meets the curtain ~10 cm up and never reaches the floor behind it
    assert sense(curtain_at(0.5), 'both') == []


def test_off_centre_and_turned_lidar_and_depth():
    found = sense(curtain_at(1.0), 'both', base=(0.0, 0.15), yaw=math.radians(20))
    assert len(found) == 1
    assert found[0].centre[0] == pytest.approx(1.0, abs=0.05)


@pytest.mark.parametrize('d', [0.6, 0.8, 1.2])
@pytest.mark.parametrize('mode', ['lidar', 'depth', 'both'])
def test_close_walls_hurdles_and_narrow_curtains_are_not_curtains(d, mode):
    wall = corridor((d, -0.45, 0.0, d + 0.05, 0.45, 0.45))
    high = curtain_at(d, gap=0.20)        # also: lidar corner bins where side walls meet it
    on_floor = curtain_at(d, gap=0.03)
    narrow = corridor() + [(d, -0.45, 0.0, d + 0.05, -0.25, 0.45),
                           (d, 0.25, 0.0, d + 0.05, 0.45, 0.45),
                           (d, -0.25, 0.08, d + 0.02, 0.25, 0.8)]
    for boxes in (wall, high, on_floor, narrow):
        assert sense(boxes, mode) == []


# ------------------------------------------------------------------ extru walls
# resource/extru wall.stl (test/extru_wall.py): a 0.9 m aluminium frame with a corrugated board
# on one face, and a 10 cm x 37 cm window right through it at each end (0.31-0.41 m from the
# centre, z 0.04-0.41). Floor is seen through the windows; only the top rail is in the band.

def extru_front(d, board_side=-1):
    """Extru wall across the corridor, d m ahead; board_side -1 = board facing the robot."""
    return extru_boxes((d, -0.45, 0.0), (d + 0.04, 0.45, 0.45), board_side)


def extru_corridor(front=(), length=5.0, half=0.45):
    """Corridor along +x of 0.9 m extru walls (10 cm gaps at the cell corners), boards inside."""
    boxes = []
    for c in np.arange(0.0, length, 1.0):
        boxes += extru_boxes((c - 0.45, half, 0.0), (c + 0.45, half + 0.04, 0.45), -1)
        boxes += extru_boxes((c - 0.45, -half - 0.04, 0.0), (c + 0.45, -half, 0.45), +1)
    return boxes + list(front)


def test_extru_boxes_leave_see_through_windows():
    lo, hi = (1.0, -0.45, 0.0), (1.04, 0.45, 0.45)
    w = windows(lo, hi)
    assert w[0] == pytest.approx((-0.41, -0.31))
    assert w[1] == pytest.approx((0.31, 0.41))
    # a lidar ray along x at window height and y = 0.36 passes the wall; at y = 0 it stops
    backstop = (2.0, -2.0, 0.0, 2.05, 2.0, 1.0)
    pts = raycast(extru_boxes(lo, hi) + [backstop], origin=(0.0, 0.0, 0.2), az_range=(-30, 30),
                  el_range=(-0.25, 0.25), max_r=4.0)
    hits = {round(float(np.degrees(np.arctan2(p[1], p[0])))): p[0] for p in pts}
    assert all(abs(x - 1.0) < 0.01 for a, x in hits.items() if abs(a) <= 15)   # board
    assert all(x > 1.9 for a, x in hits.items() if 18 <= abs(a) <= 21)         # window


@pytest.mark.parametrize('d', [1.2, 1.6, 2.2])
@pytest.mark.parametrize('board_side', [-1, 1])
def test_extru_wall_ahead_is_a_wall_not_a_curtain(d, board_side):
    boxes = corridor() + extru_front(d, board_side)
    assert detect(boxes) == []
    cells = classify_bins(raycast(boxes), (0, 0), 0.0, 0.0)
    kinds = [c['kind'] for c in cells if c['r'] is not None]
    assert 'curtain' not in kinds
    ahead = {c['kind'] for c in cells if c['r'] is not None and abs(c['bearing']) < 0.15}
    assert ahead == {'wall'}            # the board reaches the floor


@pytest.mark.parametrize('d', [0.6, 0.8, 1.2])
@pytest.mark.parametrize('mode', ['lidar', 'depth', 'both'])
def test_extru_wall_ahead_lidar_and_depth(d, mode):
    for side in (-1, 1):
        assert sense(corridor() + extru_front(d, side), mode) == []
        assert sense(extru_corridor(extru_front(d, side)), mode) == []


def test_extru_corridor_has_no_curtains_but_a_real_curtain_is_found():
    assert detect(extru_corridor()) == []
    assert sense(extru_corridor(), 'both') == []
    found = detect(extru_corridor([curtain_box(x=1.6)]))
    assert len(found) == 1
    assert found[0].centre[0] == pytest.approx(1.6, abs=0.05)
    assert found[0].width > 0.7
    for d in (0.8, 1.2):
        found = sense(extru_corridor([curtain_box(x=d)]), 'both')
        assert len(found) == 1
        assert found[0].centre[0] == pytest.approx(d, abs=0.05)


def test_extru_corridor_seen_at_an_angle_has_no_curtains():
    for xy, yaw in (((0.0, 0.2), math.radians(25)), ((0.3, -0.2), math.radians(-30)),
                    ((0.5, 0.0), math.radians(60))):
        assert detect(extru_corridor(extru_front(1.8)), xy=xy, yaw=yaw) == []


# ------------------------------------------------------------------ wall-scan override

def test_crossed_and_crossing_round_trip_and_transform():
    from maze_nav.curtains import scan_curtains
    a = Curtain(np.array([1.0, -0.5]), np.array([1.0, 0.5]), np.array([1.0, 0.0]), id=1,
                crossed=True)
    b = Curtain(np.array([3.0, -0.5]), np.array([3.0, 0.5]), np.array([1.0, 0.0]), id=2,
                crossing=True)
    c = Curtain(np.array([5.0, -0.5]), np.array([5.0, 0.5]), np.array([1.0, 0.0]), id=3)
    _, out = curtains_from_json(curtains_to_json([a, b, c], 'map'))
    assert [(o.crossed, o.crossing) for o in out] == [(True, False), (False, True),
                                                      (False, False)]
    assert [o.id for o in scan_curtains(out)] == [1, 2]       # never crossed: stays a wall
    assert a.transformed(1.0, 2.0, 0.3).crossed and b.transformed(0, 0, 1).crossing
    # old messages (no fields) read as not crossed
    _, old = curtains_from_json('{"frame": "odom", "curtains": [{"p1": [0, 0], "p2": [1, 0],'
                                ' "n": [0, 1]}]}')
    assert scan_curtains(old) == []


def test_invert_pose_undoes_transformed():
    from maze_nav.curtains import invert_pose
    c = Curtain(np.array([1.0, -0.5]), np.array([1.2, 0.5]), np.array([1.0, -0.2]))
    t = (0.7, -1.3, 0.9)
    back = c.transformed(*t).transformed(*invert_pose(*t))
    assert np.allclose(back.p1, c.p1) and np.allclose(back.p2, c.p2)
    assert np.allclose(back.n, c.n)


def test_curtain_mask_widens_near_the_robot_but_keeps_side_walls():
    from maze_nav.curtains import curtain_mask
    # curtain across a 1 m corridor at x = 2, side walls at y = +-0.5
    c = Curtain(np.array([2.0, -0.48]), np.array([2.0, 0.48]), np.array([1.0, 0.0]))
    pts = np.array([[2.0, 0.0],        # on the line
                    [2.3, 0.1],        # bulge 0.3 m past it
                    [1.7, -0.2],       # bulge 0.3 m before it
                    [2.3, 0.5],        # side wall next to the curtain
                    [2.6, 0.0]])       # 0.6 m past: beyond the wide strip
    far = curtain_mask(pts, [c], robot_xy=(0.0, 0.0))
    assert far.tolist() == [True, False, False, False, False]
    near = curtain_mask(pts, [c], robot_xy=(2.2, 0.0))
    assert near.tolist() == [True, True, True, False, False]
    assert (curtain_mask(pts, [c]) == in_curtains(pts, [c])).all()
