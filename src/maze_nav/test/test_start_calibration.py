"""Start-cell calibration (stairs_lidar.StartCalibration, floor_tilt_from_depth, tilt_fix):
standing height and depth-camera tilt from the start floor, robot standing still."""
import math

import numpy as np
import pytest

import stair_scene
from maze_nav import stairs_lidar as L
from maze_nav.stairs import mount_disagreement
from stair_scene import K, R_BASE_CAM, T_BASE_CAM, render

BASE = (0.0, 0.0, 0.35)          # standing on the start floor (z = 0)
# start cell walls (1 m cell, robot at its centre facing +x): the camera must ignore them
CELL = [((-0.5, -0.52, 0.0), (0.5, -0.48, 0.45)), ((-0.5, 0.48, 0.0), (0.5, 0.52, 0.45)),
        ((-0.52, -0.5, 0.0), (-0.48, 0.5, 0.45))]


def rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def frame_with_true_mount(monkeypatch, r_err, seed=0):
    """Depth seen with the camera really at r_err @ R_BASE_CAM (a mount off by r_err)."""
    monkeypatch.setattr(stair_scene, 'R_BASE_CAM', r_err @ R_BASE_CAM)
    depth, r_ob = render(CELL, BASE, 0.0, seed=seed)
    monkeypatch.setattr(stair_scene, 'R_BASE_CAM', R_BASE_CAM)
    return depth, r_ob


def tilt(depth, r_ob, r_cam):
    pts = L.depth_floor_points(depth, K, r_ob, r_cam, T_BASE_CAM, BASE, stride=4)
    return L.floor_tilt_from_depth(pts, BASE, 0.0, 0.0)


def test_right_mount_reads_a_level_floor():
    depth, r_ob = render(CELL, BASE, 0.0, seed=0)
    a, b, off, n = tilt(depth, r_ob, R_BASE_CAM)
    assert abs(math.degrees(math.atan(a))) < 0.3 and abs(math.degrees(math.atan(b))) < 0.3
    assert abs(off) < 0.01 and n > 150


@pytest.mark.parametrize('r_err', [rot('y', 2.0), rot('y', -1.5), rot('x', 1.5),
                                   rot('y', 1.0) @ rot('x', -1.0)])
def test_mount_error_found_and_fixed(monkeypatch, r_err):
    """A 1-2 deg mount error shows up as a floor slope; tilt_fix of that slope is the error,
    and with it the floor is level and at the right height again."""
    depth, r_ob = frame_with_true_mount(monkeypatch, r_err)
    a, b, _, _ = tilt(depth, r_ob, R_BASE_CAM)
    assert max(abs(a), abs(b)) > 0.012                       # visibly sloped (> ~0.7 deg)
    fix = L.tilt_fix(a, b)
    ang, _ = mount_disagreement((fix, T_BASE_CAM), (r_err, T_BASE_CAM))
    assert ang < 0.3
    a2, b2, off2, _ = tilt(depth, r_ob, fix @ R_BASE_CAM)
    assert abs(a2) < 0.004 and abs(b2) < 0.004 and abs(off2) < 0.01


def test_calibration_collects_while_still_and_restarts_after_a_move():
    c = L.StartCalibration(duration=3.0)
    for k in range(10):
        c.add_pose(0.5 * k, (0.0, 0.0, 0.337), 0.0, 0.0)
        c.add_tilt(0.02, 0.0)
    assert c.ready(4.5)
    c.add_pose(5.0, (0.10, 0.0, 0.337), 0.0, 0.0)            # moved 10 cm: start over
    assert not c.ready(5.0) and c.heights == [pytest.approx(0.337)]
    for k in range(1, 8):
        c.add_pose(5.0 + 0.5 * k, (0.10, 0.0, 0.337), 0.0, 0.0)
        c.add_tilt(0.02, -0.01)
    assert c.ready(8.5)
    r = c.result()
    assert r['standing_height'] == pytest.approx(0.337)
    assert r['pitch_deg'] == pytest.approx(math.degrees(math.atan(0.02)))
    assert r['roll_deg'] == pytest.approx(math.degrees(math.atan(-0.01)))


def test_calibration_rejects_implausible_values():
    c = L.StartCalibration(duration=1.0, max_tilt_deg=3.0)
    for k in range(6):
        c.add_pose(0.3 * k, (0.0, 0.0, 0.10), 0.0, 0.0)      # lying, or the wrong floor
        c.add_tilt(0.10, 0.0)                                # 5.7 deg: a bad frame, not a mount
    r = c.result()
    assert r['standing_height'] is None and r['pitch_deg'] is None
    assert len(r['why']) == 2
