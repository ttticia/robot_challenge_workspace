"""Synthetic depth images for the stair detector tests: axis-aligned boxes on a floor, seen by
the Lite3's Orbbec depth camera (intrinsics and mount from the robot's /camera/depth/camera_info
and /tf_static, ROS_bags/perception_test)."""
import json
import math
import os

import numpy as np

from maze_nav.stairs import quat_to_matrix

HERE = os.path.dirname(os.path.abspath(__file__))
K = (372.79, 372.79, 319.40, 241.5)          # fx, fy, cx, cy
W, H = 640, 480
# base_link -> camera_link (20 deg down), camera_link -> camera_depth_optical_frame
T_BASE_CAM = np.array([0.2523, -0.0005, 0.0741])
R_BASE_CAM = quat_to_matrix(0.0, 0.1736, 0.0, 0.9848) @ quat_to_matrix(-0.5, 0.5, -0.5, 0.5)
STANDING = 0.35


def staircase(x0, y_c=0.0, risers=3, rise=0.12, tread=0.15, width=0.9, base=0.0, top=0.3):
    """Boxes for stairs going up along +x, first riser at x = x0, centred on y = y_c. Other
    stair directions are tested by turning the robot instead."""
    end = x0 + (risers - 1) * tread + top
    return [((x0 + i * tread, y_c - width / 2, base), (end, y_c + width / 2, base + (i + 1) * rise))
            for i in range(risers)]


def maze_boxes():
    """The example maze (test/example_maze.json): tiles, mats, walls, rails and the stairs."""
    with open(os.path.join(HERE, 'example_maze.json')) as f:
        solids = json.load(f)['solids']
    out = []
    for s in solids:
        lo, hi = s['min'], s['max']
        if s['kind'] in ('tile', 'mat', 'wall', 'rail', 'block'):
            out.append((tuple(lo), tuple(hi)))
        elif s['kind'] == 'stairs':              # fake_maze_lite3: 3 up, 3 down, 0.15 / 0.12 m
            c = (lo[0] + hi[0]) / 2
            for n, half in ((1, 0.375), (2, 0.225), (3, 0.075)):
                out.append(((c - half, lo[1], 0.0), (c + half, hi[1], 0.12 * n)))
    return out


def raycast(boxes, origin, dirs, floor_z=0.0, max_range=6.0):
    """Ray parameter t of the first hit (floor plane at floor_z or a box) per ray; inf = none.
    Boxes further than max_range from the origin are skipped (beyond what the tests use)."""
    t_best = np.full(len(dirs), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (floor_z - origin[2]) / dirs[:, 2]
        t_best = np.where((t > 0), t, np.inf)
        inv = 1.0 / dirs
        for lo, hi in boxes:
            if max(abs((lo[0] + hi[0]) / 2 - origin[0]), abs((lo[1] + hi[1]) / 2 - origin[1])) \
                    > max_range + max(hi[0] - lo[0], hi[1] - lo[1]) / 2:
                continue
            t1 = (np.asarray(lo) - origin) * inv
            t2 = (np.asarray(hi) - origin) * inv
            tmin = np.nanmax(np.minimum(t1, t2), axis=1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
            t_best = np.where(hit & (tmin < t_best), tmin, t_best)
    return t_best


def body_rotation(yaw, pitch=0.0, roll=0.0):
    """base_link orientation in odom (pitch > 0 = nose down)."""
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return (np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


# Livox Mid-360 on the Lite3: base_link -> livox_frame (0.187, 0, 0.125), pitched 15 deg down;
# vertical FOV -7..52 deg in the sensor frame, 360 deg around; ~200 k points/s (20 k a scan at
# 10 Hz), range noise ~2 cm (datasheet, robot_info.md).
T_BASE_LIDAR = np.array([0.18678, 0.0, 0.12498])
R_BASE_LIDAR = quat_to_matrix(0.0, 0.1305, 0.0, 0.9914)


def lidar_scan(boxes, base_xyz, yaw, floor_z=0.0, n=20000, seed=0, pitch=0.0, roll=0.0,
               noise=0.02, max_range=6.0):
    """One simulated Mid-360 scan with occlusion: (N, 3) points in livox_frame, plus the
    base_link rotation in odom. Rays are spread uniformly over the sensor's FOV (the real
    non-repetitive pattern fills it over time)."""
    rng = np.random.default_rng(seed)
    az = rng.uniform(-math.pi, math.pi, n)
    el = np.arcsin(rng.uniform(math.sin(math.radians(-7)), math.sin(math.radians(52)), n))
    d_l = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], 1)
    r_ob = body_rotation(yaw, pitch, roll)
    r_ol = r_ob @ R_BASE_LIDAR
    origin = np.asarray(base_xyz, float) + r_ob @ T_BASE_LIDAR
    t = raycast(boxes, origin, d_l @ r_ol.T, floor_z, max_range)
    ok = np.isfinite(t) & (t > 0.1) & (t < max_range)
    r = t[ok] + (rng.normal(0.0, noise, ok.sum()) if noise else 0.0)
    return d_l[ok] * r[:, None], r_ob


def render(boxes, base_xyz, yaw, floor_z=0.0, noise=True, seed=0, pitch=0.0, roll=0.0):
    """uint16 depth (mm) seen from base_link at base_xyz with heading yaw (rad).

    Returns (depth, r_odom_base) so the test can build the level transform the node would.
    """
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]])
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    cr, sr = math.cos(roll), math.sin(roll)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    r_ob = rz @ ry @ rx
    r_oc = r_ob @ R_BASE_CAM
    origin = np.asarray(base_xyz, float) + r_ob @ T_BASE_CAM
    fx, fy, cx, cy = K
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1).reshape(-1, 3)
    dirs = rays @ r_oc.T                                   # odom direction per unit camera z
    t_best = raycast(boxes, origin, dirs, floor_z, max_range=3.5)
    z = t_best.reshape(H, W)                               # camera z = t (rays have z = 1)
    z[~np.isfinite(z)] = 0.0
    if noise:
        rng = np.random.default_rng(seed)
        z = z + rng.normal(0.0, 1.0, z.shape) * (0.001 + 0.005 * z * z)
        z[rng.random(z.shape) < 0.02] = 0.0
    z[(z < 0.1) | (z > 10.0)] = 0.0
    return np.round(z * 1000).astype(np.uint16), r_ob
