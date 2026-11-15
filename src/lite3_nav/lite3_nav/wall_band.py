"""Wall band scan helpers (numpy only, no ROS, unit-tested).

Borrowed from maze_nav (origin/navigation 419f205, maze_nav/core.py: cloud_xyz, wall_ranges,
estimate_floor), unchanged apart from this header. Used by lite3_nav/wall_scan.py.
"""
import math

import numpy as np


# PointField datatype codes -> numpy types (sensor_msgs/PointField)
_PF_TYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32,
             7: np.float32, 8: np.float64}


def cloud_xyz(data, fields, point_step, row_step, width, height, big_endian=False):
    """x, y, z of a PointCloud2 as (N, 3) float64, NaNs/infs dropped.

    Reads only the x/y/z fields by their offsets, so any other fields (the Livox driver adds
    intensity float32, tag/line uint8, timestamp float64) can have any type.
    sensor_msgs_py's read_points_numpy refuses clouds with mixed field types.
    fields: iterable of objects with .name, .offset, .datatype (sensor_msgs/PointField).
    """
    by_name = {f.name: f for f in fields}
    missing = [n for n in 'xyz' if n not in by_name]
    if missing:
        raise ValueError(f'cloud has no {missing} field(s)')
    order = '>' if big_endian else '<'
    dtype = np.dtype({'names': list('xyz'),
                      'formats': [np.dtype(_PF_TYPES[by_name[n].datatype]).newbyteorder(order)
                                  for n in 'xyz'],
                      'offsets': [by_name[n].offset for n in 'xyz'],
                      'itemsize': point_step})
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if height > 1 and row_step != width * point_step:     # padded rows: trim each row
        buf = buf[:height * row_step].reshape(height, row_step)[:, :width * point_step]
        buf = np.ascontiguousarray(buf).ravel()
    pts = buf[:width * height * point_step].view(dtype)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def wall_ranges(points_odom, robot_xy, robot_yaw, floor_z, band, angle_min, angle_inc,
                n_beams, range_min, range_max):
    """Flatten a point cloud to a LaserScan that sees maze walls and nothing else.

    points_odom: (N, 3) points in a gravity-aligned frame (odom). The height band is measured
    from the maze floor (floor_z, odom), not from the robot, so the band stays put while the
    robot's body rises and tilts on ramps, mats and stairs.
    band: (low, high) metres above the floor. Example maze: walls 0..0.45 m, terrain <= 0.15 m,
    stair treads <= 0.26 m, hand rails <= 0.535 m -> (0.30, 0.43) sees walls and rails only.
    Returns ranges (n_beams,) with +inf where nothing was hit; beam angles are in the robot's
    heading frame (yaw only).
    """
    ranges = np.full(n_beams, np.inf, dtype=np.float32)
    if len(points_odom) == 0:
        return ranges
    p = np.asarray(points_odom, dtype=np.float64)
    h = p[:, 2] - floor_z
    p = p[(h >= band[0]) & (h <= band[1])]
    if len(p) == 0:
        return ranges
    dx, dy = p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1]
    r = np.hypot(dx, dy)
    a = np.arctan2(dy, dx) - robot_yaw
    a = (a - angle_min) % (2 * math.pi)          # wrap into [0, 2pi) from angle_min
    idx = np.floor(a / angle_inc).astype(np.int64)
    keep = (r >= range_min) & (r <= range_max) & (idx >= 0) & (idx < n_beams)
    np.minimum.at(ranges, idx[keep], r[keep].astype(np.float32))
    return ranges


def estimate_floor(points_odom, robot_xy, robot_z, expected_drop=0.35, window=0.15,
                   min_points=200, layer_fraction=0.2):
    """Floor height (odom z) from the cloud around the standing robot, or None.

    Takes the LOWEST dense 1 cm height layer within `window` of the expected floor
    (robot_z - expected_drop) in a ring 0.4..2.5 m around the robot. Lowest, not densest: the
    lidar can't see the floor closer than ~1.2 m ahead, so terrain mats in the next cell
    (0.1-0.12 m high in the example maze) can hold more points than the floor itself.
    """
    p = np.asarray(points_odom, dtype=np.float64)
    if len(p) == 0:
        return None
    r = np.hypot(p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1])
    guess = robot_z - expected_drop
    z = p[(r > 0.4) & (r < 2.5) & (np.abs(p[:, 2] - guess) < window), 2]
    if len(z) < min_points:
        return None
    hist, edges = np.histogram(z, bins=np.arange(guess - window, guess + window + 0.01, 0.01))
    dense = np.nonzero(hist >= max(layer_fraction * hist.max(), min_points / 10))[0]
    i = int(dense[0])
    near = z[(z >= edges[i] - 0.01) & (z <= edges[i + 1] + 0.01)]
    return float(np.median(near))
