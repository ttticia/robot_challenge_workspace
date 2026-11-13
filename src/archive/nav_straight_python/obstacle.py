"""Pure geometry for the forward obstacle check (no ROS, unit-testable).

Frames: base_link is x forward, y left, z up, origin at the body centre, about
0.35 m above the floor. The Mid-360 is mounted 15 deg nose-down (robot_info.md §5).
"""
import math

import numpy as np

# base_link -> livox_frame from robot_info.md §5, used if /tf_static is not available.
DEFAULT_LIDAR_XYZ = (0.18678, 0.0, 0.12498)
DEFAULT_LIDAR_QUAT = (0.0, 0.1305, 0.0, 0.9914)  # x, y, z, w

_PF_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
              5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def cloud_to_xyz(msg):
    """sensor_msgs/PointCloud2 -> (N, 3) float64 array of finite points.

    Reads the x/y/z field offsets from the message, so it works with the
    Livox layout (point_step 22, extra fields) and any other.
    """
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in 'xyz'):
        raise ValueError('cloud has no x/y/z fields')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [np.dtype(_PF_DTYPES[fields[k].datatype]).newbyteorder(endian)
                    for k in 'xyz'],
        'offsets': [fields[k].offset for k in 'xyz'],
        'itemsize': msg.point_step,
    })
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3))
    if msg.row_step == msg.width * msg.point_step:
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
    else:  # padded rows
        buf = bytes(msg.data)
        raw = np.concatenate([
            np.frombuffer(buf, dtype=dtype, count=msg.width, offset=r * msg.row_step)
            for r in range(msg.height)])
    pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
    return pts[np.all(np.isfinite(pts), axis=1)]


def transform(points, rotation, translation):
    """Apply child->parent transform (rotation 3x3, translation 3) to (N, 3) points."""
    return points @ np.asarray(rotation).T + np.asarray(translation)


def forward_clearance(points_base, front_x, half_width, z_min, z_max,
                      max_lookahead, min_points=1):
    """Free distance (m) from the robot's front edge to the nearest obstacle ahead.

    Considers only points (in base_link) inside the corridor the robot will sweep:
    front_x < x < front_x + max_lookahead, |y| <= half_width, z_min <= z <= z_max.
    Returns the distance to the min_points-th nearest point (ignores single-point
    noise), or math.inf if fewer points than that are in the corridor.
    """
    p = points_base
    if p.size == 0:
        return math.inf
    m = ((p[:, 0] > front_x) & (p[:, 0] < front_x + max_lookahead)
         & (np.abs(p[:, 1]) <= half_width) & (p[:, 2] >= z_min) & (p[:, 2] <= z_max))
    xs = p[m, 0]
    k = max(1, int(min_points))
    if xs.size < k:
        return math.inf
    return float(np.partition(xs, k - 1)[k - 1] - front_x)


def speed_for_clearance(clearance, stop_distance, slow_distance, max_speed, min_speed):
    """Forward speed for a given clearance; 0 means stop."""
    if clearance <= stop_distance:
        return 0.0
    if clearance >= slow_distance or slow_distance <= stop_distance:
        return max_speed
    frac = (clearance - stop_distance) / (slow_distance - stop_distance)
    return min_speed + frac * (max_speed - min_speed)
