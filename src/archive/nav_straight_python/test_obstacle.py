import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from nav_straight.obstacle import (DEFAULT_LIDAR_QUAT, DEFAULT_LIDAR_XYZ, cloud_to_xyz,
                                   forward_clearance, quat_to_matrix, speed_for_clearance,
                                   transform, wrap_angle, yaw_from_quat)

BOX = dict(front_x=0.32, half_width=0.30, z_min=-0.25, z_max=0.40, max_lookahead=3.0)


def livox_cloud(points):
    """Build a cloud with the Livox layout (point_step 22, extra fields)."""
    dt = np.dtype({'names': ['x', 'y', 'z', 't', 'intensity', 'tag', 'line'],
                   'formats': ['<f4', '<f4', '<f4', '<u4', '<f4', 'u1', 'u1'],
                   'offsets': [0, 4, 8, 12, 16, 20, 21], 'itemsize': 22})
    arr = np.zeros(len(points), dtype=dt)
    arr['x'], arr['y'], arr['z'] = np.asarray(points).T
    m = PointCloud2()
    m.height, m.width, m.point_step = 1, len(points), 22
    m.row_step = 22 * len(points)
    m.fields = [PointField(name='x', offset=0, datatype=7, count=1),
                PointField(name='y', offset=4, datatype=7, count=1),
                PointField(name='z', offset=8, datatype=7, count=1),
                PointField(name='t', offset=12, datatype=6, count=1),
                PointField(name='intensity', offset=16, datatype=7, count=1),
                PointField(name='tag', offset=20, datatype=2, count=1),
                PointField(name='line', offset=21, datatype=2, count=1)]
    m.data = arr.tobytes()
    return m


def test_cloud_to_xyz_livox_layout():
    pts = [(1.0, 2.0, 3.0), (-0.5, 0.25, float('nan')), (4.0, 5.0, 6.0)]
    out = cloud_to_xyz(livox_cloud(pts))
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out, [[1, 2, 3], [4, 5, 6]])


def test_lidar_pitch_points_forward_and_down():
    r = quat_to_matrix(*DEFAULT_LIDAR_QUAT)
    # A point 1 m straight along the lidar's x axis lands ahead of and below the lidar.
    p = transform(np.array([[1.0, 0, 0]]), r, DEFAULT_LIDAR_XYZ)[0]
    assert math.isclose(p[0] - DEFAULT_LIDAR_XYZ[0], math.cos(math.radians(15)), abs_tol=0.01)
    assert math.isclose(DEFAULT_LIDAR_XYZ[2] - p[2], math.sin(math.radians(15)), abs_tol=0.01)


def test_clearance_ignores_floor_side_and_overhead():
    pts = np.array([
        [1.0, 0.0, -0.35],   # floor
        [1.0, 0.5, 0.0],     # beside the corridor
        [1.0, 0.0, 0.8],     # overhead
        [0.2, 0.0, 0.0],     # inside the body / legs
    ])
    assert forward_clearance(pts, **BOX) == math.inf


def test_clearance_nearest_and_min_points():
    wall = np.array([[2.0, y, 0.0] for y in np.linspace(-0.2, 0.2, 10)])
    noise = np.array([[0.8, 0.0, 0.0]])
    pts = np.vstack([wall, noise])
    assert math.isclose(forward_clearance(pts, **BOX), 0.8 - 0.32)
    assert math.isclose(forward_clearance(pts, **BOX, min_points=5), 2.0 - 0.32)


def test_speed_profile():
    args = dict(stop_distance=0.5, slow_distance=1.2, max_speed=0.1, min_speed=0.05)
    assert speed_for_clearance(0.4, **args) == 0.0
    assert speed_for_clearance(0.5, **args) == 0.0
    assert speed_for_clearance(5.0, **args) == 0.1
    assert speed_for_clearance(math.inf, **args) == 0.1
    mid = speed_for_clearance(0.85, **args)
    assert 0.05 < mid < 0.1


def test_angles():
    assert math.isclose(yaw_from_quat(0, 0, math.sin(0.5), math.cos(0.5)), 1.0)
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2)
