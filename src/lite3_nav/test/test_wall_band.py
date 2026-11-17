"""lite3_nav.wall_band (no ROS needed). Tests borrowed from maze_nav (origin/navigation 419f205,
test/test_core.py, wall scan section), adapted to the lite3_nav module name."""
import math

import numpy as np
import pytest

from lite3_nav.wall_band import cloud_xyz, estimate_floor, wall_ranges

INC = math.radians(1.0)
N = 360


def ranges(points, xy=(0, 0), yaw=0.0, floor=-0.35, band=(0.30, 0.43)):
    return wall_ranges(np.array(points, float), xy, yaw, floor, band, -math.pi, INC, N, 0.25, 12.0)


def beam(angle):
    return int((angle + math.pi) / INC) % N


# ------------------------------------------------------------------ wall scan

def test_wall_in_band_is_seen_terrain_and_stairs_are_not():
    floor = -0.35
    pts = [(1.0, 0.0, floor + 0.35),           # wall, 0.35 m above floor
           (0.0, 1.5, floor + 0.15),           # ramp top
           (0.0, -1.0, floor + 0.26),          # top stair tread
           (-1.0, 0.0, floor + 0.02)]          # floor
    r = ranges(pts, floor=floor)
    assert r[beam(0.0)] == pytest.approx(1.0, abs=1e-3)
    assert np.isinf(r[beam(math.pi / 2)]) and np.isinf(r[beam(-math.pi / 2)])
    assert np.isinf(r[beam(math.pi - 0.01)])


def test_band_is_floor_referenced_not_robot_referenced():
    # Robot body 0.15 m higher (on a ramp) does not move the band: the band uses floor_z only.
    pts = [(1.0, 0.0, -0.35 + 0.40)]
    assert ranges(pts)[beam(0.0)] == pytest.approx(1.0, abs=1e-3)


def test_nearest_hit_wins_and_yaw_is_applied():
    pts = [(2.0, 0.0, 0.0), (1.2, 0.0, 0.0), (5.0, 5.0, 0.0)]
    r = ranges(pts, xy=(0.2, 0.0), yaw=math.pi / 2)
    assert r[beam(-math.pi / 2)] == pytest.approx(1.0, abs=1e-3)   # wall is to the right


def test_floor_estimate():
    rng = np.random.default_rng(0)
    xy = rng.uniform(-1.5, 1.5, (5000, 2))
    floor = np.c_[xy, -0.33 + rng.normal(0, 0.005, len(xy))]
    wall = np.c_[np.full(500, 0.9), rng.uniform(-1, 1, 500), rng.uniform(-0.33, 0.12, 500)]
    z = estimate_floor(np.r_[floor, wall], (0, 0), 0.0)
    assert z == pytest.approx(-0.33, abs=0.01)
    assert estimate_floor(wall[:50], (0, 0), 0.0) is None
    # A denser terrain mat 0.12 m above the floor in the next cell must not win.
    mat = np.c_[rng.uniform(1.2, 2.2, (12000, 2)), -0.21 + rng.normal(0, 0.005, 12000)]
    assert estimate_floor(np.r_[floor, wall, mat], (0, 0), 0.0) == pytest.approx(-0.33, abs=0.01)


def test_cloud_xyz_reads_livox_mixed_field_types():
    """Livox layout: x y z intensity float32, tag line uint8, timestamp float64 (step 26)."""
    from types import SimpleNamespace as F
    fields = [F(name='x', offset=0, datatype=7), F(name='y', offset=4, datatype=7),
              F(name='z', offset=8, datatype=7), F(name='intensity', offset=12, datatype=7),
              F(name='tag', offset=16, datatype=2), F(name='line', offset=17, datatype=2),
              F(name='timestamp', offset=18, datatype=8)]
    dt = np.dtype({'names': ['x', 'y', 'z', 'i', 'tag', 'line', 't'],
                   'formats': ['<f4'] * 4 + ['u1', 'u1', '<f8'],
                   'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})
    a = np.zeros(3, dt)
    a['x'], a['y'], a['z'], a['t'] = [1, 2, np.nan], [3, 4, 5], [6, 7, 8], 1e9
    xyz = cloud_xyz(a.tobytes(), fields, 26, 26 * 3, 3, 1)
    assert xyz.tolist() == [[1, 3, 6], [2, 4, 7]]             # NaN point dropped
    with pytest.raises(ValueError):
        cloud_xyz(a.tobytes(), fields[1:], 26, 78, 3, 1)
