"""core.terrain_band: the wall band on uneven ground (no ROS).
python3 -m pytest -q test/test_terrain_band.py"""
import numpy as np
import pytest

from maze_nav.core import terrain_band, wall_ranges

BAND = (0.30, 0.50)
FLOOR = -0.294          # 10:31 run start floor (odom z)


def test_even_ground_keeps_the_band():
    assert terrain_band(BAND, FLOOR) == pytest.approx(BAND)
    assert terrain_band(BAND, FLOOR, FLOOR + 0.01) == pytest.approx((0.31, 0.50))


def test_rough_ground_raises_the_bottom_only():
    low, high = terrain_band(BAND, FLOOR, None, rough=True)
    assert low == pytest.approx(0.38) and high == pytest.approx(0.50)


def test_standing_on_a_tile_lifts_the_bottom_by_the_tile():
    low, high = terrain_band(BAND, FLOOR, FLOOR + 0.10)
    assert low == pytest.approx(0.40) and high == pytest.approx(0.50)


def test_lift_is_capped_and_band_keeps_its_width():
    low, high = terrain_band(BAND, FLOOR, FLOOR + 0.45, rough=True)   # on top of the stairs
    assert high == pytest.approx(0.50)
    assert low == pytest.approx(0.42)                                  # 0.30 + max_lift 0.12
    low, _ = terrain_band((0.30, 0.40), FLOOR, FLOOR + 0.12, rough=True)
    assert low == pytest.approx(0.32)                                  # >= 8 cm wide


def test_odom_drifted_down_lowers_the_top():
    low, high = terrain_band(BAND, FLOOR, FLOOR - 0.04)
    assert low == pytest.approx(0.30) and high == pytest.approx(0.46)


def test_rubble_top_tilted_stays_out_but_wall_stays_in():
    # rubble on a tile: 0.25 m top + 4 cm of tilt / odom z error = 0.29..0.33 m
    rubble = np.array([[1.0, 0.0, FLOOR + z] for z in np.linspace(0.29, 0.33, 9)])
    wall = np.array([[0.0, 1.0, FLOOR + z] for z in np.linspace(0.0, 0.53, 30)])
    pts = np.vstack([rubble, wall])
    n, inc = 360, 2 * np.pi / 360

    def hits(band):
        r = wall_ranges(pts, (0.0, 0.0), 0.0, FLOOR, band, -np.pi, inc, n, 0.25, 12.0)
        ahead = r[int(round(np.pi / inc))]              # bearing 0: the rubble
        left = r[int(round(1.5 * np.pi / inc))]         # bearing +90: the wall
        return np.isfinite(ahead), np.isfinite(left)

    assert hits(BAND) == (True, True)                                  # old band: rubble = wall
    assert hits(terrain_band(BAND, FLOOR, None, rough=True)) == (False, True)
