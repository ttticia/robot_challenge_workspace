import math
from types import SimpleNamespace

import numpy as np
import yaml

from navigation.map_io import (FREE, OCCUPIED, UNKNOWN, link_latest, to_image, write_map,
                               yaw_from_quaternion)


def read_pgm(path):
    raw = path.read_bytes()
    magic, size, maxval, pixels = raw.split(b'\n', 3)
    width, height = map(int, size.split())
    assert magic == b'P5' and maxval == b'255'
    return np.frombuffer(pixels, dtype=np.uint8).reshape(height, width)


def test_thresholds_match_nav2_trinary():
    image = to_image([-1, 0, 25, 26, 64, 65, 100], 7, 1)
    assert image.tolist() == [[UNKNOWN, FREE, FREE, UNKNOWN, UNKNOWN, OCCUPIED, OCCUPIED]]


def test_top_image_row_is_largest_y():
    # Grid row 0 is the bottom of the map (smallest y); images store the top row first.
    image = to_image([100, 100, 0, 0], 2, 2)
    assert image.tolist() == [[FREE, FREE], [OCCUPIED, OCCUPIED]]


def test_write_map_round_trip(tmp_path):
    data = [-1] * 6 + [0] * 3 + [100] * 3
    yml = write_map(tmp_path / 'maps', 'room', data, 3, 4, 0.05000000074505806,
                    (-1.5, 2.25, math.pi / 2))
    meta = yaml.safe_load(yml.read_text())
    assert meta == {'image': 'room.pgm', 'mode': 'trinary', 'resolution': 0.05,
                    'origin': [-1.5, 2.25, 1.570796], 'negate': 0,
                    'occupied_thresh': 0.65, 'free_thresh': 0.25}
    image = read_pgm(tmp_path / 'maps' / 'room.pgm')
    assert image.shape == (4, 3)
    assert (image[0] == OCCUPIED).all() and (image[1] == FREE).all()
    assert (image[2:] == UNKNOWN).all()
    assert not list((tmp_path / 'maps').glob('*.tmp'))


def test_latest_link_follows_newest_map(tmp_path):
    for name in ('first', 'second'):
        write_map(tmp_path, name, [0], 1, 1, 0.05, (0, 0, 0))
        link_latest(tmp_path, name)
    assert (tmp_path / 'latest.yaml').resolve() == (tmp_path / 'second.yaml').resolve()


def test_yaw_from_quaternion():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))
    assert math.isclose(yaw_from_quaternion(q), 0.8)
