"""Write an occupancy grid as a Nav2 map_server map (PGM image + YAML)."""
import math
import os
from pathlib import Path

import numpy as np

# Same pixel values and thresholds as nav2_map_server's map_saver (trinary mode).
FREE, OCCUPIED, UNKNOWN = 254, 0, 205
FREE_THRESH, OCCUPIED_THRESH = 0.25, 0.65


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_image(data, width, height):
    """Occupancy values (-1 unknown, 0..100) -> image rows, top row = largest y."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    image = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    image[(grid >= 0) & (grid <= FREE_THRESH * 100)] = FREE
    image[grid >= OCCUPIED_THRESH * 100] = OCCUPIED
    return np.flipud(image)


def _atomic_write(path, payload):
    # Write to a temp file and rename, so a crash never leaves a half-written map.
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(payload)
    os.replace(tmp, path)


def write_map(directory, name, data, width, height, resolution, origin):
    """Save <name>.pgm and <name>.yaml in directory; origin is (x, y, yaw).

    Returns the YAML path.
    """
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    pgm, yml = directory / f'{name}.pgm', directory / f'{name}.yaml'
    header = f'P5\n{width} {height}\n255\n'.encode()
    _atomic_write(pgm, header + to_image(data, width, height).tobytes())
    x, y, yaw = (round(float(v), 6) for v in origin)
    _atomic_write(yml, (
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        f'resolution: {round(float(resolution), 6)}\n'
        f'origin: [{x}, {y}, {yaw}]\n'
        'negate: 0\n'
        f'occupied_thresh: {OCCUPIED_THRESH}\n'
        f'free_thresh: {FREE_THRESH}\n').encode())
    return yml


def link_latest(directory, name):
    """Point latest.yaml at <name>.yaml so the newest map always has a fixed path."""
    directory = Path(directory).expanduser()
    tmp = directory / 'latest.yaml.tmp'
    tmp.unlink(missing_ok=True)
    tmp.symlink_to(f'{name}.yaml')
    os.replace(tmp, directory / 'latest.yaml')
