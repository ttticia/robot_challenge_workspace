"""Estimate visible stair treads and a full-width landing from 3D depth points."""

import math

import numpy as np


BIN = 0.05
LANES = 5
WIDTH = 0.65
LANDING = 0.90
LEVEL_TOLERANCE = 0.035
MIN_RISE = 0.075
MAX_RISER = 0.18


def depth_points(image, camera):
    """Project a registered rectified depth image into its optical frame."""
    formats = {'16UC1': ('u2', 0.001), '32FC1': ('f4', 1.0)}
    if image.encoding not in formats or image.header.frame_id != camera.header.frame_id:
        return None
    if (image.width, image.height) != (camera.width, camera.height):
        return None
    fx, fy, cx, cy = camera.p[0], camera.p[5], camera.p[2], camera.p[6]
    if min(fx, fy) <= 0 or not all(map(math.isfinite, (fx, fy, cx, cy))):
        return None
    type_code, scale = formats[image.encoding]
    dtype = np.dtype(('>' if image.is_bigendian else '<') + type_code)
    if image.step < dtype.itemsize * image.width:
        return None
    depth = np.ndarray((image.height, image.width), dtype, buffer=bytes(image.data),
                       strides=(image.step, dtype.itemsize))
    z = depth[::2, ::2].astype(float) * scale
    rows, cols = np.nonzero(np.isfinite(z) & (z >= 0.15) & (z <= 4.0))
    z = z[rows, cols]
    return np.column_stack(((2 * cols - cx) * z / fx, (2 * rows - cy) * z / fy, z))


def transform_points(points, transform):
    """Transform optical-frame points into a gravity-aligned coordinate frame."""
    q = transform.rotation
    xyz = np.array([q.x, q.y, q.z])
    cross = 2 * np.cross(xyz, points)
    t = transform.translation
    return points + q.w * cross + np.cross(xyz, cross) + [t.x, t.y, t.z]


def estimate_stairs(points):
    """Return (run, rise) in metres for a visible straight stair corridor."""
    x, y, z = points.T
    keep = (x >= 0.10) & (x < 4.0) & (np.abs(y) < WIDTH / 2)
    lanes = ((y[keep] + WIDTH / 2) * LANES / WIDTH).astype(int)
    bins = (x[keep] / BIN).astype(int)
    cells = {}
    for index, lane, height in zip(bins, lanes, z[keep]):
        cells.setdefault((index, lane), []).append(height)

    # All lanes must agree on the surface height in each forward bin.
    bands = []
    for index in sorted(set(bins)):
        heights = [cells.get((index, lane), []) for lane in range(LANES)]
        if min(map(len, heights)) < 3:
            continue
        ranges = [np.percentile(samples, [10, 90]) for samples in heights]
        low, high = min(r[0] for r in ranges), max(r[1] for r in ranges)
        if high - low > LEVEL_TOLERANCE:
            continue
        h = (low + high) / 2
        if bands and index == bands[-1][1] + 1 and abs(h - bands[-1][2]) <= LEVEL_TOLERANCE:
            first, last, level = bands[-1]
            bands[-1] = first, index, (level * (last - first + 1) + h) / (index - first + 1)
        else:
            bands.append((index, index, h))

    surfaces = [band for band in bands if band[1] - band[0] >= 1]
    if (len(surfaces) < 3 or surfaces[0][0] * BIN > 0.30
            or (surfaces[0][1] + 1) * BIN < 0.35):
        return None
    for lower, upper in zip(surfaces, surfaces[1:]):
        if upper[0] - lower[1] > 3 or not MIN_RISE <= upper[2] - lower[2] <= MAX_RISER:
            return None
    first, last, height = surfaces[-1]
    if (last - first + 1) * BIN < LANDING:
        return None
    return first * BIN + LANDING / 2, height - surfaces[0][2]
