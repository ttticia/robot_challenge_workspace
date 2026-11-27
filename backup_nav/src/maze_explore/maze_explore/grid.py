"""The maze as a grid of square cells (no ROS in here).

The grid frame has its origin at the centre of the start cell, +u along the robot's heading
at the start. Cell (i, j) has its centre at (i, j) * size. Directions d = 0..3 are +u, +v,
-u, -v (ahead, left, back, right of the start heading).
"""
import math

import numpy as np

DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def nb(cell, d):
    return cell[0] + DIRS[d][0], cell[1] + DIRS[d][1]


def opp(d):
    return (d + 2) % 4


class Grid:
    def __init__(self, x0=0.0, y0=0.0, yaw=0.0, size=1.02):
        self.x0, self.y0, self.yaw, self.size = x0, y0, yaw, size

    def to_grid(self, x, y):
        """Map-frame points -> grid-frame metres (u, v). Works on arrays."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = np.asarray(x) - self.x0, np.asarray(y) - self.y0
        return c * dx + s * dy, -s * dx + c * dy

    def to_map(self, u, v):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        u, v = np.asarray(u), np.asarray(v)
        return self.x0 + c * u - s * v, self.y0 + s * u + c * v

    def cell_of(self, x, y):
        u, v = self.to_grid(x, y)
        return int(round(float(u) / self.size)), int(round(float(v) / self.size))

    def centre(self, cell):
        x, y = self.to_map(cell[0] * self.size, cell[1] * self.size)
        return float(x), float(y)

    def heading(self, d):
        return wrap(self.yaw + d * math.pi / 2)

    def nearest_dir(self, yaw):
        return int(round(wrap(yaw - self.yaw) / (math.pi / 2))) % 4

    def box(self, cell, d, along, depth):
        """Map-frame sample points in a box next to cell's edge d. `depth` is measured from
        the edge into the neighbour (negative = back into `cell`), `along` sideways along the
        edge. Returns (x, y) arrays of shape (len(along), len(depth))."""
        a, q = np.meshgrid(np.asarray(along), np.asarray(depth), indexing='ij')
        du, dv = DIRS[d]
        cu, cv = cell[0] * self.size, cell[1] * self.size
        fwd = self.size / 2 + q
        u = cu + du * fwd - dv * a
        v = cv + dv * fwd + du * a
        return self.to_map(u, v)


def fit_grid(grid, px, py, max_yaw=math.radians(12), max_shift=0.30, bin_m=0.02):
    """Line the grid up with the walls. px, py: map-frame wall points (occupied map cells).

    Walls lie on the cell borders, half a cell from the centres. For each trial rotation the
    points' u and v coordinates are folded onto one cell width; walls then pile up in one bin
    each. The rotation with the sharpest piles wins, and the piles' positions give the shift.
    Returns (Grid, quality) with quality = share of the points within 6 cm of a border, or
    (None, 0) when there are too few points.
    """
    if len(px) < 40:
        return None, 0.0
    n = int(round(grid.size / bin_m))
    best = None
    for dyaw in np.radians(np.arange(-math.degrees(max_yaw), math.degrees(max_yaw) + 0.25, 0.5)):
        g = Grid(grid.x0, grid.y0, grid.yaw + dyaw, grid.size)
        u, v = g.to_grid(px, py)
        score, shift = 0.0, []
        for w in (u, v):
            f = (w / g.size - 0.5) % 1.0                # 0 = on a border
            h = np.bincount((f * n).astype(int) % n, minlength=n).astype(float)
            h = h + np.roll(h, 1) + np.roll(h, -1)
            k = int(np.argmax(h))
            score += h[k]
            off = (k + 0.5) / n
            shift.append((off if off < 0.5 else off - 1.0) * g.size)
        if best is None or score > best[0]:
            best = (score, g, shift)
    _, g, (su, sv) = best
    if abs(su) > max_shift or abs(sv) > max_shift:
        return None, 0.0
    x0, y0 = g.to_map(su, sv)
    out = Grid(float(x0), float(y0), g.yaw, g.size)
    u, v = out.to_grid(px, py)
    du = np.abs((u / out.size - 0.5 + 0.5) % 1.0 - 0.5) * out.size
    dv = np.abs((v / out.size - 0.5 + 0.5) % 1.0 - 0.5) * out.size
    return out, float(np.mean(np.minimum(du, dv) < 0.06))
