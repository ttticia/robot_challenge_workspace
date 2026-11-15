"""Exit / final-cell detection on the SLAM map (numpy/scipy only, no ROS, unit-tested).

The organisers end the run when the robot reaches the final grid cell, which has an opening out
of the arena. Two ways to recognise it:

- ExitDetector: borrowed from maze_nav (origin/navigation 419f205, maze_nav/core.py), adapted to
  lite3_nav.core.Grid (rotated maps). Out of the arena = a disk of radius >= open_radius made only
  of cells SLAM has seen as free (walls and unknown both count as blocked, so open space has to be
  seen, not guessed) lies within near_radius of the robot, >= min_displacement from the start, the
  robot was in narrow space once (clearance <= enclosed_radius), and all of that holds for dwell s.
  Inside 0.9 m corridors no such disk exists (max ~0.45 m).
- final_cell_xy / final_cell_radius: if the final cell's map coordinates are known, being within
  final_cell_radius of them also counts (immediately, no dwell).

exit_masked_grid() hides the open floor beyond the exit (the same disks) from frontier selection
so exploration finishes the arena before it drives out (lite3_nav addition, not in maze_nav).
"""
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt

FREE_MAX = 25


def seen_free_clearance(grid):
    """Metres from each cell to the nearest cell that is not seen-free (wall, unknown, map edge)."""
    free = (grid.data >= 0) & (grid.data <= FREE_MAX)
    return distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1]*grid.resolution


def cell_centres(grid):
    """World (x, y) of every cell centre, each (H, W)."""
    h, w = grid.data.shape
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    xy = grid.world(np.stack([cols.ravel(), rows.ravel()], axis=1))
    return xy[:, 0].reshape(h, w), xy[:, 1].reshape(h, w)


def open_region(grid, open_radius, start=None, min_displacement=0., clearance=None):
    """Cells inside a seen-free disk of radius open_radius whose centre is >= min_displacement
    from start: the open floor beyond the exit (an open start area is not the exit)."""
    clearance = seen_free_clearance(grid) if clearance is None else clearance
    centres = clearance >= open_radius
    if start is not None and min_displacement > 0 and centres.any():
        x, y = cell_centres(grid)
        centres &= np.hypot(x-start[0], y-start[1]) >= min_displacement
    if not centres.any():
        return centres
    # Disk union: every cell within open_radius of an open centre.
    return distance_transform_edt(~centres)*grid.resolution <= open_radius


def exit_masked_grid(grid, open_radius, start=None, min_displacement=0., clearance=None):
    """Copy of grid with the open floor beyond the exit marked occupied, or None if there is none.
    Frontier selection on it cannot pick goals out there (lite3_nav addition)."""
    region = open_region(grid, open_radius, start, min_displacement, clearance)
    if not region.any():
        return None
    # Slightly larger than the disks so no frontier border is left at their rim.
    region = binary_dilation(region, iterations=2)
    data = grid.data.copy()
    data[region] = 100
    return type(grid)(data, grid.resolution, grid.origin, grid.yaw)


@dataclass
class ExitDetector:
    """Borrowed from maze_nav (origin/navigation 419f205) ExitDetector; see the module docstring."""
    enclosed_radius: float = 0.7
    open_radius: float = 1.1
    near_radius: float = 0.8
    min_displacement: float = 1.5
    dwell: float = 2.0
    final_cell_xy: tuple = None
    final_cell_radius: float = 0.4
    start: tuple = None
    seen_enclosed: bool = False
    since: float = None
    found: bool = False
    reason: str = ''
    clearance: float = 0.0          # seen-free clearance at the robot
    open_nearby: float = 0.0        # best clearance within near_radius (away from start)
    last_time: float = field(default=None, repr=False)

    def reset(self, start=None):
        self.start = None if start is None else tuple(start[:2])
        self.seen_enclosed = self.found = False
        self.since = self.last_time = None
        self.reason = ''
        self.clearance = self.open_nearby = 0.0

    def update(self, grid, xy, now, clearance=None, open_test=True):
        """True once the robot has left through the exit (latched). clearance: optional cached
        seen_free_clearance(grid). open_test=False checks only final_cell_xy."""
        if self.found:
            return True
        xy = (float(xy[0]), float(xy[1]))
        if self.start is None:
            self.start = xy
        if self.final_cell_xy is not None and \
                math.hypot(xy[0]-self.final_cell_xy[0], xy[1]-self.final_cell_xy[1]) <= self.final_cell_radius:
            self.found = True
            self.reason = 'FINAL_CELL'
            return True
        if not open_test:
            self.since = None
            return False
        if self.last_time is not None and not 0.0 <= now-self.last_time <= 2.0:
            self.since = None                   # paused / clock jumped: start over
        self.last_time = now
        self.clearance = self.open_nearby = 0.0
        c = grid.cell(xy)
        if grid.contains(c):
            d = seen_free_clearance(grid) if clearance is None else clearance
            self.clearance = float(d[c[1], c[0]])
            res = grid.resolution
            r = int(math.ceil(self.near_radius/res))
            y0, y1 = max(0, c[1]-r), min(d.shape[0], c[1]+r+1)
            x0, x1 = max(0, c[0]-r), min(d.shape[1], c[0]+r+1)
            yy, xx = np.mgrid[y0:y1, x0:x1]
            w = grid.world(np.stack([xx.ravel(), yy.ravel()], axis=1))
            wx, wy = w[:, 0].reshape(xx.shape), w[:, 1].reshape(xx.shape)
            ok = ((np.hypot(wx-xy[0], wy-xy[1]) <= self.near_radius)
                  & (np.hypot(wx-self.start[0], wy-self.start[1]) >= self.min_displacement))
            if ok.any():
                self.open_nearby = float(d[y0:y1, x0:x1][ok].max())
        self.seen_enclosed |= 0 < self.clearance <= self.enclosed_radius
        moved = math.hypot(xy[0]-self.start[0], xy[1]-self.start[1]) >= self.min_displacement
        if self.seen_enclosed and moved and self.open_nearby >= self.open_radius:
            self.since = now if self.since is None else self.since
            if now-self.since >= self.dwell:
                self.found = True
                self.reason = 'OPEN_FLOOR'
        else:
            self.since = None
        return self.found
