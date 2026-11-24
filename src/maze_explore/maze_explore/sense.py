"""What is on each side of a cell, read from the SLAM map and a height map (no ROS in here).

Two sources:
  occupancy grid (slam_toolbox /map): built from the lidar slice 0.32-0.48 m above the floor,
      so it shows everything a wall-height obstacle: walls, curtains, the upper stair steps.
  HeightMap: the highest lidar return above the floor per 5 cm square. Walls stop at 0.53 m;
      a curtain hangs from 0.9 m; stairs rise step by step.

kind(cell, d) puts the two together:
  wall      something wall-high along the edge
  curtain   as a wall, but it carries on above wall height along the edge
  unknown   the edge has not been seen well enough yet
  exit      open, and beyond it there are no walls at all (outside the maze)
  object    open, but something taller than terrain stands inside the cell behind (a toy
            set, a prop): never walked into, looked at from here like a room
  room      open, the cell behind it is closed on its three other sides (dead end)
  stairs    the cell behind is blocked by something that rises like steps (open border, or a
            border that is lower than a wall with steps rising behind it)
  blocked   open, but the cell behind is blocked by something else
  free      open, and the cell behind can be walked into
"""
import numpy as np

from maze_explore.grid import nb, opp

OCC = 50          # occupancy value from which a map cell counts as occupied


class Occupancy:
    def __init__(self, data, res, ox, oy):
        self.data, self.res, self.ox, self.oy = data, res, ox, oy

    def at(self, x, y):
        """Occupancy at map points (arrays); -1 = unknown or outside the map."""
        c = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        r = np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        h, w = self.data.shape
        ok = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        out = np.full(np.shape(x), -1, np.int16)
        out[ok] = self.data[r[ok], c[ok]]
        return out

    def occupied_points(self):
        r, c = np.nonzero(self.data >= OCC)
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res


class HeightMap:
    """Highest return above the floor per square, and how often a square was seen tall."""

    def __init__(self, half=25.0, res=0.05, tall=0.65, obj=0.38):
        # obj: above terrain (<= 0.25 m, plus height error), below the props (>= 0.5 m)
        self.res, self.half, self.tall, self.obj = res, half, tall, obj
        n = int(2 * half / res)
        self.top = np.full((n, n), np.nan, np.float32)
        self.tall_hits = np.zeros((n, n), np.uint16)
        self.obj_hits = np.zeros((n, n), np.uint16)

    def _idx(self, x, y):
        c = np.floor((np.asarray(x) + self.half) / self.res).astype(int)
        r = np.floor((np.asarray(y) + self.half) / self.res).astype(int)
        n = self.top.shape[0]
        return r, c, (r >= 0) & (r < n) & (c >= 0) & (c < n)

    def add(self, x, y, h):
        r, c, ok = self._idx(x, y)
        r, c, h = r[ok], c[ok], np.asarray(h, np.float32)[ok]
        cur = self.top[r, c]
        self.top[r, c] = np.where(np.isnan(cur), h, cur)      # first value, then the max
        np.maximum.at(self.top, (r, c), h)
        t = h >= self.tall
        np.add.at(self.tall_hits, (r[t], c[t]), 1)
        o = h >= self.obj
        np.add.at(self.obj_hits, (r[o], c[o]), 1)

    def top_at(self, x, y):
        r, c, ok = self._idx(x, y)
        out = np.full(np.shape(x), np.nan, np.float32)
        out[ok] = self.top[r[ok], c[ok]]
        return out

    def tall_at(self, x, y, hits=2):
        r, c, ok = self._idx(x, y)
        out = np.zeros(np.shape(x), bool)
        out[ok] = self.tall_hits[r[ok], c[ok]] >= hits
        return out


    def obj_at(self, x, y, hits=3):
        """Squares seen at object height (>= obj) at least `hits` times."""
        r, c, ok = self._idx(x, y)
        out = np.zeros(np.shape(x), bool)
        out[ok] = self.obj_hits[r[ok], c[ok]] >= hits
        return out


ALONG = np.arange(-0.30, 0.301, 0.05)        # the middle 0.6 m of an edge (posts left out)
ACROSS = np.arange(-0.10, 0.101, 0.05)       # the edge line +- 0.10 m
CORRIDOR = np.arange(-0.25, 0.251, 0.05)     # the middle of the cell behind an edge
# A cell's interior, 0.25 m clear of its borders: walls, posts and curtains stand on the
# borders, and in the height map a wall smears up to 0.2 m to each side (pose noise). Checked on
# bag run1: with 0.25 m the four cells the robot walked through read 0 squares, the cell with
# the market stall 21 or more.
INSIDE = np.arange(-0.25, 0.251, 0.05)
INSIDE_DEPTH = np.arange(0.26, 0.761, 0.05)


class Sense:
    def __init__(self, grid, occ, heights, params=None):
        self.grid, self.occ, self.hm = grid, occ, heights
        p = dict(wall_frac=0.40, open_frac=0.15, seen_frac=0.70, curtain_frac=0.50,
                 blocked_frac=0.50, stair_top=(0.33, 0.60), stair_front=(0.08, 0.42),
                 exit_depth=2.0, object_squares=8,
                 wall_top_min=0.50)
        p.update(params or {})
        self.p = p

    def edge(self, cell, d):
        """('wall' | 'open' | 'unknown', tall) for the edge on side d of cell."""
        x, y = self.grid.box(cell, d, ALONG, ACROSS)
        o = self.occ.at(x, y)
        wall = (o >= OCC).any(axis=1)
        seen = (o >= 0).mean(axis=1) >= 0.6
        tall = self.hm.tall_at(x, y).any(axis=1).mean() >= self.p['curtain_frac']
        if wall.mean() >= self.p['wall_frac']:
            return 'wall', tall
        if wall.mean() <= self.p['open_frac'] and seen.mean() >= self.p['seen_frac']:
            return 'open', tall
        return 'unknown', tall

    def interior(self, cell, d):
        """Looking from `cell` through edge d into the neighbour: (blocked, front, top).
        blocked: something wall-high across the middle of the neighbour. front / top: the
        highest returns 0.05-0.30 m and 0.35-0.65 m past the edge (NaN = nothing seen)."""
        x, y = self.grid.box(cell, d, CORRIDOR, np.arange(0.15, 0.851, 0.05))
        blocked = (self.occ.at(x, y) >= OCC).any(axis=1).mean() >= self.p['blocked_frac']

        def high(d0, d1):
            hx, hy = self.grid.box(cell, d, CORRIDOR, np.arange(d0, d1 + 0.001, 0.05))
            t = self.hm.top_at(hx, hy)
            t = t[~np.isnan(t)]
            return float(np.percentile(t, 90)) if len(t) >= 6 else float('nan')
        return blocked, high(0.05, 0.30), high(0.35, 0.65)

    def objects(self, cell, d):
        """Something taller than terrain stands inside the neighbour on side d: at least
        object_squares 5 cm squares of its interior were seen at object height."""
        x, y = self.grid.box(cell, d, INSIDE, INSIDE_DEPTH)
        return int(self.hm.obj_at(x, y).sum()) >= self.p['object_squares']

    def outside(self, cell, d):
        """Open edge with no walls for exit_depth beyond it, nor beside the next cell."""
        x, y = self.grid.box(cell, d, np.arange(-0.45, 0.451, 0.05),
                             np.arange(0.20, self.p['exit_depth'] + 0.001, 0.05))
        o = self.occ.at(x, y)
        if (o >= OCC).mean() > 0.01 or (o == 0).mean() < 0.5:
            return False
        b = nb(cell, d)
        return all(self.edge(b, k)[0] != 'wall' for k in ((d + 1) % 4, (d + 3) % 4))

    def kind(self, cell, d, may_exit=True):
        state, tall = self.edge(cell, d)
        if state == 'wall':
            if tall:
                return 'curtain'
            # Stairs that start right at the border look like a wall in the map. A wall is
            # 0.53 m at the border; stairs are lower there and rise behind it.
            x, y = self.grid.box(cell, d, ALONG, ACROSS)
            t = self.hm.top_at(x, y)
            t = t[~np.isnan(t)]
            low_edge = len(t) >= 6 and float(np.percentile(t, 90)) < self.p['wall_top_min']
            return 'stairs' if low_edge and self.steps(*self.interior(cell, d)[1:]) else 'wall'
        if state == 'unknown':
            return 'unknown'
        if may_exit and self.outside(cell, d):
            return 'exit'
        b = nb(cell, d)
        blocked, front, top = self.interior(cell, d)
        # stairs are tall inside too: they are let through only with the full picture
        # (wall-high in the map, a lower step seen in front of a higher one)
        stairs = blocked and not np.isnan(front) and self.steps(front, top)
        if self.objects(cell, d) and not stairs:
            return 'object'
        others = [self.edge(b, k)[0] for k in range(4) if k != opp(d)]
        if all(s == 'wall' for s in others) or \
                (blocked and others.count('wall') >= 2 and 'open' not in others):
            return 'room'
        if blocked:
            return 'stairs' if self.steps(front, top) else 'blocked'
        return 'free'

    def steps(self, front, top):
        """Heights just past the border (front) and 0.35-0.65 m past it (top) rise like steps."""
        lo, hi = self.p['stair_top']
        flo, fhi = self.p['stair_front']
        return bool(lo <= top <= hi and (np.isnan(front) or (flo <= front <= fhi and front < top)))
