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
                 exit_depth=2.0, object_squares=8, terrain_min=0.06, terrain_squares=15,
                 wall_top_min=0.50, low_wall_tall=0.20, low_wall_seen=0.50)
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

    def floor_seen(self, cell, d):
        """How many of the 5 cm squares that rough() looks at the lidar has seen at all."""
        x, y = self.grid.box(cell, d, INSIDE, INSIDE_DEPTH)
        return int(np.isfinite(self.hm.top_at(x, y)).sum())

    def rough(self, cell, d):
        """Uneven terrain in the neighbour on side d: at least terrain_squares 5 cm squares
        of its interior stand terrain_min or more above the floor, below object height."""
        x, y = self.grid.box(cell, d, INSIDE, INSIDE_DEPTH)
        t = self.hm.top_at(x, y)
        return int(((t >= self.p['terrain_min']) & (t < self.hm.obj)).sum()) >= self.p['terrain_squares']

    def rough_from_points(self, cell, d, x, y, h, min_seen=40, min_points=3):
        """Uneven terrain in the neighbour on side d, from dense points (the depth camera):
        map-frame x, y and height above the floor h. Same squares and thresholds as rough(),
        but each square takes the MEDIAN of its points (dense data: one stray point must not
        decide). Returns (rough, squares seen); rough is None with fewer than min_seen squares."""
        med = self.square_heights(cell, d, x, y, h, min_points)
        if len(med) < min_seen:
            return None, int(len(med))
        n = int(((med >= self.p['terrain_min']) & (med < self.hm.obj)).sum())
        return n >= self.p['terrain_squares'], int(len(med))

    def square_heights(self, cell, d, x, y, h, min_points=3):
        """Median height of each 5 cm square of the neighbour's interior (the squares
        rough() looks at) that holds at least min_points of the given points."""
        x, y, h = (np.asarray(v, float).ravel() for v in (x, y, h))
        b = nb(cell, d)
        bu, bv = b[0] * self.grid.size, b[1] * self.grid.size
        u, v = self.grid.to_grid(x, y)
        half = INSIDE[-1] + 0.025                  # the squares rough() samples, edge to edge
        ok = (np.abs(u - bu) <= half) & (np.abs(v - bv) <= half) & np.isfinite(h)
        if not ok.any():
            return np.zeros(0)
        iu = np.floor((u[ok] - bu + half) / 0.05).astype(int)
        iv = np.floor((v[ok] - bv + half) / 0.05).astype(int)
        key = iu * 1000 + iv
        order = np.argsort(key)
        key, hs = key[order], h[ok][order]
        starts = np.r_[0, np.nonzero(np.diff(key))[0] + 1]
        counts = np.diff(np.r_[starts, len(key)])
        med = np.array([np.median(hs[s:s + n]) for s, n in zip(starts, counts)])
        return med[counts >= min_points]

    def outside(self, cell, d):
        """Open edge with no walls for exit_depth beyond it, nor beside the next cell."""
        x, y = self.grid.box(cell, d, np.arange(-0.45, 0.451, 0.05),
                             np.arange(0.20, self.p['exit_depth'] + 0.001, 0.05))
        o = self.occ.at(x, y)
        if (o >= OCC).mean() > 0.01 or (o == 0).mean() < 0.5:
            return False
        b = nb(cell, d)
        return all(self.edge(b, k)[0] != 'wall' for k in ((d + 1) % 4, (d + 3) % 4))

    def last_cell(self, cell, d):
        """`cell` is shaped like the last cell, with its way out on side d: a stub that
        sticks out of the maze, walled on both sides of the way out and entered from
        behind (user, 2026-11-25, arena plan: the finish cell is one cell out from the
        block, then the open hall). Open floor alone is not an exit: on 2026-11-25 09:38
        the way from the cell after the start into the open terrain cells read as one."""
        sides = all(self.edge(cell, k)[0] == 'wall' for k in ((d + 1) % 4, (d + 3) % 4))
        return sides and self.edge(cell, (d + 2) % 4)[0] != 'wall'

    def low_wall(self, cell, d):
        """A wall of ordinary height stands on this edge: wall in the map, its top measured
        along at least half of it, and (almost) nothing curtain-high seen there. The colour
        camera sees a curtain OVER such a wall (walls are 0.53 m, curtains hang from 0.9 m)
        and has no depth, so a sighting in that direction is of a curtain further back, not
        of this edge (user, 2026-11-25). An edge whose height is not known is not one."""
        x, y = self.grid.box(cell, d, ALONG, ACROSS)
        if (self.occ.at(x, y) >= OCC).any(axis=1).mean() < self.p['wall_frac']:
            return False
        if self.hm.tall_at(x, y).any(axis=1).mean() >= self.p['low_wall_tall']:
            return False
        measured = (~np.isnan(self.hm.top_at(x, y))).any(axis=1).mean()
        return bool(measured >= self.p['low_wall_seen'])

    def tall_inside(self, cell, d):
        """(squares, measured) for the cell behind side d, its three other walls included:
        how many 5 cm squares were seen taller than an ordinary wall (HeightMap.tall, twice
        or more), and whether the far wall's top was measured along at least half of it
        (so 'nothing tall' means looked and found nothing, not 'not seen'). The four
        corners of the cell are left out."""
        # Only what is IN the cell counts (user, 2026-11-26): the squares up to the cell's own
        # borders (its walls stand on them, 0.51 m from the centre line, 1.02 m deep) and
        # nothing beyond. Until then the box reached 4 cm past the side borders and 10 cm
        # past the far one, so something tall just behind a wall, in the next cell, made
        # this cell a 'room with something in it'.
        along, depth = np.arange(-0.50, 0.501, 0.05), np.arange(0.05, 1.001, 0.05)
        x, y = self.grid.box(cell, d, along, depth)
        # Not the corners (0.2 m): a curtain or a frame wall on a NEIGHBOUR's edge ends at
        # the corner post and reads tall there (simulator, 2026-11-26: the empty room
        # beside the curtain of challenge_a gave 7 squares, all from the curtain's end).
        a, e = np.meshgrid(along, depth, indexing='ij')
        corner = (np.abs(a) > 0.35) & ((e < 0.30) | (e > 0.82))
        tall = self.hm.tall_at(x, y)
        squares = int((tall & ~corner).sum())
        fx, fy = self.grid.box(nb(cell, d), d, ALONG, ACROSS)
        measured = (~np.isnan(self.hm.top_at(fx, fy))).any(axis=1).mean() >= self.p['low_wall_seen']
        return squares, bool(measured)

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
        if may_exit and self.outside(cell, d) and self.last_cell(cell, d):
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
