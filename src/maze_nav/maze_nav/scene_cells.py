"""Maze cells, and when the robot has reached one it hasn't been in before (no ROS).

The maze is a square grid of `size` m cells. CellGrid maps the SLAM `map` frame to cell
indices (i, j): cell (0, 0) is the one centred on `origin` (by default the map origin, i.e.
where the robot stood at bringup: on the start cell, its rear ~8 cm from the back wall, puts
base_link within a few cm of the start cell's centre), and +i points along `yaw` (0 = the
robot's heading at bringup, so a robot placed square in the start cell lines the grid up
with the maze).

NewCellTrigger: tracks which cell the robot is in (`current`, and `previous`: the cell it
came from) and returns one SceneEvent the first time the robot enters each cell, including
the start cell (`trigger_at_start`). Going back into a cell entered before never fires.
"Entered" uses hysteresis: base_link must be `enter_margin` m inside a cell (away from all
its edges), so straddling a boundary doesn't flip cells or fire early.

JunctionDetector: once per cell, when the robot first gets within `centre_radius` of a
cell's centre, the cell's open sides other than the way in; two or more = a junction (the
explorer then faces each way, left to right). DeadEndDetector: the robot faces (within
`heading_tol`) a neighbouring cell whose three other sides are all walls on /map: a dead
end, reported once per cell while the robot is still in the cell before it (that is what
fires scene understanding).
side_walls(): the middles of a dead end's left and right walls, which the explorer turns
the camera to from the cell before it (posters and QR codes on the side walls).

run_active() reads maze_explorer's /maze/status, so the node only fires while the robot is
actually running the maze (not while it holds still before /maze/start, nor after the end).

opening() tells, from the occupancy grid, whether a side of a cell is open or a wall; the
node uses it to list the new cell's ways out.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage

from maze_nav.core import FREE_MAX, OCCUPIED_MIN, Grid

# Neighbour directions in cell indices, with names for the messages.
DIRECTIONS = {'+i': (1, 0), '+j': (0, 1), '-i': (-1, 0), '-j': (0, -1)}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class CellGrid:
    size: float = 1.0
    origin: tuple = (0.0, 0.0)   # map (x, y) of the centre of cell (0, 0)
    yaw: float = 0.0             # rad: map-frame heading of the grid's +i axis

    def local(self, xy):
        """Map (x, y) -> continuous cell coordinates (u, v); cell centres are integers."""
        dx, dy = xy[0] - self.origin[0], xy[1] - self.origin[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy) / self.size, (-s * dx + c * dy) / self.size

    def world(self, uv):
        """Continuous cell coordinates (u, v) -> map (x, y)."""
        u, v = uv[0] * self.size, uv[1] * self.size
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.origin[0] + c * u - s * v, self.origin[1] + s * u + c * v

    def cell(self, xy):
        u, v = self.local(xy)
        return int(math.floor(u + 0.5)), int(math.floor(v + 0.5))

    def centre(self, cell):
        return self.world(cell)

    def depth_inside(self, xy, cell):
        """Metres from xy to the nearest edge of `cell` (negative: xy is outside it)."""
        u, v = self.local(xy)
        return (0.5 - max(abs(u - cell[0]), abs(v - cell[1]))) * self.size

    def heading(self, d):
        """Map-frame heading (rad) of neighbour direction d = (di, dj)."""
        return wrap(self.yaw + math.atan2(d[1], d[0]))


def exit_cell(cells, xy, cols, rows, margin=0.15):
    """The cell xy is in if it lies outside the maze, else None.

    The maze is `cols` cells along +i by `rows` along +j with the start cell (0, 0) in its
    corner (cells (0..cols-1, 0..rows-1)). xy has to be at least `margin` m past the maze's
    outer edge, so a point in the exit opening itself doesn't count. cols or rows <= 0 = no
    known maze size: never an exit cell.
    """
    if cols <= 0 or rows <= 0:
        return None
    u, v = cells.local(xy)
    m = margin / cells.size
    if -0.5 - m <= u <= cols - 0.5 + m and -0.5 - m <= v <= rows - 0.5 + m:
        return None
    return cells.cell(xy)


def fit_grid(points, cells, max_yaw=math.radians(8.0), step=math.radians(0.25)):
    """The cell grid that best lines up with the maze walls: (CellGrid, score), or None.

    points: (N, 2) map (x, y) of wall pixels near the maze. Walls lie on cell edges, at
    (k + 1/2) cell sizes from cell (0, 0)'s centre along both axes, so for the right yaw the
    wall coordinates bunch up at one phase of the cell size. For each yaw within max_yaw of
    `cells.yaw`, score = |mean exp(2 pi i u / size)| + same for v (0 = spread evenly, 2 =
    every point on a grid line); the best yaw's mean phases give the shift that puts the
    lines on cell edges (at most half a cell). Same size, cell (0, 0) stays the cell nearest
    the old origin.
    """
    pts = np.asarray(points, float).reshape(-1, 2)
    if len(pts) == 0:
        return None
    k = 2 * math.pi / cells.size
    d = pts - np.asarray(cells.origin, float)
    best = None
    for yaw in cells.yaw + np.arange(-max_yaw, max_yaw + 1e-9, step):
        c, s = math.cos(yaw), math.sin(yaw)
        zu = np.exp(1j * k * (c * d[:, 0] + s * d[:, 1])).mean()
        zv = np.exp(1j * k * (-s * d[:, 0] + c * d[:, 1])).mean()
        score = abs(zu) + abs(zv)
        if best is None or score > best[0]:
            best = (score, yaw, zu, zv)
    score, yaw, zu, zv = best
    # wall phase pi (k + 1/2 sizes) when aligned; the offset is what moves it there
    du = wrap(np.angle(zu) - math.pi) / k
    dv = wrap(np.angle(zv) - math.pi) / k
    c, s = math.cos(yaw), math.sin(yaw)
    origin = (cells.origin[0] + c * du - s * dv, cells.origin[1] + s * du + c * dv)
    return CellGrid(cells.size, origin, wrap(yaw)), float(score)


def boundary_sides(cols, rows):
    """[(cell, d)] every outer side of the cols x rows maze: cell inside, d = (di, dj) out."""
    out = []
    for i in range(cols):
        out += [((i, 0), (0, -1)), ((i, rows - 1), (0, 1))]
    for j in range(rows):
        out += [((0, j), (-1, 0)), ((cols - 1, j), (1, 0))]
    return out


def opening(grid, cells, cell, d, half_width=0.35, depth=0.1, min_blocked=0.3,
            min_known=0.5):
    """'open', 'wall' or 'unknown': the side of `cell` towards neighbour direction d.

    Looks at a strip of the occupancy grid `grid` (core.Grid) across the shared edge:
    `half_width` m either side of its middle (the corner posts at +-0.5 m stay out) and
    `depth` m either side of the edge line (walls are 0.05 m thick and SLAM may draw them a
    cell off). Each lateral step is one column: it is blocked if any of its samples is a
    wall, known if any is free or wall. A wall if >= min_blocked of the columns are blocked;
    unknown if < min_known are known (or the strip is off the map); otherwise open.
    """
    res = grid.resolution
    step = res / 2
    lateral = np.arange(-half_width, half_width + 1e-9, step) / cells.size
    along = np.arange(-depth, depth + 1e-9, step) / cells.size
    perp = (-d[1], d[0])
    edge = (cell[0] + 0.5 * d[0], cell[1] + 0.5 * d[1])
    blocked = known = 0
    for t in lateral:
        col_blocked = col_known = False
        for s in along:
            x, y = cells.world((edge[0] + s * d[0] + t * perp[0],
                                edge[1] + s * d[1] + t * perp[1]))
            gc = grid.cell((x, y))
            if not grid.contains(gc):
                continue
            value = grid.data[gc[1], gc[0]]
            if value >= OCCUPIED_MIN:
                col_blocked = col_known = True
            elif 0 <= value <= FREE_MAX:
                col_known = True
        blocked += col_blocked
        known += col_known
    n = len(lateral)
    if blocked >= min_blocked * n:
        return 'wall'
    if known < min_known * n:
        return 'unknown'
    return 'open'


# maze_explorer states (/maze/status, "STATE" or "STATE: why") in which the run is under way.
ACTIVE_STATES = ('CREEPING', 'SCANNING', 'EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR',
                 'JUNCTION', 'DEAD_END')


def run_active(status, active_states=ACTIVE_STATES):
    """True if a /maze/status string says the robot is running the maze."""
    if not status:
        return False
    return status.split(':', 1)[0].strip() in active_states


@dataclass
class SceneEvent:
    cell: tuple             # the cell just entered for the first time
    from_cell: tuple        # the cell the robot came from (None for the start cell)
    direction: str          # '+i', '+j', '-i' or '-j' moved; '' at the start or after a jump
    yaw: float              # rad, map frame: robot heading when it entered


class NewCellTrigger:
    """Feed it the robot position; it returns a SceneEvent on entering a cell for the first time."""

    def __init__(self, cells, enter_margin=0.15, trigger_at_start=True):
        self.cells = cells
        self.enter_margin = enter_margin
        self.trigger_at_start = trigger_at_start
        self.current = None          # cell the robot is in (last one entered)
        self.previous = None         # the cell it entered `current` from (None at the start)
        self.visited = set()

    def update(self, xy, yaw):
        """One pose sample (map frame x, y and yaw in rad)."""
        c = self.cells.cell(xy)
        if c == self.current:
            return None
        if self.current is not None and self.cells.depth_inside(xy, c) < self.enter_margin:
            return None              # near the edge: not entered yet
        prev, self.current = self.current, c
        self.previous = prev
        if c in self.visited:
            return None
        self.visited.add(c)
        if prev is None and not self.trigger_at_start:
            return None
        step = None if prev is None else (c[0] - prev[0], c[1] - prev[1])
        name = next((n for n, d in DIRECTIONS.items() if d == step), '')
        return SceneEvent(c, prev, name, yaw)


def opposite(d):
    return (-d[0], -d[1])


def name_of(d):
    return next(n for n, v in DIRECTIONS.items() if v == d)


def trace_way(openings, cell, d, max_cells=10):
    """Where does the way out of `cell` towards d lead? Follows it cell by cell on the map
    while each cell has exactly one way on (corridors, bends included).

    `openings(cell, d)` -> 'open' / 'wall' / 'unknown' (opening() on /map). Returns a dict:
      {'kind': 'dead_end', 'cell': dead-end cell, 'before': the cell before it (`cell` itself
       when the dead end is next door), 'dir': the direction from 'before' into it, 'length':
       cells walked to reach it}
      {'kind': 'junction' | 'unknown' | 'open', 'cell': where the trace stopped, 'length'}
    'unknown': a side not yet seen (so it may still be a dead end); 'open': longer than
    `max_cells` without an end.
    """
    prev, cur, into = cell, (cell[0] + d[0], cell[1] + d[1]), d
    for n in range(1, max_cells + 1):
        back = opposite(into)
        sides = {v: openings(cur, v) for v in DIRECTIONS.values() if v != back}
        if any(s == 'unknown' for s in sides.values()):
            return {'kind': 'unknown', 'cell': cur, 'length': n}
        ways = [v for v, s in sides.items() if s == 'open']
        if not ways:
            return {'kind': 'dead_end', 'cell': cur, 'before': prev, 'dir': into, 'length': n}
        if len(ways) > 1:
            return {'kind': 'junction', 'cell': cur, 'length': n}
        prev, into = cur, ways[0]
        cur = (cur[0] + into[0], cur[1] + into[1])
    return {'kind': 'open', 'cell': cur, 'length': max_cells}


def side_walls(cells, cell, d):
    """Map (x, y) of the middles of `cell`'s left and right sides, seen looking along d (the
    way in): where the explorer points the camera to scan a dead end's side walls."""
    left = (-d[1], d[0])
    return {'left': cells.world((cell[0] + 0.5 * left[0], cell[1] + 0.5 * left[1])),
            'right': cells.world((cell[0] - 0.5 * left[0], cell[1] - 0.5 * left[1]))}


def side_heading(xy, point, facing=None, max_off=math.radians(60)):
    """Map yaw from xy to a side-wall point; with `facing` (the dead end's heading) given, at
    most `max_off` rad either side of it (a robot standing well off-centre would otherwise
    turn round to a wall it can't see past the corner)."""
    h = math.atan2(point[1] - xy[1], point[0] - xy[0])
    if facing is None:
        return h
    off = max(-max_off, min(max_off, wrap(h - facing)))
    return wrap(facing + off)


class JunctionDetector:
    """Once per cell: is it a junction (2+ open ways on besides the way in)?

    `openings(cell, d)` -> 'open' / 'wall' / 'unknown' (scene_cells.opening on /map). The
    check runs the first time base_link is within `centre_radius` m (in both axes) of the
    cell's centre, so the explorer's turns happen near the middle of the cell, and only
    when the way in is known (not the start cell, not after a pose jump). Unknown sides
    don't count as open. Returns the open direction names (sorted), or None.
    """

    def __init__(self, cells, centre_radius=0.25, min_paths=2):
        self.cells = cells
        self.centre_radius = centre_radius
        self.min_paths = min_paths
        self.checked = set()

    def update(self, xy, cell, came_from, openings):
        if cell is None or cell in self.checked or self.cells.cell(xy) != cell:
            return None
        if self.cells.depth_inside(xy, cell) < 0.5 * self.cells.size - self.centre_radius:
            return None
        self.checked.add(cell)
        if came_from is None:
            return None
        back = (came_from[0] - cell[0], came_from[1] - cell[1])
        if back not in DIRECTIONS.values():
            return None                              # jumped: no single way in
        ways = sorted(n for n, d in DIRECTIONS.items()
                      if d != back and openings(cell, d) == 'open')
        return ways if len(ways) >= self.min_paths else None


class DeadEndDetector:
    """The robot, in `cell`, faces a neighbouring dead end: report it once.

    Facing = the robot's yaw within `heading_tol` rad of a grid direction d. The dead end is
    the neighbour n = cell + d: the side between the two must be open and n's other three
    sides (its left, right and far side seen from the robot) all 'wall' (a large obstacle
    across a side reads as a wall too; 'unknown' doesn't count). base_link must not be in n
    yet. Cells in `skip` (e.g. already visited) are never reported. Returns
    (n, d_name, {side name: opening}) or None.
    """

    def __init__(self, cells, heading_tol=math.radians(25)):
        self.cells = cells
        self.heading_tol = heading_tol
        self.reported = set()

    def facing(self, yaw):
        for name, d in DIRECTIONS.items():
            if abs(wrap(yaw - self.cells.heading(d))) <= self.heading_tol:
                return name, d
        return None, None

    def update(self, xy, yaw, cell, openings, skip=()):
        if cell is None:
            return None
        name, d = self.facing(yaw)
        if d is None:
            return None
        n = (cell[0] + d[0], cell[1] + d[1])
        if n in self.reported or n in skip or self.cells.cell(xy) == n:
            return None
        if openings(cell, d) != 'open':
            return None
        sides = {}
        for k, v in DIRECTIONS.items():
            if v == opposite(d):
                sides[k] = 'open'
                continue
            sides[k] = openings(n, v)
            if sides[k] != 'wall':
                return None
        self.reported.add(n)
        return n, name, sides


def cell_objects(grid, cells, border=0.12, min_extent=0.2, min_pixels=4, min_known=0.5):
    """{cell: (extent_m, (x, y))}: cells with an object inside (user 2026-11-21: rooms may
    hold objects with a footprint of at least 0.3 x 0.3 m; the robot must not enter them).

    An object is a FREE-STANDING connected blob of occupied map cells (8-connected, over the
    whole map): every one of its map cells lies in one cell's interior (more than `border` m
    from its edges, where the walls and SLAM's blur of them are), it is at least `min_extent`
    m across and `min_pixels` map cells big. A blob that reaches out of the interior is part
    of the walls, however far into the cell it comes: the real walls are not on the grid lines
    (the grid starts at the robot's start pose; robot 2026-11-21 08:32: walls 0.10-0.15 m off
    them, 5 cells blocked) and some maze walls run through a cell's middle (sim example maze:
    9 cells "0.75 m across" blocked). The price: an object touching a wall on the map is not
    found (Nav2 still drives around it). The lidar only sees an object's faces, so a 0.3 m box
    can show as a single 0.3 m line. Cells less than `min_known` mapped inside are skipped.
    Only things in the wall band (wall_scan: 0.30-0.45 m above the floor) are on the map;
    lower objects can't be seen here.
    """
    h, w = grid.data.shape
    res = grid.resolution
    occ_all = grid.data >= OCCUPIED_MIN
    if not occ_all.any():
        return {}
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    u, v = cells.local((grid.origin[0] + (cols + 0.5) * res, grid.origin[1] + (rows + 0.5) * res))
    ci, cj = np.floor(u + 0.5).astype(int), np.floor(v + 0.5).astype(int)
    inner = np.maximum(np.abs(u - ci), np.abs(v - cj)) <= 0.5 - border / cells.size
    lab, n = ndimage.label(occ_all, structure=np.ones((3, 3), int))
    idx = np.arange(1, n + 1)
    # Wall-connected: any map cell of the blob outside every cell interior.
    walls = np.bincount(lab[occ_all & ~inner], minlength=n + 1)[1:] > 0
    npx = np.bincount(lab[occ_all], minlength=n + 1)[1:]
    out = {}
    cand = idx[~walls & (npx >= min_pixels)]
    if len(cand) == 0:
        return out
    umin, umax = ndimage.minimum(u, lab, cand), ndimage.maximum(u, lab, cand)
    vmin, vmax = ndimage.minimum(v, lab, cand), ndimage.maximum(v, lab, cand)
    umean, vmean = ndimage.mean(u, lab, cand), ndimage.mean(v, lab, cand)
    for k, u0, u1, v0, v1, um, vm in zip(cand, umin, umax, vmin, vmax, umean, vmean):
        extent = max(u1 - u0, v1 - v0) * cells.size + res
        if extent < min_extent:
            continue
        cell = (int(math.floor(um + 0.5)), int(math.floor(vm + 0.5)))   # all in one interior
        in_cell = inner & (ci == cell[0]) & (cj == cell[1])
        if (grid.data[in_cell] >= 0).mean() < min_known:
            continue
        if cell not in out or extent > out[cell][0]:
            xy = cells.world((float(um), float(vm)))
            out[cell] = (float(extent), (float(xy[0]), float(xy[1])))
    return out


def object_box(grid, cells, cell, border=0.12):
    """(x0, y0, x1, y1) map-frame box around the occupied map cells inside `cell`'s interior
    (more than `border` m from its edges: walls and their SLAM blur stay out), or None.
    The lidar sees only an object's faces, so this grows as more of it is seen; callers
    keep the union over maps (grow_box)."""
    h, w = grid.data.shape
    res = grid.resolution
    corners = np.array([cells.world((cell[0] + a, cell[1] + b))
                        for a in (-0.5, 0.5) for b in (-0.5, 0.5)])
    lo = np.floor((corners.min(axis=0) - np.array(grid.origin)) / res).astype(int)
    hi = np.ceil((corners.max(axis=0) - np.array(grid.origin)) / res).astype(int)
    c0, r0 = max(lo[0], 0), max(lo[1], 0)
    c1, r1 = min(hi[0], w - 1), min(hi[1], h - 1)
    if c1 < c0 or r1 < r0:
        return None
    cols, rows = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
    x, y = grid.origin[0] + (cols + 0.5) * res, grid.origin[1] + (rows + 0.5) * res
    u, v = cells.local((x, y))
    inner = np.maximum(np.abs(u - cell[0]), np.abs(v - cell[1])) <= 0.5 - border / cells.size
    occ = inner & (grid.data[rows, cols] >= OCCUPIED_MIN)
    if not occ.any():
        return None
    half = res / 2
    return (float(x[occ].min() - half), float(y[occ].min() - half),
            float(x[occ].max() + half), float(y[occ].max() + half))


def grow_box(box, new, min_size=0.3):
    """Union of two boxes (either may be None), at least min_size m on each side (the user's
    smallest object footprint, 0.3 x 0.3 m, 2026-11-21: a box seen as one face is a line)."""
    if new is not None:
        box = new if box is None else (min(box[0], new[0]), min(box[1], new[1]),
                                       max(box[2], new[2]), max(box[3], new[3]))
    if box is None:
        return None
    x0, y0, x1, y1 = box
    if x1 - x0 < min_size:
        c = (x0 + x1) / 2
        x0, x1 = c - min_size / 2, c + min_size / 2
    if y1 - y0 < min_size:
        c = (y0 + y1) / 2
        y0, y1 = c - min_size / 2, c + min_size / 2
    return (x0, y0, x1, y1)


def block_boxes(grid, boxes, margin=0.0, value=100):
    """Copy of grid with every map cell inside the given (x0, y0, x1, y1) map boxes, grown
    by margin m, set to `value`: an object's footprint only, not its whole maze cell
    (presentation mazes 2026-11-21: the object fills about half of a room)."""
    if not boxes:
        return grid
    data = grid.data.copy()
    h, w = data.shape
    res = grid.resolution
    ox, oy = grid.origin
    for x0, y0, x1, y1 in boxes:
        c0 = max(int(math.floor((x0 - margin - ox) / res)), 0)
        r0 = max(int(math.floor((y0 - margin - oy) / res)), 0)
        c1 = min(int(math.ceil((x1 + margin - ox) / res)) - 1, w - 1)
        r1 = min(int(math.ceil((y1 + margin - oy) / res)) - 1, h - 1)
        if c1 >= c0 and r1 >= r0:
            data[r0:r1 + 1, c0:c1 + 1] = value
    return Grid(data, grid.resolution, grid.origin)


def in_boxes(xy, boxes, margin=0.0):
    """Is the point inside any (x0, y0, x1, y1) box grown by margin?"""
    x, y = xy
    return any(x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin
               for x0, y0, x1, y1 in boxes)


def block_cells(grid, cells, blocked, value=100):
    """Copy of grid (core.Grid) with every map cell inside the given maze cells set to
    `value` (occupied), so no frontier goal and no Nav2 path goes into them."""
    if not blocked:
        return grid
    data = grid.data.copy()
    h, w = data.shape
    res = grid.resolution
    for cell in blocked:
        corners = np.array([cells.world((cell[0] + a, cell[1] + b))
                            for a in (-0.5, 0.5) for b in (-0.5, 0.5)])
        lo = np.floor((corners.min(axis=0) - np.array(grid.origin)) / res).astype(int)
        hi = np.ceil((corners.max(axis=0) - np.array(grid.origin)) / res).astype(int)
        c0, r0 = max(lo[0], 0), max(lo[1], 0)
        c1, r1 = min(hi[0], w - 1), min(hi[1], h - 1)
        if c1 < c0 or r1 < r0:
            continue
        cols, rows = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
        u, v = cells.local((grid.origin[0] + (cols + 0.5) * res,
                            grid.origin[1] + (rows + 0.5) * res))
        inside = np.maximum(np.abs(u - cell[0]), np.abs(v - cell[1])) <= 0.5
        data[rows[inside], cols[inside]] = value
    return Grid(data, grid.resolution, grid.origin)
