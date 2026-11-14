"""Pure maze logic (numpy/scipy only, no ROS) so it can be unit-tested.

- wall_ranges():   3D points (odom frame) -> 2D laser ranges of the maze walls only
- Grid:            occupancy grid helpers (world <-> cell)
- choose_frontier: next exploration goal on the SLAM map
- ExitDetector:    "we were inside narrow corridors and now we are in open space"
"""
from dataclasses import dataclass, field
import math

import numpy as np
from scipy import ndimage

FREE_MAX = 25        # occupancy values 0..FREE_MAX count as free
OCCUPIED_MIN = 65    # values >= this count as walls


# --------------------------------------------------------------------------- wall scan

# PointField datatype codes -> numpy types (sensor_msgs/PointField)
_PF_TYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32,
             7: np.float32, 8: np.float64}


def cloud_xyz(data, fields, point_step, row_step, width, height, big_endian=False):
    """x, y, z of a PointCloud2 as (N, 3) float64, NaNs/infs dropped.

    Reads only the x/y/z fields by their offsets, so any other fields (the Livox driver adds
    intensity float32, tag/line uint8, timestamp float64) can have any type.
    sensor_msgs_py's read_points_numpy refuses clouds with mixed field types.
    fields: iterable of objects with .name, .offset, .datatype (sensor_msgs/PointField).
    """
    by_name = {f.name: f for f in fields}
    missing = [n for n in 'xyz' if n not in by_name]
    if missing:
        raise ValueError(f'cloud has no {missing} field(s)')
    order = '>' if big_endian else '<'
    dtype = np.dtype({'names': list('xyz'),
                      'formats': [np.dtype(_PF_TYPES[by_name[n].datatype]).newbyteorder(order)
                                  for n in 'xyz'],
                      'offsets': [by_name[n].offset for n in 'xyz'],
                      'itemsize': point_step})
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if height > 1 and row_step != width * point_step:     # padded rows: trim each row
        buf = buf[:height * row_step].reshape(height, row_step)[:, :width * point_step]
        buf = np.ascontiguousarray(buf).ravel()
    pts = buf[:width * height * point_step].view(dtype)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def wall_ranges(points_odom, robot_xy, robot_yaw, floor_z, band, angle_min, angle_inc,
                n_beams, range_min, range_max):
    """Flatten a point cloud to a LaserScan that sees maze walls and nothing else.

    points_odom: (N, 3) points in a gravity-aligned frame (odom). The height band is measured
    from the maze floor (floor_z, odom), not from the robot, so the band stays put while the
    robot's body rises and tilts on ramps, mats and stairs.
    band: (low, high) metres above the floor. Example maze: walls 0..0.45 m, terrain <= 0.15 m,
    stair treads <= 0.26 m, hand rails <= 0.535 m -> (0.30, 0.43) sees walls and rails only.
    Returns ranges (n_beams,) with +inf where nothing was hit; beam angles are in the robot's
    heading frame (yaw only).
    """
    ranges = np.full(n_beams, np.inf, dtype=np.float32)
    if len(points_odom) == 0:
        return ranges
    p = np.asarray(points_odom, dtype=np.float64)
    h = p[:, 2] - floor_z
    p = p[(h >= band[0]) & (h <= band[1])]
    if len(p) == 0:
        return ranges
    dx, dy = p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1]
    r = np.hypot(dx, dy)
    a = np.arctan2(dy, dx) - robot_yaw
    a = (a - angle_min) % (2 * math.pi)          # wrap into [0, 2pi) from angle_min
    idx = np.floor(a / angle_inc).astype(np.int64)
    keep = (r >= range_min) & (r <= range_max) & (idx >= 0) & (idx < n_beams)
    np.minimum.at(ranges, idx[keep], r[keep].astype(np.float32))
    return ranges


def estimate_floor(points_odom, robot_xy, robot_z, expected_drop=0.35, window=0.15,
                   min_points=200, layer_fraction=0.2):
    """Floor height (odom z) from the cloud around the standing robot, or None.

    Takes the LOWEST dense 1 cm height layer within `window` of the expected floor
    (robot_z - expected_drop) in a ring 0.4..2.5 m around the robot. Lowest, not densest: the
    lidar can't see the floor closer than ~1.2 m ahead, so terrain mats in the next cell
    (0.1-0.12 m high in the example maze) can hold more points than the floor itself.
    """
    p = np.asarray(points_odom, dtype=np.float64)
    if len(p) == 0:
        return None
    r = np.hypot(p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1])
    guess = robot_z - expected_drop
    z = p[(r > 0.4) & (r < 2.5) & (np.abs(p[:, 2] - guess) < window), 2]
    if len(z) < min_points:
        return None
    hist, edges = np.histogram(z, bins=np.arange(guess - window, guess + window + 0.01, 0.01))
    dense = np.nonzero(hist >= max(layer_fraction * hist.max(), min_points / 10))[0]
    i = int(dense[0])
    near = z[(z >= edges[i] - 0.01) & (z <= edges[i + 1] + 0.01)]
    return float(np.median(near))


# --------------------------------------------------------------------------- grid

@dataclass
class Grid:
    data: np.ndarray        # (H, W) int8 occupancy, row = y
    resolution: float
    origin: tuple           # world (x, y) of cell (0, 0)'s corner

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return cls(data, info.resolution, (info.origin.position.x, info.origin.position.y))

    def cell(self, xy):
        return (int(math.floor((xy[0] - self.origin[0]) / self.resolution)),
                int(math.floor((xy[1] - self.origin[1]) / self.resolution)))

    def world(self, cell):
        return (self.origin[0] + (cell[0] + 0.5) * self.resolution,
                self.origin[1] + (cell[1] + 0.5) * self.resolution)

    def contains(self, cell):
        return 0 <= cell[0] < self.data.shape[1] and 0 <= cell[1] < self.data.shape[0]

    @property
    def free(self):
        return (self.data >= 0) & (self.data <= FREE_MAX)

    @property
    def unknown(self):
        return self.data < 0

    @property
    def occupied(self):
        return self.data >= OCCUPIED_MIN


# --------------------------------------------------------------------------- frontiers

@dataclass
class Frontier:
    goal: tuple             # world (x, y) the robot should drive to
    yaw: float              # facing the unknown
    path_length: float      # metres along free space from the robot
    size: int               # frontier cells in the cluster
    centroid: tuple
    width: float = 0.0      # metres, longest extent of the cluster


def cluster_width(cells, res):
    """Longest extent (m) of a cluster of (row, col) cells.

    Range of the cells along x, y and both diagonals, plus one cell; within ~8 % of the true
    longest span, and cheap for big clusters.
    """
    r, c = cells[:, 0].astype(float), cells[:, 1].astype(float)
    spans = [np.ptp(c), np.ptp(r), np.ptp(c + r) / math.sqrt(2), np.ptp(c - r) / math.sqrt(2)]
    return (max(spans) + 1.0) * res


def clearance_map(grid):
    """Metres from each cell to the nearest wall (unknown is not a wall)."""
    return ndimage.distance_transform_edt(~grid.occupied) * grid.resolution


def geodesic_distance(passable, start):
    """Shortest path length in cells (8-connected) over `passable` from start=(col,row)."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    h, w = passable.shape
    dist = np.full((h, w), np.inf)
    if not (0 <= start[0] < w and 0 <= start[1] < h) or not passable[start[1], start[0]]:
        return dist
    ids = np.full((h, w), -1, dtype=np.int64)
    rows, cols = np.nonzero(passable)
    ids[rows, cols] = np.arange(len(rows))
    src, dst, cost = [], [], []
    for dy, dx, c in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
        # pair cell (y, x) with (y + dy, x + dx); dy >= 0 always
        a = ids[0:h - dy, max(0, -dx):w - max(0, dx)]
        b = ids[dy:h, max(0, dx):w - max(0, -dx)]
        m = (a >= 0) & (b >= 0)
        src.append(a[m])
        dst.append(b[m])
        cost.append(np.full(m.sum(), c))
    n = len(rows)
    g = coo_matrix((np.concatenate(cost), (np.concatenate(src), np.concatenate(dst))),
                   shape=(n, n)).tocsr()
    d = dijkstra(g, directed=False, indices=int(ids[start[1], start[0]]))
    dist[rows, cols] = d
    return dist


def find_frontiers(grid, robot_xy, robot_radius=0.25, goal_clearance=0.33, min_size=4,
                   blacklist=(), blacklist_radius=0.4, min_width=0.0):
    """All reachable frontiers, nearest (along free space) first.

    A frontier cell is a known-free cell next to unknown space. Clusters smaller than
    min_size cells are noise (single unknown pixels between scan rays); clusters narrower than
    min_width metres (longest extent) are openings the robot can't fit through, and clusters
    with no cell reachable along passable free space are skipped. For each cluster the
    goal is the reachable cell with at least goal_clearance to the walls that is closest to
    the cluster, so the footprint fits there (corridor half width in the example is 0.45 m).
    """
    res = grid.resolution
    free, unknown = grid.free, grid.unknown
    clear = clearance_map(grid)
    passable = free & (clear >= robot_radius)
    start = grid.cell(robot_xy)
    if not grid.contains(start) or not passable[start[1], start[0]]:
        # Robot drawn inside an inflated wall, on unknown cells under itself, or just off
        # the map edge (the lidar's blind sector behind it at startup leaves nothing mapped
        # there): start from the nearest passable cell.
        idx = np.argwhere(passable)
        if len(idx) == 0:
            return []
        d = np.hypot(idx[:, 1] - start[0], idx[:, 0] - start[1])
        if d.min() * res > 0.6:
            return []
        start = (int(idx[d.argmin(), 1]), int(idx[d.argmin(), 0]))
    dist = geodesic_distance(passable, start) * res

    near_unknown = ndimage.binary_dilation(unknown, structure=np.ones((3, 3), bool))
    frontier = free & near_unknown
    labels, n = ndimage.label(frontier, structure=np.ones((3, 3), bool))
    if n == 0:
        return []

    goal_ok = passable & (clear >= goal_clearance) & np.isfinite(dist)
    goal_cells = np.argwhere(goal_ok)                      # (row, col)
    if len(goal_cells) == 0:
        return []
    out = []
    for k in range(1, n + 1):
        cells = np.argwhere(labels == k)
        if len(cells) < min_size:
            continue
        width = cluster_width(cells, res)
        if width < min_width:
            continue
        centroid_rc = cells.mean(0)
        # Unreachable along passable free space = seen through a gap the robot doesn't fit
        # (a slit in a wall, over a low wall): its goal would land on the near side of the
        # wall and lead nowhere (live run 2026-11-14 09:09: three goals wasted that way).
        fd = dist[cells[:, 0], cells[:, 1]]
        if not np.isfinite(fd).any():
            continue
        # Goal: nearest goal-able cell to the cluster's closest cell along free space.
        anchor = cells[np.argmin(fd)]
        gd = np.hypot(goal_cells[:, 0] - anchor[0], goal_cells[:, 1] - anchor[1]) * res
        j = int(np.argmin(gd))
        if gd[j] > 0.7:                                    # nowhere to stand near it
            continue
        g = goal_cells[j]
        goal = grid.world((int(g[1]), int(g[0])))
        if any(math.hypot(goal[0] - b[0], goal[1] - b[1]) < blacklist_radius for b in blacklist):
            continue
        c = grid.world((centroid_rc[1], centroid_rc[0]))
        # Face the unknown if it is clearly ahead of the goal, else the direction of travel.
        # (Nav2's goal checker ignores yaw here; this only avoids a pointless last turn.)
        far = math.hypot(c[0] - goal[0], c[1] - goal[1]) > 0.3
        ref, tip = (goal, c) if far else (robot_xy, goal)
        out.append(Frontier(goal=goal, yaw=math.atan2(tip[1] - ref[1], tip[0] - ref[0]),
                            path_length=float(dist[g[0], g[1]]), size=len(cells),
                            centroid=c, width=width))
    out.sort(key=lambda f: f.path_length)
    return out


def choose_frontier(frontiers, current=None, robot_yaw=None, robot_xy=None,
                    switch_margin=0.5, heading_weight=0.3):
    """Pick the next goal: shortest path, with a little bias to keep going the same way.

    Keeps `current` (a previous Frontier) while a frontier still exists near it and no other
    one is switch_margin metres closer. This stops the robot flip-flopping between two
    frontiers at similar distance, which wastes time in a maze.
    """
    if not frontiers:
        return None

    def score(f):
        s = f.path_length
        if robot_yaw is not None and robot_xy is not None:
            a = math.atan2(f.goal[1] - robot_xy[1], f.goal[0] - robot_xy[0])
            turn = abs(math.atan2(math.sin(a - robot_yaw), math.cos(a - robot_yaw)))
            s += heading_weight * turn
        return s

    best = min(frontiers, key=score)
    if current is not None:
        same = [f for f in frontiers
                if math.hypot(f.goal[0] - current.goal[0], f.goal[1] - current.goal[1]) < 0.4]
        if same and score(same[0]) - switch_margin <= score(best):
            return same[0]
    return best


def heading_towards(grid, robot_xy, goal, robot_radius=0.25, lookahead=0.3):
    """Heading (rad, map frame) to drive off from robot_xy so as to follow free space to goal.

    Not the straight line to the goal, which can cross a wall: the direction to the cell about
    `lookahead` m away that is closest to the goal along free space. Falls back to the straight
    line when no such cell is found (goal unreachable, robot off the map).
    """
    straight = math.atan2(goal[1] - robot_xy[1], goal[0] - robot_xy[0])
    res = grid.resolution
    passable = grid.free & (clearance_map(grid) >= robot_radius)
    gc = grid.cell(goal)
    if not grid.contains(gc):
        return straight
    d = geodesic_distance(passable, gc)
    rows, cols = np.nonzero(np.isfinite(d))
    if len(rows) == 0:
        return straight
    xs = grid.origin[0] + (cols + 0.5) * res
    ys = grid.origin[1] + (rows + 0.5) * res
    r = np.hypot(xs - robot_xy[0], ys - robot_xy[1])
    ring = np.abs(r - lookahead) <= 1.5 * res
    if not ring.any():
        return straight
    i = np.flatnonzero(ring)[np.argmin(d[rows[ring], cols[ring]])]
    return math.atan2(ys[i] - robot_xy[1], xs[i] - robot_xy[0])


# --------------------------------------------------------------------------- exit

@dataclass
class ExitDetector:
    """Out of the maze = open floor right next to the robot, far from where it started.

    Open floor: a disk of radius >= open_radius made only of cells SLAM has seen as free
    (walls and unknown both count as blocked, so open space has to be seen, not guessed).
    Inside 1 m maze corridors no such disk exists (max ~0.45 m), outside the maze it does.
    The disk has to be within near_radius of the robot (frontier goals at the maze mouth are
    never in the middle of open floor themselves) and >= min_displacement from the start
    (an open entrance next to the start cell is not the exit). The robot must also have been
    in narrow space once (enclosed_radius) and it must hold for `dwell` seconds.
    """
    enclosed_radius: float = 0.7
    open_radius: float = 1.1
    near_radius: float = 0.8
    min_displacement: float = 1.5
    dwell: float = 2.0
    start: tuple = None
    seen_enclosed: bool = False
    since: float = None
    found: bool = False
    clearance: float = 0.0          # at the robot
    open_nearby: float = 0.0        # best clearance within near_radius (away from start)
    last_time: float = field(default=None, repr=False)

    def update(self, grid, xy, now):
        if self.start is None:
            self.start = tuple(xy)
        if self.last_time is not None and not 0.0 <= now - self.last_time <= 2.0:
            self.since = None                   # paused / clock jumped: start over
        self.last_time = now
        c = grid.cell(xy)
        self.clearance = self.open_nearby = 0.0
        if grid.contains(c):
            res = grid.resolution
            d = ndimage.distance_transform_edt(np.pad(grid.free, 1, constant_values=False))
            d = d[1:-1, 1:-1] * res
            self.clearance = float(d[c[1], c[0]])
            r = int(math.ceil(self.near_radius / res))
            y0, y1 = max(0, c[1] - r), min(d.shape[0], c[1] + r + 1)
            x0, x1 = max(0, c[0] - r), min(d.shape[1], c[0] + r + 1)
            yy, xx = np.mgrid[y0:y1, x0:x1]
            wx = grid.origin[0] + (xx + 0.5) * res
            wy = grid.origin[1] + (yy + 0.5) * res
            ok = ((np.hypot(wx - xy[0], wy - xy[1]) <= self.near_radius)
                  & (np.hypot(wx - self.start[0], wy - self.start[1]) >= self.min_displacement))
            if ok.any():
                self.open_nearby = float(d[y0:y1, x0:x1][ok].max())
        self.seen_enclosed |= self.clearance <= self.enclosed_radius
        moved = math.hypot(xy[0] - self.start[0], xy[1] - self.start[1]) >= self.min_displacement
        if self.seen_enclosed and moved and self.open_nearby >= self.open_radius:
            self.since = now if self.since is None else self.since
            self.found |= now - self.since >= self.dwell
        else:
            self.since = None
        return self.found
