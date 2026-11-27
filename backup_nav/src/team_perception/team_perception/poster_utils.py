"""
Tell posters of people from real people, with no ROS imports so it can be tested anywhere.

Two independent tests, combined by combine() and voted over time by PosterTracker:

1. Edge-ring test (depth image only).
   A poster is part of a surface (paper on a board or wall): just outside the person's
   box that same surface continues. A real person stands IN FRONT of their background:
   just outside the box the depth jumps backwards.
   - Fit the surface of the torso region (central part of the box) as a plane. For a
     plane, inverse depth is exactly linear in pixel coordinates, 1/z = a*u + b*v + c,
     so tilted posters are handled without camera intrinsics.
   - Extrapolate that plane into a thin ring just outside the box (left, right, top;
     not bottom, where a real person's feet touch the floor at the same depth).
   - Poster: ring pixels lie on the plane. Real: ring pixels are well behind it.

2. Static-map test (map position + nav_msgs/OccupancyGrid).
   A poster hangs on something that is in the static map (wall, board), so its map
   position is within a few cm of an occupied cell. A real person stands in free space.
   Only valid with a map built WITHOUT people in it and with posters' walls/boards in it.

Each test returns 'poster', 'real' or 'unknown' plus the numbers behind the decision,
so thresholds can be tuned from logs.
"""
from collections import deque
from dataclasses import dataclass
import itertools
import math

import numpy as np

POSTER, REAL, UNKNOWN = 'poster', 'real', 'unknown'


# --- depth helpers -----------------------------------------------------------------------
def to_metres(depth):
    """16UC1 (mm) or 32FC1 (m) depth -> float32 metres with invalid pixels as NaN."""
    d = depth.astype(np.float32)
    if depth.dtype == np.uint16:
        d /= 1000.0
    d[~np.isfinite(d) | (d < 0.05)] = np.nan
    return d


def scale_box(box, image_size, depth_shape):
    """Map a box from colour-image pixels to depth-image pixels (aligned, same FOV)."""
    if not image_size:
        return [float(v) for v in box]
    sx = depth_shape[1] / float(image_size[0])
    sy = depth_shape[0] / float(image_size[1])
    x1, y1, x2, y2 = box
    return [x1 * sx, y1 * sy, x2 * sx, y2 * sy]


# --- 1. edge-ring test -------------------------------------------------------------------
@dataclass
class RingParams:
    torso_x: tuple = (0.30, 0.70)   # torso region as fractions of the box (x range)
    torso_y: tuple = (0.15, 0.55)   # (y range, from the top of the box)
    gap_frac: float = 0.03          # skip this fraction of box width just outside the box
    ring_frac: float = 0.10         # ring thickness as a fraction of box width
    min_ring_px: int = 4            # ...but at least this many pixels
    tol_m: float = 0.05             # "on the plane" tolerance at 0 m ...
    tol_quad: float = 0.01          # ... plus this * z^2 (stereo noise grows with z^2)
    step_m: float = 0.25            # "behind the plane" = more than this behind it
    min_valid_frac: float = 0.3     # a region needs this share of valid depth pixels
    min_valid_px: int = 25          # ... and at least this many
    poster_frac: float = 0.6        # side counts as continuous if this share is on-plane
    real_frac: float = 0.5          # side counts as a jump if this share is behind
    min_sides: int = 2              # poster needs at least this many usable sides
    max_range_m: float = 4.0        # beyond this depth is too noisy: unknown


def _fit_inverse_depth_plane(us, vs, zs):
    """Least-squares fit 1/z = a*u + b*v + c, refit once without outliers."""
    w = 1.0 / zs
    A = np.column_stack([us, vs, np.ones_like(us)])
    coef = np.linalg.lstsq(A, w, rcond=None)[0]
    res = np.abs(1.0 / np.clip(A @ coef, 1e-6, None) - zs)
    keep = res <= max(0.02, 3.0 * np.median(res))
    if keep.sum() >= 10 and not keep.all():
        coef = np.linalg.lstsq(A[keep], w[keep], rcond=None)[0]
    return coef


def _region(depth, x1, y1, x2, y2):
    """Return valid pixels of depth[y1:y2, x1:x2] as (us, vs, zs, n_total), clipped."""
    h, w = depth.shape
    x1, x2 = max(0, int(round(x1))), min(w, int(round(x2)))
    y1, y2 = max(0, int(round(y1))), min(h, int(round(y2)))
    if x2 <= x1 or y2 <= y1:
        return None
    patch = depth[y1:y2, x1:x2]
    vs, us = np.nonzero(np.isfinite(patch))
    return us + x1, vs + y1, patch[vs, us], patch.size


def edge_ring_test(depth_m, box, p=RingParams()):
    """
    Classify one person box from an aligned depth image in metres (NaN = no data).

    box is [x1, y1, x2, y2] in depth-image pixels. Returns (label, metrics).
    """
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    m = {'sides': {}}
    if bw < 8 or bh < 8:
        return UNKNOWN, dict(m, reason='box too small')

    tx = (x1 + p.torso_x[0] * bw, x1 + p.torso_x[1] * bw)
    ty = (y1 + p.torso_y[0] * bh, y1 + p.torso_y[1] * bh)
    torso = _region(depth_m, tx[0], ty[0], tx[1], ty[1])
    if torso is None:
        return UNKNOWN, dict(m, reason='torso outside image')
    us, vs, zs, n = torso
    if len(zs) < p.min_valid_px or len(zs) < p.min_valid_frac * n:
        return UNKNOWN, dict(m, reason='too little depth on torso',
                             torso_valid=round(len(zs) / max(n, 1), 2))
    z_torso = float(np.median(zs))
    m['torso_z'] = round(z_torso, 3)
    if z_torso > p.max_range_m:
        return UNKNOWN, dict(m, reason=f'beyond max_range_m ({z_torso:.1f} m)')
    coef = _fit_inverse_depth_plane(us.astype(np.float64), vs.astype(np.float64),
                                    zs.astype(np.float64))
    tol = p.tol_m + p.tol_quad * z_torso ** 2
    m['tol_m'] = round(tol, 3)

    gap = p.gap_frac * bw
    t = max(p.min_ring_px, p.ring_frac * bw)
    sides = {'left': (x1 - gap - t, y1, x1 - gap, y1 + 0.75 * bh),
             'right': (x2 + gap, y1, x2 + gap + t, y1 + 0.75 * bh),
             'top': (x1, y1 - gap - t, x2, y1 - gap)}
    n_poster = n_real = n_used = 0
    for name, (a, b, c, d) in sides.items():
        reg = _region(depth_m, a, b, c, d)
        if reg is None or (c - a) < 1 or (d - b) < 1:
            m['sides'][name] = 'outside image'
            continue
        su, sv, sz, sn = reg
        # Sides cut short by the image border are unreliable: require most of the ring.
        expected = max(1.0, (c - a) * (d - b))
        if len(sz) < p.min_valid_px or len(sz) < p.min_valid_frac * expected:
            m['sides'][name] = f'no depth ({len(sz)}/{int(expected)} px)'
            continue
        pred = 1.0 / np.clip(coef[0] * su + coef[1] * sv + coef[2], 1e-6, None)
        diff = sz - pred                      # > 0: ring is behind the torso plane
        on = float(np.mean(np.abs(diff) < tol))
        behind = float(np.mean(diff > p.step_m))
        n_used += 1
        n_poster += on >= p.poster_frac
        n_real += behind >= p.real_frac
        m['sides'][name] = {'on_plane': round(on, 2), 'behind': round(behind, 2),
                            'median_diff_m': round(float(np.median(diff)), 3)}
    m.update(sides_used=n_used, sides_on_plane=n_poster, sides_behind=n_real)

    if n_real >= 1:
        return REAL, dict(m, reason='depth jumps behind the person at the edge')
    if n_used >= p.min_sides and n_poster == n_used:
        return POSTER, dict(m, reason='surface continues past the edges')
    if n_used < p.min_sides:
        return UNKNOWN, dict(m, reason=f'only {n_used} usable side(s)')
    return UNKNOWN, dict(m, reason='edges neither clearly continuous nor clearly behind')


def torso_pixel_and_depth(depth_m, box, p=RingParams()):
    """Centre of the torso region and its median depth (m), or None if no depth."""
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    u = x1 + 0.5 * (p.torso_x[0] + p.torso_x[1]) * bw
    v = y1 + 0.5 * (p.torso_y[0] + p.torso_y[1]) * bh
    reg = _region(depth_m, x1 + p.torso_x[0] * bw, y1 + p.torso_y[0] * bh,
                  x1 + p.torso_x[1] * bw, y1 + p.torso_y[1] * bh)
    if reg is None or len(reg[2]) < p.min_valid_px:
        return None
    return u, v, float(np.median(reg[2]))


# --- 2. static-map test ------------------------------------------------------------------
@dataclass
class MapParams:
    on_radius_m: float = 0.30     # occupied cell this close -> on a static surface
    free_radius_m: float = 0.60   # no occupied cell this close -> standing in free space
    occupied: int = 65            # OccupancyGrid value counted as occupied (0..100)


class StaticMap:
    """nav_msgs/OccupancyGrid data as a numpy array plus its geometry."""

    def __init__(self, data, width, height, resolution, origin_xy, origin_yaw=0.0):
        self.grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.cos, self.sin = math.cos(origin_yaw), math.sin(origin_yaw)

    def to_cell(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        # rotate into the grid frame (origin yaw is almost always 0)
        gx = self.cos * dx + self.sin * dy
        gy = -self.sin * dx + self.cos * dy
        return int(math.floor(gx / self.res)), int(math.floor(gy / self.res))

    def nearest_occupied(self, x, y, radius, occupied=65):
        """Distance (m) to the nearest occupied cell within radius, else None."""
        cx, cy = self.to_cell(x, y)
        r = int(math.ceil(radius / self.res))
        h, w = self.grid.shape
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return None
        ys, xs = np.nonzero(self.grid[y0:y1, x0:x1] >= occupied)
        if xs.size == 0:
            return None
        d = np.hypot(xs + x0 - cx, ys + y0 - cy) * self.res
        d = d[d <= radius]
        return float(d.min()) if d.size else None

    def cell_value(self, x, y):
        cx, cy = self.to_cell(x, y)
        h, w = self.grid.shape
        return int(self.grid[cy, cx]) if 0 <= cx < w and 0 <= cy < h else None


def static_map_test(static_map, x, y, p=MapParams()):
    """Classify a map position. Returns (label, metrics)."""
    if static_map is None:
        return UNKNOWN, {'reason': 'no map'}
    cell = static_map.cell_value(x, y)
    if cell is None:
        return UNKNOWN, {'reason': 'outside the map'}
    d = static_map.nearest_occupied(x, y, p.free_radius_m, p.occupied)
    m = {'nearest_occupied_m': None if d is None else round(d, 3), 'cell': cell}
    if d is not None and d <= p.on_radius_m:
        return POSTER, dict(m, reason='on/next to a static surface in the map')
    if d is None:
        if cell < 0:
            return UNKNOWN, dict(m, reason='unexplored area of the map')
        return REAL, dict(m, reason=f'free space (nothing within {p.free_radius_m} m)')
    return UNKNOWN, dict(m, reason='near, but not on, a static surface')


# --- combining and voting ----------------------------------------------------------------
MAP_MODES = ('require', 'veto', 'off')


def combine(ring, map_label, map_mode='veto'):
    """
    Combine the two tests into one label for this frame.

    map_mode:
      'require' - poster only if the ring test says poster AND the map confirms it.
      'veto'    - ring test decides; the map can only overrule it by saying 'free space'.
                  Use when some posters may stand on boards that are not in the map.
      'off'     - ring test only.
    A ring 'real' always wins: a jump behind the person is direct evidence.
    """
    if ring == REAL:
        return REAL
    if map_mode == 'off':
        return ring
    if ring == POSTER:
        if map_label == POSTER:
            return POSTER
        if map_mode == 'require':
            return UNKNOWN
        return UNKNOWN if map_label == REAL else POSTER
    # ring unknown: the map alone may only say "real" (free space), never "poster"
    return REAL if map_label == REAL else UNKNOWN


@dataclass
class Track:
    id: int
    x: float
    y: float
    z: float
    labels: deque
    last_seen: float
    reported_at: float = -1e9
    count: int = 0
    room: tuple = None        # survivor's room point (pulled back from the wall), track frame
    n_room: int = 0
    cell: tuple = None        # cell last reported for this poster ...
    cell_frame: str = None    # ... and the frame it was placed in (odom before SLAM is up)
    need: int = None          # poster votes needed for this track (None = tracker default)

    def add_room(self, xyz):
        """Running average of the room point, like the track position."""
        self.room = _avg(self.room, xyz, min(self.n_room, 9))
        self.n_room += 1


def _avg(old, new, k):
    if old is None or k == 0:
        return tuple(float(v) for v in new)
    return tuple((o * k + n) / (k + 1) for o, n in zip(old, new))


@dataclass
class MazeCells:
    """
    The maze's square cells in the map frame: SAME convention as maze_nav's
    scene_understanding.CellGrid (and the same parameter names in scene_trigger), so a cell
    index means the same cell to both packages. Cell (0, 0) is centred on `origin` (the map
    origin = where the robot stood at bringup, on the start cell); +i points along `yaw`.
    """
    size: float = 1.0
    origin: tuple = (0.0, 0.0)
    yaw: float = 0.0

    def cell(self, xy):
        dx, dy = xy[0] - self.origin[0], xy[1] - self.origin[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        u, v = (c * dx + s * dy) / self.size, (-s * dx + c * dy) / self.size
        return int(math.floor(u + 0.5)), int(math.floor(v + 0.5))

    def centre(self, cell):
        u, v = cell[0] * self.size, cell[1] * self.size
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.origin[0] + c * u - s * v, self.origin[1] + s * u + c * v

    def local(self, xy):
        dx, dy = xy[0] - self.origin[0], xy[1] - self.origin[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy) / self.size, (-s * dx + c * dy) / self.size

    def room_cell(self, poster_xy, toward_xy):
        """The cell a poster on a wall belongs to: the poster hangs on the cell edge nearest
        it, and its room is the cell on that edge's side `toward_xy` (a point on the camera's
        side, e.g. the pulled-back room point) lies on; along the edge the poster's own
        position counts. A ray pull-back alone put posters on the near half of a side wall
        (where QR posters hang, booklet FAQ 4), seen from the cell before the room, into that
        cell (2026-11-23)."""
        pu, pv = self.local(poster_xy)
        tu, tv = self.local(toward_xy)
        eu, ev = math.floor(pu) + 0.5, math.floor(pv) + 0.5      # nearest edge lines
        eu = eu if abs(pu - eu) <= abs(pu - (eu - 1)) else eu - 1
        ev = ev if abs(pv - ev) <= abs(pv - (ev - 1)) else ev - 1
        if abs(pu - eu) <= abs(pv - ev):                          # on a constant-u edge
            i = int(math.floor(eu)) + (1 if tu > eu else 0)
            return i, int(math.floor(pv + 0.5))
        j = int(math.floor(ev)) + (1 if tv > ev else 0)
        return int(math.floor(pu + 0.5)), j


def pull_back(xyz, dist):
    """
    Point `dist` m from xyz back towards the camera (xyz in a camera frame, camera at 0).

    A poster hangs on a wall, and walls sit on cell edges, so the poster point itself can
    fall in either cell. The survivor's room is the one the camera sees the poster from:
    stepping back towards the camera by about a quarter cell lands well inside it.
    """
    r = math.sqrt(sum(v * v for v in xyz))
    if r <= dist or r == 0.0:
        return (0.0, 0.0, 0.0) if r == 0.0 else tuple(v * 0.5 for v in xyz)
    k = 1.0 - dist / r
    return tuple(v * k for v in xyz)


class PosterTracker:
    """
    Group detections of the same object by position and vote over recent frames.

    A track is confirmed as a poster once it has at least `need` poster votes among its
    last `window` labels. Confirmed posters are reported once, then again only after
    repeat_after_s. Positions should be in the map frame so they stay put as the robot
    moves.
    """

    def __init__(self, merge_radius_m=0.5, window=8, need=5, forget_s=30.0,
                 repeat_after_s=60.0):
        self.merge = merge_radius_m
        self.window = window
        self.need = need
        self.forget = forget_s
        self.repeat = repeat_after_s
        self.tracks = []
        self._ids = itertools.count(1)

    def update(self, xyz, label, now, need=None):
        """Add one classification. Returns (track, report_now). need: votes needed for THIS
        track from now on (e.g. fewer in a room with a QR code); None = the default."""
        self.tracks = [t for t in self.tracks if now - t.last_seen < self.forget]
        x, y, z = xyz
        best, best_d = None, self.merge
        for t in self.tracks:
            d = math.dist((t.x, t.y, t.z), (x, y, z))
            if d <= best_d:
                best, best_d = t, d
        if best is None:
            best = Track(next(self._ids), x, y, z, deque(maxlen=self.window), now)
            self.tracks.append(best)
        else:   # running average keeps the position steady
            k = min(best.count, 9)
            best.x = (best.x * k + x) / (k + 1)
            best.y = (best.y * k + y) / (k + 1)
            best.z = (best.z * k + z) / (k + 1)
        best.labels.append(label)
        best.last_seen = now
        best.count += 1
        if need is not None:
            best.need = need
        confirmed = self.is_poster(best)
        report = confirmed and now - best.reported_at >= self.repeat
        if report:
            best.reported_at = now
        return best, report

    def is_poster(self, t):
        need = t.need if t.need is not None else self.need
        return sum(1 for lab in t.labels if lab == POSTER) >= need

    def votes(self, t):
        return f'{sum(1 for lab in t.labels if lab == POSTER)}/{len(t.labels)}'


class Announced:
    """Positions already announced ("Human found"), all in ONE fixed frame (odom)."""

    def __init__(self, merge_radius_m=0.5):
        self.merge = merge_radius_m
        self.positions = []

    def add(self, xyz):
        """True (and remembered) if nothing was announced within merge radius of xyz."""
        if any(math.dist(xyz, p) <= self.merge for p in self.positions):
            return False
        self.positions.append(tuple(xyz))
        return True


@dataclass
class QrRoom:
    """A room known to hold at least one survivor because a QR code was read in it."""
    text: str
    room: tuple                 # room point (pulled back from the code's wall), odom
    first_seen: float
    n: int = 1
    has_poster: bool = False    # a poster was confirmed in this room
    look_requested: bool = False
    visited: bool = False       # the robot has been within leave_radius of the room point
    backup: bool = False        # survivor reported from the QR alone ...
    backup_claimed: bool = False  # ... and later matched by a confirmed poster
    cell: tuple = None          # cell last reported for the QR-only survivor ...
    cell_frame: str = None      # ... and the frame it was placed in


class QrRooms:
    """
    Rooms with QR codes, all in ONE fixed frame (odom). QR codes only appear in rooms with
    survivors (but not every survivor room has one), so a read code means "at least one
    survivor here". Used to confirm posters with fewer votes near a code, to ask the explorer
    to look around a code's room when no poster has been confirmed there yet, and to report
    a survivor from the code alone once the robot has left such a room.

    Rooms need not be single cells, so "in the room" is a distance: room points (posters and
    codes are both pulled back from their walls towards the camera) within `radius` m.
    """

    def __init__(self, radius=1.0):
        self.radius = radius
        self.rooms = {}             # text -> QrRoom

    def add(self, text, room, now):
        """Record a read code (repeat reads refine its room point). Returns the QrRoom."""
        r = self.rooms.get(text)
        if r is None:
            r = self.rooms[text] = QrRoom(text, tuple(room), now)
        else:
            r.room = _avg(r.room, room, min(r.n, 9))
            r.n += 1
        return r

    def near(self, xyz):
        """QR rooms within radius of xyz, nearest first."""
        rs = [(math.dist(r.room[:2], xyz[:2]), r) for r in self.rooms.values()]
        return [r for d, r in sorted(rs, key=lambda dr: dr[0]) if d <= self.radius]

    def poster_confirmed(self, xyz):
        """
        A poster was confirmed at room point xyz: mark its QR room(s). Returns the room whose
        QR-only survivor report this poster turns out to be (no new "Human found"), or None.
        """
        claim = None
        for r in self.near(xyz):
            r.has_poster = True
            if claim is None and r.backup and not r.backup_claimed:
                r.backup_claimed = True
                claim = r
        return claim

    def to_look(self, now, delay):
        """Rooms read `delay` s ago with no poster confirmed and no look-around asked yet."""
        return [r for r in self.rooms.values()
                if not r.has_poster and not r.look_requested and now - r.first_seen >= delay]

    def to_report(self, robot_xyz, now, leave_radius, timeout):
        """
        Rooms to report a survivor in from the code alone: no poster confirmed there, and
        the robot has been in the room (within leave_radius of its room point) and left
        again, or `timeout` s have passed since the code was read. A code read from afar
        doesn't count as "left": the robot may still go in and find the poster.
        """
        out = []
        for r in self.rooms.values():
            if r.has_poster or r.backup:
                continue
            d = None if robot_xyz is None else math.dist(robot_xyz[:2], r.room[:2])
            if d is not None and d <= leave_radius:
                r.visited = True
            left = r.visited and d is not None and d > leave_radius
            if left or now - r.first_seen >= timeout:
                out.append(r)
        return out
