"""Stairs from the lidar: an odom height map, step profiles, and fusion with the camera.

Pure numpy, no ROS (stair_detector.py feeds it /livox/points). Why the lidar helps the depth
camera (stairs.py):
  - range and direction: the Mid-360 sees 360 deg out to many metres; the camera 90 deg wide
    and its detector out to 2.5 m;
  - a measured floor: Fast-LIO's odom is gravity-aligned, so the height map knows the floor
    height. The camera alone has to assume the body is 0.35 m up (+-5 cm decided whether a
    step was hidden, 2026-11-18);
  - memory: cells stay in the odom map, so the bottom riser, seen from 1-2 m away, is still
    known when the robot is close and neither sensor sees it any more.
Limits: the lidar is pitched 15 deg down with its FOV from -7 deg, so ahead it sees the floor
only from ~1.2 m and a 12 cm tread from ~1.0 m (live robot 2026-11-18: floor from 1.25 m). It
never sees the bottom step close up; the map remembers it if it was seen from further away.

HeightMap: square grid in odom (`size` m, `cell` m) of the TOP surface per cell: each scan's
highest point in the cell, smoothed over scans (newest weighted most, `alpha`). Not the mean:
the lidar's ~2 cm range noise runs along nearly horizontal rays, so riser hits smear into the
next tread's cells and a mean drags treads down (rises came out 7-9 cm for 12 cm steps). The
robot's own body (a box around base_link) and points far above or below it are dropped.

Profiles: cells in a band along a direction, binned along it (median height per bin) and split
into flat levels. A step = an upward jump between consecutive levels of rise_min..rise_max,
a tread apart (tread_min..tread_max). Stairs = >= min_risers such steps with similar rises; the
bottom one is `anchored` when the level before it is the measured floor.

detect(): stair directions from the height map's slopes (uphill gradients of about a rise over
a cell or two), then profiles across lateral bands in each direction.
check(): a camera estimate against the map: confirm (with the lidar's own first riser, which
may be a step nearer than the camera's when the camera can't see the bottom one), veto (the
map is well covered there and flat: no stairs), or unknown (not enough lidar coverage).
fuse(): one estimate from the camera tracker, the lidar tracker and that check.
"""
import collections
import math
import statistics
from dataclasses import dataclass

import numpy as np

from maze_nav.stairs import StairEstimate, wrap


@dataclass
class LidarParams:
    cell: float = 0.04                  # m height-map cell
    size: float = 24.0                  # m, square map centred on the odom origin
    alpha: float = 0.3                  # weight of a new scan's cell maximum
    min_count: float = 2.0              # scans that hit a cell before it counts
    body_back: float = 0.45             # m, robot's own points (box around base_link) dropped
    body_front: float = 0.35
    body_half_width: float = 0.30
    # m below base_link kept (floor at ~0.29-0.35). 0.90, was 0.60: the bottom of descending
    # stairs (two or three steps down to the lab floor, presentation maze 2026-11-21) lies
    # 0.25-0.45 m below the robot's floor, and 0.60 cut it off.
    keep_below: float = 0.90
    keep_above: float = 0.30            # m above base_link kept (stairs and rails are lower)
    standing_height: float = 0.35       # base_link above the floor, for the floor search
    floor_window: float = 0.15          # m around the expected floor height searched
    floor_max_off: float = 0.07         # m: the floor layer found must be this close to it
    floor_min_cells: int = 40
    floor_ring: tuple = (0.5, 3.0)      # m from the robot, cells used for the floor
    profile_bin: float = 0.03           # m along the stairs
    band_half_width: float = 0.20       # m either side of the profile line
    min_bin_cells: int = 2
    level_tol: float = 0.035            # m, a bin within this of a level's height belongs to it
    max_gap_bins: int = 3               # empty bins allowed inside a level
    riser_gap_max: float = 0.15         # m between two treads' bins where the riser is
    rise_min: float = 0.06              # finding the steps; the height map reads a 13 cm step as
    #                                     7-8 cm when sparse (> 2 m), so 0.08 here lost the riser and
    #                                     put the centre 0.7 m off. The > 8 cm rule (user 2026-11-21)
    #                                     is stair_detector's check_min_rise instead
    rise_max: float = 0.22
    rise_tol: float = 0.05
    tread_min: float = 0.10
    tread_max: float = 0.50
    min_risers: int = 2
    floor_tol: float = 0.05             # m, level within this of the floor = floor (anchored)
    search_min: float = 0.6             # m from the robot, detect() looks for stairs ...
    search_max: float = 4.0             # ... up to here
    slope_min: float = 0.04             # m height change over 2 cells to count as a step edge
    min_edge_cells: int = 8
    band_step: float = 0.20             # m between lateral bands in detect()
    refine_deg: float = 15.0            # detect(): heading refined within +- this
    refine_step_deg: float = 1.5
    min_width: float = 0.30             # m of riser edge across for a lidar detection
    check_before: float = 0.50          # m before the camera's first riser profiled by check()
    check_after: float = 0.80
    veto_coverage: float = 0.7          # fraction of check bins with data to veto
    veto_flat: float = 0.04             # m: max - min height of a flat (stair-less) profile
    nearer_max: float = 0.50            # m: lidar's first riser may be this much nearer
    # Descending stairs / drops (detect_down; presentation maze 2026-11-21: the exit is two
    # steps DOWN from a raised cell). From the top the lidar (lowest forward ray ~22 deg down)
    # can't see the treads: each nosing shadows the tread below it. What it does see is the
    # robot's own floor ending at a line and, past a strip it never sees, surfaces well below
    # it (lower treads, the floor at the bottom).
    drop_min: float = 0.17              # m below the floor; > a raised plate's edge (~0.10)
    drop_max: float = 0.80
    drop_gap_max: float = 1.4           # m of unseen strip between the edge and a lower level
    drop_floor_min: float = 0.20        # m of floor-level profile right before the edge
    drop_wall_max: float = 0.10         # m over the floor between edge and drop = a wall there
    drop_search: float = 3.0            # m from the robot to the edge, at most
    drop_near: float = 0.30             # m: edges nearer than this are under the robot's nose
    drop_lat: float = 1.2               # m either side of the robot: lateral bands
    drop_min_bands: int = 3             # bands with an agreeing edge (0.2 m apart: >= 0.6 m)
    drop_edge_tol: float = 0.12         # m: band edges this close to the common one agree
    drop_min_cells: int = 10            # cells below the floor before profiling at all
    drop_rise_guess: float = 0.12       # m per riser when the treads themselves aren't seen
    # Floor under the robot's feet (floor_under): cells it walked over, seen while it came
    floor_under_radius: float = 0.50
    floor_under_window: float = 0.12    # m around base_z - standing_height
    floor_under_min_cells: int = 10


class HeightMap:
    """Mean height per cell of an odom grid, from lidar points (see module doc)."""

    def __init__(self, p=None):
        self.p = p or LidarParams()
        self.n = int(round(self.p.size / self.p.cell))
        self.half = self.n * self.p.cell / 2
        self.top = np.zeros((self.n, self.n), np.float64)     # smoothed per-scan maximum
        self.cnt = np.zeros((self.n, self.n), np.float64)     # scans that hit the cell

    def index(self, xy):
        ij = np.floor((np.asarray(xy, float) + self.half) / self.p.cell).astype(np.int64)
        return ij[..., 0], ij[..., 1]

    def add(self, points_odom, base_xyz, base_yaw):
        """Add one scan (N, 3) in odom; base_xyz / base_yaw: base_link pose in odom."""
        p = self.p
        pts = np.asarray(points_odom, float)
        if len(pts) == 0:
            return 0
        dz = pts[:, 2] - base_xyz[2]
        dx, dy = pts[:, 0] - base_xyz[0], pts[:, 1] - base_xyz[1]
        c, s = math.cos(base_yaw), math.sin(base_yaw)
        fx, fy = c * dx + s * dy, -s * dx + c * dy
        own = (fx > -p.body_back) & (fx < p.body_front) & (np.abs(fy) < p.body_half_width)
        keep = ~own & (dz > -p.keep_below) & (dz < p.keep_above)
        i, j = self.index(pts[keep, :2])
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        i, j, z = i[ok], j[ok], pts[keep][ok, 2]
        if len(z) == 0:
            return 0
        flat = i * self.n + j
        # This scan's top per cell: sort by cell, max over each run (np.maximum.reduceat).
        # Same result as np.unique + np.maximum.at, several times faster (ufunc.at is slow).
        order = np.argsort(flat, kind='stable')
        flat, z = flat[order], z[order]
        starts = np.flatnonzero(np.r_[True, flat[1:] != flat[:-1]])
        cells = flat[starts]
        mx = np.maximum.reduceat(z, starts)
        ci, cj = cells // self.n, cells % self.n
        new = self.cnt[ci, cj] == 0
        old = self.top[ci, cj]
        self.top[ci, cj] = np.where(new, mx, (1 - p.alpha) * old + p.alpha * mx)
        self.cnt[ci, cj] += 1.0
        return int(len(z))

    def cells(self, centre_xy, radius):
        """(xy (M, 2), height (M,)) of valid cells within radius of centre_xy."""
        p = self.p
        r = int(math.ceil(radius / p.cell))
        ci, cj = self.index(centre_xy)
        i0, i1 = max(0, ci - r), min(self.n, ci + r + 1)
        j0, j1 = max(0, cj - r), min(self.n, cj + r + 1)
        cnt = self.cnt[i0:i1, j0:j1]
        ok = cnt >= p.min_count
        ii, jj = np.nonzero(ok)
        h = self.top[i0:i1, j0:j1][ok]
        xy = (np.stack([ii + i0, jj + j0], 1) + 0.5) * p.cell - self.half
        near = np.hypot(xy[:, 0] - centre_xy[0], xy[:, 1] - centre_xy[1]) <= radius
        return xy[near], h[near]

    def grid(self, centre_xy, radius):
        """(heights (k, k) with nan, x0, y0) of the square around centre_xy (for slopes)."""
        p = self.p
        r = int(math.ceil(radius / p.cell))
        ci, cj = self.index(centre_xy)
        i0, j0 = max(0, ci - r), max(0, cj - r)
        i1, j1 = min(self.n, ci + r + 1), min(self.n, cj + r + 1)
        cnt = self.cnt[i0:i1, j0:j1]
        g = np.full(cnt.shape, np.nan)
        ok = cnt >= p.min_count
        g[ok] = self.top[i0:i1, j0:j1][ok]
        return g, i0 * p.cell - self.half, j0 * p.cell - self.half

    def floor(self, base_xyz):
        """Odom z of the floor around the robot (lowest dense 1 cm layer of cell heights near
        base_z - standing_height, and within floor_max_off of it), or None.

        Within floor_max_off: the lidar sees the floor only from ~1.4 m ahead of base_link,
        so near stairs the first tread can be the lowest dense layer (a 12 cm tread was taken
        for the floor, which made the stairs look anchored). None is safer than wrong."""
        p = self.p
        xy, h = self.cells(base_xyz[:2], p.floor_ring[1])
        r = np.hypot(xy[:, 0] - base_xyz[0], xy[:, 1] - base_xyz[1])
        guess = base_xyz[2] - p.standing_height
        z = h[(r >= p.floor_ring[0]) & (np.abs(h - guess) < p.floor_window)]
        if len(z) < p.floor_min_cells:
            return None
        hist, edges = np.histogram(z, bins=np.arange(guess - p.floor_window,
                                                     guess + p.floor_window + 0.011, 0.01))
        dense = np.nonzero(hist >= max(0.2 * hist.max(), 3))[0]
        if len(dense) == 0:
            return None
        k = int(dense[0])
        near = z[(z >= edges[k] - 0.01) & (z <= edges[k + 1] + 0.01)]
        f = float(np.median(near))
        return f if abs(f - guess) <= p.floor_max_off else None


    def floor_under(self, base_xyz, also=()):
        """Odom z of the floor under the robot's feet (median of the cells within
        floor_under_radius, seen from further back while it walked there, within
        floor_under_window of base_z - standing_height), or None. also: more HeightMaps
        whose cells count too (stair_detector: the depth camera's floor map).

        Not floor(): that takes the LOWEST dense layer in a 0.5-3 m ring, which on a raised
        plate is the lab floor seen past the maze walls. This is the surface the robot stands
        on: the start plate, the bare floor of a room or a curtain cell 5-10 cm lower, the
        block tops on rough terrain. Lying down it is out of the window: None (keep the last).

        The lidar alone rarely sees it: its lowest forward ray is ~22 deg down, so the nearest
        floor it hits is ~1.4 m ahead of base_link on level floor and ~1.75 m ahead looking
        down onto a floor 10-16 cm lower; ~2.7 m to the sides (walls block that in a maze) and
        never behind. A cell is only under the feet if it was seen from that far back,
        straight ahead - after a turn into a 1 m cell, never. The depth camera (20 deg down,
        +-33 deg vertical FOV) sees the floor from ~0.6 m ahead: pass its map in `also`.
        No wider fallback disc: next to slats it took the slats' smeared tops (0.155) for a
        0.10 plate (simulated scans) - callers fall back to wall_scan's start floor instead."""
        p = self.p
        guess = base_xyz[2] - p.standing_height
        xy, h = self.cells(base_xyz[:2], p.floor_under_radius)
        for m in also:
            if m is not None:
                h = np.concatenate([h, m.cells(base_xyz[:2], p.floor_under_radius)[1]])
        z = h[np.abs(h - guess) <= p.floor_under_window]
        return float(np.median(z)) if len(z) >= p.floor_under_min_cells else None


def floor_tilt_from_depth(pts_odom, base_xyz, base_yaw, floor_z, ahead=(0.45, 1.2),
                          half_width=0.35, window=0.10, inlier=0.02, min_pts=150):
    """(forward slope, sideways slope, offset m, points) of the floor seen by the depth
    camera, or None. pts_odom: depth_floor_points() of one frame; floor_z: the known floor
    (start cell: wall_scan's /maze/floor_z).

    Fits dz = offset + a * forward + b * left over points ahead / beside base_link (level
    frame) within `window` of floor_z, then once more over the points within `inlier` of the
    first fit. On a flat floor a and b are 0 when the camera's extrinsic is right; a slope is
    a mount angle error (1 deg ~ 1.7 cm per m of range). Picks the start cell's own floor:
    0.45-1.2 m ahead, +-0.35 m sideways (inside a 1 m cell's walls)."""
    p = np.asarray(pts_odom, float)
    if len(p) == 0:
        return None
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    dx, dy = p[:, 0] - base_xyz[0], p[:, 1] - base_xyz[1]
    fx, fy = c * dx + s * dy, -s * dx + c * dy
    dz = p[:, 2] - floor_z
    sel = (fx >= ahead[0]) & (fx <= ahead[1]) & (np.abs(fy) <= half_width) & \
        (np.abs(dz) <= window)
    if sel.sum() < min_pts:
        return None
    a = np.stack([np.ones(sel.sum()), fx[sel], fy[sel]], 1)
    sol = np.linalg.lstsq(a, dz[sel], rcond=None)[0]
    keep = np.abs(a @ sol - dz[sel]) <= inlier
    if keep.sum() < min_pts:
        return None
    sol = np.linalg.lstsq(a[keep], dz[sel][keep], rcond=None)[0]
    return float(sol[1]), float(sol[2]), float(sol[0]), int(keep.sum())


def tilt_fix(forward_slope, side_slope):
    """3x3 rotation (base_link frame) that levels a floor seen with these slopes: corrected
    r_base_cam = tilt_fix(...) @ r_base_cam (rotation about the camera's own origin)."""
    ph, ps = math.atan(forward_slope), -math.atan(side_slope)
    ry = np.array([[math.cos(ph), 0, math.sin(ph)], [0, 1, 0],
                   [-math.sin(ph), 0, math.cos(ph)]])
    rx = np.array([[1, 0, 0], [0, math.cos(ps), -math.sin(ps)],
                   [0, math.sin(ps), math.cos(ps)]])
    return rx @ ry


class StartCalibration:
    """Calibration on the start cell (its floor = the maze's main floor; raised floors are
    only the uneven-terrain cells), while the robot stands still before /maze/start:

      standing_height: median of base_z - floor_z (the real height in the current gait,
                       instead of the 0.35 m default the floor searches are centred on);
      camera tilt:     median slopes of the start floor seen by the depth camera
                       (floor_tilt_from_depth) -> tilt_fix for the camera extrinsic.

    add_pose() each tick (moving more than still_xy m / still_yaw rad since the last call
    starts the collection over); add_tilt() per depth frame. ready() once `duration` s of
    still samples (>= min_samples poses). result(): dict with standing_height / pitch_deg /
    roll_deg (None when missing or outside min/max_height, max_tilt_deg: then the default
    stays and 'why' says so)."""

    def __init__(self, duration=3.0, min_samples=5, min_height=0.28, max_height=0.42,
                 max_tilt_deg=3.0, still_xy=0.02, still_yaw=math.radians(1.0)):
        self.duration, self.min_samples = duration, min_samples
        self.min_height, self.max_height = min_height, max_height
        self.max_tilt = math.radians(max_tilt_deg)
        self.still_xy, self.still_yaw = still_xy, still_yaw
        self.reset()

    def reset(self):
        self.since, self.last = None, None
        self.heights, self.tilts = [], []

    def add_pose(self, now, base_xyz, yaw, floor_z):
        if self.last is not None:
            lx, ly, lyaw = self.last
            if math.hypot(base_xyz[0] - lx, base_xyz[1] - ly) > self.still_xy or \
                    abs(math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw))) > \
                    self.still_yaw:
                self.reset()
        self.last = (base_xyz[0], base_xyz[1], yaw)
        if self.since is None:
            self.since = now
        self.heights.append(float(base_xyz[2]) - float(floor_z))

    def add_tilt(self, forward_slope, side_slope):
        if self.since is not None:
            self.tilts.append((forward_slope, side_slope))

    def ready(self, now):
        return self.since is not None and now - self.since >= self.duration and \
            len(self.heights) >= self.min_samples

    def result(self):
        out = {'standing_height': None, 'pitch_deg': None, 'roll_deg': None, 'why': []}
        if self.heights:
            h = float(np.median(self.heights))
            out['measured_height'] = round(h, 4)
            if self.min_height <= h <= self.max_height:
                out['standing_height'] = h
            else:
                out['why'].append(f'standing height {h:.3f} m outside '
                                  f'{self.min_height:.2f}..{self.max_height:.2f}')
        if len(self.tilts) >= 3:
            a, b = (float(np.median([t[i] for t in self.tilts])) for i in (0, 1))
            pitch, roll = math.atan(a), math.atan(b)
            out['slopes'] = (a, b)
            if abs(pitch) <= self.max_tilt and abs(roll) <= self.max_tilt:
                out['pitch_deg'], out['roll_deg'] = math.degrees(pitch), math.degrees(roll)
            else:
                out['why'].append(f'camera tilt {math.degrees(pitch):+.1f} / '
                                  f'{math.degrees(roll):+.1f} deg (pitch / roll) over '
                                  f'{math.degrees(self.max_tilt):.0f}: not applied')
        else:
            out['why'].append('no depth floor frames')
        return out


class FloorTracker:
    """Floor under the feet between measurements: the last measured floor (floor_under),
    carried along by how much base_link has risen / dropped since, while standing.

    Why not base_z - standing_height: cmd_vel_bridge measures posture as base_z minus the
    floor under the feet; a floor derived from base_z would always say "standing". This only
    tracks CHANGES of base_z from a measured anchor, only while the robot is known standing
    (cmd_vel_bridge's lie / pose state: commanded, not measured from this floor), and its
    output stays off /maze/local_floor_z (measured only, cmd_vel_bridge's input).

    update() -> (floor z or None, source): 'measured' (handed in this call), 'tracked'
    (anchor + base_z change), 'frozen' (not standing: lying, getting up, a Pose look - the
    last tracked value, so a 15-20 cm lie-down drop never reads as a step), None (no anchor,
    or it expired: older than max_age s, more than max_travel m walked, or base_z more than
    max_rise m from the anchor - Fast-LIO's z drifts a few cm per minute). After standing
    again, tracking waits settle s (still rising) and re-anchors on the frozen value at the
    new base_z (a different gait height after a stand-up is not a floor change).

    Gait bob: base_z goes up and down with every step (trot ~2-3 Hz), and a 2 Hz update()
    lands on a random phase of it. Two filters:
      * base_z is the median over the last `window` s of observe() samples (fed at ~10 Hz:
        several steps), for the anchor and for the tracking alike (instantaneous base_z
        when fewer than 3 samples);
      * steps only: the tracked floor moves only once anchor + base_z change differs from
        it by >= step_min for step_hold s, and then by the whole difference (following the
        median for one more window, until it holds only post-step heights). Maze floors
        change in steps >= 5 cm (plate edges, curtain cells, treads); smaller changes (what
        is left of the bob, slow odom z drift) never move it."""

    def __init__(self, max_age=60.0, max_travel=2.0, max_rise=0.6, settle=1.5, window=1.0,
                 step_min=0.04, step_hold=0.5):
        self.max_age, self.max_travel = max_age, max_travel
        self.max_rise, self.settle = max_rise, settle
        self.window, self.step_min, self.step_hold = window, step_min, step_hold
        self.floor = self.anchor_z = self.anchor_t = self.last_xy = None
        self.travel = 0.0
        self.value = None               # last returned floor (frozen while not standing)
        self.standing_since = None
        self.reanchor = False           # stood up again: anchor anew once settled
        self.zs = collections.deque()   # (t, base_z) from observe()
        self.step_since = None          # monotonic s the candidate first left the value
        self.refine_until = -1e9        # after a step: follow the median for one window

    def observe(self, now, z):
        """A base_z sample (~10 Hz) for the bob filter."""
        self.zs.append((now, float(z)))
        while self.zs and now - self.zs[0][0] > self.window:
            self.zs.popleft()

    def _z(self, now, z):
        """base_z with the bob filtered out: median of the last `window` s of samples."""
        recent = [v for t, v in self.zs if now - t <= self.window]
        return float(np.median(recent)) if len(recent) >= 3 else z

    def _anchor(self, floor, z, t, xy, travel=0.0):
        self.floor, self.anchor_z, self.anchor_t = floor, z, t
        self.last_xy, self.travel, self.value = xy, travel, floor
        self.reanchor, self.step_since, self.refine_until = False, None, -1e9

    def update(self, now, base_xyz, measured=None, standing=True):
        xy = (float(base_xyz[0]), float(base_xyz[1]))
        if not standing:
            if self.standing_since is not None and self.value is not None:
                self.reanchor = True
            self.standing_since = None
            self.zs.clear()             # lying / posing heights must not reach the median
            return (self.value, 'frozen') if self.value is not None else (None, None)
        z = self._z(now, float(base_xyz[2]))
        if self.standing_since is None:
            self.standing_since = now
        if measured is not None:
            self._anchor(float(measured), z, now, xy)
            return self.value, 'measured'
        if self.floor is None:
            return None, None
        if self.reanchor:              # stood up again: wait until settled, then anchor the
            if now - self.standing_since < self.settle:    # frozen value at this base_z
                return self.value, 'frozen'
            self._anchor(self.value, z, self.anchor_t, xy, self.travel)
        if self.last_xy is not None:
            self.travel += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
        self.last_xy = xy
        rise = z - self.anchor_z
        if now - self.anchor_t > self.max_age or self.travel > self.max_travel or \
                abs(rise) > self.max_rise:
            self.floor = self.value = None
            return None, None
        cand = self.floor + rise
        if now < self.refine_until:
            self.value = cand               # just after a step: until the median window
            return self.value, 'tracked'    # holds only post-step heights
        if abs(cand - self.value) < self.step_min:
            self.step_since = None          # bob / drift: keep the value
        elif self.step_since is None:
            self.step_since = now
        if self.step_since is not None and now - self.step_since >= self.step_hold:
            self.value, self.step_since = cand, None      # a real step: all of it
            self.refine_until = now + self.window
        return self.value, 'tracked'


def depth_floor_points(depth, k, r_odom_base, r_base_cam, t_base_cam, base_xyz, stride=8,
                       min_depth=0.3, max_depth=2.0):
    """Odom points (N, 3) from a depth image, for a floor-only HeightMap (floor_under).

    depth: HxW uint16 mm or float m (0 / nan = none); k = (fx, fy, cx, cy); r_odom_base:
    base_link rotation in odom (roll / pitch / yaw); (r_base_cam, t_base_cam): the optical
    frame's pose in base_link; base_xyz: base_link in odom. Every stride-th pixel, camera z
    in min_depth..max_depth only (depth noise grows with range: ~1 cm at 1 m, a few cm past
    2 m on the robot's camera). No median filter: a flying pixel is one sample of a cell's
    smoothed top, and floor_under takes a median over many cells."""
    full = np.asarray(depth)
    d = full[::stride, ::stride]
    z = d.astype(np.float64) * 0.001 if d.dtype == np.uint16 else d.astype(np.float64)
    fx, fy, cx, cy = k
    v, u = np.mgrid[0:full.shape[0]:stride, 0:full.shape[1]:stride]
    ok = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    z, u, v = z[ok], u[ok], v[ok]
    if len(z) == 0:
        return np.zeros((0, 3))
    cam = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1)
    r_oc = np.asarray(r_odom_base) @ np.asarray(r_base_cam)
    origin = np.asarray(base_xyz, float) + np.asarray(r_odom_base) @ np.asarray(t_base_cam)
    return cam @ r_oc.T + origin


# ---------------------------------------------------------------- profiles
@dataclass
class Level:
    u0: float                           # m along the profile, first bin
    u1: float                           # last bin
    h: float                            # odom z


def profile(xy, h, origin, heading, lateral, half_width, u_min, u_max, p):
    """(u centres, median height or nan, cells per bin) along `heading` from `origin` (odom),
    cells within half_width of the line `lateral` m to its left."""
    d = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-d[1], d[0]])
    rel = xy - np.asarray(origin, float)
    u, v = rel @ d, rel @ n - lateral
    sel = (np.abs(v) <= half_width) & (u >= u_min) & (u < u_max)
    nb = max(1, int(math.ceil((u_max - u_min) / p.profile_bin)))
    centres = u_min + (np.arange(nb) + 0.5) * p.profile_bin
    med = np.full(nb, np.nan)
    count = np.zeros(nb, int)
    if sel.any():
        b = np.minimum(((u[sel] - u_min) / p.profile_bin).astype(int), nb - 1)
        hs = h[sel]
        order = np.lexsort((hs, b))
        b, hs = b[order], hs[order]
        starts = np.r_[0, np.nonzero(np.diff(b))[0] + 1]
        ends = np.r_[starts[1:], len(b)]
        for s, e in zip(starts, ends):
            count[b[s]] = e - s
            if e - s >= p.min_bin_cells:
                med[b[s]] = hs[s + (e - s) // 2]
    return centres, med, count


def levels(centres, med, p):
    """Flat runs of a profile (Levels), riser bins in between dropped."""
    out, cur, gap = [], None, 0
    for u, hv in zip(centres, med):
        if np.isnan(hv):
            gap += 1
            if cur is not None and gap > p.max_gap_bins:
                out.append(cur)
                cur = None
            continue
        # statistics.median, not np.median: same value, ~20x faster on these short lists
        # (np.median per bin was most of detect()'s and detect_down()'s time, 2026-11-21).
        if cur is not None and abs(hv - statistics.median(cur[2])) <= p.level_tol:
            cur[1] = u
            cur[2].append(hv)
        else:
            if cur is not None:
                out.append(cur)
            cur = [u, u, [hv]]
        gap = 0
    if cur is not None:
        out.append(cur)
    # One-bin runs are riser cells (mixed heights), not treads.
    return [Level(a, b, float(statistics.median(hs))) for a, b, hs in out if len(hs) >= 2]


def steps_in(lv, floor_z, p):
    """Longest run of upward steps in a list of Levels: (riser u positions, rises, anchored)."""
    best = ([], [], False)
    for i in range(len(lv) - 1):
        us, rises = [], []
        for a, b in zip(lv[i:], lv[i + 1:]):
            rise = b.h - a.h
            u = (a.u1 + b.u0) / 2                     # riser between the two treads
            if not (p.rise_min <= rise <= p.rise_max) or b.u0 - a.u1 > p.riser_gap_max:
                break
            if us and not (p.tread_min <= u - us[-1] <= p.tread_max):
                break
            if rises and abs(rise - float(np.mean(rises))) > p.rise_tol:
                break
            us.append(u)
            rises.append(rise)
        anchored = floor_z is not None and abs(lv[i].h - floor_z) <= p.floor_tol
        if len(us) > len(best[0]) or (len(us) == len(best[0]) and anchored and not best[2]):
            best = (us, rises, anchored)
    return best if len(best[0]) >= p.min_risers else ([], [], False)


def steps_down(lv, floor_z, p):
    """Longest run of downward steps from a level at the floor: (riser u positions, drops).
    The mirror of steps_in for stairs seen from the top (only when the treads are seen,
    e.g. from close up or pitched nose down)."""
    best = ([], [])
    if floor_z is None:
        return best
    for i in range(len(lv) - 1):
        if abs(lv[i].h - floor_z) > p.floor_tol:
            continue
        us, drops = [], []
        for a, b in zip(lv[i:], lv[i + 1:]):
            d = a.h - b.h
            u = (a.u1 + b.u0) / 2
            if not (p.rise_min <= d <= p.rise_max) or b.u0 - a.u1 > p.riser_gap_max:
                break
            if us and not (p.tread_min <= u - us[-1] <= p.tread_max):
                break
            if drops and abs(d - float(np.mean(drops))) > p.rise_tol:
                break
            us.append(u)
            drops.append(d)
        if len(us) > len(best[0]):
            best = (us, drops)
    return best


# ---------------------------------------------------------------- detection
def slope_directions(hm, base_xy, p, k=3):
    """Up to k uphill directions (rad, odom) of the map's step edges near the robot."""
    g, _, _ = hm.grid(base_xy, p.search_max)
    if g.shape[0] < 7 or g.shape[1] < 7:
        return []
    ok = np.isfinite(g)
    v = np.where(ok, g, 0.0)
    acc, cnt = np.zeros_like(v), np.zeros_like(v)
    for di in (-1, 0, 1):                             # nan-aware 3x3 mean
        for dj in (-1, 0, 1):
            acc += np.roll(np.roll(v, di, 0), dj, 1)
            cnt += np.roll(np.roll(ok, di, 0), dj, 1)
    g = np.where(cnt >= 5, acc / np.maximum(cnt, 1), np.nan)
    gi = (g[4:, 2:-2] - g[:-4, 2:-2]) / 2             # per 2 cells along odom x (first index)
    gj = (g[2:-2, 4:] - g[2:-2, :-4]) / 2             # along odom y
    mag = np.hypot(gi, gj)
    edge = np.isfinite(mag) & (mag >= p.slope_min) & (mag <= 2 * p.rise_max)
    if edge.sum() < p.min_edge_cells:
        return []
    ang = np.degrees(np.arctan2(gj[edge], gi[edge]))
    hist = np.bincount(((ang + 180) // 5).astype(int) % 72, minlength=72).astype(float)
    sm = hist + 0.5 * (np.roll(hist, 1) + np.roll(hist, -1))
    out = []
    for b in np.argsort(-sm):
        if sm[b] < p.min_edge_cells:
            break
        a = math.radians(b * 5 - 180 + 2.5)
        if all(abs(wrap(a - o)) > math.radians(20) for o in out):
            out.append(a)
        if len(out) >= k:
            break
    return out


def _bands(xy, h, base_xy, heading, floor_z, p, lats=None):
    """[(lateral, riser us, rises, anchored)] for profiles across lateral bands."""
    d = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-d[1], d[0]])
    v_all = (xy - np.asarray(base_xy, float)) @ n
    if lats is None:
        lats = np.arange(-p.search_max, p.search_max + 1e-9, p.band_step)
    out = []
    for lat in lats:
        if (np.abs(v_all - lat) <= p.band_half_width).sum() < 10:
            continue
        c, med, _ = profile(xy, h, base_xy, heading, lat, p.band_half_width,
                            -p.search_max, p.search_max, p)
        us, rises, anchored = steps_in(levels(c, med, p), floor_z, p)
        if us and us[0] > 0 and p.search_min <= math.hypot(us[0], lat) <= p.search_max:
            out.append((float(lat), us, rises, anchored))
    return out


def riser_span(xy, h, base_xy, heading, u0, rise, p, around=0.0):
    """(v_lo, v_hi) sideways of base_xy (left +) where the height jumps by >= 0.6 rise across
    the riser at u0 along `heading`: the stairs' sides, measured on the riser itself in 4 cm
    slices (lateral profile bands are 0.4 m wide and reach past the sides). None if none."""
    d = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-d[1], d[0]])
    rel = xy - np.asarray(base_xy, float)
    u, v = rel @ d, rel @ n
    before = (u > u0 - 0.12) & (u < u0 - 0.03)
    after = (u > u0 + 0.03) & (u < u0 + 0.12)
    step = 0.04
    vb = np.arange(around - p.search_max, around + p.search_max + 1e-9, step)
    hb = np.full(len(vb), np.nan)
    ha = np.full(len(vb), np.nan)
    for k, c in enumerate(vb):
        sl = np.abs(v - c) <= step / 2 + 0.01
        if (sl & before).any() and (sl & after).any():
            hb[k], ha[k] = np.median(h[sl & before]), np.median(h[sl & after])
    # A riser slice: the lower level before it and the upper level after it, like the rest of
    # the riser (medians over the slices near `around`), and a jump of at least half a rise.
    # Wall tops beside the stairs also "jump" but are at neither level.
    mid = np.isfinite(hb) & (np.abs(vb - around) <= 0.3)
    if not mid.any():
        return None
    lo, hi = np.median(hb[mid]), np.median(ha[mid])
    jump = np.isfinite(hb) & (np.abs(hb - lo) <= 0.05) & (np.abs(ha - hi) <= 0.05) & \
        (ha - hb >= 0.5 * rise)
    best, run, gap = None, None, 0
    for k in range(len(vb)):                          # longest run, 1-slice gaps allowed
        if jump[k]:
            run = [k, k] if run is None else [run[0], k]
            gap = 0
        elif run is not None:
            gap += 1
            if gap > 1:
                if best is None or run[1] - run[0] > best[1] - best[0]:
                    best = run
                run, gap = None, 0
    if run is not None and (best is None or run[1] - run[0] > best[1] - best[0]):
        best = run
    if best is None:
        return None
    return float(vb[best[0]] - step / 2), float(vb[best[1]] + step / 2)


def fit_riser_heading(xy, h, base_xy, heading, u0, around, p, half=0.6):
    """Heading corrected by the first riser's own line: in 8 cm slices across the stairs, the
    riser is where the height first passes half way between the levels before and after it;
    a line through those points (one outlier pass) gives the correction. heading unchanged if
    too few slices."""
    d = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-d[1], d[0]])
    rel = xy - np.asarray(base_xy, float)
    u, v = rel @ d, rel @ n
    near = np.abs(u - u0) <= 0.25
    lo_c = near & (u < u0 - 0.04) & (np.abs(v - around) <= 0.3)
    hi_c = near & (u > u0 + 0.04) & (np.abs(v - around) <= 0.3)
    if lo_c.sum() < 3 or hi_c.sum() < 3:
        return heading
    lo, hi = np.median(h[lo_c]), np.median(h[hi_c])
    mid = (lo + hi) / 2
    vs, us = [], []
    for c in np.arange(around - half, around + half + 1e-9, 0.08):
        sl = near & (np.abs(v - c) <= 0.04)
        b4 = sl & (u > u0 - 0.15) & (u < u0 - 0.04)
        af = sl & (u > u0 + 0.04) & (u < u0 + 0.15)
        # Riser slices only: lower level before, upper level after (wall tops beside the
        # stairs are high all along and would tilt the line).
        if not (b4.any() and af.any()) or abs(np.median(h[b4]) - lo) > 0.05 or \
                abs(np.median(h[af]) - hi) > 0.05:
            continue
        up = sl & (h >= mid)
        if up.sum() >= 2 and (sl & (h < mid)).any():
            vs.append(c)
            us.append(float(np.percentile(u[up], 20)))
    if len(vs) < 4:
        return heading
    vs, us = np.array(vs), np.array(us)
    k, b = np.polyfit(vs, us, 1)
    res = np.abs(us - (k * vs + b))
    keep = res <= max(0.03, 2.5 * np.median(res))
    if keep.sum() >= 4:
        k, b = np.polyfit(vs[keep], us[keep], 1)
    return wrap(heading - math.atan(k))


def _common_first(bands):
    """First-riser position shared by the most bands (not the band with the most steps: an
    edge band can have one noisy extra step)."""
    us = np.array([b[1][0] for b in bands])
    votes = [(np.abs(us - x) <= 0.08).sum() for x in us]
    best = us[np.abs(us - us[int(np.argmax(votes))]) <= 0.08]
    return float(np.median(best))


def _score(bands):
    """Most risers in the best band, then most bands with stairs."""
    return (max((len(b[1]) for b in bands), default=0), len(bands))


def detect(hm, base_xyz, floor_z, p=None):
    """Lidar-only stairs near the robot: list of StairEstimates (odom), best first."""
    p = p or hm.p
    base_xy = np.asarray(base_xyz[:2], float)
    xy, h = hm.cells(base_xy, p.search_max + 0.6)
    if len(h) == 0:
        return []
    out = []
    for coarse in slope_directions(hm, base_xy, p):
        bands = _bands(xy, h, base_xy, coarse, floor_z, p)
        if not bands:
            continue
        # Refine the heading on the bands that found steps: the sharpest profile wins.
        lats = sorted({b[0] for b in bands})
        best = (_score(bands), coarse, bands)
        for dh in np.arange(-p.refine_deg, p.refine_deg + 1e-9, p.refine_step_deg):
            hd = coarse + math.radians(dh)
            bb = _bands(xy, h, base_xy, hd, floor_z, p, lats)
            key = _score(bb)
            if key > best[0] or (key == best[0] and abs(dh) < abs(math.degrees(wrap(best[1] - coarse)))):
                best = (key, hd, bb)
        _, heading, bands = best
        bands = _bands(xy, h, base_xy, heading, floor_z, p)     # full width at that heading
        if not bands:
            continue
        first = _common_first(bands)
        same = [b for b in bands if abs(b[1][0] - first) <= 0.08]
        rise = float(np.mean([np.mean(b[2]) for b in same]))
        around = float(np.median([b[0] for b in same]))
        for _ in range(2):                  # the riser line's own direction, then re-profile
            heading = fit_riser_heading(xy, h, base_xy, heading, first, around, p)
            bb = _bands(xy, h, base_xy, heading, floor_z, p,
                        np.arange(around - 0.6, around + 0.6 + 1e-9, p.band_step))
            if not bb:
                break
            first = _common_first(bb)
            same = [b for b in bb if abs(b[1][0] - first) <= 0.08]
            around = float(np.median([b[0] for b in same]))
        span = riser_span(xy, h, base_xy, heading, first, rise, p, around=around)
        if span is None or span[1] - span[0] < p.min_width:
            continue
        est = StairEstimate(heading=wrap(heading), along=0.0, risers=max(len(b[1]) for b in same),
                            rise=rise, source='lidar', anchored=any(b[3] for b in same),
                            width=float(span[1] - span[0]))
        est.along = float(base_xy @ est.d()) + float(np.median([b[1][0] for b in same]))
        est.centre = float(base_xy @ est.n()) + (span[0] + span[1]) / 2
        if not any(o.close_to(est, 0.15, math.radians(10)) for o in out):
            out.append(est)
    return out


def drop_edge(centres, med, floor_z, p):
    """(edge u, drop m, levels seen below) of one profile, or None.

    A floor level (within floor_tol of floor_z, >= drop_floor_min long) ending at u > 0, then
    within drop_gap_max a run of ever lower levels (unseen bins in between are fine: the
    nosing shadows them) reaching drop_min..drop_max below the floor, with nothing higher
    than floor + drop_wall_max in between (a wall between the robot and lower ground seen
    past it is not a drop). The edge is where the floor level ends: the nosing is the
    occluding edge, so the floor is seen right up to it."""
    lv = levels(centres, med, p)
    for k, a in enumerate(lv[:-1]):
        if abs(a.h - floor_z) > p.floor_tol or a.u1 <= 0.0 or \
                a.u1 - a.u0 < p.drop_floor_min - 1e-9:
            continue
        run, prev = [], a
        for b in lv[k + 1:]:
            if b.h >= prev.h - 0.03 or b.u0 - prev.u1 > p.drop_gap_max:
                break
            run.append(b)
            prev = b
        if not run:
            continue
        drop = floor_z - run[-1].h
        if not (p.drop_min <= drop <= p.drop_max):
            continue
        between = (centres > a.u1) & (centres < run[-1].u0) & np.isfinite(med)
        if between.any() and np.nanmax(med[between]) > floor_z + p.drop_wall_max:
            continue
        return float(a.u1), float(drop), len(run)      # last floor bin: never late
    return None


def band_profiles(xy, h, base_xy, heading, lats, p, u_max):
    """(u centres, median height per (lat, bin) or nan) of all lateral bands at once.

    profile() per band costs a pass over every cell plus a Python loop per bin; detect_down
    profiles ~100 bands per call (336 ms near a drop). Here the cells are rasterised once per
    heading (profile_bin along x map cell across; the highest cell wins a raster pixel) and
    each band is a nan-median over its raster rows."""
    d = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-d[1], d[0]])
    rel = xy - np.asarray(base_xy, float)
    u, v = rel @ d, rel @ n
    lats = np.asarray(lats, float)
    v0 = lats.min() - p.band_half_width
    nv = int(math.ceil((lats.max() + p.band_half_width - v0) / p.cell)) + 1
    nb = max(1, int(math.ceil(u_max / p.profile_bin)))
    ui = np.floor(u / p.profile_bin).astype(np.int64)
    vi = np.floor((v - v0) / p.cell).astype(np.int64)
    ok = (ui >= 0) & (ui < nb) & (vi >= 0) & (vi < nv)
    ui, vi, hh = ui[ok], vi[ok], h[ok]
    order = np.argsort(hh, kind='stable')
    ras = np.full((nv, nb), np.nan)
    ras[vi[order], ui[order]] = hh[order]
    centres = (np.arange(nb) + 0.5) * p.profile_bin
    rows_v = v0 + (np.arange(nv) + 0.5) * p.cell
    med = np.full((len(lats), nb), np.nan)
    with np.errstate(all='ignore'):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            for k, lat in enumerate(lats):
                rows = ras[np.abs(rows_v - lat) <= p.band_half_width]
                if len(rows) == 0:
                    continue
                cnt = np.isfinite(rows).sum(axis=0)
                m = np.nanmedian(rows, axis=0)
                med[k] = np.where(cnt >= p.min_bin_cells, m, np.nan)
    return centres, med


def _drop_bands(xy, h, base_xy, heading, floor_z, p, lats=None):
    """[(lateral, edge u, drop, levels below)] across lateral bands at one heading."""
    if lats is None:
        lats = np.arange(-p.drop_lat, p.drop_lat + 1e-9, p.band_step)
    c, med = band_profiles(xy, h, base_xy, heading, lats, p, p.drop_search + p.drop_gap_max)
    out = []
    for lat, row in zip(lats, med):
        if np.isfinite(row).sum() < 4:
            continue
        e = drop_edge(c, row, floor_z, p)
        if e is not None and p.drop_near <= e[0] <= p.drop_search:
            out.append((float(lat),) + e)
    return out


def _drop_group(bands, p):
    """The bands whose edges agree with the most others (within drop_edge_tol)."""
    if not bands:
        return []
    us = np.array([b[1] for b in bands])
    votes = [(np.abs(us - x) <= p.drop_edge_tol).sum() for x in us]
    ref = us[int(np.argmax(votes))]
    return [b for b in bands if abs(b[1] - ref) <= p.drop_edge_tol]


def detect_down(hm, base_xyz, floor_z, p=None):
    """Descending stairs (or a drop of >= drop_min) near the robot: StairEstimates with
    down=True, heading pointing down them, `along` at the top edge, best first.

    floor_z: the floor the robot stands on (HeightMap.floor_under, not floor()). Cheap when
    there is nothing: cells drop_min below that floor are counted first. Headings: bearings
    of those cells from the robot (10 deg bins, up to 3), each tried +-20 deg, then corrected
    by the line through the bands' edges. Treads seen (steps_down) give the riser count;
    otherwise drop / drop_rise_guess, at least 2."""
    p = p or hm.p
    if floor_z is None:
        return []
    base_xy = np.asarray(base_xyz[:2], float)
    xy, h = hm.cells(base_xy, p.drop_search + p.drop_gap_max)
    low = (h < floor_z - p.drop_min) & (h > floor_z - p.drop_max)
    if low.sum() < p.drop_min_cells:
        return []
    rel = xy[low] - base_xy
    ang = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))
    hist = np.bincount(((ang + 180) // 10).astype(int) % 36, minlength=36).astype(float)
    sm = hist + 0.5 * (np.roll(hist, 1) + np.roll(hist, -1))
    out = []
    tried = []
    for b in np.argsort(-sm)[:6]:
        if sm[b] < p.drop_min_cells or len(tried) >= 3:
            break
        coarse = math.radians(b * 10 - 180 + 5)
        if any(abs(wrap(coarse - t)) < math.radians(25) for t in tried):
            continue
        tried.append(coarse)
        best = None
        for dh in (0.0, -15.0, 15.0):
            hd = wrap(coarse + math.radians(dh))
            grp = _drop_group(_drop_bands(xy, h, base_xy, hd, floor_z, p), p)
            if not grp:
                continue
            spread = float(np.ptp([g[1] for g in grp])) if len(grp) > 1 else 0.0
            key = (len(grp), -spread)
            if best is None or key > best[0]:
                best = (key, hd, grp)
        if best is None or len(best[2]) < p.drop_min_bands:
            continue
        _, heading, grp = best
        if len(grp) >= 3:                   # the edge line's own direction, then re-profile
            lats, us = np.array([g[0] for g in grp]), np.array([g[1] for g in grp])
            k, _ = np.polyfit(lats, us, 1)
            hd = wrap(heading - math.atan(k))
            g2 = _drop_group(_drop_bands(xy, h, base_xy, hd, floor_z, p), p)
            if len(g2) >= len(grp):
                heading, grp = hd, g2
        if len(grp) < p.drop_min_bands:
            continue
        lats = np.array([g[0] for g in grp])
        edge = float(np.median([g[1] for g in grp]))
        drop = float(np.median([g[2] for g in grp]))
        # Treads seen? Profile the centre band again for downward steps from the floor.
        mid = float(np.median(lats))
        c, med, _ = profile(xy, h, base_xy, heading, mid, p.band_half_width,
                            0.0, p.drop_search + p.drop_gap_max, p)
        us, drops = steps_down(levels(c, med, p), floor_z, p)
        if len(us) >= 2 and abs(us[0] - edge) <= p.drop_edge_tol + 0.05:
            risers, rise = len(us), float(np.mean(drops))
            risers = max(risers, int(round(drop / rise)))
        else:
            # Treads unseen: a drop >= drop_min isn't walked off in the normal gait, so it is
            # stairs (>= 2 risers, the user's rule for stairs) whatever the real count.
            risers = max(2, int(round(drop / p.drop_rise_guess)))
            rise = drop / risers
        est = StairEstimate(heading=wrap(heading), along=0.0, risers=risers, rise=rise,
                            source='lidar', anchored=True,
                            width=float(np.ptp(lats) + p.band_step), down=True)
        est.along = float(base_xy @ est.d()) + edge
        est.centre = float(base_xy @ est.n()) + mid
        # One staircase reached from two coarse bearings: same edge point, similar heading.
        if not any(np.linalg.norm(o.edge_point(base_xy) - est.edge_point(base_xy)) < 0.3 and
                   abs(wrap(o.heading - est.heading)) < math.radians(30) for o in out):
            out.append(est)
    out.sort(key=lambda e: e.distance(base_xy))
    return out


# ---------------------------------------------------------------- terrain ahead
@dataclass
class TerrainParams:
    """terrain(): what the robot is about to walk on (presentation mazes 2026-11-21: the only
    way out crosses two uneven cells - a raised box / ramp, transverse slats, a field of
    blocks of mixed heights). Heights relative to the floor under the robot."""
    ahead: tuple = (0.25, 1.20)         # m ahead of base_link: the strip looked at
    half_width: float = 0.30            # m either side (the body is 0.46 m wide)
    under: float = 0.45                 # m around base_link: what it stands on now
    min_cells: int = 12
    ignore_above: float = 0.30          # m: walls and stairs (their own detectors)
    rough_step: float = 0.05            # m (95th percentile) -> rough
    rough_spread: float = 0.035         # m (90th - 10th percentile) -> rough
    obstacle_step: float = 0.10         # m -> obstacle (slats, blocks, a plate edge up)
    obstacle_spread: float = 0.08
    drop_low: float = 0.06              # m below the floor (5th percentile) -> drop
    # m spread under the robot -> standing on rough ground. Simulated scans (2 cm range noise):
    # flat 0.01, slats 0.025, the presentation block field 0.037-0.042 (its 9 cm real spread
    # is squashed by the per-cell maximum and the smoothing).
    under_rough: float = 0.03


def terrain(hm, base_xyz, yaw, floor_z, tp=None):
    """dict: level ('even', 'rough', 'obstacle', 'drop' or 'unknown') of the strip ahead,
    its max_step / low / spread (m, vs floor_z), cells, and under_spread / on_rough for the
    ground under the robot (None if not seen). Cheap: two small discs of the height map."""
    tp = tp or TerrainParams()
    out = {'level': 'unknown', 'max_step': None, 'low': None, 'spread': None, 'cells': 0,
           'under_spread': None, 'on_rough': False}
    if floor_z is None:
        return out
    bxy = np.asarray(base_xyz[:2], float)
    xy, h = hm.cells(bxy, math.hypot(tp.ahead[1], tp.half_width))
    rel = xy - bxy
    c, s = math.cos(yaw), math.sin(yaw)
    u, v = rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c
    z = h - floor_z
    ok = z <= tp.ignore_above
    near = ok & (np.hypot(u, v) <= tp.under)
    if near.sum() >= tp.min_cells:
        sp = float(np.percentile(z[near], 90) - np.percentile(z[near], 10))
        out['under_spread'], out['on_rough'] = sp, sp >= tp.under_rough
    strip = ok & (u >= tp.ahead[0]) & (u <= tp.ahead[1]) & (np.abs(v) <= tp.half_width)
    out['cells'] = int(strip.sum())
    if strip.sum() < tp.min_cells:
        return out
    zs = z[strip]
    step, low = float(np.percentile(zs, 95)), float(np.percentile(zs, 5))
    spread = float(np.percentile(zs, 90) - np.percentile(zs, 10))
    out.update(max_step=step, low=low, spread=spread)
    if low <= -tp.drop_low:
        out['level'] = 'drop'
    elif step >= tp.obstacle_step or spread >= tp.obstacle_spread:
        out['level'] = 'obstacle'
    elif step >= tp.rough_step or spread >= tp.rough_spread:
        out['level'] = 'rough'
    else:
        out['level'] = 'even'
    return out


# ---------------------------------------------------------------- camera check and fusion
@dataclass
class Check:
    kind: str                           # 'confirm', 'veto' or 'unknown'
    along: float = None                 # confirm: the lidar's first riser (odom, along est.d())
    anchored: bool = False
    why: str = ''


def check(hm, est, floor_z, base_xy, p=None):
    """A camera StairEstimate against the lidar map (see module doc)."""
    p = p or hm.p
    lat = est.lateral(np.asarray(base_xy, float)) if est.centre is not None else 0.0
    origin = np.asarray(base_xy, float)
    a0 = est.distance(origin)
    xy, h = hm.cells(origin, a0 + p.check_after + 1.0)
    # Profile along the stairs through their centre (robot's line if the centre is unknown).
    c, med, _ = profile(xy, h, origin, est.heading, -lat, p.band_half_width,
                        a0 - p.check_before, a0 + p.check_after, p)
    covered = np.isfinite(med)
    us, rises, anchored = steps_in(levels(c, med, p), floor_z, p)
    if us:
        first = us[0]
        if -p.nearer_max - 0.05 <= first - a0 <= 0.15:
            return Check('confirm', along=float(origin @ est.d()) + first, anchored=anchored,
                         why=f'{len(us)} lidar steps, first {first - a0:+.2f} m from camera\'s')
        return Check('unknown', why=f'lidar steps {first - a0:+.2f} m from the camera\'s')
    if covered.mean() >= p.veto_coverage and \
            np.nanmax(med) - np.nanmin(med) < p.veto_flat:
        return Check('veto', why=f'lidar map flat there ({covered.mean() * 100:.0f} % covered, '
                                 f'{(np.nanmax(med) - np.nanmin(med)) * 100:.1f} cm spread)')
    return Check('unknown', why=f'lidar coverage {covered.mean() * 100:.0f} %, no steps')


def fuse(cam, lid, chk, base_xy, nearer_min=0.10, yaw_tol=math.radians(10), nearer_max=0.5):
    """(estimate or None, why) from the camera tracker's estimate, the lidar tracker's and
    check(cam).
    - vetoed camera stairs are dropped;
    - the camera's first riser (accurate to ~1 cm) is kept, unless the lidar has an ANCHORED
      first riser (rising from the measured floor) nearer_min..nearer_max nearer: then the
      camera's bottom step was out of its view and the lidar's is taken. Within nearer_min
      the difference is lidar position noise (5-10 cm);
    - camera and lidar trackers both: agree (direction, first riser within nearer_max) -> one
      estimate, camera's centre preferred; disagree -> the nearer (stopping early is the safe
      mistake);
    - one of them only: that one."""
    xy = np.asarray(base_xy, float)
    why = []
    if cam is not None and chk is not None:
        if chk.kind == 'veto':
            why.append(f'camera stairs vetoed: {chk.why}')
            cam = None
        elif chk.kind == 'confirm':
            nearer = cam.distance(xy) - (chk.along - float(xy @ cam.d()))
            if chk.anchored and nearer_min <= nearer <= nearer_max:
                why.append(f'lidar map has the first riser {nearer:.2f} m nearer')
                cam = StairEstimate(heading=cam.heading, along=chk.along, centre=cam.centre,
                                    risers=cam.risers + 1, rise=cam.rise, source='both',
                                    anchored=chk.anchored, width=cam.width)
            else:
                cam = StairEstimate(heading=cam.heading, along=cam.along, centre=cam.centre,
                                    risers=cam.risers, rise=cam.rise, source='both',
                                    anchored=chk.anchored, width=cam.width)
            why.append(f'lidar confirms ({chk.why})')
    if cam is None or lid is None:
        out = cam if cam is not None else lid
        return out, '; '.join(why) or (f'{out.source} only' if out is not None else 'none')
    if cam.down != lid.down or abs(wrap(cam.heading - lid.heading)) > yaw_tol or \
            abs(cam.distance(xy) - lid.distance(xy)) > nearer_max:
        near = cam if cam.distance(xy) <= lid.distance(xy) else lid
        why.append(f'camera and lidar disagree; nearer ({near.source}) taken')
        return near, '; '.join(why)
    nearer = cam.distance(xy) - lid.distance(xy)
    near = lid if (lid.anchored and nearer_min <= nearer) else cam
    out = StairEstimate(heading=cam.heading, along=0.0,
                        centre=cam.centre if cam.centre is not None else lid.centre,
                        risers=max(cam.risers, lid.risers), rise=cam.rise, source='both',
                        anchored=cam.anchored or lid.anchored,
                        width=max(cam.width, lid.width), down=cam.down)
    out.along = float(np.dot(near.d() * near.along, out.d()))
    why.append(f'camera and lidar agree; first riser from {"lidar" if near is lid else "camera"}')
    return out, '; '.join(why)
