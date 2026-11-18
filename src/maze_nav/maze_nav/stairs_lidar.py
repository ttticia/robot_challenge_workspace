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
import math
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
    keep_below: float = 0.60            # m below base_link kept (floor at ~0.35)
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
    rise_min: float = 0.06
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
        cells, inv = np.unique(flat, return_inverse=True)
        mx = np.full(len(cells), -np.inf)
        np.maximum.at(mx, inv, z)                       # this scan's top per cell
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
        if cur is not None and abs(hv - np.median(cur[2])) <= p.level_tol:
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
    return [Level(a, b, float(np.median(hs))) for a, b, hs in out if len(hs) >= 2]


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
                            rise=rise, source='lidar', anchored=any(b[3] for b in same))
        est.along = float(base_xy @ est.d()) + float(np.median([b[1][0] for b in same]))
        est.centre = float(base_xy @ est.n()) + (span[0] + span[1]) / 2
        if not any(o.close_to(est, 0.15, math.radians(10)) for o in out):
            out.append(est)
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
                                    anchored=chk.anchored)
            else:
                cam = StairEstimate(heading=cam.heading, along=cam.along, centre=cam.centre,
                                    risers=cam.risers, rise=cam.rise, source='both',
                                    anchored=chk.anchored)
            why.append(f'lidar confirms ({chk.why})')
    if cam is None or lid is None:
        out = cam if cam is not None else lid
        return out, '; '.join(why) or (f'{out.source} only' if out is not None else 'none')
    if abs(wrap(cam.heading - lid.heading)) > yaw_tol or \
            abs(cam.distance(xy) - lid.distance(xy)) > nearer_max:
        near = cam if cam.distance(xy) <= lid.distance(xy) else lid
        why.append(f'camera and lidar disagree; nearer ({near.source}) taken')
        return near, '; '.join(why)
    nearer = cam.distance(xy) - lid.distance(xy)
    near = lid if (lid.anchored and nearer_min <= nearer) else cam
    out = StairEstimate(heading=cam.heading, along=0.0,
                        centre=cam.centre if cam.centre is not None else lid.centre,
                        risers=max(cam.risers, lid.risers), rise=cam.rise, source='both',
                        anchored=cam.anchored or lid.anchored)
    out.along = float(np.dot(near.d() * near.along, out.d()))
    why.append(f'camera and lidar agree; first riser from {"lidar" if near is lid else "camera"}')
    return out, '; '.join(why)
