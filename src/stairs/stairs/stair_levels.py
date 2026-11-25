# Copied unchanged from ~/stairs_detect/stair_levels.py (Rey, 2026-11-24) so that maze_explore
# is self-contained. Change the detector there first, then copy it here again.
"""Stairs from the depth camera by their tread levels (numpy only, no ROS).

The challenge stairs (Rey, 23 Nov 2026): up and over, 3 steps up, 12 cm rise, 20 cm tread,
95 cm between the handrails, top ~0.36 m. Seen from the cell in front, the depth camera sees
the treads as flat patches at ~0.12 / 0.24 / 0.36 m above the floor, each ~one tread further
away. Terrain in the arena (8 cm raised cells, block field <= ~0.22 m, slats, wedges) never has
a flat patch near 0.36 m, so the TOP level is required.

detect(depth_m, k, r_cam, t_cam, p) -> StairSeen or None
  1. depth -> grid of points (every stride px) in the LEVEL frame: base_link position, x along
     the robot's heading, z up along gravity (roll/pitch from the IMU taken out).
  2. local flatness from the organised grid: a point is on a horizontal surface if its 3D
     neighbours (right and below in the image) lie within flat_dz of its height while being
     at least a few cm away sideways. Floor height: lowest dense flat layer near -body_height.
  3. flat points binned by height above the floor: level k (k = 1..steps) = points within
     level_tol of k * rise. The top level (k = steps) must hold >= min_level_points.
  4. stair direction: the front edge of the top level and of every other level found
     (nearest flat point per lateral slice) fitted as lines; heading = their mean normal.
  5. along that heading: each level's front edge u_k. Levels must be one tread apart
     (tread +- tread_tol) and in order. distance (base_link -> bottom riser) = u_1 if level 1
     is seen, else u_lowest - (lowest_k - 1) * tread (close up the bottom tread is below the
     view). Width of the top level across the heading >= min_width.
"""
import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class LevelParams:
    rise: float = 0.12
    tread: float = 0.20
    steps: int = 3                      # levels up to the top
    body_height: float = 0.35           # base_link above the floor when standing
    floor_search: float = 0.12          # floor layer within this of -body_height
    stride: int = 4                     # px
    min_depth: float = 0.15
    max_depth: float = 3.0
    max_ahead: float = 2.5              # m
    max_side: float = 1.5               # m
    flat_dz: float = 0.012              # m: neighbours this level count as flat
    min_step_xy: float = 0.005          # m between neighbours (else ignored)
    level_tol: float = 0.04             # m around k * rise
    min_level_points: int = 25          # top level
    min_other_points: int = 12          # lower levels (optional)
    slice_w: float = 0.05               # m lateral slices for the front edge
    min_slices: int = 6                 # front edge needs this many slices
    tread_tol: float = 0.07
    min_width: float = 0.35             # m, top level across the heading
    half_width: float = 0.55            # m, lateral crop around the steps' centre
    # Height-range checks (Rey, live test 23 Nov: an obstacle with stair-like steps passed)
    height_tol: float = 0.025           # each tread k*rise +- this above the floor, and each
    #                                     step this close to one rise above the one below
    top_min: float = 0.33               # top tread above the floor
    top_max: float = 0.40
    above_lo: float = 0.45              # nothing flat between these heights on the footprint
    above_hi: float = 0.80
    above_max_points: int = 30
    edge_half_span: float = 0.40   # treads checked across the middle 0.8 m (rails at +-0.475)
    edge_bin: float = 0.05
    edge_min_pts: int = 3          # flat points of another height that make a strip foreign
    edge_max_foreign: int = 2      # of 16 strips
    edge_tol: float = 0.03         # front edge straightness
    edge_gap: int = 6              # holes (no depth) bridged, in strips (24 Nov: sparse top tread)
    edge_min_run: float = 0.30
    edge_other_dh: float = 0.06    # a flat patch this far from the tread height is another level
    max_yaw_deg: float = 60.0


@dataclass
class StairSeen:
    distance: float                     # m, base_link to the bottom riser along heading
    yaw: float                          # rad, stair heading (into the stairs) in the level frame
    lateral: float                      # m, stair centre sideways of base_link (left +)
    width: float                        # m, top level width
    levels: list                        # [(k, u_front, n_points, height)] seen
    floor_z: float                      # level-frame z of the floor
    inferred: bool = False              # bottom riser not seen (distance from a higher level)
    why: str = ''

    def summary(self):
        ks = ','.join(f'{k}@{lv[3]:.2f}' for k, *lv in [(l[0], *l) for l in self.levels])
        return (f'stairs {self.distance:.2f} m at {math.degrees(self.yaw):+.0f} deg, '
                f'lateral {self.lateral:+.2f}, width {self.width:.2f}, levels {ks}'
                + (' (bottom inferred)' if self.inferred else ''))


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# Orbbec optical frame (x right, y down, z forward) relative to camera_link (x fwd, z up).
R_LINK_OPTICAL = quat_to_matrix(-0.5, 0.5, -0.5, 0.5)


def roll_pitch_only(r_world_base):
    """Body orientation with its yaw removed (level frame keeps the robot's heading)."""
    r = np.asarray(r_world_base, float)
    yaw = math.atan2(r[1, 0], r[0, 0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ r


def level_camera(r_level_base, r_base_link, t_base_link):
    """(R, t): optical frame -> level frame, from base_link -> camera_link."""
    r_bc = np.asarray(r_base_link) @ R_LINK_OPTICAL
    return r_level_base @ r_bc, r_level_base @ np.asarray(t_base_link, float)


def grid_points(depth, k, r, t, p):
    """(H', W', 3) level-frame points on a stride grid, NaN where no depth."""
    d = np.asarray(depth)
    d = d.astype(np.float64) * (0.001 if d.dtype == np.uint16 else 1.0)
    d = d[::p.stride, ::p.stride]
    fx, fy, cx, cy = k
    hh, ww = d.shape
    u = np.arange(ww) * p.stride
    v = np.arange(hh) * p.stride
    uu, vv = np.meshgrid(u, v)
    ok = (d > p.min_depth) & (d < p.max_depth)
    z = np.where(ok, d, np.nan)
    cam = np.stack([(uu - cx) / fx * z, (vv - cy) / fy * z, z], -1)
    return cam @ np.asarray(r).T + np.asarray(t)


def flat_mask(g, p):
    """Points whose right and lower grid neighbours are level with them (horizontal surface)."""
    z = g[..., 2]
    flat = np.zeros(z.shape, bool)
    for dy, dx in ((0, 1), (1, 0)):
        a = g[:g.shape[0] - dy, :g.shape[1] - dx]
        b = g[dy:, dx:]
        dz = np.abs(a[..., 2] - b[..., 2])
        dxy = np.hypot(a[..., 0] - b[..., 0], a[..., 1] - b[..., 1])
        ok = (dz < p.flat_dz) & (dxy > p.min_step_xy) & (dz < 0.5 * dxy)
        m = np.zeros(z.shape, bool)
        m[:g.shape[0] - dy, :g.shape[1] - dx] = ok
        flat &= m if (dy, dx) != (0, 1) else True
        flat |= m if (dy, dx) == (0, 1) else False
    return flat & np.isfinite(z)


def find_floor(pts, p):
    z = pts[:, 2]
    guess = -p.body_height
    sel = z[np.abs(z - guess) < p.floor_search]
    if len(sel) < 30:
        return guess, False
    hist, edges = np.histogram(sel, bins=np.arange(guess - p.floor_search,
                                                   guess + p.floor_search + 0.01, 0.01))
    dense = np.nonzero(hist >= max(0.2 * hist.max(), 10))[0]
    if not len(dense):
        return guess, False
    i = int(dense[0])
    near = sel[(sel >= edges[i] - 0.01) & (sel <= edges[i + 1] + 0.01)]
    return float(np.median(near)), True


def choose_floor(pts, p):
    """Floor height for the level test. Close up the floor leaves the view and the lowest flat
    layer is the first tread (real stairs, mid-cell shot: 'floor' 0.12 m too high, top level
    missed). Candidates: that layer and one or two rises below it, within floor_search of
    -body_height; pick the one that puts the most levels (top first) on the k * rise lattice."""
    layer, measured = find_floor(pts, p)
    best = None
    for k in range(p.steps):
        f = layer - k * p.rise
        if abs(f + p.body_height) > p.floor_search:
            continue
        h = pts[:, 2] - f
        counts = [int((np.abs(h - kk * p.rise) < p.level_tol).sum()) for kk in range(1, p.steps + 1)]
        score = (counts[-1] >= p.min_level_points,
                 sum(c >= p.min_other_points for c in counts), -abs(f + p.body_height))
        if best is None or score > best[0]:
            best = (score, f)
    return (best[1], measured) if best else (layer, measured)


def front_edge(xy, heading, p):
    """Nearest point along heading per lateral slice: (v_centres, u_front) arrays."""
    c, s = math.cos(heading), math.sin(heading)
    u = xy[:, 0] * c + xy[:, 1] * s
    v = -xy[:, 0] * s + xy[:, 1] * c
    b = np.floor(v / p.slice_w).astype(int)
    vs, us = [], []
    for k in np.unique(b):
        m = b == k
        if m.sum() < 2:
            continue
        vs.append((k + 0.5) * p.slice_w)
        us.append(np.percentile(u[m], 5))
    return np.array(vs), np.array(us)


def fit_heading(levels, p, step_deg=1.0):
    """Heading (into the stairs) that makes every tread's near and far edges sharpest along it:
    turned off its heading, a 0.95 m wide tread's edges smear along u. levels: list of (N, 2)
    xy arrays. Cost per level: (u p20 - p2) + (u p98 - p80). On the real stairs (23 Nov) this
    matched the riser normals within ~1 deg; the plain spread (IQR) was 7-12 deg off because
    the shiny middle of each tread leaves holes in the depth."""
    lim = p.max_yaw_deg
    levels = [np.asarray(xy, float) for xy in levels]

    def best_of(degs):
        angs = np.radians(degs)
        cs = np.stack([np.cos(angs), np.sin(angs)])        # (2, A): every heading at once
        cost = np.zeros(len(angs))
        for xy in levels:
            q = np.percentile(xy @ cs, (2, 20, 80, 98), axis=0)   # (4, A)
            cost += (q[1] - q[0]) + (q[3] - q[2])
        return float(degs[int(np.argmin(cost))])

    return math.radians(best_of(np.arange(-lim, lim + 1e-9, step_deg)))


def edge_check(chain, centre_v, uvh, p):
    """None if every tread is one unbroken surface across the stairs, else why not.

    Real treads run the full 95 cm between the handrails. The block-field obstacle (20 cm
    cubes at 12/24/36 cm) has the same heights in a patchwork: along a "tread" some 5 cm
    strips hold a cube of ANOTHER height. A real tread can have holes (shiny spots give no
    depth) but never another level inside it. Per tread: across the middle 2*edge_half_span,
    in the band one tread deep behind its front edge, count the strips where flat points at
    another height outnumber the tread's own (edge_max_foreign of them allowed), and the
    front edge must be one straight line over at least edge_min_run (holes bridged)."""
    u_all, v_all, h_all = uvh
    edges = np.arange(centre_v - p.edge_half_span, centre_v + p.edge_half_span + 1e-9, p.edge_bin)
    nb = len(edges) - 1
    for i, lvl in enumerate(chain):
        kk, v, u = lvl[0], lvl[4], lvl[6]
        bv = np.floor((v - edges[0]) / p.edge_bin).astype(int)
        inside = (bv >= 0) & (bv < nb)
        fronts, idx = [], []
        if inside.any():
            order = np.lexsort((u[inside], bv[inside]))       # by strip, then u
            bs, us = bv[inside][order], u[inside][order]
            starts = np.r_[0, np.nonzero(np.diff(bs))[0] + 1]
            ends = np.r_[starts[1:], len(bs)]
            for a0, b0 in zip(starts, ends):
                fronts.append(float(np.percentile(us[a0:b0], 5)))
                idx.append(int(bs[a0]))
        if len(fronts) < 3:
            return f'tread {kk} too little of it seen across the middle'
        f, idx = np.array(fronts), np.array(idx)
        uf = float(np.median(f))
        # foreign levels inside the tread band
        band = (u_all > uf + 0.03) & (u_all < uf + p.tread - 0.03)
        # another height = more than half a rise from THIS tread's measured height (the floor
        # estimate can be a few cm off when the floor is out of view, 24 Nov between mid/near)
        hk = lvl[3]
        other = band & (np.abs(h_all - hk) > p.edge_other_dh) & (np.abs(h_all - (hk - kk * p.rise)) > p.edge_other_dh)
        own_in = (u > uf + 0.03) & (u < uf + p.tread - 0.03)
        def strip_counts(vv, m):
            b_ = np.floor((vv[m] - edges[0]) / p.edge_bin).astype(int)
            return np.bincount(b_[(b_ >= 0) & (b_ < nb)], minlength=nb)
        no, nk = strip_counts(v_all, other), strip_counts(v, own_in)
        foreign = int(((no >= p.edge_min_pts) & (no > nk)).sum())
        if foreign > p.edge_max_foreign:
            return f'tread {kk} broken: another height in {foreign} of {nb} strips'
        if i == 0:
            continue        # the lowest tread's near edge can be the bottom of the image
        best = 0
        for c0 in f:
            ii = idx[np.abs(f - c0) <= p.edge_tol]
            start = ii[0]
            for x, y in zip(ii, ii[1:]):
                if y - x - 1 > p.edge_gap:
                    best = max(best, x - start + 1)
                    start = y
            best = max(best, ii[-1] - start + 1)
        if best * p.edge_bin < p.edge_min_run:
            return f'tread {kk} front edge straight for only {best * p.edge_bin:.2f} m'
    return None


def detect(depth, k, r, t, p=None):
    """StairSeen or None; why the frame failed in detect.last_why."""
    p = p or LevelParams()
    g = grid_points(depth, k, r, t, p)
    flat = flat_mask(g, p)
    pts = g[flat]
    keep = (pts[:, 0] > 0.15) & (pts[:, 0] < p.max_ahead) & (np.abs(pts[:, 1]) < p.max_side)
    pts = pts[keep]
    if len(pts) < p.min_level_points:
        detect.last_why = 'no flat surfaces'
        return None
    floor, _ = choose_floor(pts, p)
    h = pts[:, 2] - floor
    lv = {kk: pts[np.abs(h - kk * p.rise) < p.level_tol] for kk in range(1, p.steps + 1)}
    top = lv[p.steps]
    if len(top) < p.min_level_points:
        detect.last_why = f'no top level at {p.steps * p.rise:.2f} m ({len(top)} pts)'
        return None
    # lateral crop: the stairs are one corridor ~1 m wide; rails and side frames outside it
    # also show flat bits near 0.36 m (seen on the real stairs, 23 Nov)
    lower = [lv[kk] for kk in range(1, p.steps) if len(lv[kk]) >= p.min_other_points]
    ref = np.concatenate(lower) if lower else top
    centre = np.median(ref[:, :2], axis=0)
    ang0 = math.atan2(centre[1], centre[0]) if centre[0] > 0.2 else 0.0
    def crop(q, heading):
        c, s = math.cos(heading), math.sin(heading)
        v = -(q[:, 0] - centre[0]) * s + (q[:, 1] - centre[1]) * c
        return q[np.abs(v) < p.half_width]
    lv = {kk: crop(q, ang0) for kk, q in lv.items()}
    lower = [lv[kk][:, :2] for kk in range(1, p.steps) if len(lv[kk]) >= p.min_other_points]
    heading = fit_heading(lower if lower else [lv[p.steps][:, :2]], p)
    c, s = math.cos(heading), math.sin(heading)
    seen = []
    prev_u = None
    for kk in range(1, p.steps + 1):
        q = lv[kk]
        need = p.min_level_points if kk == p.steps else p.min_other_points
        u = q[:, 0] * c + q[:, 1] * s
        if prev_u is not None and len(q):
            m = (u > prev_u + 0.5 * p.tread) & (u < prev_u + 2.5 * p.tread)
            q, u = q[m], u[m]
        if len(q) < need:
            continue
        v = -q[:, 0] * s + q[:, 1] * c
        uf = float(np.percentile(u, 5))
        ub = float(np.percentile(u, 95))
        seen.append((kk, uf, int(len(q)), float(np.median(q[:, 2] - floor)), v, ub, u))
        prev_u = uf
    if not seen or seen[-1][0] != p.steps:
        detect.last_why = 'top level not in line with the lower steps'
        return None
    # keep the levels that are one tread apart in order, ending with the top
    chain = [seen[-1]]
    for lvl in reversed(seen[:-1]):
        gap = chain[0][1] - lvl[1]
        expect = (chain[0][0] - lvl[0]) * p.tread
        if abs(gap - expect) <= p.tread_tol * (chain[0][0] - lvl[0]):
            chain.insert(0, lvl)
    if len(chain) < 2:
        detect.last_why = 'top level alone (no lower step one tread nearer)'
        return None
    # height range: every tread at k * rise, every step one rise, the top in its range
    offs = [hh - kk * p.rise for kk, _, _, hh, *_ in chain]
    if max(offs) - min(offs) > 2 * p.height_tol or abs(float(np.mean(offs))) > 0.04:
        detect.last_why = ('tread heights ' + ', '.join(f'{lv[3]:.3f}' for lv in chain)
                           + f' not on a {p.rise:.2f} m staircase')
        return None
    for a_, b_ in zip(chain, chain[1:]):
        step = (b_[3] - a_[3]) / (b_[0] - a_[0])
        if abs(step - p.rise) > p.height_tol:
            detect.last_why = f'step {step:.3f} m between treads {a_[0]} and {b_[0]}'
            return None
    top_h = chain[-1][3]
    if not p.top_min <= top_h <= p.top_max:
        detect.last_why = f'top at {top_h:.3f} m, outside {p.top_min}-{p.top_max}'
        return None
    low_k, low_u = chain[0][0], chain[0][1]
    # Bottom riser from every edge seen: a lower tread's FAR edge is the next riser (never cut
    # off by the view while the tread above it is seen); a near edge can be (close up the
    # bottom of the image cuts the tread, sim: +7 cm at 0.45 m). Far edges first.
    far = [lvl[5] - lvl[0] * p.tread for lvl in chain[:-1]
           if any(o[0] == lvl[0] + 1 for o in chain)]
    dist = float(np.median(far)) if far else low_u - (low_k - 1) * p.tread
    allv = np.concatenate([lvl[4] for lvl in chain])
    lo, hi = np.percentile(allv, (3, 97))
    width = float(hi - lo + p.slice_w)
    tv = chain[-1][4]
    top_w = float(np.percentile(tv, 97) - np.percentile(tv, 3) + p.slice_w)
    if top_w < p.min_width:
        detect.last_why = f'top level only {top_w:.2f} m wide'
        return None
    lat_v = float((hi + lo) / 2)
    hp = pts[:, 2] - floor
    u_all = pts[:, 0] * c + pts[:, 1] * s
    v_all = -pts[:, 0] * s + pts[:, 1] * c
    bad = edge_check(chain, float((np.percentile(tv, 97) + np.percentile(tv, 3)) / 2),
                     (u_all, v_all, hp), p)
    if bad:
        detect.last_why = bad
        return None
    # nothing flat above the top on the stairs' footprint (a taller stepped obstacle)
    above = ((hp > p.above_lo) & (hp < p.above_hi) & (u_all > dist + 0.05) & (u_all < dist + 0.6)
             & (np.abs(v_all - lat_v) < 0.3))
    if above.sum() > p.above_max_points:
        detect.last_why = f'{int(above.sum())} flat points {p.above_lo}-{p.above_hi} m above the floor on the steps'
        return None
    # lateral of the stair centre line relative to base_link, perpendicular to heading
    det = StairSeen(distance=float(dist), yaw=float(heading), lateral=lat_v, width=width,
                    levels=[(a, round(b, 3), n, round(hh, 3)) for a, b, n, hh, *_ in chain],
                    floor_z=floor, inferred=low_k > 1)
    detect.last_why = 'ok'
    return det


detect.last_why = ''


class Confirm:
    """Per-frame detections -> confirmed stairs, without odometry.

    Robot-frame estimates are only compared within `window_sec` (the robot moves <= ~5 cm in
    0.6 s at 0.1 m/s). Confirmed once `need` of the last `window` frames (all within
    window_sec) agree with the newest hit: distance within dist_tol, heading within yaw_tol,
    lateral within lat_tol. A close-up frame that puts the bottom riser one tread further (the
    bottom tread left the view) disagrees and simply doesn't vote. The confirmed estimate is
    the median of the agreeing frames. It stays confirmed for hold_sec after the last
    agreement (a few missed frames don't flicker the status)."""

    def __init__(self, window=5, need=3, window_sec=1.2, dist_tol=0.08,
                 yaw_tol=math.radians(8), lat_tol=0.10, hold_sec=1.5):
        self.window, self.need, self.window_sec = window, need, window_sec
        self.dist_tol, self.yaw_tol, self.lat_tol, self.hold_sec = dist_tol, yaw_tol, lat_tol, hold_sec
        self.recent = []                    # (t, StairSeen or None)
        self.confirmed = None               # (t, StairSeen)

    def add(self, t, seen):
        self.recent = [(a, b) for a, b in self.recent if t - a <= self.window_sec][-(self.window - 1):]
        self.recent.append((t, seen))
        if seen is not None:
            agree = [s for _, s in self.recent if s is not None
                     and abs(s.distance - seen.distance) <= self.dist_tol
                     and abs(math.atan2(math.sin(s.yaw - seen.yaw), math.cos(s.yaw - seen.yaw))) <= self.yaw_tol
                     and abs(s.lateral - seen.lateral) <= self.lat_tol]
            if len(agree) >= self.need:
                med = StairSeen(
                    distance=float(np.median([s.distance for s in agree])),
                    yaw=float(np.median([s.yaw for s in agree])),
                    lateral=float(np.median([s.lateral for s in agree])),
                    width=float(np.median([s.width for s in agree])),
                    levels=seen.levels, floor_z=seen.floor_z,
                    inferred=sum(s.inferred for s in agree) * 2 > len(agree))
                self.confirmed = (t, med)
        if self.confirmed is not None and t - self.confirmed[0] > self.hold_sec:
            self.confirmed = None
        return self.state()

    def state(self):
        return self.confirmed[1] if self.confirmed is not None else None


@dataclass
class RailParams:
    """The handrails' unique shape (lidar, real stairs 23 Nov): along each side, the rail top
    climbs from ~0.5-0.6 m at the bottom riser to ~0.85 m over the top tread (u 0.4-0.7 m),
    then falls again: a triangle. Maze walls are flat-topped at ~0.77-0.81 m, curtain frames
    are posts with a bar across, props are single objects. Heights above the floor."""
    v_in: float = 0.30                  # m from the stairs' centre line (rails at ~+-0.48)
    v_out: float = 0.80
    bin_u: float = 0.10
    low_u: tuple = (-0.10, 0.25)        # u range (from the bottom riser) of the low rail end
    high_u: tuple = (0.30, 0.80)        # u range of the rail's peak
    low_max: float = 0.70               # rail top there at most this
    high_min: float = 0.78              # peak at least this
    min_climb: float = 0.15             # peak - low end
    min_bin_points: int = 5


def lidar_level(points_livox, r_level_base, r_base_lidar, t_base_lidar):
    """Livox points -> the same level frame as detect() (base_link origin, gravity z)."""
    p = np.asarray(points_livox, float) @ np.asarray(r_base_lidar).T + np.asarray(t_base_lidar)
    return p @ np.asarray(r_level_base).T


def rail_profile(pts_level, seen, side, rp):
    """Rail top height (98th pct above the floor) per u bin along one side, NaN = no points."""
    c, s_ = math.cos(seen.yaw), math.sin(seen.yaw)
    u = pts_level[:, 0] * c + pts_level[:, 1] * s_ - seen.distance
    v = -pts_level[:, 0] * s_ + pts_level[:, 1] * c - seen.lateral
    h = pts_level[:, 2] - seen.floor_z
    m = (side * v > rp.v_in) & (side * v < rp.v_out) & (h > 0.3) & (h < 1.2)
    edges = np.arange(rp.low_u[0], rp.high_u[1] + 1e-9, rp.bin_u)
    out = []
    for e in edges:
        mm = m & (u >= e) & (u < e + rp.bin_u)
        out.append(float(np.percentile(h[mm], 98)) if mm.sum() >= rp.min_bin_points else float('nan'))
    return edges, np.array(out)


def rails_check(pts_level, seen, rp=None):
    """(ok, why): the triangle-shaped handrails on BOTH sides of the stairs (see RailParams)."""
    rp = rp or RailParams()
    if pts_level is None or len(pts_level) == 0:
        return False, 'no lidar points'
    out = []
    for side, name in ((1, 'left'), (-1, 'right')):
        e, prof = rail_profile(pts_level, seen, side, rp)
        lo = prof[(e >= rp.low_u[0] - 1e-9) & (e < rp.low_u[1])]
        hi = prof[(e >= rp.high_u[0] - 1e-9) & (e < rp.high_u[1])]
        if np.all(np.isnan(lo)) or np.all(np.isnan(hi)):
            return False, f'no {name} handrail seen'
        low, high = float(np.nanmin(lo)), float(np.nanmax(hi))
        if low > rp.low_max or high < rp.high_min or high - low < rp.min_climb:
            return False, f'{name} side is not a stair rail (low {low:.2f}, peak {high:.2f} m)'
        out.append(f'{name} {low:.2f}->{high:.2f}')
    return True, 'rails ' + ', '.join(out)
