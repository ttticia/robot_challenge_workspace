"""Curtains: walls with a low gap at the bottom that the robot walks through (no ROS here).

A curtain hangs in a maze opening. To the wall band (wall_scan, 0.30-0.45 m above the floor)
it looks like a wall. What gives it away is the gap at its bottom: lidar beams that pass
under it hit the floor (or anything low) just behind it. A real wall can't do that. The
walls are 0.45 m tall and the lidar sits ~0.475 m above the floor, so a beam that clears
a wall top only comes down ~20 wall-distances further on, never within 1 m behind it.

detect_curtains(): points in odom (lidar, depth camera via depth_points(), or both) ->
curtain segments. It bins the points by bearing from the origin (the lidar, or the camera
with depth only) inside +-fov ahead. Each bin:
  face    nearest cluster of wall-band points (>= min_face within 6 cm), range r_w
  ground  median height of low points 0.05..ground_window m in front of the face (the
          local floor: ramps and mats move it); none there -> the lowest floor layer seen
          past the face, else floor_z
  gap     lowest face point (|r - r_w| < face_depth, more than 3 cm above the ground) minus
          the ground. That is the curtain's bottom edge, or the lowest a sensor sees there
  behind  floor returns (within floor_tol of the ground), behind_start..behind m past it
  -> 'wall' if the gap is < min_gap (the face reaches the floor; a wall reads 3-4 cm);
     else 'curtain' if behind >= min_behind and gap < max_gap; 'high' if seen under but the
     gap is >= max_gap (a hurdle or a hand rail); 'face' if nothing is seen under it (its
     lowest point may only be that sensor's lowest ray there: no claim);
     None if the bin has no face.
Neighbouring curtain and face bins (holes of up to max_hole_bins None / high bins; a wall
bin ends the run) whose face points lie on one line (line_tol) and span more than min_width
make a curtain, if its longest stretch of curtain bins alone is min_core (0.35 m) wide.
Range from base_link (Lite3 mounts, synthetic tests in test/test_curtains.py):
  lidar   ~1.2-2.6 m: its lowest beam (~21 deg down, 0.475 m up) only passes under a 5-15 cm
          gap from ~1 m from the lidar
  depth   ~0.7-2.0 m, +-40 deg: the camera (0.42 m up, 20 deg down, lowest ray 53 deg down)
          sees under the curtain from ~0.55 m, but needs ~0.7 m to see 0.7 m of its width
  both    ~0.6-2.6 m: the camera sees under the middle, the lidar the face's ends (width)
Below ~0.55 m no ray reaches under a 5-15 cm gap: a curtain and a wall look the same.

diagnose(): why nothing was found, for the log (user 2026-11-20): the widest run of
non-wall bins ahead and what stopped it - gap >= max_gap (a hurdle), too narrow, never seen
under. CurtainTracker.votes(): hits so far for a not yet confirmed detection.

CurtainTracker: votes over the last few detections. A confirmed curtain is kept for the
whole run, refined until the robot is within freeze_distance. curtain_detector keeps it in
the map frame, so SLAM corrects Fast-LIO drift and jumps for it.

Map override helpers: scan_curtains() (which curtains wall_scan drops: crossed or being
crossed), curtain_mask() / in_curtains() (points to drop from the wall scan), clear_curtains()
(occupancy grid with the curtains set free), invert_pose().

Sensor blockage (curtain draped over the sensor): lidar_blockage() / BlockageMonitor for the
lidar, depth_blockage() for the depth camera.
"""
from dataclasses import dataclass, field
import collections
import json
import math

import numpy as np

try:
    import cv2
except ImportError:          # median filter is optional
    cv2 = None


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class CurtainParams:
    max_gap: float = 0.15          # m; gap at the bottom must be smaller (to-do spec)
    # m; and at least this. A wall on the floor reads ~0.03-0.04 (face points start 3 cm
    # above the ground), so stray returns behind a wall must not make it a curtain.
    min_gap: float = 0.05
    # m; a bin seen under with a gap of candidate_min_gap..min_gap is 'low' (logged, never a
    # curtain or a candidate): practice-maze curtains hang 1-3 cm up (user 2026-11-22), but
    # extru walls read the same (floor seen through their side windows, test_curtains), so
    # 'low' bins can't be used until something else tells the two apart (to_do.md).
    candidate_min_gap: float = 0.01
    # m; runs of curtain-like bins whose facing ends are at most this far apart, on one
    # line, are one curtain: the practice-maze curtain is two halves with a slit between
    # (photos 2026-11-22), each ~0.45 m wide, under min_width alone. 0 = off (default):
    # joining also stitched side-wall pieces into a curtain along a corridor
    # (test_rolling_pin, 2026-11-22); needs a facing / doorway check first (to_do.md).
    slit_max: float = 0.0
    min_width: float = 0.60        # m; curtain must be wider (spec 0.7; the measured
    #                                width runs 5-10 cm short: folds, frame posts; 2026-11-20)
    band: tuple = (0.30, 0.45)     # m above the floor: wall band (= wall_scan band_low /
    #                                band_high; curtain_detector takes them from the launch)
    fov: float = math.radians(60)  # rad either side of the heading ("the wall in front")
    bin_width: float = math.radians(1.5)
    min_range: float = 0.25        # m from the origin (lidar, or the camera with depth only)
    max_range: float = 2.6
    min_face: int = 3              # band points that make a face in a bin
    face_depth: float = 0.08       # m; face points within this of r_w
    # m past the face where floor returns start to count. Close up, the floor seen under a
    # curtain is only ~0.3 x its distance deep (depth camera 0.42 m up, 10 cm gap), so it
    # starts right at the face; heights (floor_tol vs the face's > 3 cm) tell them apart.
    behind_start: float = 0.04
    behind: float = 1.0            # m behind the face searched for floor returns
    floor_tol: float = 0.05        # m; returns this close to the local ground count as floor
    min_behind: int = 3            # floor returns behind = beams went under
    ground_window: float = 0.40    # m in front of the face for the local ground
    ground_max: float = 0.28       # m above the floor: higher points aren't ground
    max_hole_bins: int = 1
    line_tol: float = 0.15         # m off the fitted line (fabric hangs in folds)
    min_bins: int = 4
    min_core: float = 0.35         # m of curtain bins in one stretch (see _segment)


@dataclass
class Curtain:
    """A curtain segment in a fixed frame. n points from where it was first seen to its far
    side; side +1 = crossing it along n.

    crossed: the robot has been walked through it (PASSED, or STUCK / ABORTED with base_link
    already past the line): it is really soft, so wall_scan may drop it. A false detection
    (a real wall with a low gap) never gets there, so it stays a wall in the costmaps.
    crossing: curtain_detector is walking the robot through it right now."""
    p1: np.ndarray
    p2: np.ndarray
    n: np.ndarray
    gap: float = 0.0
    id: int = 0
    passed: list = field(default_factory=list)   # sides (+1 / -1) already walked through
    crossed: bool = False
    crossing: bool = False

    @property
    def centre(self):
        return (self.p1 + self.p2) / 2

    @property
    def width(self):
        return float(np.linalg.norm(self.p2 - self.p1))

    @property
    def u(self):
        d = self.p2 - self.p1
        return d / max(np.linalg.norm(d), 1e-9)

    def signed(self, xy):
        """Distance of xy past the curtain line along n (negative = the side it was seen from)."""
        return float(np.dot(np.asarray(xy, float) - self.centre, self.n))

    def lateral(self, xy):
        """Offset of xy along the curtain from its centre."""
        return float(np.dot(np.asarray(xy, float) - self.centre, self.u))

    def distance(self, xy):
        """Distance from xy to the segment."""
        xy = np.asarray(xy, float)
        t = np.clip(np.dot(xy - self.p1, self.u), 0.0, self.width)
        return float(np.linalg.norm(xy - (self.p1 + t * self.u)))

    def side_of(self, xy):
        """The side xy would cross the curtain towards: +1 if xy is before it along n."""
        return 1 if self.signed(xy) < 0 else -1

    def transformed(self, tx, ty, th):
        c, s = math.cos(th), math.sin(th)
        r = np.array([[c, -s], [s, c]])
        t = np.array([tx, ty])
        return Curtain(r @ self.p1 + t, r @ self.p2 + t, r @ self.n, self.gap, self.id,
                       list(self.passed), self.crossed, self.crossing)

    def to_dict(self):
        return {'id': self.id, 'p1': [round(float(v), 3) for v in self.p1],
                'p2': [round(float(v), 3) for v in self.p2],
                'n': [round(float(v), 4) for v in self.n], 'gap': round(self.gap, 3),
                'width': round(self.width, 3), 'passed': list(self.passed),
                'crossed': bool(self.crossed), 'crossing': bool(self.crossing)}

    @classmethod
    def from_dict(cls, d):
        return cls(np.array(d['p1'], float), np.array(d['p2'], float), np.array(d['n'], float),
                   float(d.get('gap', 0.0)), int(d.get('id', 0)), list(d.get('passed', [])),
                   bool(d.get('crossed', False)), bool(d.get('crossing', False)))


def invert_pose(tx, ty, th):
    """(tx, ty, th) of the inverse 2D transform: Curtain.transformed(*invert_pose(*T)) undoes
    Curtain.transformed(*T)."""
    c, s = math.cos(th), math.sin(th)
    return -(c * tx + s * ty), -(-s * tx + c * ty), -th


def curtains_to_json(curtains, frame='odom'):
    return json.dumps({'frame': frame, 'curtains': [c.to_dict() for c in curtains]})


def curtains_from_json(text):
    """(frame, [Curtain]) from curtains_to_json(); ('', []) if unreadable."""
    try:
        d = json.loads(text)
        return d.get('frame', 'odom'), [Curtain.from_dict(c) for c in d.get('curtains', [])]
    except (ValueError, KeyError, TypeError):
        return '', []


# --------------------------------------------------------------------------- detection

def _face_range(r_sorted, min_face, window=0.06):
    """Nearest r with >= min_face values in [r, r + window], or None."""
    n = len(r_sorted)
    if n < min_face:
        return None
    hi = np.searchsorted(r_sorted, r_sorted + window, side='right')
    ok = np.flatnonzero(hi - np.arange(n) >= min_face)
    return float(r_sorted[ok[0]]) if len(ok) else None


def classify_bins(pts, lidar_xy, yaw, floor_z, p=None):
    """Per-bearing-bin classification (see the module doc).

    Returns a list of dicts (one per bin, left to right... i.e. by increasing bearing) with
    'bearing' (rad, robot heading = 0), 'kind' ('curtain' / 'high' / 'wall' / None),
    'r' (face range), 'xy' (face point, same frame as pts), 'gap', 'behind'.
    """
    p = p or CurtainParams()
    pts = np.asarray(pts, float)
    rel = pts[:, :2] - np.asarray(lidar_xy, float)[:2]
    r = np.hypot(rel[:, 0], rel[:, 1])
    b = np.arctan2(rel[:, 1], rel[:, 0]) - yaw
    b = (b + math.pi) % (2 * math.pi) - math.pi
    h = pts[:, 2] - floor_z
    n_bins = int(round(2 * p.fov / p.bin_width))
    keep = (np.abs(b) < p.fov) & (r > p.min_range - p.ground_window) & \
        (r < p.max_range + p.behind)
    r, b, h = r[keep], b[keep], h[keep]
    k = np.clip(((b + p.fov) / p.bin_width).astype(int), 0, n_bins - 1)
    order = np.argsort(k, kind='stable')
    r, b, h, k = r[order], b[order], h[order], k[order]
    starts = np.searchsorted(k, np.arange(n_bins + 1))
    out = []
    for i in range(n_bins):
        bearing = -p.fov + (i + 0.5) * p.bin_width
        cell = {'bearing': bearing, 'kind': None, 'r': None, 'xy': None, 'gap': None,
                'behind': 0}
        out.append(cell)
        s, e = starts[i], starts[i + 1]
        if e - s == 0:
            continue
        rb, hb = r[s:e], h[s:e]
        band = (hb >= p.band[0]) & (hb <= p.band[1]) & (rb >= p.min_range) & \
            (rb <= p.max_range)
        rw = _face_range(np.sort(rb[band]), p.min_face)
        if rw is None:
            continue
        cell['r'] = rw
        a = bearing + yaw
        cell['xy'] = (lidar_xy[0] + rw * math.cos(a), lidar_xy[1] + rw * math.sin(a))
        front = (rb > rw - p.ground_window) & (rb < rw - 0.05) & (hb < p.ground_max)
        past = (rb > rw + p.behind_start) & (rb < rw + p.behind)
        if front.sum() >= 2:
            ground = float(np.median(hb[front]))
        else:
            # No floor in front (too close for the lidar, or the camera's lower edge): the
            # floor seen past the face (under a curtain), else the global floor (floor_z).
            # Its lowest layer only: side walls past the face reach down to it too.
            low = hb[past & (np.abs(hb) < p.floor_tol + 0.12)]
            ground = 0.0
            if len(low) >= p.min_behind:
                g0 = float(np.percentile(low, 10))
                ground = float(np.median(low[low < g0 + p.floor_tol]))
        face = (np.abs(rb - rw) < p.face_depth) & (hb > ground + 0.03) & (hb <= p.band[1])
        if not face.any():
            continue
        gap = float(hb[face].min()) - ground
        behind = int((past & (np.abs(hb - ground) < p.floor_tol)).sum())
        cell['gap'], cell['behind'] = gap, behind
        if gap < p.candidate_min_gap or gap < p.min_gap and behind < p.min_behind:
            cell['kind'] = 'wall'      # the face reaches the floor
        elif gap < p.min_gap:
            cell['kind'] = 'low'       # seen under, but too low to tell from a wall
        elif behind >= p.min_behind:
            cell['kind'] = 'curtain' if gap < p.max_gap else 'high'
        else:
            # Nothing seen under it: its lowest point may only be the lowest ray that sensor
            # has at that range (lidar < ~1.2 m, camera < ~0.6 m), not the real bottom.
            cell['kind'] = 'face'
    return out


def _segment(cells, p):
    """Curtain (p1, p2, gap) from a run of curtain cells, or None. End bins off the line
    (side walls seen at a slant next to the curtain) are trimmed first."""
    while len(cells) >= p.min_bins:
        xy = np.array([c['xy'] for c in cells])
        m = xy.mean(axis=0)
        _, _, vt = np.linalg.svd(xy - m)
        u = vt[0]
        t = (xy - m) @ u
        off = np.abs((xy - m) @ vt[1])
        if off.max() <= p.line_tol:
            break
        cells = cells[1:] if off[0] >= off[-1] else cells[:-1]
    else:
        return None
    # Bin centres -> outer edges: half a bin's arc at each end.
    # Core: the longest stretch of curtain bins (holes up to max_hole_bins). It alone must be
    # min_core wide; 'face' bins only widen it. Without this, a few false curtain bins where
    # a side wall meets the face, at both ends of a 'face' run, would add up to a curtain.
    idx = [i for i, c in enumerate(cells) if c['kind'] == 'curtain']
    groups, g = [], []
    for i in idx:
        if g and i - g[-1] > p.max_hole_bins + 1:
            groups.append(g)
            g = []
        g.append(i)
    if g:
        groups.append(g)
    core = max(groups, key=len) if groups else []
    if len(core) < p.min_bins:
        return None
    a, b = np.array(cells[core[0]]['xy']), np.array(cells[core[-1]]['xy'])
    if np.linalg.norm(b - a) + p.bin_width * cells[core[0]]['r'] < p.min_core:
        return None
    gaps = [cells[i]['gap'] for i in core]
    pad = 0.5 * p.bin_width * np.array([cells[0]['r'], cells[-1]['r']])
    lo, hi = t.min() - pad[0], t.max() + pad[1]
    return m + lo * u, m + hi * u, float(np.median(gaps))


def join_runs(runs, p):
    """Runs of bins (left to right) with the facing ends of neighbours at most slit_max
    apart joined into one: the two halves of a split curtain, with the slit between them
    (bins that looked through it are not in any run, or form a short run further away,
    which is dropped). Whether the result is one straight line is _segment's check."""
    if p.slit_max <= 0:
        return [r for r in runs if r]
    out = []
    for run in runs:
        if not run:
            continue
        if out:
            a = out[-1]
            gap = math.hypot(run[0]['xy'][0] - a[-1]['xy'][0], run[0]['xy'][1] - a[-1]['xy'][1])
            if gap <= p.slit_max:
                out[-1] = a + run
                continue
            # a short run seen through the slit (further than both halves): skip it
            if len(out) >= 2 and len(a) <= 3:
                b = out[-2]
                far = min(c['r'] for c in a) > max(b[-1]['r'], run[0]['r']) + 0.3
                gap2 = math.hypot(run[0]['xy'][0] - b[-1]['xy'][0],
                                  run[0]['xy'][1] - b[-1]['xy'][1])
                if far and gap2 <= p.slit_max:
                    out.pop()
                    out[-1] = b + run
                    continue
        out.append(list(run))
    return out


def detect_curtains(pts, lidar_xy, yaw, floor_z, p=None, cells_out=None):
    """[Curtain] in the frame of pts (odom) from one batch of points. n points away from
    the lidar. cells_out (a list): gets classify_bins()'s bins, for diagnose()."""
    p = p or CurtainParams()
    cells = classify_bins(pts, lidar_xy, yaw, floor_z, p)
    if cells_out is not None:
        cells_out.extend(cells)
    runs, cur, holes = [], [], 0
    for c in cells + [{'kind': 'wall'}]:          # sentinel closes the last run
        kind = c['kind']
        # 'face' bins (bottom unseen) only widen a run of curtain bins: close up the depth
        # camera (+-40 deg) sees under the middle while the lidar (+-60) sees the face's ends.
        # _segment() needs min_bins curtain bins in a run.
        if kind in ('curtain', 'face'):
            if cur and math.hypot(c['xy'][0] - cur[-1]['xy'][0],
                                  c['xy'][1] - cur[-1]['xy'][1]) > \
                    0.15 + (holes + 1) * p.bin_width * c['r']:
                runs.append(cur)
                cur = []
            cur.append(c)
            holes = 0
        elif kind in (None, 'high') and cur and holes < p.max_hole_bins:
            # one 'high' bin is a hole too: where the camera's view ends, a bin can get its
            # face from the lidar (bottom cut off) and its floor from the camera
            holes += 1
        else:
            if cur:
                runs.append(cur)
            cur, holes = [], 0
    found = []
    lxy = np.asarray(lidar_xy, float)[:2]
    for run in join_runs(runs, p):
        seg = _segment(run, p)
        if seg is None:
            continue
        p1, p2, gap = seg
        if np.linalg.norm(p2 - p1) <= p.min_width:
            continue
        d = p2 - p1
        n = np.array([d[1], -d[0]]) / np.linalg.norm(d)
        if np.dot((p1 + p2) / 2 - lxy, n) < 0:
            n = -n
        found.append(Curtain(p1, p2, n, gap))
    return found


def diagnose(cells, p=None):
    """Why no curtain came out of these bins, as one log line, or None (nothing curtain-like).

    Looks at the widest run of bins with a face that isn't a wall ('curtain' / 'high' /
    'face', holes up to max_hole_bins) that holds at least 2 bins seen under ('curtain' or
    'high') - plain walls too close to see under ('face' only) say nothing, every close wall
    is like that. Reasons, first that fits: most seen-under bins have gap >= max_gap (a
    hurdle / the curtain hangs too high); fewer than min_bins curtain bins; narrower than
    min_width; curtain core narrower than min_core.
    """
    p = p or CurtainParams()
    runs, cur, holes = [], [], 0
    for c in list(cells) + [{'kind': 'wall'}]:
        if c['kind'] in ('curtain', 'high', 'face'):
            cur.append(c)
            holes = 0
        elif c['kind'] is None and cur and holes < p.max_hole_bins:
            holes += 1
        else:
            if cur:
                runs.append(cur)
            cur, holes = [], 0
    runs = [r for r in runs if sum(c['kind'] in ('curtain', 'high') for c in r) >= 2]
    if not runs:
        return None
    run = max(runs, key=len)
    cur_b = [c for c in run if c['kind'] == 'curtain']
    high = [c for c in run if c['kind'] == 'high']
    width = float(np.hypot(run[-1]['xy'][0] - run[0]['xy'][0],
                           run[-1]['xy'][1] - run[0]['xy'][1]))
    rng = float(np.median([c['r'] for c in run]))
    where = (f'{width:.2f} m wide, {rng:.2f} m away, bearing '
             f'{math.degrees((run[0]["bearing"] + run[-1]["bearing"]) / 2):+.0f} deg')
    if len(high) >= max(len(cur_b), 2):
        g = float(np.median([c['gap'] for c in high]))
        return (f'wall with a {g * 100:.0f} cm gap under it ({where}): gap >= max_gap '
                f'{p.max_gap * 100:.0f} cm, taken as a hurdle (lab curtain: curtain_max_gap:=0.25)')
    if len(cur_b) < p.min_bins:
        return (f'seen under only {len(cur_b) + len(high)} of {len(run)} bins ({where}); '
                f'{p.min_bins} curtain bins needed - get 0.7-2 m away, facing it')
    if width <= p.min_width:
        return f'curtain-like but only {width:.2f} m wide < min_width {p.min_width:.2f} ({where})'
    return (f'curtain-like ({where}, {len(cur_b)} curtain bins) but its curtain core is '
            f'< {p.min_core:.2f} m or off a straight line')


def near_miss_segment(cells, p=None, max_gap=0.30, min_width=0.60):
    """(p1, p2, gap) of the widest run of non-wall bins seen under (see diagnose) whose
    median seen-under gap is < max_gap and that spans > min_width, else None. For the
    curtain inspection (user 2026-11-20): a wall with a suitable gap that isn't proven a
    curtain yet - too narrow, too few bins, or a gap just over max_gap."""
    p = p or CurtainParams()
    runs, cur, holes = [], [], 0
    for c in list(cells) + [{'kind': 'wall'}]:
        if c['kind'] in ('curtain', 'high', 'face'):
            cur.append(c)
            holes = 0
        elif c['kind'] is None and cur and holes < p.max_hole_bins:
            holes += 1
        else:
            if cur:
                runs.append(cur)
            cur, holes = [], 0
    best = None
    for run in join_runs(runs, p):
        under = [c for c in run if c['kind'] in ('curtain', 'high')]
        if len(under) < 2:
            continue
        gap = float(np.median([c['gap'] for c in under]))
        p1, p2 = np.array(run[0]['xy'], float), np.array(run[-1]['xy'], float)
        width = float(np.linalg.norm(p2 - p1))
        if gap >= max_gap or width <= min_width:
            continue
        if best is None or width > best[3]:
            best = (p1, p2, gap, width)
    return None if best is None else best[:3]


def floor_at_line(pts, p1, p2, n, floor_z, near=0.45, behind=0.8, clear=0.10, search=0.15,
                  tol=0.03, min_pts=6, end_margin=0.10):
    """Floor height (frame of pts) right at a curtain line, measured from the points
    themselves, or None.

    The floor the robot stands on is a poor reference for a curtain: the curtain cell can be
    5-10 cm lower than the start plate, and the lidar never sees the floor under the robot
    (nearest floor ~1.4 m ahead; stairs_lidar.floor_under), so the floor handed in is often
    the start plate's. Here: points along the line (end_margin m in from each end: the side
    walls the curtain hangs between reach down to the floor there), `clear` m or more off it
    (the fabric's folds), within `search` of floor_z; their lowest dense layer (10th
    percentile + tol). In front of the line (up to `near` m: the side the robot crosses
    from) first, else behind it (up to `behind` m). pts / p1 / p2 / n as in gap_at_line.
    """
    pts = np.asarray(pts, float)
    p1, p2, n = (np.asarray(v, float)[:2] for v in (p1, p2, n))
    width = float(np.linalg.norm(p2 - p1))
    if width < 1e-6 or len(pts) == 0:
        return None
    rel = pts[:, :2] - p1
    t, d = rel @ ((p2 - p1) / width), rel @ n
    z = pts[:, 2]
    m = min(end_margin, 0.25 * width)
    ok = (t >= m) & (t <= width - m) & (np.abs(z - floor_z) <= search)
    for side in ((d <= -clear) & (d >= -near), (d >= clear) & (d <= behind)):
        zs = z[ok & side]
        if len(zs) >= min_pts:
            low = zs[zs <= np.percentile(zs, 10) + tol]
            return float(np.median(low))
    return None


def gap_at_line(pts, p1, p2, n, floor_z, face_depth=0.10, floor_tol=0.04, behind=0.8,
                top=0.45, slices=8, local_floor=True):
    """Gap under a known curtain line, measured while the robot lies down (user 2026-11-20):
    the sensors sit ~0.15-0.25 m up, about level with the hem, so they see straight under it.

    pts (N, 3) in the frame of p1, p2 (curtain ends) and n (normal pointing away from the
    robot); floor_z the floor's height there (local_floor: only a starting point, re-measured
    at the line with floor_at_line when the points show the floor there). The curtain is cut
    into `slices` along its width. Per slice: hem = lowest point on the curtain (within
    face_depth of the line, > floor + floor_tol, <= top), floor behind = points 0.05..behind
    past the line within floor_tol of the floor. Returns (gap, coverage, slices_seen): gap =
    median hem height of the slices that have both (None if none), coverage = that fraction
    of the slices.
    """
    pts = np.asarray(pts, float)
    p1, p2, n = (np.asarray(v, float)[:2] for v in (p1, p2, n))
    width = float(np.linalg.norm(p2 - p1))
    if width < 1e-6 or len(pts) == 0:
        return None, 0.0, 0
    if local_floor:
        f = floor_at_line(pts, p1, p2, n, floor_z, behind=behind)
        if f is not None:
            floor_z = f
    u = (p2 - p1) / width
    rel = pts[:, :2] - p1
    t = rel @ u                                  # along the curtain
    d = rel @ n                                  # past the line (+ = behind it)
    h = pts[:, 2] - floor_z
    inside = (t >= 0) & (t <= width)
    face = inside & (np.abs(d) < face_depth) & (h > floor_tol) & (h <= top)
    floor = inside & (d > 0.05) & (d < behind) & (np.abs(h) < floor_tol)
    k = np.clip((t / width * slices).astype(int), 0, slices - 1)
    hems = []
    for i in range(slices):
        fi = face & (k == i)
        if fi.sum() >= 3 and (floor & (k == i)).sum() >= 3:
            hems.append(float(np.percentile(h[fi], 5)))
    if not hems:
        return None, 0.0, 0
    return float(np.median(hems)), len(hems) / slices, len(hems)


def depth_points(depth_m, k, r, t, stride=4, min_depth=0.15, max_depth=2.0, median=5):
    """Points (N, 3) from a depth image in metres (0 / nan = none), in the frame (r, t) maps
    the optical frame into (p = r @ p_optical + t). k = (fx, fy, cx, cy). A median filter
    first takes out most flying pixels at the curtain's bottom edge."""
    d = np.asarray(depth_m, np.float32)
    d = np.where(np.isfinite(d), d, 0.0).astype(np.float32)
    if cv2 is not None and median in (3, 5):
        d = cv2.medianBlur(np.ascontiguousarray(d), median)
    fx, fy, cx, cy = k
    rows = np.arange(0, d.shape[0], stride)
    cols = np.arange(0, d.shape[1], stride)
    z = d[np.ix_(rows, cols)].astype(np.float64)
    ok = (z >= min_depth) & (z <= max_depth)
    x = ((cols[None, :] - cx) / fx * z)[ok]
    y = ((rows[:, None] - cy) / fy * z)[ok]
    pts = np.stack([x, y, z[ok]], -1)
    return pts @ np.asarray(r, float).T + np.asarray(t, float)


class CurtainTracker:
    """Votes curtain detections over time; confirmed ones are kept (with an id)."""

    def __init__(self, window=3.0, need=3, pos_tol=0.25, yaw_tol=math.radians(20),
                 freeze_distance=1.0, alpha=0.3):
        self.window, self.need, self.pos_tol, self.yaw_tol = window, need, pos_tol, yaw_tol
        self.freeze_distance, self.alpha = freeze_distance, alpha
        self.tracks = []          # {'c': Curtain, 'hits': deque of times, 'confirmed': bool}
        self.next_id = 1

    def _match(self, c):
        best, best_d = None, self.pos_tol
        for t in self.tracks:
            d = float(np.linalg.norm(t['c'].centre - c.centre))
            ang = abs(np.dot(t['c'].u, c.u))          # |cos| of the angle between the lines
            if d < best_d and ang > math.cos(self.yaw_tol):
                best, best_d = t, d
        return best

    def add(self, now, curtains, robot_xy=None):
        """Feed one detection batch. Returns the curtains newly confirmed by it."""
        new = []
        for c in curtains:
            t = self._match(c)
            if t is None:
                self.tracks.append({'c': c, 'hits': collections.deque([now]),
                                    'confirmed': False})
                continue
            t['hits'].append(now)
            frozen = robot_xy is not None and t['c'].distance(robot_xy) < self.freeze_distance
            if not frozen:
                old = t['c']
                p1, p2 = c.p1, c.p2
                if np.dot(p2 - p1, old.p2 - old.p1) < 0:
                    p1, p2 = p2, p1
                a = self.alpha
                old.p1 = (1 - a) * old.p1 + a * p1
                old.p2 = (1 - a) * old.p2 + a * p2
                old.gap = (1 - a) * old.gap + a * c.gap
                d = old.p2 - old.p1
                nn = np.array([d[1], -d[0]]) / max(np.linalg.norm(d), 1e-9)
                old.n = nn if np.dot(nn, old.n) >= 0 else -nn
            while t['hits'] and now - t['hits'][0] > self.window:
                t['hits'].popleft()
            if not t['confirmed'] and len(t['hits']) >= self.need:
                t['confirmed'] = True
                t['c'].id = self.next_id
                self.next_id += 1
                new.append(t['c'])
        # Drop unconfirmed tracks that stopped getting hits.
        self.tracks = [t for t in self.tracks if t['confirmed'] or
                       (t['hits'] and now - t['hits'][-1] <= self.window)]
        return new

    def confirm(self, c, now):
        """Add c as confirmed (an inspection measured its gap); returns the stored curtain
        (an existing track it matches, else a new one) with its id."""
        t = self._match(c)
        if t is None:
            t = {'c': c, 'hits': collections.deque([now]), 'confirmed': False}
            self.tracks.append(t)
        if not t['confirmed']:
            t['confirmed'] = True
            t['c'].id = self.next_id
            self.next_id += 1
        return t['c']

    def votes(self, c):
        """(hits in the window, confirmed) of the track c matches, or (0, False)."""
        t = self._match(c)
        return (len(t['hits']), t['confirmed']) if t is not None else (0, False)

    @property
    def confirmed(self):
        return [t['c'] for t in self.tracks if t['confirmed']]


# --------------------------------------------------------------------------- map override

def in_curtains(xy, curtains, half_thickness=0.12, extend=0.03):
    """Mask of points (N, >=2) that lie inside any curtain's rectangle."""
    xy = np.asarray(xy, float)
    mask = np.zeros(len(xy), bool)
    for c in curtains:
        d = xy[:, :2] - c.p1
        t = d @ c.u
        off = d @ c.n
        mask |= (t >= -extend) & (t <= c.width + extend) & (np.abs(off) <= half_thickness)
    return mask


def scan_curtains(curtains):
    """The curtains wall_scan drops: walked through once (crossed) or being walked through
    now (crossing). Confirmed but never crossed ones stay walls in the scan."""
    return [c for c in curtains if c.crossed or c.crossing]


def curtain_mask(xy, curtains, robot_xy=None, half_thickness=0.12, extend=0.03, near=1.0,
                 wide=0.40, inset=0.05):
    """Mask of points to drop from the wall scan: in_curtains(), plus, for a curtain within
    `near` m of the robot, a strip `wide` m either side of its line. That one stops `inset` m
    short of the ends, so the walls the curtain hangs between stay. Fabric the robot pushes
    or drags bulges well past the 0.12 m strip, right around the robot."""
    mask = in_curtains(xy, curtains, half_thickness, extend)
    if robot_xy is None or wide <= half_thickness:
        return mask
    xy = np.asarray(xy, float)
    for c in curtains:
        if c.distance(robot_xy) >= near or c.width <= 2 * inset:
            continue
        d = xy[:, :2] - c.p1
        t = d @ c.u
        mask |= (t >= inset) & (t <= c.width - inset) & (np.abs(d @ c.n) <= wide)
    return mask


def clear_curtains(grid, curtains, half_thickness=0.10, extend=0.0):
    """Copy of grid (core.Grid, same frame as the curtains) with curtain cells set free."""
    from maze_nav.core import Grid
    if not curtains:
        return grid
    data = grid.data.copy()
    h, w = data.shape
    res = grid.resolution
    for c in curtains:
        corners = [c.p1 + s * half_thickness * c.n + e for s in (-1, 1)
                   for e in (-extend * c.u, (c.width + extend) * c.u)]
        corners = np.array(corners)
        lo = np.floor((corners.min(axis=0) - np.array(grid.origin)) / res).astype(int)
        hi = np.ceil((corners.max(axis=0) - np.array(grid.origin)) / res).astype(int)
        c0, r0 = max(lo[0], 0), max(lo[1], 0)
        c1, r1 = min(hi[0], w - 1), min(hi[1], h - 1)
        if c1 < c0 or r1 < r0:
            continue
        cols, rows = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
        centres = np.stack([grid.origin[0] + (cols.ravel() + 0.5) * res,
                            grid.origin[1] + (rows.ravel() + 0.5) * res], axis=1)
        inside = in_curtains(centres, [c], half_thickness + res / 2, extend + res / 2)
        data[rows.ravel()[inside], cols.ravel()[inside]] = 0
    return Grid(data, grid.resolution, grid.origin)


def wall_between(grid, a, b, stop_short=0.25, start_after=0.3, occupied=65):
    """Is there a map wall on the straight line from a (the robot) to b (a curtain
    candidate's middle), both in the grid's frame? The last `stop_short` m before b are left
    out (the candidate itself is on the map as a wall) and so are the first `start_after` m
    (the robot's own cell, where its footprint or self-returns may mark cells).

    Curtain candidates seen through a gap, or through see-through sim walls (2026-11-21: one
    2 m away behind a wall), can't be reached for the inspection: the robot drove at the wall
    until the 75 s timeout. Cells off the map count as no wall."""
    a, b = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
    d = float(np.linalg.norm(b - a))
    if d <= start_after + stop_short:
        return False
    res = grid.resolution
    t = np.arange(start_after, d - stop_short, res / 2) / d
    pts = a + t[:, None] * (b - a)
    cols = np.floor((pts[:, 0] - grid.origin[0]) / res).astype(int)
    rows = np.floor((pts[:, 1] - grid.origin[1]) / res).astype(int)
    h, w = grid.data.shape
    ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    return bool((grid.data[rows[ok], cols[ok]] >= occupied).any())


# --------------------------------------------------------------------------- blockage

def lidar_blockage(pts, lidar_xyz, base_xy, base_yaw, body=(0.42, 0.32, 0.28), near=0.5):
    """(near, far) point counts of one lidar cloud (pts in odom): near = within `near` m of
    the lidar, robot body box (back, front, half width in base_link) left out."""
    pts = np.asarray(pts, float)
    if len(pts) == 0:
        return 0, 0
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    dx, dy = pts[:, 0] - base_xy[0], pts[:, 1] - base_xy[1]
    bx, by = c * dx + s * dy, -s * dx + c * dy
    back, front, half = body
    in_body = (bx > -back) & (bx < front) & (np.abs(by) < half) & \
        (pts[:, 2] < lidar_xyz[2] + 0.05)
    d = np.linalg.norm(pts - np.asarray(lidar_xyz, float), axis=1)
    n_near = int(((d < near) & ~in_body).sum())
    n_far = int(((d >= near) & ~in_body).sum())
    return n_near, n_far


class BlockageMonitor:
    """Lidar blocked = many near returns, or far returns well below their usual count.

    The Mid-360 drops returns closer than 0.1 m, so fabric right on the window shows up as
    missing points rather than near ones. The usual far count is the median of clouds
    passed to update(..., learn=True) (normal driving, no curtain close)."""

    def __init__(self, near_frac=0.3, far_frac=0.4, history=50, min_history=5):
        self.near_frac, self.far_frac = near_frac, far_frac
        self.far_counts = collections.deque(maxlen=history)
        self.min_history = min_history

    @property
    def baseline(self):
        if len(self.far_counts) < self.min_history:
            return None
        return float(np.median(self.far_counts))

    def update(self, n_near, n_far, learn):
        """(blocked, why) for one cloud."""
        total = n_near + n_far
        frac = n_near / total if total else 1.0
        base = self.baseline
        if frac > self.near_frac:
            blocked, why = True, f'{frac:.0%} of returns within reach'
        elif base is not None and n_far < self.far_frac * base:
            blocked, why = True, f'{n_far} far returns (usually {base:.0f})'
        else:
            blocked, why = False, f'{frac:.0%} near, {n_far} far'
        if learn and not blocked:
            self.far_counts.append(n_far)
        return blocked, why


def depth_blockage(depth_m, near=0.30, min_clear=0.25, step=4):
    """(blocked, clear fraction) of a depth image in metres (0 / nan = no reading): clear =
    a reading further than `near`. Fabric on the lens reads as no depth or very close."""
    d = np.asarray(depth_m, dtype=np.float32)[::step, ::step]
    clear = np.isfinite(d) & (d > near)
    frac = float(clear.mean()) if d.size else 0.0
    return frac < min_clear, frac
