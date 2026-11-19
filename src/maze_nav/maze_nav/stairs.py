"""Stairs from a depth image, and the approach that stops the robot square in front of them.

Pure numpy (cv2 only for an optional median filter), no ROS: stair_detector.py is the node.

Frames
  camera optical: x right, y down, z forward (sensor_msgs/Image).
  level frame: origin at base_link, x forward along the robot's heading, y left, z up along
  gravity (odom z). The body's roll and pitch are taken out and its yaw kept
  (level_transform). The floor under the robot is at z ~ -standing_height.

Detection works on RISERS, not treads. Seen from 0.42 m up, the floor 2 m ahead is only ~12 deg
below the line of sight, so one grid row (4 px) covers ~10 cm of floor: a 15 cm tread gets
1-2 rows. Risers face the camera and stay dense. detect_stairs:
  1. depth (median-filtered) -> a point every stride_x x stride_y pixels -> level frame
  2. normals from the organised grid (cross product of row and column tangents over
     +-normal_step_x / _y grid points, then a 3x3 mean). Vertical face: |n_z| < sin(25 deg).
  3. candidate stair directions: histogram of the inward azimuth of vertical points (1 deg
     bins, within +-max_yaw of straight ahead), strongest peaks. A wall facing the robot is a
     candidate too; step 5 rejects it.
  4. per candidate theta: u = distance along theta, v = sideways. Vertical points facing
     theta are clustered by u (histogram peaks) into faces: u, height range [lo, hi] above
     the floor, lateral span.
  5. stairs = >= min_risers faces in a row where each face is first_rise_min..rise_max tall,
     the middles of consecutive faces are rise_min..rise_max apart in height, averaging at
     least mean_rise_min over the chain (the rise: the
     smoothed normals trim a few cm off both ends of a face, the middles don't move) and
     tread_min..tread_max apart along u, each face starts where the one before ends
     (-chain_tol .. +chain_gap), the rises agree within rise_tol, the lateral spans overlap,
     and the tread between two faces (where the depth has one) is at their join height. A
     0.45 m wall, a single mat edge or a ramp never qualifies; nor does anything narrower
     than min_stair_width, nearer than min_distance or beyond max_yaw.
  6. theta is refined within +-refine_deg to make the faces thinnest along u. The first
     face's u is the distance to the stairs. The lateral centre is the middle of the faces'
     span, and centre_ok is false if that span touches the image edge or is narrower than
     min_width.
  Close up (base_link < ~0.5 m from the bottom riser) that riser is below the camera's view and
  the first face seen is the 2nd step. Steps below the view are NOT inferred: without a
  measured floor height (no live feed of it), +-5 cm of body height decides whether one is
  missing; tried on the live robot 2026-11-18 it added a step that wasn't there.

Tested on synthetic depth (test/stair_scene.py, incl. the example maze), on the real lab
frames of ROS_bags/perception_test (no stairs; 3 false hits in 1432, all while standing up)
and on live frames of real stairs (test/data/live_stairs_close.npz), ~30 ms a frame.

StairTracker fuses per-frame detections (in odom) once vote_need of the last vote_window agree;
refine_lock keeps the driven estimate steady close up (it was locked from further away, where
the bottom riser is in view). The bottom riser is assumed visible from a distance.

camera_from_link(): the depth camera's pose from base_link -> camera_link plus the Orbbec's
fixed optical rotation. On 2026-11-18 the robot's /tf_static had a second, unpitched parent
for camera_depth_optical_frame (base_link, 0.25/0/0.05, no 20 deg pitch), and TF resolved
through it: every height came out skewed.
Approach turns the robot to face the stairs, drives it onto their centre line up to
stop_distance from the first riser, then squares it up (perpendicular to the step edges).
"""
import math
from dataclasses import dataclass, field

import numpy as np

try:
    import cv2
except ImportError:                     # the filter is optional; detection works without it
    cv2 = None


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class StairParams:
    stride_x: int = 4                   # px between grid columns
    stride_y: int = 2                   # px between grid rows (risers are short in the image)
    median_ksize: int = 5               # 0 = no median filter
    min_depth: float = 0.15             # m, camera z
    max_depth: float = 3.0
    normal_step_x: int = 3              # grid points either side for the row tangent
    normal_step_y: int = 2              # grid points either side for the column tangent
    max_range: float = 2.5              # m ahead of base_link, along theta
    roi_half_width: float = 1.5         # m sideways
    standing_height: float = 0.35       # base_link above the floor
    floor_band: float = 0.10            # floor estimate within this of -standing_height
    floor_max_x: float = 1.5            # m ahead; floor points nearer than this
    min_floor_points: int = 40
    vertical_max_tilt_deg: float = 25.0     # normal this close to horizontal = vertical face
    horizontal_max_tilt_deg: float = 15.0   # normal this close to up = horizontal surface
    max_yaw_deg: float = 60.0           # stair direction within this of the robot's heading
    facing_tol_deg: float = 12.0        # face normals within this of theta belong to it
    candidates: int = 3                 # directions tried per frame
    min_face_points: int = 12
    face_bin: float = 0.01              # m, u histogram
    face_smooth: float = 0.03           # m, half width of the box smoothing
    face_half_window: float = 0.06      # m either side of a peak
    min_height: float = -0.25           # faces considered from this far below the floor
    max_height: float = 0.75            # ... to this far above it
    rise_min: float = 0.06              # per step; a faint first step must not break the chain
    mean_rise_min: float = 0.075        # whole chain: 6-7 cm chains were real-lab false hits
    rise_max: float = 0.22
    first_rise_min: float = 0.03
    rise_tol: float = 0.05
    chain_tol: float = 0.04             # a face may start this far below the one before ends
    chain_gap: float = 0.08             # ... or this far above (face ends come out short)
    tread_min: float = 0.10
    tread_max: float = 0.50
    tread_tol: float = 0.05
    min_tread_points: int = 5
    front_depth: float = 0.25           # m of ground in front of the first riser for front_seen
    min_risers: int = 2
    min_width: float = 0.35             # m, narrower riser span = no lateral centre
    min_stair_width: float = 0.30       # m, narrower = not stairs (real lab: 0.15 m clutter)
    min_distance: float = 0.30          # m; nearer is under the robot's own front (0.27 m)
    edge_margin: float = 0.03           # fraction of image width counted as the image edge
    refine_deg: float = 6.0
    stray_frac: float = 0.25            # a face with < this x the points of the face before it
    #                                     that doesn't fit the chain is skipped, not the end of
    #                                     it (a ~15-point smear behind the bottom riser's top
    #                                     at 1.5 m started the stairs one tread late)
    min_support_frac: float = 0.3       # directions with < this x the points of the best are
    #                                     dropped (a 190-point 4-face ghost beat 2300 points)
    infer_bottom: bool = True           # a riser below the view, see detect_stairs
    missing_lo: float = 0.06            # m: first face starts this far above a MEASURED floor


@dataclass
class Face:
    u: float                            # m along theta from base_link
    lo: float                           # m above the floor
    hi: float
    vlo: float                          # m sideways (left +)
    vhi: float
    n: int
    col_min: int                        # image columns it covers
    col_max: int
    spread: float = 0.0                 # m, std of u
    idx: np.ndarray = field(repr=False, default=None)

    @property
    def rise(self):
        """Seen height of the face (a few cm short of the real riser)."""
        return self.hi - self.lo

    @property
    def mid(self):
        return (self.hi + self.lo) / 2


@dataclass
class Detection:
    yaw: float                          # rad, stair direction (into the stairs) in the level frame
    distance: float                     # m, base_link to the first riser along yaw
    lateral: float                      # m, stair centre line sideways of base_link (left +)
    centre_ok: bool                     # lateral is trustworthy
    risers: int
    rise: float                         # m, mean of the fully seen risers
    tread: float                        # m, mean riser spacing
    width: float                        # m, riser span
    floor_z: float                      # level-frame z of the floor used
    thickness: float = 0.0              # m, mean spread of the faces along yaw (smaller = better aligned)
    span_lo: float = 0.0                # m, riser span sideways of base_link (left +), along
    span_hi: float = 0.0                #   the stair direction: 0 inside = stairs straight ahead
    front_seen: bool = True             # ground seen just in front of the first riser (info only:
    #                                     often false close up, where the floor leaves the view)
    first_rise: float = 0.0             # m, bottom step's rise (from the 2nd face; 0 = unknown)
    floor_measured: bool = False        # floor_z measured (camera floor points or given), not
    #                                     the standing_height default
    inferred_bottom: bool = False       # distance is a riser below the view (detect_stairs)
    support: int = 0                    # face points in the chain
    faces: list = field(repr=False, default_factory=list)

    def summary(self):
        return (f'{self.risers} risers {self.rise * 100:.0f} cm / {self.tread * 100:.0f} cm, '
                f'{self.distance:.2f} m at {math.degrees(self.yaw):+.0f} deg, '
                f'{self.width:.2f} m wide' + ('' if self.centre_ok else ' (edge cut off)')
                + (' (bottom riser inferred)' if self.inferred_bottom else ''))


# ---------------------------------------------------------------- geometry
def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# camera_link -> camera_depth_optical_frame on the Orbbec (camera_link -> camera_depth_frame is
# the identity, then the usual x-right/y-down/z-forward optical rotation; /tf_static 2026-11-14).
R_LINK_OPTICAL = quat_to_matrix(-0.5, 0.5, -0.5, 0.5)


def camera_from_link(r_base_link, t_base_link):
    """(R, t) of the depth optical frame in base_link from camera_link's pose in base_link."""
    return np.asarray(r_base_link) @ R_LINK_OPTICAL, np.asarray(t_base_link, float)


def mount_disagreement(cam_a, cam_b):
    """(deg, m) between two camera poses in base_link (rotation angle, translation)."""
    r = np.asarray(cam_a[0]).T @ np.asarray(cam_b[0])
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2))))
    return ang, float(np.linalg.norm(np.asarray(cam_a[1]) - np.asarray(cam_b[1])))


def level_transform(r_odom_base, r_base_cam, t_base_cam):
    """(R, t, yaw): camera optical -> level frame, and base_link's yaw in odom.

    r_odom_base: 3x3 rotation of base_link in odom (only roll/pitch are used here).
    r_base_cam, t_base_cam: static camera optical pose in base_link.
    """
    yaw = math.atan2(r_odom_base[1, 0], r_odom_base[0, 0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    tilt = rz @ r_odom_base
    return tilt @ r_base_cam, tilt @ np.asarray(t_base_cam, float), yaw


def depth_to_metres(depth):
    """uint16 (mm) or float (m) -> float32 metres, 0 = no data."""
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * 0.001
    d = depth.astype(np.float32)
    d[~np.isfinite(d)] = 0.0
    return d


def grid_points(depth_m, k, p):
    """Organised camera-frame points (rows, cols, 3) and their pixel columns.

    k = (fx, fy, cx, cy). Invalid points are nan.
    """
    fx, fy, cx, cy = k
    d = depth_m
    if cv2 is not None and p.median_ksize in (3, 5):
        d = cv2.medianBlur(np.ascontiguousarray(d, np.float32), p.median_ksize)
    rows = np.arange(0, d.shape[0], p.stride_y)
    cols = np.arange(0, d.shape[1], p.stride_x)
    z = d[np.ix_(rows, cols)].astype(np.float64)
    z[(z < p.min_depth) | (z > p.max_depth)] = np.nan
    x = (cols[None, :] - cx) / fx * z
    y = (rows[:, None] - cy) / fy * z
    return np.stack([x, y, z], -1), cols


def grid_normals(pts, p):
    """Unit normals of an organised grid, pointing towards the camera; nan where unknown."""
    sx, sy = p.normal_step_x, p.normal_step_y
    n = np.full(pts.shape, np.nan)
    tx = pts[sy:-sy, 2 * sx:] - pts[sy:-sy, :-2 * sx]
    ty = pts[2 * sy:, sx:-sx] - pts[:-2 * sy, sx:-sx]
    c = np.cross(tx, ty)
    # Occlusion edges: tangents far longer than the pixel footprint allows.
    z = pts[sy:-sy, sx:-sx, 2]
    lim_x = np.maximum(0.10, 12.0 * 2 * sx * p.stride_x * z / 370.0)
    lim_y = np.maximum(0.10, 12.0 * 2 * sy * p.stride_y * z / 370.0)
    bad = (np.linalg.norm(tx, axis=-1) > lim_x) | (np.linalg.norm(ty, axis=-1) > lim_y)
    c[bad] = np.nan
    n[sy:-sy, sx:-sx] = c
    # 3x3 mean (nan-aware), then normalise.
    ok = np.isfinite(n).all(-1)
    acc = np.where(ok[..., None], n, 0.0)
    norm = np.linalg.norm(acc, axis=-1, keepdims=True)
    acc = np.divide(acc, norm, out=np.zeros_like(acc), where=norm > 0)
    cnt = ok.astype(float)
    s = np.zeros_like(acc)
    sc = np.zeros_like(cnt)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            s += np.roll(np.roll(acc, dy, 0), dx, 1)
            sc += np.roll(np.roll(cnt, dy, 0), dx, 1)
    s[sc < 5] = np.nan
    s[~ok] = np.nan
    s /= np.linalg.norm(s, axis=-1, keepdims=True)
    flip = np.einsum('ijk,ijk->ij', s, pts) > 0          # point towards the camera
    s[flip] *= -1
    return s


# ---------------------------------------------------------------- faces and chains
def _peaks(signal, min_value, min_sep):
    """Indices of local maxima >= min_value, strongest first, at least min_sep apart."""
    s = signal
    cand = np.where((s >= min_value) & (s >= np.roll(s, 1)) & (s > np.roll(s, -1)))[0]
    cand = cand[np.argsort(-s[cand], kind='stable')]
    keep = []
    for i in cand:
        if all(abs(i - j) >= min_sep for j in keep):
            keep.append(i)
    return keep


def find_faces(u, h, v, cols, p):
    """Cluster vertical points (already facing theta) by u into Faces, nearest first."""
    if len(u) < p.min_face_points:
        return []
    nb = int(p.max_range / p.face_bin) + 2
    b = np.clip((u / p.face_bin).astype(int), 0, nb - 1)
    hist = np.bincount(b, minlength=nb).astype(float)
    k = max(1, int(round(p.face_smooth / p.face_bin)))
    sm = np.convolve(hist, np.ones(2 * k + 1), 'same')
    peaks = sorted(_peaks(sm, p.min_face_points, int(0.6 * p.tread_min / p.face_bin)))
    faces = []
    centres = [(i + 0.5) * p.face_bin for i in peaks]
    for j, c in enumerate(centres):
        w = p.face_half_window
        if j > 0:
            w = min(w, (c - centres[j - 1]) / 2)
        lo_w = w
        w = p.face_half_window
        if j + 1 < len(centres):
            w = min(w, (centres[j + 1] - c) / 2)
        sel = np.where((u > c - lo_w) & (u < c + w))[0]
        if len(sel) < p.min_face_points:
            continue
        hs, vs = h[sel], v[sel]
        faces.append(Face(u=float(np.median(u[sel])),
                          lo=float(np.percentile(hs, 3)), hi=float(np.percentile(hs, 97)),
                          vlo=float(np.percentile(vs, 3)), vhi=float(np.percentile(vs, 97)),
                          n=len(sel), col_min=int(cols[sel].min()),
                          col_max=int(cols[sel].max()), spread=float(np.std(u[sel])), idx=sel))
    return faces


def tread_height(u, v, h, a, b, p):
    """Median height of horizontal points between two faces, None if too few."""
    sel = (u > a.u + 0.03) & (u < b.u - 0.03) & \
        (v > max(a.vlo, b.vlo)) & (v < min(a.vhi, b.vhi))
    if sel.sum() < p.min_tread_points:
        return None
    return float(np.median(h[sel]))


def find_chain(faces, p, tread_check=None):
    """Longest run of faces that looks like stairs (see module doc), [] if none.

    tread_check(a, b) -> tread height between faces a and b, or None if unknown.
    """
    def fits(chain, f):
        prev = chain[-1]
        # Face heights come out a few cm short at both ends (normals are smoothed over the
        # step's corners), so the rise is the spacing of the face middles, and a face
        # may start up to chain_gap above where the one below ends.
        step = f.mid - prev.mid
        if not (p.first_rise_min <= f.rise <= p.rise_max) or \
                not (p.rise_min <= step <= p.rise_max):
            return False
        if not (-p.chain_tol <= f.lo - prev.hi <= p.chain_gap):
            return False
        # The first step is left out: its rise may differ (a 10 cm bottom step under 13 cm
        # ones) and face 0 may be cut off by the view.
        steps = [b.mid - a.mid for a, b in zip(chain[1:], chain[2:])]
        if steps and abs(step - float(np.mean(steps))) > p.rise_tol:
            return False
        overlap = min(f.vhi, prev.vhi) - max(f.vlo, prev.vlo)
        if overlap < 0.5 * min(f.vhi - f.vlo, prev.vhi - prev.vlo):
            return False
        if tread_check is not None:
            t = tread_check(prev, f)
            if t is not None and abs(t - (prev.hi + f.lo) / 2) > p.tread_tol:
                return False
        return True

    best = []
    for i, f0 in enumerate(faces):
        if not (p.first_rise_min <= f0.rise <= p.rise_max):
            continue
        chain = [f0]
        for f in faces[i + 1:]:
            prev = chain[-1]
            gap = f.u - prev.u
            if gap < p.tread_min:
                continue                        # a stray cluster too close behind: skip it
            if gap > p.tread_max:
                break
            if not fits(chain, f):
                if f.n < p.stray_frac * prev.n:
                    continue                    # a weak stray cluster (smear on a tread edge)
                break
            chain.append(f)
        if len(chain) > len(best):
            best = chain
    return best if len(best) >= p.min_risers else []


# ---------------------------------------------------------------- detection
class Cloud:
    """Level-frame points of one frame with what detect_stairs needs."""

    def __init__(self, depth, k, r, t, p):
        pts, cols = grid_points(depth_to_metres(depth), k, p)
        nrm = grid_normals(pts, p)
        width = depth.shape[1]
        ok = np.isfinite(pts).all(-1) & np.isfinite(nrm).all(-1)
        pts, nrm = pts[ok], nrm[ok]
        self.cols = np.broadcast_to(cols[None, :], ok.shape)[ok]
        self.xyz = pts @ np.asarray(r).T + np.asarray(t)
        self.nrm = nrm @ np.asarray(r).T
        self.width = width
        keep = (self.xyz[:, 0] > 0.1) & (self.xyz[:, 0] < p.max_range + 0.3) & \
            (np.abs(self.xyz[:, 1]) < p.roi_half_width)
        self.xyz, self.nrm, self.cols = self.xyz[keep], self.nrm[keep], self.cols[keep]


def estimate_floor(cloud, p, measured=False):
    """Level-frame z of the floor near the robot (default: -standing_height).
    measured=True: (z, True if floor points were found rather than the default)."""
    z0 = -p.standing_height
    z = cloud.xyz[:, 2]
    sel = (cloud.nrm[:, 2] > math.cos(math.radians(p.horizontal_max_tilt_deg))) & \
        (cloud.xyz[:, 0] < p.floor_max_x) & (np.abs(z - z0) < p.floor_band)
    if sel.sum() < p.min_floor_points:
        return (z0, False) if measured else z0
    f = float(np.median(z[sel]))
    return (f, True) if measured else f


def detect_stairs(depth, k, r, t, p=None, cloud=None, floor_z=None):
    """Detection or None. depth: HxW uint16 mm or float m; k = (fx, fy, cx, cy);
    (r, t): camera optical -> level frame (level_transform).

    floor_z: level-frame z of a measured floor (stair_detector: the lidar height map's floor,
    which remembers the floor close up where the camera no longer sees it); None = from this
    frame's floor points, else -standing_height.

    Bottom riser below the view (p.infer_bottom): close up the first face found can be the
    2nd riser. It is inferred one tread nearer only on evidence: the floor is measured, the
    first face starts >= missing_lo above it, and a tread is seen in front of that face at the
    height where it starts (>= rise_min above the floor). Body height alone never decides (on
    2026-11-18 that added a step that wasn't there)."""
    p = p or StairParams()
    cloud = cloud or Cloud(depth, k, r, t, p)
    if len(cloud.xyz) == 0:
        return None
    if floor_z is not None and abs(floor_z + p.standing_height) <= 2 * p.floor_band:
        floor, measured = float(floor_z), True
    else:
        floor, measured = estimate_floor(cloud, p, measured=True)
    xyz, nrm = cloud.xyz, cloud.nrm
    h = xyz[:, 2] - floor
    nz = nrm[:, 2]
    vert = (np.abs(nz) < math.sin(math.radians(p.vertical_max_tilt_deg))) & \
        (h > p.min_height) & (h < p.max_height)
    horiz = nz > math.cos(math.radians(p.horizontal_max_tilt_deg))
    if vert.sum() < p.min_face_points:
        return None
    az = np.arctan2(-nrm[vert, 1], -nrm[vert, 0])        # inward: into the face
    max_yaw = math.radians(p.max_yaw_deg)
    deg = np.degrees(az[np.abs(az) < max_yaw])
    if len(deg) < p.min_face_points:
        return None
    nbins = int(2 * p.max_yaw_deg) + 1
    hist = np.bincount(np.clip((deg + p.max_yaw_deg).astype(int), 0, nbins - 1),
                       minlength=nbins).astype(float)
    sm = np.convolve(hist, np.ones(5), 'same')
    cands = _peaks(sm, 2 * p.min_face_points, 10)[:p.candidates]
    vi = np.where(vert)[0]
    hi_ = np.where(horiz)[0]

    dets = []
    for c in cands:
        theta = math.radians(c + 0.5 - p.max_yaw_deg)
        det = _try_direction(theta, xyz, h, nrm, vi, hi_, cloud, floor, p)
        for _ in range(3):              # refine: the angle that makes the faces thinnest
            if det is None:
                break
            pts = np.concatenate([f.idx for f in det.faces])
            owner = np.concatenate([np.full(len(f.idx), i) for i, f in enumerate(det.faces)])
            sub = xyz[vi[pts], :2]
            best_t, best_s = det.yaw, None
            for dt in np.radians(np.linspace(-p.refine_deg, p.refine_deg, 25)):
                tt = det.yaw + dt
                uu = sub[:, 0] * math.cos(tt) + sub[:, 1] * math.sin(tt)
                s = sum(np.std(uu[owner == i]) for i in range(len(det.faces)))
                if best_s is None or s < best_s:
                    best_t, best_s = tt, s
            if abs(wrap(best_t - det.yaw)) < math.radians(0.3):
                break
            refined = _try_direction(best_t, xyz, h, nrm, vi, hi_, cloud, floor, p)
            if refined is None or refined.risers < det.risers:
                break
            det = refined
        if det is not None:
            dets.append(det)
    if not dets:
        return None
    # Most risers; then the sharpest faces. Not the most points: a direction a few deg off
    # still collects the real risers (facing_tol), but smears them along u. Directions with
    # only a fraction of the best one's points are ghosts (riser ends seen obliquely).
    top = max(d.support for d in dets)
    dets = [d for d in dets if d.support >= p.min_support_frac * top]
    det = max(dets, key=lambda d: (d.risers, -d.thickness))
    det.floor_measured = measured
    return _infer_bottom(det, xyz, h, nrm, p) if measured and p.infer_bottom else det


def _infer_bottom(det, xyz, h, nrm, p):
    """det with the bottom riser added one tread nearer if a tread is seen in front of its
    first face at the height that face starts (see detect_stairs), else det unchanged."""
    f0 = det.faces[0]
    if f0.lo < p.missing_lo or len(det.faces) < 2:
        return det
    ct, st = math.cos(det.yaw), math.sin(det.yaw)
    horiz = nrm[:, 2] > math.cos(math.radians(p.horizontal_max_tilt_deg))
    u = xyz[:, 0] * ct + xyz[:, 1] * st
    v = -xyz[:, 0] * st + xyz[:, 1] * ct
    sel = horiz & (u > f0.u - det.tread + 0.03) & (u < f0.u - 0.03) & \
        (v > f0.vlo) & (v < f0.vhi)
    if sel.sum() < p.min_tread_points:
        return det
    t = float(np.median(h[sel]))
    if t < p.rise_min or abs(t - f0.lo) > p.tread_tol or t > p.rise_max:
        return det
    det.distance = f0.u - det.tread
    det.risers += 1
    det.first_rise = t
    det.inferred_bottom = True
    det.front_seen = False
    return det


def _try_direction(theta, xyz, h, nrm, vi, hi_, cloud, floor, p):
    ct, st = math.cos(theta), math.sin(theta)
    az = np.arctan2(-nrm[vi, 1], -nrm[vi, 0])
    facing = np.abs(np.arctan2(np.sin(az - theta), np.cos(az - theta))) < \
        math.radians(p.facing_tol_deg)
    fv = vi[facing]
    u = xyz[fv, 0] * ct + xyz[fv, 1] * st
    v = -xyz[fv, 0] * st + xyz[fv, 1] * ct
    inside = (u > 0.1) & (u < p.max_range)
    sel = np.where(inside)[0]
    faces = find_faces(u[sel], h[fv][sel], v[sel], cloud.cols[fv][sel], p)
    for f in faces:                                     # indices into vi
        f.idx = np.where(facing)[0][sel[f.idx]]
    if len(faces) < p.min_risers:
        return None
    hu = xyz[hi_, 0] * ct + xyz[hi_, 1] * st
    hv = -xyz[hi_, 0] * st + xyz[hi_, 1] * ct
    hh = h[hi_]
    chain = find_chain(faces, p, lambda a, b: tread_height(hu, hv, hh, a, b, p))
    if not chain:
        return None
    steps = [b.mid - a.mid for a, b in zip(chain, chain[1:])]
    # Rise from the steps above the bottom one when there are any: face 0's middle mixes the
    # bottom step's rise (it may be lower, e.g. 10 cm under 13 cm steps) with the next one's.
    upper = steps[1:] or steps
    rise = float(np.mean(upper))
    vlo = float(np.median([f.vlo for f in chain]))
    vhi = float(np.median([f.vhi for f in chain]))
    width = vhi - vlo
    if width < p.min_stair_width or chain[0].u < p.min_distance or \
            abs(theta) > math.radians(p.max_yaw_deg) or rise < p.mean_rise_min:
        return None
    m = p.edge_margin * cloud.width
    cut = any(f.col_min <= m or f.col_max >= cloud.width - 1 - m for f in chain)
    f0 = chain[0]
    front = (hu > f0.u - p.front_depth) & (hu < f0.u - 0.03) & (hv > f0.vlo) & (hv < f0.vhi) & \
        (hh < f0.lo + p.chain_gap)
    front_seen = int(front.sum()) >= p.min_tread_points
    tread = float(np.mean(np.diff([f.u for f in chain])))
    # Face 1 spans [r1, r1 + r2] above the floor: its middle less half a rise is r1.
    first_rise = chain[1].mid - rise / 2 if len(steps) >= 2 else 0.0
    return Detection(yaw=wrap(theta), distance=f0.u, lateral=(vlo + vhi) / 2,
                     centre_ok=(not cut) and width >= p.min_width, risers=len(chain),
                     rise=rise, tread=tread,
                     width=width, floor_z=floor, faces=chain,
                     thickness=float(np.mean([f.spread for f in chain])),
                     front_seen=front_seen, span_lo=vlo, span_hi=vhi,
                     first_rise=first_rise, support=int(sum(f.n for f in chain)))


# ---------------------------------------------------------------- odom bookkeeping
@dataclass
class StairEstimate:
    """Stairs in odom: first riser on the line p.d = along, centre line p.n = centre
    (d = (cos heading, sin heading) into the stairs, n = d turned 90 deg left)."""
    heading: float
    along: float
    centre: float = None                # None = lateral position unknown
    risers: int = 0
    rise: float = 0.0
    source: str = 'camera'              # 'camera', 'lidar' or 'both' (stairs_lidar.fuse)
    anchored: bool = False              # lidar: bottom riser seen rising from the measured floor

    def d(self):
        return np.array([math.cos(self.heading), math.sin(self.heading)])

    def n(self):
        return np.array([-math.sin(self.heading), math.cos(self.heading)])

    def distance(self, xy):
        """m from xy to the first riser, along the stair direction."""
        return self.along - float(np.dot(xy, self.d()))

    def lateral(self, xy):
        """m xy is left of the centre line (0 if the centre is unknown)."""
        return 0.0 if self.centre is None else float(np.dot(xy, self.n())) - self.centre

    def edge_point(self, xy):
        """Middle of the first riser (or the point on it straight ahead of xy)."""
        c = self.centre if self.centre is not None else float(np.dot(xy, self.n()))
        return self.d() * self.along + self.n() * c

    def close_to(self, other, pos_tol, yaw_tol):
        if abs(wrap(self.heading - other.heading)) > yaw_tol:
            return False
        return abs(self.along - float(np.dot(other.d() * other.along, self.d()))) <= pos_tol


def to_odom(det, xy, yaw):
    """Detection (level frame at base_link pose xy, yaw in odom) -> StairEstimate."""
    heading = wrap(yaw + det.yaw)
    est = StairEstimate(heading=heading, along=0.0, risers=det.risers, rise=det.rise)
    xy = np.asarray(xy, float)
    est.along = float(np.dot(xy, est.d())) + det.distance
    if det.centre_ok:
        est.centre = float(np.dot(xy, est.n())) + det.lateral
    return est


class StairTracker:
    """Confirms stairs once vote_need of the last vote_window detections (within max_age s)
    agree, and fuses those."""

    def __init__(self, window=6, need=3, pos_tol=0.15, yaw_tol=math.radians(10), max_age=3.0,
                 nearer_need=0, nearer_range=(0.10, 0.50)):
        """nearer_need > 0: a group of >= nearer_need detections nearer_range m nearer than
        the largest group (same direction) wins over it. Missing the bottom riser (out of
        view, or a short 10 cm step) starts the stairs one tread late; the reverse (a riser
        that isn't there, in front of real stairs) needs a whole extra riser chain. Stopping
        early is the safe mistake: one tread late puts the robot's front on the bottom step."""
        self.window, self.need = window, need
        self.pos_tol, self.yaw_tol, self.max_age = pos_tol, yaw_tol, max_age
        self.nearer_need, self.nearer_range = nearer_need, nearer_range
        self.hist = []                  # (t, StairEstimate or None)

    def add(self, t, est):
        """est None = a processed frame without stairs (it counts against them)."""
        self.hist.append((t, est))
        self.hist = self.hist[-self.window:]

    def estimate(self, t):
        """Fused estimate of the largest group of agreeing detections, None if < need."""
        recent = [e for (s, e) in self.hist if e is not None and t - s <= self.max_age]
        best = []
        for ref in recent:
            group = [e for e in recent if ref.close_to(e, self.pos_tol, self.yaw_tol)]
            if len(group) > len(best):
                best = group
        if len(best) < self.need:
            return None
        if self.nearer_need:
            ref = best[0]
            near = None
            for e in recent:
                ahead = ref.along - float(np.dot(e.d() * e.along, ref.d()))
                if abs(wrap(e.heading - ref.heading)) > self.yaw_tol or \
                        not self.nearer_range[0] <= ahead <= self.nearer_range[1]:
                    continue
                group = [g for g in recent if e.close_to(g, self.pos_tol, self.yaw_tol)]
                key = (len(group), ahead)
                if len(group) >= self.nearer_need and (near is None or key > near[0]):
                    near = (key, group)
            if near is not None:
                best = near[1]
        hd = math.atan2(sum(math.sin(e.heading) for e in best),
                        sum(math.cos(e.heading) for e in best))
        out = StairEstimate(heading=hd, along=0.0,
                            risers=max(e.risers for e in best),
                            rise=float(np.mean([e.rise for e in best])),
                            source=best[-1].source, anchored=any(e.anchored for e in best))
        d, n = out.d(), out.n()
        out.along = float(np.mean([np.dot(e.d() * e.along, d) for e in best]))
        cs = [np.dot(e.edge_point(e.d() * e.along), n) for e in best if e.centre is not None]
        out.centre = float(np.mean(cs)) if cs else None
        return out

    def clear(self):
        self.hist = []


def in_front(est, xy, det=None, max_offset=0.35, margin=0.05):
    """(ok, offset): is the robot in front of the stairs, not beside them?

    Seen from a neighbouring cell the stairs can be close along their own direction while
    the robot is beside them (example maze: from the studded mat, 0.8 m off their centre
    line); lining up from there ends square but beside the stairs, or blocked by a wall.
    - centre known: |offset from the centre line| <= max_offset (stairs 0.87 m wide).
    - centre unknown: the robot's own line along the stairs passes through this frame's
      riser span (det), `margin` inside it. No det: (False, None).
    offset is the distance from the centre line (None if unknown).
    """
    xy = np.asarray(xy, float)
    if est.centre is not None:
        off = abs(est.lateral(xy))
        return off <= max_offset, off
    if det is None:
        return False, None
    return det.span_lo + margin <= 0.0 <= det.span_hi - margin, None


def refine_lock(lock, found, xy, freeze_distance, pos_tol, yaw_tol):
    """The estimate to drive to: `found` (the tracker's newest) replaces `lock` only if it
    agrees (pos_tol, yaw_tol) and the robot is further than freeze_distance from the first
    riser. Closer, the camera loses the bottom riser (it sees the floor only from ~0.5 m ahead
    of base_link), so a frame could start the stairs one tread late. A lateral centre, once
    known, is kept."""
    if found is None or lock.distance(np.asarray(xy, float)) <= freeze_distance or \
            not found.close_to(lock, pos_tol, yaw_tol):
        return lock
    if found.centre is None:
        found.centre = lock.centre
    return found


# ---------------------------------------------------------------- the approach
class WallCheck:
    """The robot's footprint against the SLAM occupancy grid, for the stair line-up.

    Same rules as maze_explorer: footprint box (back, front, half width; m) plus `margin`,
    turns checked every 2 deg of their sweep (core.max_turn). Turns and the stop pose fall
    back to `min_margin` when `margin` doesn't fit: in the cell before the example maze's
    stairs the rear corners' 0.437 m turning circle is 1 cm short of the side wall (0.427 m
    from the cell centre), as in any 0.9 m corridor. The per-tick check (safe) always uses
    min_margin: it is the last line of defence, it must only stop real contact. The line-up drives in odom and
    the grid is in map: `offset` = (x, y, yaw) of odom in map (TF map -> odom), applied to
    every pose. Unknown cells count as free. Walls within ~0.5 m beside the robot that the
    lidar never saw from further away are not on the map: this only knows what SLAM mapped.
    """

    def __init__(self, grid, box=(0.37, 0.27, 0.23), margin=0.03, min_margin=0.01,
                 offset=(0.0, 0.0, 0.0), horizon=0.5, dt=0.1):
        self.grid, self.box, self.margin, self.min_margin = grid, box, margin, min_margin
        self.offset, self.horizon, self.dt = offset, horizon, dt

    def margins(self):
        return (self.margin,) if self.min_margin >= self.margin else (self.margin,
                                                                       self.min_margin)

    def to_map(self, xy, yaw):
        ox, oy, oyaw = self.offset
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * xy[0] - s * xy[1], oy + s * xy[0] + c * xy[1]), wrap(yaw + oyaw)

    def overlap(self, xy, yaw, margin=None):
        from maze_nav.core import footprint_overlap
        mxy, myaw = self.to_map(xy, yaw)
        return footprint_overlap(self.grid, mxy, myaw, self.box,
                                 self.margin if margin is None else margin)

    def pose_clear(self, xy, yaw, margin=None):
        return self.overlap(xy, yaw, margin) == 0

    def turn_clear(self, xy, yaw, angle, margin=None):
        """The whole on-the-spot turn by `angle` (rad, +left) keeps the footprint clear."""
        from maze_nav.core import max_turn
        if abs(angle) < 1e-6:
            return True
        mxy, myaw = self.to_map(xy, yaw)
        m = self.margin if margin is None else margin
        return max_turn(self.grid, mxy, myaw, self.box, 1.0 if angle > 0 else -1.0,
                        abs(angle), m) >= abs(angle) - 1e-6

    def free_ahead(self, xy, yaw, max_d):
        from maze_nav.core import free_distance_ahead
        mxy, myaw = self.to_map(xy, yaw)
        return free_distance_ahead(self.grid, mxy, myaw, self.box, max_d, self.min_margin)

    def _keeps_clear(self, xy, yaw, v, w, now_overlap):
        """Driving (v, w) for `horizon` s: clear all the way, or (already touching a wall)
        never deeper into it than now."""
        x, y, a = float(xy[0]), float(xy[1]), yaw
        for _ in range(max(1, int(round(self.horizon / self.dt)))):
            a = a + w * self.dt
            x, y = x + v * self.dt * math.cos(a), y + v * self.dt * math.sin(a)
            if self.overlap((x, y), a, self.min_margin) > now_overlap:
                return False
        return True

    def safe(self, xy, yaw, v, w):
        """(v, w, why): the command if its next `horizon` s stay clear, else the part of it
        that does (turn only, or drive only), else (0, 0). why = '' if unchanged."""
        if v == 0.0 and w == 0.0:
            return v, w, ''
        now = self.overlap(xy, yaw, self.min_margin)
        if self._keeps_clear(xy, yaw, v, w, now):
            return v, w, ''
        if v != 0.0 and w != 0.0 and self._keeps_clear(xy, yaw, 0.0, w, now):
            return 0.0, w, 'driving on would hit a wall'
        if v != 0.0 and w != 0.0 and self._keeps_clear(xy, yaw, v, 0.0, now):
            return v, 0.0, 'turning would hit a wall'
        return 0.0, 0.0, ('turning' if v == 0.0 else 'driving') + ' would hit a wall'


class Approach:
    """Velocity commands that line the robot up with the stairs and stop in front of them.

    ALIGN        turn on the spot towards the stop point (on the centre line, stop_distance
                 before the first riser), or straight at the stairs if the centre is unknown
    APPROACH     drive there (never backwards), steering on the way
    FINAL_ALIGN  turn on the spot until perpendicular to the step edges, held settle_time s
    DONE         zero
    Closer than stop_distance already: no driving, it only squares up.

    With `walls` (a WallCheck; None = no wall checks, as before):
      - the stop pose must fit: if the footprint (+ margin) touches a wall there, it is moved
        up to max_shift sideways, else up to max_extra further back (`note` says so);
      - each turn on the spot is checked over its whole sweep first: the short way, else the
        long way round, else (ALIGN) MAKE_ROOM: creep straight ahead, up to room_max, until
        the turn is clear (+ room_extra), then turn; (APPROACH) steer while driving instead;
        (FINAL_ALIGN) within square_loose_tol is good enough, the note says how far off;
      - every command is checked `walls.horizon` s ahead (WallCheck.safe) and cut to the part
        that stays clear; nothing safe for blocked_time s -> BLOCKED.
    BLOCKED      zero; `note` says why. The robot is at the stairs but not lined up: the node
                 holds (AT_STAIRS) for the operator rather than giving up.
    """

    def __init__(self, stop_distance=0.45, align_tol=math.radians(5),
                 final_tol=math.radians(2.5), dist_tol=0.02, max_v=0.08, min_v=0.03, k_v=0.5,
                 max_w=0.35, min_w=0.15, k_w=1.0, max_offset=math.radians(25),
                 turn_first=math.radians(12), settle_time=0.6, centre=True, walls=None,
                 square_loose_tol=math.radians(8), room_max=0.3, room_extra=0.04,
                 max_shift=0.10, max_extra=0.15, blocked_time=1.0):
        self.stop_distance, self.align_tol, self.final_tol = stop_distance, align_tol, final_tol
        self.dist_tol, self.max_v, self.min_v, self.k_v = dist_tol, max_v, min_v, k_v
        self.max_w, self.min_w, self.k_w = max_w, min_w, k_w
        self.max_offset, self.turn_first, self.settle_time = max_offset, turn_first, settle_time
        self.centre = centre
        self.walls = walls
        self.square_loose_tol, self.room_max, self.room_extra = (square_loose_tol, room_max,
                                                                 room_extra)
        self.max_shift, self.max_extra, self.blocked_time = max_shift, max_extra, blocked_time
        self.phase = 'ALIGN'
        self.settled_since = None
        self.note = ''                  # wall-related remarks for the log / AT_STAIRS reason
        self.stop_shift = 0.0           # m left of the centre line the stop point was moved
        self.stop_extra = 0.0           # m further back than stop_distance
        self.stop_lateral = None        # centre unknown: the lateral position it keeps
        self.planned = False
        self.turn_sign = None           # direction chosen for the current turn on the spot
        self.room_from = None           # MAKE_ROOM: where the creep started
        self.room_clear_at = None       # MAKE_ROOM: crept distance where the turn got clear
        self.blocked_since = None

    def _note(self, text):
        self.note = text if not self.note else f'{self.note}; {text}'

    def remaining(self, xy, est):
        return est.distance(xy) - self.stop_distance - self.stop_extra

    def stop_point(self, est):
        along = est.along - self.stop_distance - self.stop_extra
        lat = (est.centre + self.stop_shift if self.centre and est.centre is not None
               else self.stop_lateral)
        return None if lat is None else est.d() * along + est.n() * lat

    def target_heading(self, xy, est):
        rem = self.remaining(xy, est)
        if not self.centre or est.centre is None or rem < 0.08:
            return est.heading
        stop = self.stop_point(est)
        bearing = math.atan2(stop[1] - xy[1], stop[0] - xy[0])
        off = max(-self.max_offset, min(self.max_offset, wrap(bearing - est.heading)))
        return wrap(est.heading + off)

    def plan_stop(self, xy, est):
        """Move the stop pose off any wall (sideways first, then further back)."""
        self.planned = True
        if self.walls is None:
            return
        centred = self.centre and est.centre is not None
        if not centred:
            self.stop_lateral = float(np.dot(xy, est.n()))
        shifts = [0.0] + [s * k for k in np.arange(0.02, self.max_shift + 1e-9, 0.02)
                          for s in (1.0, -1.0)]
        for margin in self.walls.margins():
            for extra in np.arange(0.0, self.max_extra + 1e-9, 0.05):
                for shift in shifts:
                    self.stop_extra = float(extra)
                    if centred:
                        self.stop_shift = float(shift)
                    else:
                        self.stop_lateral = float(np.dot(xy, est.n())) + float(shift)
                    if self.walls.pose_clear(self.stop_point(est), est.heading, margin):
                        if extra or shift:
                            self._note(f'stop pose moved {shift * 100:+.0f} cm sideways, '
                                       f'{extra * 100:.0f} cm back: wall')
                        return
        self.stop_shift, self.stop_extra = 0.0, 0.0
        if not centred:
            self.stop_lateral = float(np.dot(xy, est.n()))
        self._note('no wall-free stop pose found; driving checks still apply')

    def _turn(self, err, floor=True):
        w = max(-self.max_w, min(self.max_w, self.k_w * err))
        if floor and abs(w) < self.min_w:
            w = math.copysign(self.min_w, err)
        return w

    def _planned_turn(self, xy, yaw, err):
        """w for turning by err on the spot, the short way or (blocked) the long way round;
        None if both ways are blocked. Without walls: the short way, as before."""
        if self.walls is None:
            return self._turn(err)
        if self.turn_sign is None:
            long_err = err - math.copysign(2 * math.pi, err)
            for margin in self.walls.margins():
                if self.walls.turn_clear(xy, yaw, err, margin):
                    self.turn_sign = math.copysign(1.0, err)
                    break
                if self.walls.turn_clear(xy, yaw, long_err, margin):
                    self.turn_sign = -math.copysign(1.0, err)
                    self._note(f'turned {math.degrees(long_err):+.0f} deg the long way '
                               'round: wall')
                    break
            else:
                return None
        e = err if math.copysign(1.0, err) == self.turn_sign else \
            err - math.copysign(2 * math.pi, err)
        return self._turn(e)

    def _set_phase(self, phase):
        if phase != self.phase:
            self.phase, self.turn_sign = phase, None

    def update(self, now, xy, yaw, est):
        """-> (v, w) in m/s, rad/s for this tick; self.phase says where it is."""
        xy = np.asarray(xy, float)
        if not self.planned:
            self.plan_stop(xy, est)
        v, w = self._command(now, xy, yaw, est)
        if self.walls is None or self.phase in ('DONE', 'BLOCKED') or (v == 0.0 and w == 0.0):
            return v, w
        v2, w2, why = self.walls.safe(xy, yaw, v, w)
        if v2 == 0.0 and abs(w2) < self.min_w:       # nothing useful left (no livelock)
            v2, w2 = 0.0, 0.0
        if v2 == 0.0 and w2 == 0.0:
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since >= self.blocked_time:
                self._note(f'{self.phase}: {why}')
                self._set_phase('BLOCKED')
            return 0.0, 0.0
        self.blocked_since = None
        return v2, w2

    def _command(self, now, xy, yaw, est):
        rem = self.remaining(xy, est)
        if self.phase == 'MAKE_ROOM':
            moved = float(np.hypot(*(xy - self.room_from)))
            err = wrap(self.target_heading(xy, est) - yaw)
            long_err = err - math.copysign(2 * math.pi, err)
            m = self.walls.min_margin
            clear = (self.walls.turn_clear(xy, yaw, err, m)
                     or self.walls.turn_clear(xy, yaw, long_err, m))
            if clear and self.room_clear_at is None:
                self.room_clear_at = moved
            if self.room_clear_at is not None and (
                    moved >= self.room_clear_at + self.room_extra or not clear):
                self._note(f'crept {moved * 100:.0f} cm ahead to turn: wall')
                self._set_phase('ALIGN')
            elif moved >= self.room_max or rem <= self.dist_tol:
                self._note(f'turn to face the stairs ({math.degrees(err):+.0f} deg) blocked '
                           f'by walls, no room within {moved * 100:.0f} cm ahead')
                self._set_phase('BLOCKED')
                return 0.0, 0.0
            else:
                return self.min_v, 0.0
        if self.phase == 'ALIGN':
            if rem <= self.dist_tol:
                self._set_phase('FINAL_ALIGN')
            else:
                err = wrap(self.target_heading(xy, est) - yaw)
                if abs(err) < self.align_tol:
                    self._set_phase('APPROACH')
                else:
                    w = self._planned_turn(xy, yaw, err)
                    if w is not None:
                        return 0.0, w
                    room = min(self.room_max, rem)
                    if self.walls.free_ahead(xy, yaw, room) > 0.05:
                        self.room_from, self.room_clear_at = xy.copy(), None
                        self._set_phase('MAKE_ROOM')
                        return self.min_v, 0.0
                    self._note(f'turn to face the stairs ({math.degrees(err):+.0f} deg) '
                               'blocked by walls both ways, no room ahead')
                    self._set_phase('BLOCKED')
                    return 0.0, 0.0
        if self.phase == 'APPROACH':
            if rem <= self.dist_tol:
                self._set_phase('FINAL_ALIGN')
            else:
                err = wrap(self.target_heading(xy, est) - yaw)
                w = self._turn(err, floor=False)
                if abs(err) > self.turn_first:
                    wt = self._planned_turn(xy, yaw, err)
                    if wt is not None:
                        return 0.0, wt
                    # Can't turn on the spot here: steer while driving (checked ahead).
                    return self.min_v * max(0.0, math.cos(err)), self._turn(err)
                self.turn_sign = None
                v = max(self.min_v, min(self.max_v, self.k_v * rem)) * math.cos(err)
                return v, w
        if self.phase == 'FINAL_ALIGN':
            err = wrap(est.heading - yaw)
            if abs(err) < self.final_tol:
                if self.settled_since is None:
                    self.settled_since = now
                elif now - self.settled_since >= self.settle_time:
                    self._set_phase('DONE')
                return 0.0, 0.0
            self.settled_since = None
            w = self._planned_turn(xy, yaw, err)
            if w is not None:
                return 0.0, w
            if abs(err) <= self.square_loose_tol:
                self._note(f'squared up to {math.degrees(err):+.1f} deg only: wall')
                self._set_phase('DONE')
            else:
                self._note(f'squaring up blocked by walls at {math.degrees(err):+.0f} deg '
                           'off perpendicular')
                self._set_phase('BLOCKED')
            return 0.0, 0.0
        return 0.0, 0.0
