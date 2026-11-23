"""Per-cell features from lidar points ABOVE and IN the wall band (no ROS).

Practice arena (maze_info.md, photos 2026-11-22/23): walls are 0.53 m tall, but
  - the toy sets in the rooms (kitchen, market stall, workbench, medical trolley, ...) stand
    0.8-1.2 m tall at the back of the room cell: points 0.58-1.30 m above the floor in a
    cell's INTERIOR can only be a toy set (people and the other arena are outside the maze);
  - the stairs are a 3-level pyramid (15 cm rise, top 0.45 m, inside the 0.30-0.50 m wall
    band) between two truss railings ~0.9 m tall: a cell whose interior has points in the
    wall band but none above it, with high points along two OPPOSITE edges (the rails);
  - a curtain hangs from ~0.9 m on a cell edge: points 0.58-0.85 m along the edge (fabric),
    where a 0.53 m wall with a 10-25 cm gap has none. (curtains.py checks this itself on
    each detection; the edges here are for the log / RViz and a cross-check.)
The /map (wall band only) can't show any of this: toys against the back wall merge into it
(to_do.md, toy-market sim test #10) and the stair top looks like any obstacle.

CellFeatures keeps a sparse raster (res m) of hit counts per height band in the map frame,
forgetting bins not seen for `forget` s (people walking past, SLAM corrections), and
classify(cells) reads it against the current cell grid (scene_cells.CellGrid):
  cell kinds: 'object' (toy set: room), 'stairs' (raised interior between rails, axis =
  the way up), 'raised' (raised interior, no rails seen yet, no toy set);
  edges: 'tall' when >= tall_cover of the edge's length has fabric-band points.
"""
from dataclasses import dataclass
import math
import time

import numpy as np


@dataclass
class FeatureParams:
    res: float = 0.05               # m raster bins
    high: tuple = (0.58, 1.30)      # m above the floor: above the 0.53 m walls
    fabric: tuple = (0.58, 0.85)    # m: curtain fabric above the walls, under its rail
    raised: tuple = (0.30, 0.50)    # m: the wall band; the stair top (0.45) is in it
    edge_margin: float = 0.12       # m either side of an edge line: the edge strip
    corner: float = 0.10            # m at each end of an edge left out (corner posts)
    min_hits: int = 2               # hits for a bin to count
    object_area: float = 0.03       # m2 of high bins in a cell's interior: a toy set
    stair_area: float = 0.05        # m2 of raised bins in a cell's interior
    stair_max_high: float = 0.01    # m2: more high interior than this is a toy set, not stairs
    rail_cover: float = 0.5         # fraction of an edge with high bins: a railing
    tall_cover: float = 0.4         # fraction of an edge with fabric bins: tall edge
    body_radius: float = 0.55       # m around base_link: the robot's own body / legs / drone
    max_range: float = 4.0          # m from the lidar: further points are too sparse
    forget: float = 45.0            # s a bin is kept after it was last hit
    max_count: int = 50


LAYERS = ('high', 'fabric', 'raised')


class CellFeatures:
    def __init__(self, p=None):
        self.p = p or FeatureParams()
        self.bins = {k: {} for k in LAYERS}     # layer -> {(ix, iy): [count, last seen]}

    def add(self, pts, floor_z, robot_xy, sensor_xy=None, now=None):
        """pts (N, 3) in the map frame (z = odom z: slam_toolbox's map -> odom is 2D)."""
        p = self.p
        now = time.monotonic() if now is None else now
        pts = np.asarray(pts, float).reshape(-1, 3)
        if len(pts) == 0:
            return
        h = pts[:, 2] - floor_z
        rxy = np.asarray(robot_xy, float)[:2]
        sxy = rxy if sensor_xy is None else np.asarray(sensor_xy, float)[:2]
        near_body = np.hypot(pts[:, 0] - rxy[0], pts[:, 1] - rxy[1]) < p.body_radius
        far = np.hypot(pts[:, 0] - sxy[0], pts[:, 1] - sxy[1]) > p.max_range
        ok = ~near_body & ~far
        for name in LAYERS:
            lo, hi = getattr(p, name)
            m = ok & (h >= lo) & (h <= hi)
            if not m.any():
                continue
            ij = np.floor(pts[m, :2] / p.res).astype(np.int64)
            keys, counts = np.unique(ij, axis=0, return_counts=True)
            layer = self.bins[name]
            for (ix, iy), c in zip(keys.tolist(), counts.tolist()):
                b = layer.get((ix, iy))
                if b is None:
                    layer[(ix, iy)] = [min(c, p.max_count), now]
                else:
                    b[0] = min(b[0] + c, p.max_count)
                    b[1] = now

    def forget_old(self, now=None):
        now = time.monotonic() if now is None else now
        for layer in self.bins.values():
            for k in [k for k, b in layer.items() if now - b[1] > self.p.forget]:
                del layer[k]

    def occupied(self, name):
        """(M, 2) map xy of the bin centres with >= min_hits in layer `name`."""
        layer = self.bins[name]
        keys = [k for k, b in layer.items() if b[0] >= self.p.min_hits]
        if not keys:
            return np.zeros((0, 2))
        return (np.asarray(keys, float) + 0.5) * self.p.res

    def classify(self, cells):
        """{'cells': {(i, j): {...}}, 'edges': {edge key: {...}}} against `cells`.

        Cell entries: 'high' / 'raised' interior areas (m2), 'kind' ('object', 'stairs',
        'raised' or None), 'axis' (stairs: 'i' or 'j', the way across; None if unknown).
        Edge keys: ('i', i, j) = the edge between (i, j) and (i + 1, j); ('j', i, j) = between
        (i, j) and (i, j + 1). Entries: 'high' / 'fabric' cover (0..1), 'tall' (bool)."""
        p = self.p
        size = cells.size
        inner_lim = 0.5 - p.edge_margin / size
        strip = p.edge_margin / size
        along_lim = 0.5 - p.corner / size
        slots = max(1, int(round((size - 2 * p.corner) / p.res)))
        cell_out, edge_out = {}, {}
        for name in LAYERS:
            xy = self.occupied(name)
            if len(xy) == 0:
                continue
            u, v = cells.local((xy[:, 0], xy[:, 1]))
            u, v = np.asarray(u), np.asarray(v)
            ci, cj = np.floor(u + 0.5).astype(int), np.floor(v + 0.5).astype(int)
            du, dv = u - ci, v - cj
            inner = np.maximum(np.abs(du), np.abs(dv)) <= inner_lim
            if name != 'fabric' and inner.any():      # interior areas
                keys, counts = np.unique(np.c_[ci[inner], cj[inner]], axis=0,
                                         return_counts=True)
                for (i, j), n in zip(keys.tolist(), counts.tolist()):
                    e = cell_out.setdefault((int(i), int(j)), {'high': 0.0, 'raised': 0.0})
                    e[name] = float(n) * p.res * p.res
            if name == 'raised':
                continue
            # edge strips: |du -+ 0.5| < strip ('i' edges), |dv -+ 0.5| < strip ('j' edges)
            for axis, across, along, ii, jj in (('i', du, dv, ci, cj), ('j', dv, du, ci, cj)):
                for sgn in (1, -1):
                    m = (np.abs(across - 0.5 * sgn) < strip) & (np.abs(along) <= along_lim)
                    if not m.any():
                        continue
                    lo_i = ii[m] - (1 if (axis == 'i' and sgn < 0) else 0)
                    lo_j = jj[m] - (1 if (axis == 'j' and sgn < 0) else 0)
                    slot = np.clip(((along[m] + along_lim) * size / p.res).astype(int), 0,
                                   slots - 1)
                    keys = np.unique(np.c_[lo_i, lo_j, slot], axis=0)
                    pairs, counts = np.unique(keys[:, :2], axis=0, return_counts=True)
                    for (a, b), n in zip(pairs.tolist(), counts.tolist()):
                        e = edge_out.setdefault((axis, int(a), int(b)),
                                                {'high': 0.0, 'fabric': 0.0})
                        e[name] = max(e[name], float(n) / slots)
        for e in edge_out.values():
            e['tall'] = e['fabric'] >= p.tall_cover
        for (i, j), e in cell_out.items():
            e['kind'], e['axis'] = None, None
            if e['high'] >= p.object_area:
                e['kind'] = 'object'
            elif e['raised'] >= p.stair_area and e['high'] <= p.stair_max_high:
                rails_j = all(edge_out.get(k, {}).get('high', 0.0) >= p.rail_cover
                              for k in (('j', i, j), ('j', i, j - 1)))
                rails_i = all(edge_out.get(k, {}).get('high', 0.0) >= p.rail_cover
                              for k in (('i', i, j), ('i', i - 1, j)))
                if rails_j != rails_i:              # rails on one pair of sides only
                    e['kind'], e['axis'] = 'stairs', 'i' if rails_j else 'j'
                else:
                    e['kind'] = 'raised'
        return {'cells': cell_out, 'edges': edge_out}


def stair_entry(cells, cell, axis, robot_xy):
    """(edge point, heading) in the map frame to walk up stairs in `cell`: from the side of
    `axis` ('i' / 'j'; None = whichever grid axis points most towards the robot) nearer the
    robot, first riser on that cell edge, heading into the cell."""
    cx, cy = cells.centre(cell)
    bx, by = robot_xy[0] - cx, robot_xy[1] - cy
    dirs = {'i': [(1, 0), (-1, 0)], 'j': [(0, 1), (0, -1)]}
    cand = dirs[axis] if axis in dirs else dirs['i'] + dirs['j']
    # The side facing the robot: d (into the cell) points away from it.
    best = max(cand, key=lambda d: -(math.cos(cells.heading(d)) * bx +
                                     math.sin(cells.heading(d)) * by))
    heading = cells.heading(best)
    ex = cx - 0.5 * cells.size * math.cos(heading)
    ey = cy - 0.5 * cells.size * math.sin(heading)
    return (ex, ey), heading


def features_to_dict(res, cells, min_edge=0.3):
    """JSON-ready /maze/cell_features payload: cells with a kind, edges with some cover."""
    out_cells = []
    for (i, j), e in sorted(res['cells'].items()):
        if e['kind'] is None:
            continue
        cx, cy = cells.centre((i, j))
        out_cells.append({'cell': [i, j], 'kind': e['kind'], 'axis': e['axis'],
                          'high': round(e['high'], 3), 'raised': round(e['raised'], 3),
                          'centre': [round(cx, 3), round(cy, 3)]})
    out_edges = []
    for (axis, i, j), e in sorted(res['edges'].items()):
        if max(e.get('high', 0.0), e.get('fabric', 0.0)) < min_edge:
            continue
        other = [i + 1, j] if axis == 'i' else [i, j + 1]
        out_edges.append({'between': [[i, j], other], 'high': round(e.get('high', 0.0), 2),
                          'fabric': round(e.get('fabric', 0.0), 2), 'tall': e['tall']})
    return {'frame': 'map', 'size': cells.size,
            'origin': [round(cells.origin[0], 4), round(cells.origin[1], 4)],
            'yaw': round(cells.yaw, 5), 'cells': out_cells, 'edges': out_edges}
