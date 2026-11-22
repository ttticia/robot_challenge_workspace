"""Which part of the map lies down a junction's other ways (no ROS here).

User 2026-11-20: a curtain confirmed at a junction is crossed only after the junction's other
ways have been explored fully (every branch off them too). User 2026-11-21: the same for
possible stairs seen at a junction (Deferral.stairs): checked only after the other ways. maze_explorer keeps the curtain
on hold (CurtainHold) and lets only frontiers inside branch_mask() be goals until none are
left; then it releases the curtain and drives to it.

branch_mask(): free cells reachable from just outside the junction cell along each of the
given ways, without going back through the junction cell or through the curtain. A loop
that comes round to the way in or behind the curtain is part of the branch too: that is
reachable without the curtain anyway.
"""
from dataclasses import dataclass, field
import json

import numpy as np
from scipy import ndimage

from maze_nav.curtains import in_curtains


def branch_mask(grid, cells, junction_cell, ways, curtains=(), curtain_half=0.15,
                seed_depth=0.75, seed_search=0.25):
    """(H, W) bool: free cells of `grid` (core.Grid, map frame) down the junction's `ways`.

    cells: scene_cells.CellGrid; junction_cell: (i, j); ways: [(di, dj)] neighbour
    directions; curtains: curtains.Curtain in the map frame, blocked (+-curtain_half m).
    Each way is seeded `seed_depth` cells from the junction centre (just past its edge), at
    the nearest free cell within `seed_search` m."""
    free = grid.free.copy()
    h, w = free.shape
    res = grid.resolution
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    x = grid.origin[0] + (cols + 0.5) * res
    y = grid.origin[1] + (rows + 0.5) * res
    # Junction cell: blocked, so a branch can't be entered through it.
    c, s = np.cos(cells.yaw), np.sin(cells.yaw)
    dx, dy = x - cells.origin[0], y - cells.origin[1]
    u = (c * dx + s * dy) / cells.size - junction_cell[0]
    v = (-s * dx + c * dy) / cells.size - junction_cell[1]
    free &= ~((np.abs(u) <= 0.5) & (np.abs(v) <= 0.5))
    if len(curtains):
        pts = np.stack([x.ravel(), y.ravel()], axis=1)
        free &= ~in_curtains(pts, list(curtains), curtain_half, 0.05).reshape(h, w)
    labels, _ = ndimage.label(free)
    keep = set()
    r = max(1, int(round(seed_search / res)))
    for d in ways:
        sx, sy = cells.world((junction_cell[0] + seed_depth * d[0],
                              junction_cell[1] + seed_depth * d[1]))
        ci, ri = int((sx - grid.origin[0]) // res), int((sy - grid.origin[1]) // res)
        r0, r1, c0, c1 = max(ri - r, 0), min(ri + r + 1, h), max(ci - r, 0), min(ci + r + 1, w)
        if r1 <= r0 or c1 <= c0:
            continue
        patch = labels[r0:r1, c0:c1]
        if not (patch > 0).any():
            continue
        pr, pc = np.nonzero(patch > 0)
        k = np.argmin((pr + r0 - ri) ** 2 + (pc + c0 - ci) ** 2)
        keep.add(int(patch[pr[k], pc[k]]))
    return np.isin(labels, list(keep)) if keep else np.zeros_like(free)


def in_mask(mask, grid, xy):
    """True if map point xy falls on a True cell of mask (outside the grid: False)."""
    cx, cy = grid.cell(xy)
    return bool(0 <= cy < mask.shape[0] and 0 <= cx < mask.shape[1] and mask[cy, cx])


@dataclass
class Deferral:
    """A curtain confirmed at a junction, crossed once the junction's other ways are done;
    or (curtain_id None, `stairs` set; user 2026-11-21) possible stairs seen at a junction,
    checked once its other ways are done. stairs: {'id', 'edge' (map x, y of the first
    riser), 'heading' (map, up the stairs), 'done'}."""
    curtain_id: int
    junction_cell: tuple
    cells: object                          # scene_cells.CellGrid
    ways: list                             # [(di, dj)] the other ways (not in, not curtain)
    empty: int = 0                         # replans in a row with no frontier in the branch
    released: bool = False                 # branch done: driving to the curtain / crossing
    at_spot: float = None                  # monotonic time it got to the spot in front
    stairs: dict = None
    # User 2026-11-22: possible curtains (candidate-queue entries, curtain_id None) seen at
    # a junction with another way on: that way first (likely a room), then they go back
    # into the candidate queue for the lie-down check.
    candidates: list = None


@dataclass
class CurtainHold:
    """/maze/curtain_hold for curtain_detector: hold_new = don't walk through a curtain an
    inspection confirms now (report CONFIRMED, go IDLE); ids = never hand over a crossing."""
    hold_new: bool = False
    ids: list = field(default_factory=list)

    def to_json(self):
        return json.dumps({'hold_new': bool(self.hold_new), 'ids': sorted(set(self.ids))})

    @classmethod
    def from_json(cls, text):
        try:
            d = json.loads(text)
            return cls(bool(d.get('hold_new', False)), [int(i) for i in d.get('ids', [])])
        except (ValueError, TypeError, AttributeError):
            return cls()


def way_towards(cells, junction_cell, ways, xy):
    """The way (di, dj) of `ways` whose heading is closest to the bearing junction -> xy."""
    cx, cy = cells.centre(junction_cell)
    b = np.arctan2(xy[1] - cy, xy[0] - cx)
    return min(ways, key=lambda d: abs(np.angle(np.exp(1j * (cells.heading(d) - b)))))


def _label_near(labels, grid, xy, search):
    """Label of the nearest labelled cell within `search` m of xy (0 = none)."""
    res = grid.resolution
    ci, ri = int((xy[0] - grid.origin[0]) // res), int((xy[1] - grid.origin[1]) // res)
    r = max(1, int(round(search / res)))
    h, w = labels.shape
    r0, r1, c0, c1 = max(ri - r, 0), min(ri + r + 1, h), max(ci - r, 0), min(ci + r + 1, w)
    if r1 <= r0 or c1 <= c0:
        return 0
    patch = labels[r0:r1, c0:c1]
    pr, pc = np.nonzero(patch > 0)
    if len(pr) == 0:
        return 0
    k = np.argmin((pr + r0 - ri) ** 2 + (pc + c0 - ci) ** 2)
    return int(patch[pr[k], pc[k]])


def needs_crossing(grid, a, b, curtain, half=0.15, search=0.3):
    """True if b can be reached from a over free cells only through `curtain` (map frame):
    connected with the curtain open (grid has it cleared, maze_explorer.grid_now), apart
    with it blocked (+-half m). Unknown (a or b not near a free cell): False."""
    free = grid.free
    h, w = free.shape
    res = grid.resolution
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    pts = np.stack([(grid.origin[0] + (cols + 0.5) * res).ravel(),
                    (grid.origin[1] + (rows + 0.5) * res).ravel()], axis=1)
    strip = in_curtains(pts, [curtain], half, 0.05).reshape(h, w)
    shut, _ = ndimage.label(free & ~strip)
    la, lb = _label_near(shut, grid, a, search), _label_near(shut, grid, b, search)
    if la == 0 or lb == 0 or la == lb:
        return False
    opened, _ = ndimage.label(free)
    oa, ob = _label_near(opened, grid, a, search), _label_near(opened, grid, b, search)
    return oa != 0 and oa == ob
