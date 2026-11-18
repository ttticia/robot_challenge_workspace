"""Maze cells, and when the robot has reached one it hasn't been in before (no ROS).

The maze is a square grid of `size` m cells. CellGrid maps the SLAM `map` frame to cell
indices (i, j): cell (0, 0) is the one centred on `origin` (by default the map origin, i.e.
where the robot stood at bringup: on the start cell, its rear ~8 cm from the back wall, puts
base_link within a few cm of the start cell's centre), and +i points along `yaw` (0 = the
robot's heading at bringup, so a robot placed square in the start cell lines the grid up
with the maze).

NewCellTrigger: returns one SceneEvent the first time the robot enters each cell, including
the start cell (`trigger_at_start`). Going back into a cell entered before never fires.
"Entered" uses hysteresis: base_link must be `enter_margin` m inside a cell (away from all
its edges), so straddling a boundary doesn't flip cells or fire early.

run_active() reads maze_explorer's /maze/status, so the node only fires while the robot is
actually running the maze (not while it holds still before /maze/start, nor after the end).

opening() tells, from the occupancy grid, whether a side of a cell is open or a wall; the
node uses it to list the new cell's ways out.
"""
from dataclasses import dataclass
import math

import numpy as np

from maze_nav.core import FREE_MAX, OCCUPIED_MIN

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
ACTIVE_STATES = ('CREEPING', 'SCANNING', 'EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR')


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
        self.visited = set()

    def update(self, xy, yaw):
        """One pose sample (map frame x, y and yaw in rad)."""
        c = self.cells.cell(xy)
        if c == self.current:
            return None
        if self.current is not None and self.cells.depth_inside(xy, c) < self.enter_margin:
            return None              # near the edge: not entered yet
        prev, self.current = self.current, c
        if c in self.visited:
            return None
        self.visited.add(c)
        if prev is None and not self.trigger_at_start:
            return None
        step = None if prev is None else (c[0] - prev[0], c[1] - prev[1])
        name = next((n for n, d in DIRECTIONS.items() if d == step), '')
        return SceneEvent(c, prev, name, yaw)
