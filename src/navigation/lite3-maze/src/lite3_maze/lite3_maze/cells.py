"""Online cell exploration and enclosed-start/open-exit detection from observed maps.

The grid is a semantic overlay. Targets and traversability still come from the
fine occupancy grid; no world layout, route or simulator pose is consumed.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from .core import rotation, select_frontier


class CellExplorer:
    def __init__(self, cell_size=1.0, exit_radius=.9, minimum_travel=3.0, exit_hold=2.):
        self.size = cell_size
        self.exit_radius = exit_radius
        self.minimum_travel = minimum_travel
        self.exit_hold = exit_hold
        self.anchor = self.previous = None
        self.visits = {}
        self.edges = set()
        self.travel = 0.
        self.enclosed_start = False
        self.open_since = None
        self.complete = False
        self.last_cell = None
        self.clearance = None
        self.map_id = None
        self.previous_time = None

    def cell(self, point):
        local = (np.asarray(point)-self.anchor[:2]) @ rotation(self.anchor[2])
        return tuple(map(int, np.floor(local/self.size+.5)))

    def observe(self, grid, pose, now):
        if self.anchor is None:
            self.anchor = np.asarray(pose, dtype=float)
        point = np.asarray(pose[:2], dtype=float)
        here = self.cell(point)
        # Count entries, not callback frames. Slow crossings should not cost more.
        if here != self.last_cell:
            self.visits[here] = self.visits.get(here, 0)+1
            if self.last_cell is not None:
                self.edges.add(tuple(sorted((self.last_cell, here))))
            self.last_cell = here
        if self.previous is not None:
            step = np.linalg.norm(point-self.previous)
            elapsed = max(0., now-self.previous_time) if self.previous_time is not None else 0.
            if step < max(.5, .8*elapsed):  # Bound jumps while allowing accelerated simulation.
                self.travel += float(step)
        self.previous = point
        self.previous_time = now
        free = (grid.data >= 0) & (grid.data <= 25)
        if self.map_id != id(grid):
            # Pad explicitly: outside the map and unknown space cannot prove an exit.
            self.clearance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]*grid.resolution
            self.map_id = id(grid)
        start = grid.cell(self.anchor[:2])
        if self.travel < self.size and grid.contains(start):
            wall_cells = np.argwhere(grid.data >= 65)[:, ::-1]
            if len(wall_cells):
                delta = grid.world(wall_cells)-self.anchor[:2]
                near = delta[np.linalg.norm(delta, axis=1) < 1.2*self.size]
                if len(near):
                    sectors = np.floor((np.arctan2(near[:, 1], near[:, 0])+math.pi)/(math.pi/2)).astype(int) % 4
                    self.enclosed_start |= len(np.unique(sectors)) >= 3
        index = grid.cell(point)
        open_area = grid.contains(index) and self.clearance[index[1], index[0]] >= self.exit_radius
        eligible = self.enclosed_start and self.travel >= self.minimum_travel and len(self.visits) >= 3 and open_area
        if eligible:
            if self.open_since is None:
                self.open_since = now
            if now-self.open_since >= self.exit_hold:
                self.complete = True
        else:
            self.open_since = None
        return self.complete

    def choose(self, grid, pose, clearance=.25, standoff=.32, minimum_size=.12, excluded=()):
        if self.complete:
            return None, 'MAZE_EXIT_REACHED'
        # An unvisited cell gets priority while backtracking remains possible.
        preference = lambda p: 1./(1.+2.*self.visits.get(self.cell(p), 0))
        return select_frontier(grid, pose[:2], clearance, standoff, minimum_size, excluded, preference)

    def report(self):
        return dict(cell_size_m=self.size, origin='initial_robot_cell',
                    anchor=self.anchor.tolist() if self.anchor is not None else None,
                    cell=list(self.last_cell) if self.last_cell is not None else None,
                    visited=[dict(cell=list(c), entries=n) for c, n in sorted(self.visits.items())],
                    edges=[[list(a), list(b)] for a, b in sorted(self.edges)],
                    travel_m=self.travel, enclosed_start_seen=bool(self.enclosed_start),
                    exit_candidate=self.open_since is not None, complete=bool(self.complete))
