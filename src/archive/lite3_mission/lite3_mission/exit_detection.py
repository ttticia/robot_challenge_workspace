"""Geometric exit cue: leave an enclosed start and enter observed open space."""
from dataclasses import dataclass
import numpy as np
from scipy.ndimage import distance_transform_edt


def observed_clearance(grid):
    # Pad with unknown: map edges and unobserved cells are never open space.
    free=(grid.data>=0)&(grid.data<=25)
    return distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*grid.resolution


@dataclass
class ExitDetector:
    enclosed_radius: float=.8
    open_radius: float=1.2
    minimum_displacement: float=2.
    dwell_seconds: float=3.

    def __post_init__(self):
        values=[self.enclosed_radius,self.open_radius,self.minimum_displacement,self.dwell_seconds]
        if not np.isfinite(values).all() or not 0<self.enclosed_radius<self.open_radius or min(values[2:])<=0:
            raise ValueError('Exit thresholds must be finite, positive and open radius > enclosed radius')
        self.reset()

    def reset(self):
        self.start=None
        self.seen_enclosed=False
        self.since=None
        self.last_time=None
        self.found=False
        self.clearance=0.

    def update(self,grid,clearance,xy,now):
        if self.start is None:self.start=np.asarray(xy,float).copy()
        # A pause, loss of data, or backward clock breaks persistence.
        if self.last_time is not None and not 0<=now-self.last_time<=1.:
            self.since=None
        self.last_time=now
        cell=grid.cell(xy)
        self.clearance=float(clearance[cell[1],cell[0]]) if grid.contains(cell) else 0.
        self.seen_enclosed=self.seen_enclosed or self.clearance<=self.enclosed_radius
        candidate=(self.seen_enclosed and np.linalg.norm(np.asarray(xy)-self.start)>=self.minimum_displacement
                   and self.clearance>=self.open_radius)
        if not candidate:self.since=None
        elif self.since is None:self.since=now
        if self.since is not None and now-self.since>=self.dwell_seconds:self.found=True
        return self.found
