"""ROS-independent frontier selection, velocity bounds and observation bookkeeping."""
from dataclasses import dataclass
import math
import time
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def yaw_from_quaternion(q):
    values = np.asarray(q, dtype=float)
    if not np.isfinite(values).all() or abs(np.linalg.norm(values)-1.) > .02:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = values
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def transform_point(point, translation, quaternion):
    x, y, z, w = quaternion
    yaw_from_quaternion(quaternion)  # Also validates norm and finite values.
    matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    result = matrix @ np.asarray(point, float) + np.asarray(translation, float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('Invalid transformed point')
    return result.tolist()


@dataclass
class Grid:
    data: np.ndarray
    resolution: float
    origin: tuple
    yaw: float = 0.

    def __post_init__(self):
        self.data = np.asarray(self.data, dtype=np.int16)
        if self.data.ndim != 2 or not self.data.size or self.data.size > 4_000_000:
            raise ValueError('Map must be a nonempty 2D grid with at most four million cells')
        if not .005 <= self.resolution <= .5:
            raise ValueError('Invalid map resolution')
        if not np.isfinite([*self.origin, self.yaw]).all():
            raise ValueError('Invalid map origin')
        if not ((self.data >= -1) & (self.data <= 100)).all():
            raise ValueError('Invalid occupancy values')

    def cell(self, xy):
        return np.floor(((np.asarray(xy)-self.origin) @ rotation(self.yaw))/self.resolution).astype(int)

    def world(self, cells):
        return (np.asarray(cells)+.5)*self.resolution @ rotation(self.yaw).T + self.origin

    def contains(self, cell):
        x, y = cell
        return 0 <= x < self.data.shape[1] and 0 <= y < self.data.shape[0]


def select_frontier(grid, xy, clearance=.25, standoff=.32, minimum_size=.12, excluded=(), preference=None):
    """Reachable known-free approach to an unknown boundary. Nav2 plans the path.

    Clearance is an approximate width filter, not the robot's swept footprint.
    Unknown space remains blocked; unseen space behind walls is not a frontier.
    """
    known_free = (grid.data >= 0) & (grid.data <= 25)
    occupied = grid.data >= 65
    unknown = grid.data < 0
    safe = known_free & (distance_transform_edt(known_free)*grid.resolution > clearance)
    start = grid.cell(xy)
    if not grid.contains(start) or not safe[start[1], start[0]]:
        return None, 'START_NOT_CLEAR'
    components, _ = label(safe)
    reachable = safe & (components == components[start[1], start[0]])
    boundary = known_free & binary_dilation(unknown) & ~binary_dilation(occupied, iterations=2)
    borders, count = label(boundary, structure=np.ones((3,3), dtype=int))
    best, best_score = None, -math.inf
    sizes=np.bincount(borders.ravel(),minlength=count+1)
    eligible=[i for i in range(1,count+1) if sizes[i]*grid.resolution>=minimum_size]
    for i in sorted(eligible,key=lambda i:sizes[i],reverse=True)[:32]:
        border = borders == i
        if border.sum()*grid.resolution < minimum_size:
            continue
        distance = distance_transform_edt(~border)*grid.resolution
        approaches = np.argwhere(reachable & (distance <= standoff+.12))
        if not len(approaches):
            continue
        centre = grid.world(np.argwhere(border)[:, ::-1]).mean(axis=0)
        points = grid.world(approaches[:, ::-1])
        travel = np.linalg.norm(points-xy, axis=1)
        score = np.linalg.norm(points-centre, axis=1)+.15*travel
        if preference is not None:
            score /= np.array([preference(point) for point in points])
        for index in np.argsort(score):
            point = points[index]
            if travel[index] < .25 or any(np.linalg.norm(point-np.asarray(p)) < .25 for p in excluded):
                continue
            value = math.sqrt(float(border.sum()))/(travel[index]+.4)
            if preference is not None:
                value *= preference(point)
            if value > best_score:
                heading = math.atan2(centre[1]-point[1], centre[0]-point[0])
                best, best_score = (float(point[0]), float(point[1]), heading), value
            break
    return best, 'FRONTIER' if best else 'NO_REACHABLE_FRONTIER'

