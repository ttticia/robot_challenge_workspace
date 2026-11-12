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


def select_frontier(grid, xy, clearance=.25, standoff=.32, minimum_size=.12, excluded=()):
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
        for index in np.argsort(score):
            point = points[index]
            if travel[index] < .25 or any(np.linalg.norm(point-np.asarray(p)) < .25 for p in excluded):
                continue
            value = math.sqrt(float(border.sum()))/(travel[index]+.4)
            if value > best_score:
                heading = math.atan2(centre[1]-point[1], centre[0]-point[0])
                best, best_score = (float(point[0]), float(point[1]), heading), value
            break
    return best, 'FRONTIER' if best else 'NO_REACHABLE_FRONTIER'


def relative_goal(anchor, goal):
    xy = np.asarray(anchor[:2]) + rotation(anchor[2]) @ np.asarray(goal[:2])
    return float(xy[0]), float(xy[1]), math.atan2(math.sin(anchor[2]+goal[2]), math.cos(anchor[2]+goal[2]))


class VelocityLimiter:
    def __init__(self, linear=.1, angular=.25, acceleration=.2, angular_acceleration=.5):
        self.linear, self.angular = linear, angular
        self.acceleration, self.angular_acceleration = acceleration, angular_acceleration
        self.previous = np.zeros(3)

    def apply(self, command, dt, permitted):
        value = np.asarray(command, float)
        if value.shape != (3,) or not np.isfinite(value).all() or not permitted or np.linalg.norm(value) < 1e-9:
            self.previous[:] = 0.
            return self.previous.copy()
        value = value.copy()
        value[:2] *= min(1., self.linear/max(np.linalg.norm(value[:2]), 1e-9))
        value[2] = np.clip(value[2], -self.angular, self.angular)
        delta = value-self.previous
        step = max(0., min(float(dt), .1))
        delta[:2] *= min(1., self.acceleration*step/max(np.linalg.norm(delta[:2]), 1e-9))
        delta[2] = np.clip(delta[2], -self.angular_acceleration*step, self.angular_acceleration*step)
        self.previous += delta
        return self.previous.copy()


class EventStore:
    def __init__(self, radius=.25, cell_size=.8, cell_origin=None):
        if not .02<=radius<=2. or not .1<=cell_size<=10.:
            raise ValueError('Invalid dedup radius or cell size')
        if cell_origin is not None and (np.asarray(cell_origin).shape!=(2,) or not np.isfinite(cell_origin).all()):
            raise ValueError('Cell origin must be a finite XY pair')
        self.radius, self.cell_size, self.cell_origin = radius, cell_size, cell_origin
        self.records = []

    def add(self, event):
        kind, name = str(event.get('kind','')), str(event.get('label',''))
        position = np.asarray(event.get('position',[]), float)
        confidence = float(event.get('confidence',1.))
        if kind not in ('qr','survivor','room','target') or not name or len(name) > 2048:
            raise ValueError('Invalid observation kind or label')
        if position.shape != (3,) or not np.isfinite(position).all() or not 0. <= confidence <= 1.:
            raise ValueError('Invalid observation geometry/confidence')
        for record in self.records:
            if record['kind'] == kind and record['label'] == name and np.linalg.norm(position-np.asarray(record['position'])) <= self.radius:
                count = record['observations']
                record['position'] = ((np.asarray(record['position'])*count+position)/(count+1)).tolist()
                record['observations'] += 1
                record['confidence'] = max(record['confidence'],confidence)
                record['last_stamp'] = event.get('stamp')
                self._cell(record)
                return record, False
        record = {'id':len(self.records)+1,'kind':kind,'label':name,'position':position.tolist(),
                  'confidence':confidence,'observations':1,'first_stamp':event.get('stamp'),
                  'last_stamp':event.get('stamp'),'evidence':str(event.get('evidence',''))[:2048]}
        self._cell(record)
        self.records.append(record)
        return record, True

    def _cell(self, record):
        record['cell'] = (np.floor((np.asarray(record['position'][:2])-self.cell_origin)/self.cell_size).astype(int).tolist()
                          if self.cell_origin is not None else None)
