"""Initial planar LiDAR odometry, occupancy mapping and frontier navigation.

No simulator geometry, labels, route or ground-truth pose is consumed here.
This prototype has no loop closure or stair/curtain crossing controller.
"""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation, label
from scipy.spatial import cKDTree


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class ScanOdometry:
    """IMU-heading-constrained robust scan-to-scan translation ICP."""
    def __init__(self):
        self.xy = np.zeros(2)
        self.previous = None
        self.error = math.inf
        self.valid = False
        self.yaw0 = None
        self.diagnostics = {}
        self.last_good_stamp = None

    def update(self, points, imu_yaw, stamp=None):
        if self.yaw0 is None:
            self.yaw0 = imu_yaw
        yaw = math.atan2(math.sin(imu_yaw-self.yaw0), math.cos(imu_yaw-self.yaw0))
        oriented = points @ rotation(yaw).T
        if len(points) < 40:
            self.valid = False
            return self.xy.copy(), yaw
        if self.previous is None:
            self.previous = oriented.copy()
            self.valid, self.error = True, 0.
            self.last_good_stamp = stamp
            return self.xy.copy(), yaw
        tree = cKDTree(self.previous)
        # Local PCA normals make translation along long corridor walls observable
        # from end walls, rather than biasing it toward zero as point-to-point ICP does.
        _, neighbors = tree.query(self.previous, k=7)
        local = self.previous[neighbors]
        centered = local-local.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered)/7
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normals = eigenvectors[:, :, 0]
        planar = eigenvalues[:, 0] < .08*np.maximum(eigenvalues[:, 1], 1e-10)
        line_mode = planar.sum() >= 35
        delta = np.zeros(2)
        good = np.zeros(len(points), dtype=bool)
        observable = True
        for _ in range(12):
            distances, indices = tree.query(oriented + delta)
            good = distances < .12
            if line_mode:
                good &= planar[indices]
            if good.sum() < 35:
                break
            residual = self.previous[indices[good]] - (oriented[good] + delta)
            if line_mode:
                a = normals[indices[good]]
                b = np.sum(a*residual, axis=1)
                robust = np.abs(b) < .10
                information = a[robust].T @ a[robust]
                spectrum = np.linalg.eigvalsh(information)
                if spectrum[0] < .001*max(spectrum[-1], 1.):
                    observable = False
                    break
                weight = np.sqrt(np.minimum(1., .02/np.maximum(np.abs(b[robust]), 1e-6)))
                change = np.linalg.lstsq(a[robust]*weight[:, None], b[robust]*weight, rcond=None)[0]
            else:
                cutoff = max(.008, np.quantile(distances[good], .8))
                trimmed = residual[distances[good] <= cutoff]
                change = np.mean(trimmed, axis=0)
            delta += change
            if np.linalg.norm(change) < .0001:
                break
        self.error = float(np.sqrt(np.mean(distances[good]**2))) if good.any() else math.inf
        self.diagnostics = dict(correspondences=int(good.sum()), observable=bool(observable),
                                translation_m=float(np.linalg.norm(delta)))
        # A delayed scan must not permanently freeze ICP against an old reference.
        # Bound observed displacement by capture-time separation and robot speed;
        # retain the residual, correspondence and observability requirements.
        elapsed = max(0., stamp-self.last_good_stamp) if stamp is not None and self.last_good_stamp is not None else 0.
        maximum_delta = min(.30, .04+.5*elapsed) if stamp is not None else .075
        self.valid = observable and good.sum() >= 35 and self.error < .055 and np.linalg.norm(delta) < maximum_delta
        if self.valid:
            self.xy += delta
            self.previous = oriented.copy()
            self.last_good_stamp = stamp
        # Retain the last good scan on failure, rather than accepting a discontinuity.
        return self.xy.copy(), yaw


class GridMap:
    def __init__(self, resolution=.04, extent=8.):
        self.resolution, self.origin = resolution, np.array([-extent, -extent])
        self.n = int(2*extent/resolution)
        self.log = np.zeros((self.n, self.n), dtype=np.float32)
        self.half_length, self.half_width = .32, .23

    def cell(self, xy):
        return np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int)

    def world(self, cell):
        return self.origin + (np.asarray(cell)+.5)*self.resolution

    def inside(self, cells):
        return ((cells >= 0) & (cells < np.array(self.log.shape[::-1]))).all(axis=-1)

    def observe(self, xy, yaw, ranges, angle_min, angle_increment, maximum, angles=None):
        for i, measured in enumerate(ranges):
            if math.isnan(measured) or measured <= .06:
                continue
            hit = math.isfinite(measured) and .06 < measured < maximum
            distance = measured if hit else maximum
            if math.isnan(distance) or distance <= .06:
                continue
            distance = min(distance, maximum)
            angle = yaw + (angle_min+i*angle_increment if angles is None else angles[i])
            direction = np.array([math.cos(angle), math.sin(angle)])
            samples = np.arange(0, max(0, distance-.04), self.resolution*.7)
            cells = self.cell(xy + samples[:, None]*direction)
            cells = cells[self.inside(cells)]
            if len(cells):
                self.log[cells[:, 1], cells[:, 0]] -= .22
            end = self.cell(xy + distance*direction)
            if hit and self.inside(end):
                self.log[end[1], end[0]] += .85
        np.clip(self.log, -4., 4., out=self.log)

    def mark_obstacles(self, xy_points):
        cells = self.cell(xy_points)
        cells = cells[self.inside(cells)]
        if len(cells):
            self.log[cells[:, 1], cells[:, 0]] = 4.

    def planning_free(self):
        # Keep the full rotating body clear, so frontier goals cannot strand a
        # long quadruped beside a wall while trying to face the next corridor.
        blocked = self.log >= -.3
        clearance = math.hypot(self.half_length, self.half_width)+self.resolution/2
        return distance_transform_edt(np.pad(~blocked, 1))[1:-1, 1:-1]*self.resolution > clearance

    def footprint_obstructions(self, xy, yaw):
        matrix = rotation(yaw)
        length, width = self.half_length, self.half_width
        corners = np.array([[-length,-width],[-length,width],[length,-width],[length,width]]) @ matrix.T + xy
        low, high = self.cell(corners.min(axis=0)), self.cell(corners.max(axis=0))
        cells = np.array(np.meshgrid(np.arange(low[0], high[0]+1), np.arange(low[1], high[1]+1))).reshape(2, -1).T
        local = (self.world(cells)-xy) @ matrix
        margin = .5*self.resolution*(abs(math.cos(yaw))+abs(math.sin(yaw)))
        cells = cells[(np.abs(local) <= np.array([length,width])+margin).all(axis=1)]
        inside = self.inside(cells)
        blocked = ~inside
        blocked[inside] = self.log[cells[inside, 1], cells[inside, 0]] >= -.3
        return {tuple(map(int, c)) for c in cells[blocked]}

    def footprint_clear(self, xy, yaw):
        return not self.footprint_obstructions(xy, yaw)


def astar(free, start, goal):
    start, goal = tuple(start), tuple(goal)
    if any(not (0 <= p[0] < free.shape[1] and 0 <= p[1] < free.shape[0]) for p in (start, goal)):
        return []
    if not free[start[1], start[0]] or not free[goal[1], goal[0]]:
        return []
    pending = [(0., start)]
    cost, parent = {start: 0.}, {}
    while pending:
        _, here = heapq.heappop(pending)
        if here == goal:
            path = [here]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        x, y = here
        for dx, dy in ((1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)):
            nx, ny = x+dx, y+dy
            if not (0 <= ny < free.shape[0] and 0 <= nx < free.shape[1]) or not free[ny, nx]:
                continue
            if dx and dy and (not free[y, nx] or not free[ny, x]):
                continue
            neighbor = (nx, ny)
            proposed = cost[here] + math.hypot(dx, dy)
            if proposed < cost.get(neighbor, math.inf):
                cost[neighbor], parent[neighbor] = proposed, here
                heapq.heappush(pending, (proposed+math.dist(neighbor, goal), neighbor))
    return []


class FrontierPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.path = []
        self.goal = None
        self.recovering = False
        self.reason = 'NO_REACHABLE_FRONTIER'
        self.desired_heading = None
        self.forward_speed, self.reverse_speed, self.turn_speed = .3, .4, .4

    def plan(self, xy, cells=None, semantic_grid=None, yaw=0.):
        grid = self.grid
        free = grid.planning_free()
        start = grid.cell(xy)
        if not grid.inside(start) or grid.log[start[1], start[0]] >= -.3:
            self.path = []
            self.reason = 'START_NOT_OBSERVED_FREE'
            return []
        if not free[start[1], start[0]]:
            # Lateral policy drift or map quantization can put the centre just
            # outside the inflated planning region. Find a nearby clear stance
            # through observed free space; execution still checks the full body.
            self.recovering = True
            self.reason = 'START_CLEARANCE_LOW'
            raw_free = grid.log < -.3
            components, _ = label(raw_free)
            reachable = components == components[start[1], start[0]]
            candidates = np.argwhere(free & reachable)[:, ::-1]
            points = grid.world(candidates)
            distances = np.linalg.norm(points-xy, axis=1)
            eligible = np.flatnonzero((distances >= .16) & (distances <= .5))
            for i in eligible[np.argsort(distances[eligible])]:
                if not grid.footprint_clear(points[i], yaw):
                    continue
                path = astar(raw_free, start, candidates[i])
                if path:
                    self.goal = points[i]
                    self.path = [grid.world(p) for p in path]
                    return self.path
            self.path = []
            return []
        if self.recovering:
            self.goal = None
            self.recovering = False
        self.reason = 'NO_REACHABLE_FRONTIER'
        connected, _ = label(free)
        free &= connected == connected[start[1], start[0]]
        if self.goal is not None and np.linalg.norm(self.goal-xy) > .12:
            goal_cell = grid.cell(self.goal)
            path = astar(free, start, goal_cell)
            if path:
                self.path = [grid.world(p) for p in path]
                return self.path
        if cells is not None and semantic_grid is not None:
            goal, _ = cells.choose(semantic_grid, (*xy, yaw),
                clearance=math.hypot(grid.half_length, grid.half_width)+grid.resolution/2,
                standoff=.4)
            if goal:
                target = grid.cell(goal[:2])
                path = astar(free, start, target)
                if path:
                    self.goal = np.array(goal[:2])
                    self.path = [grid.world(p) for p in path]
                    return self.path
        unknown = np.abs(grid.log) < .1
        # Frontiers must border free space, away from occupied walls. Unknown
        # cells on the opposite side of a wall are not exploration targets.
        frontiers = ((grid.log < -.3) & binary_dilation(unknown)
                     & ~binary_dilation(grid.log > .5, iterations=2))
        approach = max(.42, math.hypot(grid.half_length, grid.half_width)+grid.resolution/2+.15)
        near_unknown = distance_transform_edt(~frontiers)*grid.resolution < approach if frontiers.any() else np.zeros_like(free)
        candidates = np.argwhere(free & near_unknown)
        # Prefer informative frontiers with short travel; candidate thinning bounds A* work.
        if len(candidates):
            candidates = candidates[::max(1, len(candidates)//60)]
            distances = np.linalg.norm(grid.world(candidates[:, ::-1])-xy, axis=1)
            for i in np.argsort(distances):
                if distances[i] < .3:
                    continue
                target = candidates[i, ::-1]
                path = astar(free, start, target)
                if path:
                    self.goal = grid.world(target)
                    self.path = [grid.world(p) for p in path]
                    return self.path
        self.path, self.goal = [], None
        return []

    def velocity(self, xy, yaw):
        """Face the observed route before walking so forward cameras see travel.

        Brief clearance recovery may translate laterally when the current body
        cannot rotate clear of a wall; normal path tracking never walks backward
        or sideways while holding the original maze heading.
        """
        if not self.path:
            return np.zeros(3), self.reason
        if self.recovering:
            return self._clearance_velocity(xy, yaw)
        if np.linalg.norm(self.goal-xy) < .10:
            return np.zeros(3), 'OBSERVE'
        path = np.asarray(self.path)
        nearest = int(np.argmin(np.linalg.norm(path-xy, axis=1)))
        following = path[nearest:]
        # Distance along the path, rather than searchsorted on a non-monotonic
        # radial distance sequence at corners.
        lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(following, axis=0), axis=1))]
        target = following[min(int(np.searchsorted(lengths, .40)), len(following)-1)]
        delta = target-xy
        self.desired_heading = math.atan2(delta[1], delta[0])
        error = math.atan2(math.sin(self.desired_heading-yaw), math.cos(self.desired_heading-yaw))
        turn = np.clip(1.5*error, -self.turn_speed, self.turn_speed)
        align = abs(error) > .22
        command = np.array([0. if align else self.forward_speed, 0., turn])
        overlap = self.grid.footprint_obstructions(xy, yaw)
        for t in np.linspace(.05, .35, 7):
            future_yaw = yaw+command[2]*t
            future_xy = xy+rotation(yaw+command[2]*t/2) @ command[:2]*t
            blocked = self.grid.footprint_obstructions(future_xy, future_yaw)
            if not blocked.issubset(overlap):
                # Approach a turn's clear stance before rotating the long body.
                escape, state = self._clearance_velocity(xy, yaw)
                return escape, 'ALIGN_CLEARANCE' if np.any(escape) else state
            overlap = blocked
        if overlap:
            # An existing corner overlap also needs recentering when turning
            # does not add cells but cannot clear the current one by itself.
            return self._clearance_velocity(xy, yaw)
        return command, 'ALIGN_PATH' if align else 'FOLLOW_FACING_PATH'

    def _clearance_velocity(self, xy, yaw):
        if not self.path:
            return np.zeros(3), self.reason
        path = np.array(self.path)
        nearest = int(np.argmin(np.linalg.norm(path-xy, axis=1)))
        distances = np.linalg.norm(path[nearest:]-xy, axis=1)
        lookahead = nearest + int(np.searchsorted(distances, .22))
        target = path[min(lookahead, len(path)-1)]
        delta = target-xy
        if np.linalg.norm(self.goal-xy) < .10:
            return np.zeros(3), 'OBSERVE'
        # The bundled policy often stands still at 0.2; 0.3 produces a stable walk.
        # Use the quadruped's lateral motion, keeping the initial heading instead
        # of demanding an in-place turn in a 1 m corridor.
        body_velocity = rotation(yaw).T @ delta
        body_velocity *= .30/max(np.linalg.norm(body_velocity), 1e-6)
        preferred = math.atan2(body_velocity[1], body_velocity[0])
        best, best_score = None, math.inf
        current_overlap = self.grid.footprint_obstructions(xy, yaw)
        # Roll out alternate translation directions so a diagonal shortcut at a
        # cell corner does not deadlock a rectangular body beside a straight wall.
        offsets = (0., .25, -.25, .5, -.5, .8, -.8, math.pi/2, -math.pi/2,
                   -preferred, math.pi/2-preferred, -math.pi/2-preferred, math.pi-preferred)
        for offset in offsets:
            direction = preferred+offset
            # The policy's reverse dead zone is larger than its forward one.
            speed = self.reverse_speed if math.cos(direction) < -.5 else self.forward_speed
            command = np.array([speed*math.cos(direction), speed*math.sin(direction), 0.])
            clear = True
            remaining_overlap = current_overlap
            # Cover the 0.3 s command watchdog. An 0.8 s rollout cannot take
            # the last short approach into a cell before changing direction.
            for t in np.linspace(.05, .35, 7):
                future_yaw = yaw+command[2]*t
                future_xy = xy + rotation(yaw+command[2]*t/2) @ command[:2]*t
                overlap = self.grid.footprint_obstructions(future_xy, future_yaw)
                # Pose/grid quantization can leave a few current corner cells
                # overlapped. Permit only an escape that sheds those cells,
                # enters no new blocked cells, and ends fully clear.
                if not overlap.issubset(remaining_overlap):
                    clear = False
                    break
                remaining_overlap = overlap
            if clear and not remaining_overlap:
                distance_to_path = np.min(np.linalg.norm(path[nearest:]-future_xy, axis=1))
                score = np.linalg.norm(self.goal-future_xy)+1.5*distance_to_path+.015*abs(offset)
                if score < best_score:
                    best, best_score = command, score
        if best is None:
            return np.zeros(3), 'FOOTPRINT_BLOCKED'
        return best, 'RECENTER_FOR_CLEARANCE' if self.recovering else 'EXPLORE'
