"""Nearest frontier by path length (idea and geodesic_distance from maze_nav, origin/navigation
419f205, maze_nav/core.py), on the same map model as lite3_nav.core.select_frontier.

select_frontier (the team's, core.py) ranks frontiers by size over straight-line distance, so in
a maze it can pick a frontier just behind a wall that is a long walk away, and it keeps leaving
nearby dead ends half explored. select_frontier_by_path picks the frontier whose approach point
is closest along free space, so the robot finishes the nearest branch first (closest frontier
first, as in maze_nav). Same arguments, same return values, same safety filters, plus
goal_clearance: the goal itself must be at least that far from any wall (occupied cell; unknown
is not a wall, as in maze_nav's clearance_map, 0.33 m there). Counting unknown cells too left no
goal next to a frontier at all (sim: NO_REACHABLE_FRONTIER at the start, 2026-11-15). Where no
approach point has goal_clearance, the frontier still counts with its best reachable one, as
before goal_clearance existed: it only ranks goals, it never hides a frontier. Without it the nearest approach point sat on the edge of the clearance mask,
0.25 m from a wall, and Nav2 clipped corners getting there (sim EXCESS_TILT, 2026-11-15).
"""
import math

import numpy as np
from scipy.ndimage import binary_closing, binary_dilation, distance_transform_edt, label


def geodesic_distance(passable, start):
    """Shortest path length in cells (8-connected) over `passable` from start=(col,row); inf
    where unreachable."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    h, w = passable.shape
    dist = np.full((h, w), np.inf)
    if not (0 <= start[0] < w and 0 <= start[1] < h) or not passable[start[1], start[0]]:
        return dist
    ids = np.full((h, w), -1, dtype=np.int64)
    rows, cols = np.nonzero(passable)
    ids[rows, cols] = np.arange(len(rows))
    src, dst, cost = [], [], []
    for dy, dx, c in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
        a = ids[0:h - dy, max(0, -dx):w - max(0, dx)]
        b = ids[dy:h, max(0, dx):w - max(0, -dx)]
        m = (a >= 0) & (b >= 0)
        src.append(a[m])
        dst.append(b[m])
        cost.append(np.full(m.sum(), c))
    n = len(rows)
    graph = coo_matrix((np.concatenate(cost), (np.concatenate(src), np.concatenate(dst))),
                       shape=(n, n)).tocsr()
    dist[rows, cols] = dijkstra(graph, directed=False, indices=int(ids[start[1], start[0]]))
    return dist


def fill_ray_gaps(data, iterations=2):
    """Seen-free cells, plus unknown cells in the small gaps between lidar rays. Far from the
    robot the rays fan out and leave a speckle of free and unknown cells (sim 2026-11-15, the open
    floor past the start room): taken literally, no path through it keeps clear of "unknown",
    so the frontier sat at the robot's feet and the run ended. Occupied cells never change."""
    free = (data >= 0) & (data <= 25)
    closed = binary_closing(free, structure=np.ones((3, 3), dtype=bool), iterations=iterations,
                            border_value=0)
    return free | (closed & (data < 0))


def select_frontier_by_path(grid, xy, clearance=.25, standoff=.32, minimum_size=.12, excluded=(),
                            goal_clearance=.35, planner_radius=0., ignored=(),
                            ignore_radius=.5, minimum_hop=0.):
    """Like core.select_frontier, but the goal is the approach point nearest along free space."""
    known_free = fill_ray_gaps(grid.data)
    occupied = grid.data >= 65
    unknown = (grid.data < 0) & ~known_free
    wall_distance = distance_transform_edt(known_free)*grid.resolution
    safe = known_free & (wall_distance > clearance)
    start = grid.cell(xy)
    if not grid.contains(start) or not safe[start[1], start[0]]:
        return None, 'START_NOT_CLEAR'
    path = geodesic_distance(safe, start)*grid.resolution
    reachable = np.isfinite(path)
    # planner_radius: the global costmap's robot_radius. Cells that close to a wall are lethal to
    # the planner, so a goal there is never reached: Nav2 stops within its tolerance, often where
    # the robot already is, and the same frontier comes back (sim 2026-11-15). 0 = no limit.
    occupied_distance = distance_transform_edt(~occupied)*grid.resolution
    plannable = occupied_distance > planner_radius if planner_radius > 0 else np.ones_like(occupied)
    goal_ok = reachable & plannable & (occupied_distance >= max(goal_clearance, planner_radius+grid.resolution))
    boundary = known_free & binary_dilation(unknown) & ~binary_dilation(occupied, iterations=2)
    borders, count = label(boundary, structure=np.ones((3, 3), dtype=int))
    sizes = np.bincount(borders.ravel(), minlength=count+1)
    eligible = [i for i in range(1, count+1) if sizes[i]*grid.resolution >= minimum_size]
    best, best_path = None, math.inf
    for i in sorted(eligible, key=lambda i: sizes[i], reverse=True)[:64]:
        border = borders == i
        near = distance_transform_edt(~border)*grid.resolution <= standoff+.12
        approaches = np.argwhere(goal_ok & near)
        if not len(approaches):
            approaches = np.argwhere(reachable & plannable & near)
        if not len(approaches):
            continue
        lengths = path[approaches[:, 0], approaches[:, 1]]
        if lengths.min() >= best_path:
            continue
        centre = grid.world(np.argwhere(border)[:, ::-1]).mean(axis=0)
        points = grid.world(approaches[:, ::-1])
        for index in np.argsort(lengths):
            point = points[index]
            if lengths[index] < .25 or any(np.linalg.norm(point-np.asarray(p)) < .25 for p in excluded):
                continue
            # minimum_hop: a goal inside Nav2's goal tolerance "succeeds" without moving.
            # ignored: goals that did exactly that (their frontier cannot be cleared from there).
            if math.dist(point, xy[:2]) < minimum_hop or \
                    any(np.linalg.norm(point-np.asarray(p)) < ignore_radius for p in ignored):
                continue
            heading = math.atan2(centre[1]-point[1], centre[0]-point[0])
            best, best_path = (float(point[0]), float(point[1]), heading), float(lengths[index])
            break
    return best, 'FRONTIER' if best else 'NO_REACHABLE_FRONTIER'
