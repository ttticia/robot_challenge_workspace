"""Nearest frontier by path length (lite3_nav.frontier); no ROS needed."""
import math

import numpy as np

from lite3_nav.core import Grid, select_frontier
from lite3_nav.frontier import geodesic_distance, select_frontier_by_path


def walled_map():
    """4 x 4 m at 5 cm, all seen free except: a wall at x = 2.0 m from y = 0 to 3.0 m, a large
    unknown room just behind it (x > 2.25, y < 1.5), and a small unknown pocket far away along
    free space but on the robot's side (x < 0.4, y > 3.4). Robot at (1.5, 0.5)."""
    data = np.zeros((80, 80), dtype=np.int16)
    data[:60, 40:42] = 100
    data[:30, 45:] = -1
    data[68:, :8] = -1
    return Grid(data, .05, (0., 0.))


def test_geodesic_distance_goes_around_walls():
    passable = np.ones((10, 10), bool)
    passable[:8, 5] = False
    d = geodesic_distance(passable, (0, 0))
    assert d[0, 9] > 15 and math.isinf(geodesic_distance(passable, (5, 0))[0, 0])


def test_nearest_along_free_space_not_behind_the_wall():
    grid = walled_map()
    goal, state = select_frontier_by_path(grid, (1.5, .5))
    assert state == 'FRONTIER' and goal[0] < 1.95  # the pocket on our side, not the room
    team, _ = select_frontier(grid, (1.5, .5))
    assert team[0] > 2.  # the team's picker goes for the big room round the wall


def test_goal_keeps_clear_of_the_walls():
    # 0.9 m corridor (y 0.5..1.4) from the robot to an unknown end; walls above and below.
    data = np.full((40, 80), 100, dtype=np.int16)
    data[10:28, :60] = 0
    data[10:28, 60:] = -1
    grid = Grid(data, .05, (0., 0.))
    goal, state = select_frontier_by_path(grid, (.5, .76))  # off-centre: 0.3 m clearance cell
    assert state == 'FRONTIER'
    # Goal cells at least 0.35 m (7 cells, centre to centre) from the walls: y 0.825..1.075.
    assert .8 <= goal[1] <= 1.1


def test_tight_frontier_still_counts_without_full_clearance():
    # 0.6 m wide strip: passable (> 0.25 m) down the middle, but nowhere 0.35 m from the edges.
    data = np.full((40, 80), 100, dtype=np.int16)
    data[14:26, :60] = 0
    data[14:26, 60:] = -1
    grid = Grid(data, .05, (0., 0.))
    goal, state = select_frontier_by_path(grid, (.5, 1.))
    assert state == 'FRONTIER' and goal[0] > 2.5, (goal, state)


def test_blacklist_and_start_checks_match_the_team_picker():
    grid = walled_map()
    first, _ = select_frontier_by_path(grid, (1.5, .5))
    second, state = select_frontier_by_path(grid, (1.5, .5), excluded=[first[:2]])
    assert state == 'FRONTIER' and math.dist(first[:2], second[:2]) >= .25
    assert select_frontier_by_path(grid, (2.02, .5)) == (None, 'START_NOT_CLEAR')
    open_map = Grid(np.zeros((40, 40), dtype=np.int16), .05, (0., 0.))
    assert select_frontier_by_path(open_map, (1., 1.)) == (None, 'NO_REACHABLE_FRONTIER')


def test_planner_radius_keeps_goals_where_the_planner_can_go():
    # The same 0.6 m strip: with the planner's 0.35 m radius nothing in it is plannable, so the
    # fallback must not hand Nav2 an unreachable goal (it "succeeds" in place and loops).
    data = np.full((40, 80), 100, dtype=np.int16)
    data[14:26, :60] = 0
    data[14:26, 60:] = -1
    grid = Grid(data, .05, (0., 0.))
    assert select_frontier_by_path(grid, (.5, 1.), planner_radius=.35) == (None, 'NO_REACHABLE_FRONTIER')
    # A 0.9 m corridor still has a plannable centre lane: goal on it, > 0.35 m from both walls.
    data = np.full((40, 80), 100, dtype=np.int16)
    data[10:28, :60] = 0
    data[10:28, 60:] = -1
    grid = Grid(data, .05, (0., 0.))
    goal, state = select_frontier_by_path(grid, (.5, .95), planner_radius=.35)
    assert state == 'FRONTIER' and .87 <= goal[1] <= 1.03, goal


def test_minimum_hop_and_ignored_goals():
    grid = walled_map()
    first, _ = select_frontier_by_path(grid, (1.5, .5))
    hop, state = select_frontier_by_path(grid, (1.5, .5), minimum_hop=2.)
    assert state == 'FRONTIER' and math.dist(hop[:2], (1.5, .5)) >= 2.
    other, state = select_frontier_by_path(grid, (1.5, .5), ignored=[first[:2]])
    assert state == 'FRONTIER' and math.dist(other[:2], first[:2]) >= .5


def test_lidar_ray_speckle_counts_as_seen_floor():
    # 1 m room (x < 1.0) opening east onto floor seen only as diverging lidar rays: every other
    # row free, the rows between unknown. The frontier is out past the speckle, not at the
    # robot's feet in the room (sim 2026-11-15).
    data = np.full((40, 100), 100, dtype=np.int16)
    data[5:35, 1:20] = 0                      # room, x 0.05..1.0, y 0.25..1.75
    data[5:35, 20:70] = -1                    # beyond: unknown ...
    data[5:35:2, 20:70] = 0                   # ... crossed by lidar rays
    data[5:35, 70:] = -1
    grid = Grid(data, .05, (0., 0.))
    goal, state = select_frontier_by_path(grid, (.5, 1.), planner_radius=.35, minimum_hop=.35)
    assert state == 'FRONTIER' and goal[0] > 1.5, (goal, state)
