"""Exit / final-cell detection and the beyond-exit frontier mask (no ROS graph needed)."""
import math

import numpy as np
from lite3_nav.core import Grid, select_frontier
from lite3_nav.exit_detect import ExitDetector, exit_masked_grid, open_region, seen_free_clearance

RES = .05


def arena(outside_unknown_from=None):
    """A 0.9 m wide corridor (x 0..4 m, y 1.05..1.95 m) whose east end opens onto 3 m x 3 m of
    seen-free floor (x 4..7 m). West end closed. outside_unknown_from: x from which the outside
    is still unknown (a frontier out there)."""
    data = np.full((60, 160), -1, dtype=int)            # 3 m x 8 m
    rows = slice(int(1.05/RES), int(1.95/RES))
    data[rows, 0:int(4/RES)] = 0
    data[int(1.05/RES)-1, 0:int(4/RES)] = 100
    data[int(1.95/RES), 0:int(4/RES)] = 100
    data[rows.start-1:rows.stop+1, 0] = 100
    data[:, int(4/RES):int(7/RES)] = 0
    if outside_unknown_from is not None:
        data[:, int(outside_unknown_from/RES):] = -1
    return Grid(data, RES, (0., 0.))


def run(detector, grid, xy, seconds, start_time=0.):
    t = start_time
    while t <= start_time+seconds:
        found = detector.update(grid, xy, t)
        t += .5
    return found, t


def test_corridor_is_not_the_exit_and_open_floor_is():
    grid = arena()
    d = ExitDetector()
    start = (.5, 1.5)
    assert not run(d, grid, start, 3.)[0]                   # enclosed in the corridor
    assert d.seen_enclosed and d.clearance <= .7
    assert not run(d, grid, (3., 1.5), 3., 4.)[0]          # deep in the corridor: no 1.1 m disk near
    found, _ = run(d, grid, (4.5, 1.5), 3., 8.)            # just out of the opening
    assert found and d.reason == 'OPEN_FLOOR' and d.open_nearby >= 1.1


def test_dwell_and_clock_jump_restart():
    grid = arena()
    d = ExitDetector(dwell=2.)
    d.update(grid, (.5, 1.5), 0.)
    assert not d.update(grid, (4.5, 1.5), 1.)
    assert not d.update(grid, (4.5, 1.5), 2.5)            # 1.5 s < dwell
    assert not d.update(grid, (4.5, 1.5), 10.)            # paused 7.5 s: start over
    assert not d.update(grid, (4.5, 1.5), 11.)
    assert d.update(grid, (4.5, 1.5), 12.)


def test_open_start_area_is_not_the_exit():
    grid = arena()
    d = ExitDetector()
    d.reset((5.5, 1.5))                                    # started outside, never moved
    assert not run(d, grid, (5.5, 1.5), 5.)[0]


def test_final_cell_ends_at_once_in_any_mode():
    grid = arena()
    d = ExitDetector(final_cell_xy=(3.6, 1.5), final_cell_radius=.4)
    d.update(grid, (.5, 1.5), 0., open_test=False)
    assert not d.update(grid, (3., 1.5), .5, open_test=False)
    assert d.update(grid, (3.3, 1.5), 1., open_test=False) and d.reason == 'FINAL_CELL'
    assert d.update(grid, (0., 0.), 1.5)                   # latched
    d.reset()
    assert not d.found and d.reason == ''


def test_rotated_map():
    base = arena()
    yaw = math.pi/2
    grid = Grid(base.data, RES, (2., -1.), yaw)
    to_world = lambda x, y: tuple(grid.world([[x/RES-.5, y/RES-.5]])[0])  # noqa: E731
    d = ExitDetector()
    run(d, grid, to_world(.5, 1.5), 3.)
    assert run(d, grid, to_world(4.5, 1.5), 3., 4.)[0]


def test_open_region_masks_frontiers_beyond_the_exit():
    grid = arena(outside_unknown_from=6.5)                 # frontier out on the open floor
    start = (.5, 1.5)
    goal, state = select_frontier(grid, (3.5, 1.5))
    assert state == 'FRONTIER' and goal[0] > 4.               # unmasked: drives out there
    region = open_region(grid, 1.1, start, 1.5)
    assert region.any() and not region[int(1.5/RES), int(3.5/RES)]    # corridor stays visible
    masked = exit_masked_grid(grid, 1.1, start, 1.5)
    assert masked is not None and masked.yaw == grid.yaw
    goal, state = select_frontier(masked, (3.5, 1.5))
    assert goal is None and state == 'NO_REACHABLE_FRONTIER'  # nothing left inside


def test_mask_keeps_frontiers_inside_the_arena():
    grid = arena(outside_unknown_from=6.5)
    data = grid.data.copy()
    data[int(1.05/RES):int(1.95/RES), int(1.0/RES):int(2.0/RES)] = -1   # unexplored corridor part
    grid = Grid(data, RES, (0., 0.))
    masked = exit_masked_grid(grid, 1.1, (.5, 1.5), 1.5)
    goal, state = select_frontier(masked, (3.5, 1.5), clearance=.2)
    assert state == 'FRONTIER' and goal[0] < 4.


def test_no_open_floor_no_mask():
    data = np.full((40, 40), 0)
    data[:, 18:22] = 100                                   # nothing 1.1 m wide away from walls
    grid = Grid(data, RES, (0., 0.))
    assert exit_masked_grid(grid, 1.1) is None
    assert seen_free_clearance(grid).max() < 1.1
