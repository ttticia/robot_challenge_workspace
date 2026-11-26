"""The stairs are crossed once, never again (user, 2026-11-26 night)."""
from collections import deque

import pytest

from maze_explore.grid import Grid, nb, opp
from maze_explore.plan import next_job, routes

ARGS = dict(exit_cell=None, looked=set(), greeted=set())


def test_routes_never_go_back_over_the_stairs():
    a, c = (0, 0), (2, 0)
    links = {(a, 0): ('stairs', c), (c, 2): ('stairs', a)}
    kinds = {(a, d): 'wall' for d in range(4)}
    kinds.update({(c, d): 'wall' for d in range(4)})
    reach = routes(kinds, {a, c}, {(1, 0)}, links, c)
    assert a not in reach                 # only over the stairs: not reachable any more


def test_no_second_stairs_job_after_a_crossing():
    a, c = (0, 0), (2, 0)
    links = {(a, 0): ('stairs', c), (c, 2): ('stairs', a)}
    # after the crossing, something else in front of c reads as stairs (a prop, the kitchen)
    kinds = {(c, 0): 'stairs', (c, 1): 'wall', (c, 2): 'wall', (c, 3): 'wall'}
    j = next_job(kinds, {a, c}, {(1, 0)}, links, cur=c, facing=0, **ARGS)
    assert j['job'] == 'done'
    # and without a crossing it would be crossed
    j = next_job(kinds, {c}, set(), {}, cur=c, facing=0, **ARGS)
    assert j['job'] == 'stairs'


def test_unexplored_cells_back_over_the_stairs_are_left():
    # a way into an unvisited cell beside the cell BEFORE the stairs: not walked back to
    a, c = (0, 0), (2, 0)
    links = {(a, 0): ('stairs', c), (c, 2): ('stairs', a)}
    kinds = {(a, 1): 'free', (a, 2): 'wall', (a, 3): 'wall',
             (c, 0): 'wall', (c, 1): 'wall', (c, 3): 'wall'}
    j = next_job(kinds, {a, c}, {(1, 0)}, links, cur=c, facing=0, **ARGS)
    assert j['job'] == 'done'


explorer = pytest.importorskip('maze_explore.explorer')


class Robot:
    stairs_crossed = explorer.Explorer.stairs_crossed
    settle_ai_pending = explorer.Explorer.settle_ai_pending
    arrive = explorer.Explorer.arrive
    go = explorer.Explorer.go

    class Log:
        def info(self, *a, **k): pass
        warn = info

    def __init__(self, x, y):
        self.grid, self.cell_size = Grid(0.0, 0.0, 0.0, 1.02), 1.02
        self.pose = [x, y, 0.0]
        self.links, self.closed, self.visited = {}, set(), {(0, 0)}
        self.start, self.left_start, self.kinds = (0, 0), True, {}
        self.stairs_done, self.ai_pending, self.step = False, None, None
        self.shifted_in, self.no_recentre = None, set()
        self.rough_cells, self.rough_stand = set(), False
        self.forced_wall, self.gait, self.cur_gait = set(), 'hstep', 'ai'
        self.queue, self.events = deque(), []

    def get_logger(self): return self.Log()
    def event(self, text): self.events.append(text)


def test_a_crossing_closes_the_stair_cell_and_sets_the_flag():
    r = Robot(2.04, 0.0)
    r.stairs_crossed((0, 0), 0)
    assert r.stairs_done and (1, 0) in r.closed and (2, 0) not in r.visited
    assert r.links[((0, 0), 0)] == ('stairs', (2, 0))


def test_go_refuses_a_second_crossing():
    r = Robot(2.04, 0.0)
    r.stairs_done = True
    r.go((2, 0), 0, 'stairs')
    assert not r.queue and ((2, 0), 0) in r.forced_wall and ((3, 0), 2) in r.forced_wall


def test_hand_finished_crossing_counts_at_resume():
    r = Robot(2.0, 0.05)                        # carried past the stairs after an AI stop
    r.ai_pending = ((0, 0), 0, (2, 0))
    assert r.settle_ai_pending() == (2, 0)
    assert r.stairs_done and (1, 0) in r.closed and ('resolve', (2, 0)) not in r.queue
    assert r.cur_gait == 'hstep' and r.ai_pending is None


def test_stop_after_the_ai_walk_still_counts_at_resume():
    r = Robot(1.7, 0.0)                         # e.g. /explore/stop during the leave sequence
    r.ai_pending = ((0, 0), 0, (2, 0))
    r.stairs_crossed((0, 0), 0)
    assert r.settle_ai_pending() == (2, 0) and (2, 0) in r.visited


def test_stopped_short_of_the_stairs_is_not_a_crossing():
    r = Robot(0.3, 0.0)
    r.ai_pending = ((0, 0), 0, (2, 0))
    assert r.settle_ai_pending() is None
    assert not r.stairs_done and not r.closed and r.ai_pending is None
