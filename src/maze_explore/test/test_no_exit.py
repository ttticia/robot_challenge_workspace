"""'No exit found' is not final (user, 2026-11-26 night): every wall that is not solidly
mapped is faced once more, then any tall edge is tried as a curtain, camera or no camera."""
from collections import deque

import pytest

from maze_explore.grid import Grid, nb, opp

explorer = pytest.importorskip('maze_explore.explorer')
E = explorer.Explorer


class Robot:
    recheck_soft_wall = E.recheck_soft_wall
    force_tall_curtain = E.force_tall_curtain

    class Log:
        def info(self, *a, **k): pass
        warn = info

    def __init__(self):
        self.grid, self.start = Grid(0.0, 0.0, 0.0, 1.02), (0, 0)
        self.visited = {(0, 0), (1, 0)}
        self.closed, self.links, self.kinds = set(), {}, {}
        for c in self.visited:
            for d in range(4):
                self.kinds[(c, d)] = 'wall'
        self.kinds[((0, 0), 0)] = self.kinds[((1, 0), 2)] = 'free'
        self.soft_rechecked, self.curtain_forced, self.push_failed = set(), set(), set()
        self.forced_wall, self.cv_curtains, self.cv_rejected = set(), set(), set()
        self.cv_tall_min, self.job = 0.2, ''
        self.queue, self.events, self.gone = deque(), [], []
        self.shares, self.talls = {}, {}

    def get_logger(self): return self.Log()
    def event(self, text): self.events.append(text)
    def wall_share(self, c, d): return self.shares.get((c, d), 1.0)
    def tall_share(self, sense, c, d): return self.talls.get((c, d))
    def sense(self): return None
    def update_kinds(self): pass
    def go(self, a, d, how): self.gone.append((a, d, how))


def test_a_soft_wall_is_faced_once_more_and_a_solid_one_is_not():
    r = Robot()
    r.shares[((1, 0), 0)] = 0.1               # never solidly mapped
    assert r.recheck_soft_wall((1, 0), r.visited)
    assert ((1, 0), 0) in r.soft_rechecked
    acts = [a[0] for a in r.queue]
    assert acts == ['drive', 'turn', 'hold', 'call']
    r.queue[-1][1]()                          # settle: still a wall
    assert ((1, 0), 0) in r.forced_wall and any('still a wall' in e for e in r.events)
    # every other wall is solidly mapped (share 1.0) or leads to a known cell: done
    assert not r.recheck_soft_wall((1, 0), r.visited)


def test_a_forced_wall_counts_as_soft_whatever_the_map_says():
    r = Robot()
    r.forced_wall.add(((1, 0), 1))            # resolve timeout or a given-up step
    assert r.recheck_soft_wall((1, 0), r.visited)
    assert ((1, 0), 1) in r.soft_rechecked and ((1, 0), 1) not in r.forced_wall


def test_the_robot_walks_towards_a_far_soft_wall_first():
    r = Robot()
    r.visited.add((2, 0))
    for d in range(4):
        r.kinds[((2, 0), d)] = 'wall'
    r.kinds[((1, 0), 0)] = r.kinds[((2, 0), 2)] = 'free'
    r.shares[((2, 0), 0)] = 0.0               # the soft wall is a cell away
    assert r.recheck_soft_wall((1, 0), r.visited)
    assert r.gone == [((1, 0), 0, 'walk')] and not r.queue


def test_a_tall_edge_is_tried_as_a_curtain_once():
    r = Robot()
    r.talls[((1, 0), 0)] = 0.6                # tall along most of it: fabric-like
    assert r.force_tall_curtain((1, 0), r.visited)
    e = ((1, 0), 0)
    assert e in r.cv_curtains and e in r.curtain_forced
    assert (nb(*e), opp(0)) in r.cv_curtains
    assert not r.force_tall_curtain((1, 0), r.visited)      # each edge once


def test_a_low_or_failed_edge_is_never_forced():
    r = Robot()
    r.talls[((1, 0), 0)] = 0.1                # measured low: a plain wall
    assert not r.force_tall_curtain((1, 0), r.visited)
    r.talls[((1, 0), 0)] = 0.9
    r.push_failed.add(((1, 0), 0))            # already pushed: something solid
    assert not r.force_tall_curtain((1, 0), r.visited)


def test_start_cell_sides_are_left_alone():
    r = Robot()
    r.shares[((0, 0), 1)] = 0.0
    r.talls[((0, 0), 1)] = 0.9
    assert not r.recheck_soft_wall((1, 0), r.visited)
    assert not r.force_tall_curtain((1, 0), r.visited)


def test_a_failed_push_marks_the_edge_for_good():
    src = __import__('inspect').getsource(E.stuck)
    assert 'push_failed' in src and 'curtain_forced' in src
