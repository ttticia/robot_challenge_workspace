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
        self.shares, self.talls, self.grounded = {}, {}, set()
        self.cv_seen = {}             # YOLO proposes (user, 2026-11-27): sightings per edge

    def get_logger(self): return self.Log()
    def event(self, text): self.events.append(text)
    def wall_share(self, c, d): return self.shares.get((c, d), 1.0)
    def tall_share(self, sense, c, d): return self.talls.get((c, d))

    def sense(self):
        g = self.grounded
        return type('S', (), {'solid_base': staticmethod(lambda c, d: 1.0 if (c, d) in g else 0.0)})()
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


def test_a_tall_edge_is_asked_of_the_camera_once():
    # user, 2026-11-26 night: a curtain MUST be confirmed through the camera; the last
    # resort faces the tall edge and runs cv_verify, it never pushes unconfirmed
    r = Robot()
    r.talls[((1, 0), 0)] = 0.6                # tall along most of it: fabric-like
    r.cv_seen[((1, 0), 0)] = [1.0]            # and YOLO has proposed it
    assert r.force_tall_curtain((1, 0), r.visited)
    e = ((1, 0), 0)
    assert e in r.curtain_forced and e not in r.cv_curtains   # the camera decides
    assert [a[0] for a in r.queue] == ['turn', 'shift', 'turn', 'hold', 'cv_verify']   # centred first
    assert r.queue[-1][1:] == ((1, 0), 0, True)               # final: a no is a wall
    assert any('asking the camera' in ev for ev in r.events)
    assert not r.force_tall_curtain((1, 0), r.visited)        # each edge once


def test_a_low_or_failed_edge_is_never_forced():
    r = Robot()
    r.cv_seen[((1, 0), 0)] = [1.0]
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


def test_an_edge_standing_on_the_floor_goes_last():
    # 21:12 run: the first forced candidate was a tall wall standing down to the floor.
    # A curtain leaves ~0.15 m free under it: grounded edges are tried last, not first.
    r = Robot()
    r.visited.add((2, 0))
    for d in range(4):
        r.kinds[((2, 0), d)] = 'wall'
    r.kinds[((1, 0), 0)] = r.kinds[((2, 0), 2)] = 'free'
    r.talls[((1, 0), 1)] = 0.9                # near, but stands down to the floor
    r.cv_seen.update({((1, 0), 1): [1.0], ((2, 0), 0): [1.0]})
    r.grounded.add(((1, 0), 1))
    r.talls[((2, 0), 0)] = 0.9                # a cell away, floor-gap under it
    assert r.force_tall_curtain((1, 0), r.visited)
    assert r.gone == [((1, 0), 0, 'walk')]    # walking towards the gapped one first
    r.gone.clear()
    assert r.force_tall_curtain((2, 0), r.visited)            # there now: ask the camera
    assert ((2, 0), 0) in r.curtain_forced and ((1, 0), 1) not in r.curtain_forced
    assert any('no exit found anywhere else' in e for e in r.events)
    # the grounded one is still asked about when it is all that is left (last resort)
    r.queue.clear()
    assert r.force_tall_curtain((1, 0), r.visited)
    assert ((1, 0), 1) in r.curtain_forced
    assert any('stands down to the floor' in e for e in r.events)


def test_a_failed_push_is_not_rechecked_as_a_soft_wall():
    r = Robot()
    r.shares[((1, 0), 0)] = 0.1               # soft by the map ...
    r.push_failed.add(((1, 0), 0))            # ... but the push proved it solid
    assert not r.recheck_soft_wall((1, 0), r.visited)


def test_the_first_grid_fit_forgets_the_edge_keyed_memories():
    # the cells move with the first wide fit: everything keyed by (cell, side) from
    # before it names the wrong edge (the fit can come late: 19:05 run, never at all)
    import inspect
    src = inspect.getsource(E.align_grid)
    for name in ('curtain_forced', 'push_failed', 'soft_rechecked', 'vlm_stairs',
                 'stair_rechecked', 'look_checked', 'no_views', 'cv_curtains',
                 'curtain_latched', 'cv_rejected', 'forced_wall'):
        assert f'{name}.clear()' in src or f"'{name}'" in src, name
