"""User, 2026-11-27: YOLO proposes a curtain, the lidar confirms it by height, and the
curtain-high stretch must be WIDER than 0.7 m (Sense.tall_width). No gap check."""
import inspect
from collections import deque

import numpy as np
import pytest

from maze_explore.grid import Grid, nb, opp
from maze_explore.sense import HeightMap, Sense

CELL, D = (1, 0), 0
G = Grid(0.0, 0.0, 0.0, 1.02)


class Occ:
    def at(self, x, y):
        return np.full(np.shape(x), 100, np.int8)


def heights(lo, hi, h=0.85, n=3):
    """A height map with returns at `h` along the edge (cell (1, 0), side 0, x = 1.53)
    from along = lo to along = hi (m), seen n times, and 0.53 m elsewhere on it."""
    hm = HeightMap()
    ys = np.arange(-0.50, 0.501, 0.01)
    for _ in range(n):
        z = np.where((ys >= lo) & (ys <= hi), h, 0.53)
        hm.add(np.full_like(ys, 1.53), ys, z)
    return hm


def width(lo, hi):
    return Sense(G, Occ(), heights(lo, hi)).tall_width(CELL, D)


def test_a_full_curtain_is_wide_a_tall_prop_is_not():
    assert width(-0.50, 0.50) >= 0.85
    assert width(-0.15, 0.15) <= 0.40
    assert width(-0.50, 0.50) > 0.7 > width(-0.25, 0.25)


def test_unmeasured_edge_has_no_width():
    assert Sense(G, Occ(), HeightMap()).tall_width(CELL, D) is None


def test_a_one_square_split_is_bridged():
    hm = heights(-0.50, 0.50)
    s = Sense(G, Occ(), hm)
    full = s.tall_width(CELL, D)
    hm.tall_hits[hm._idx(np.array([1.53]), np.array([0.0]))[:2]] = 0   # the 1-3 cm split
    assert s.tall_width(CELL, D) == pytest.approx(full)


explorer = pytest.importorskip('maze_explore.explorer')
E = explorer.Explorer


class Robot:
    do_cv_verify, curtain_lidar_check, tall_share = E.do_cv_verify, E.curtain_lidar_check, E.tall_share

    class Log:
        def info(self, *a, **k): pass
        warn = info

    def __init__(self, lo, hi):
        self.grid, self.heights, self.occ = G, heights(lo, hi), Occ()
        self.pose, self.t, self.act_t0 = [0.0, 0.0, 0.0], 100.0, 100.0
        self.queue, self.events = deque(), []
        self.cv_curtains, self.cv_rejected, self.cv_pending = set(), set(), set()
        self.curtain_latched, self.forced_wall = set(), set()
        self.cv_tall_min, self.curtain_width, self.cv_verify_camera = 0.2, 0.7, False
        self.curtain_height, self.curtain_flat_only = True, False
        self.cv_frames = deque()      # the camera sees nothing at the check: the lidar decides

    def now(self): return self.t
    def elapsed(self): return self.t - self.act_t0
    def get_logger(self): return self.Log()
    def event(self, text): self.events.append(text)
    def send(self, *a): pass
    def curtain_flat(self, *a): return True
    def sense(self): return Sense(self.grid, self.occ, self.heights)


def test_wide_and_tall_is_confirmed_by_the_lidar():
    r = Robot(-0.50, 0.50)
    assert r.do_cv_verify(CELL, D, True)
    assert (CELL, D) in r.cv_curtains and (nb(CELL, D), opp(D)) in r.cv_curtains
    assert any('confirmed by the lidar' in e for e in r.events)


def test_tall_but_narrow_is_not_a_curtain():
    r = Robot(-0.20, 0.20)
    assert r.do_cv_verify(CELL, D, True)
    assert (CELL, D) not in r.cv_curtains and (CELL, D) in r.cv_rejected
    assert any('wide' in e and 'lidar check failed' in e for e in r.events)


def test_low_edge_is_not_a_curtain():
    r = Robot(0.0, -1.0)                      # nothing above 0.53 m
    assert r.do_cv_verify(CELL, D, True)
    assert (CELL, D) in r.cv_rejected


def test_defaults():
    src = inspect.getsource(E.__init__)
    for s in ("p('curtain_height', True)", "p('curtain_width', 0.7)", "p('curtain_gap', False)",
              "p('cv_verify_camera', False)", "p('cv_sure', 1.0)"):
        assert s in src
