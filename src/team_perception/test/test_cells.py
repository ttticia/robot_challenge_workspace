"""Survivor location = centre of the cell the poster is seen in."""
import math
import random

import pytest

from team_perception.poster_utils import POSTER, MazeCells, PosterTracker, Track, pull_back


def test_cell_and_centre():
    c = MazeCells(1.0, (0.0, 0.0), 0.0)
    assert c.cell((0.3, -0.4)) == (0, 0)
    assert c.cell((1.6, 0.2)) == (2, 0)
    assert c.centre((2, -1)) == pytest.approx((2.0, -1.0))
    r = MazeCells(1.0, (0.1, 0.2), math.radians(90))      # rotated, shifted grid
    assert r.cell((0.1, 1.2)) == (1, 0)
    assert r.centre((1, 0)) == pytest.approx((0.1, 1.2))


def test_same_convention_as_maze_nav():
    cg = pytest.importorskip('maze_nav.scene_cells')
    rnd = random.Random(1)
    for _ in range(200):
        size, o, yaw = rnd.uniform(0.5, 1.5), (rnd.uniform(-1, 1), rnd.uniform(-1, 1)), \
            rnd.uniform(-math.pi, math.pi)
        a, b = MazeCells(size, o, yaw), cg.CellGrid(size, o, yaw)
        xy = (rnd.uniform(-5, 5), rnd.uniform(-5, 5))
        assert a.cell(xy) == b.cell(xy)
        assert a.centre(a.cell(xy)) == pytest.approx(b.centre(b.cell(xy)))


def test_poster_on_wall_is_placed_on_the_camera_side():
    # Camera optical frame: z forward. Poster 1.7 m ahead on a wall; with the robot at the
    # start-cell centre facing +x (map), the wall is the x = 1.5 edge between cells 1 and 2.
    p = pull_back((0.0, 0.0, 1.5), 0.25)
    assert p == pytest.approx((0.0, 0.0, 1.25))
    # map x = camera z here: the room point is 0.25 m into cell 1, not on the edge
    assert MazeCells().cell((p[2], 0.0)) == (1, 0)
    assert pull_back((0.0, 0.0, 0.2), 0.25) == pytest.approx((0.0, 0.0, 0.1))   # too close
    assert pull_back((0.0, 0.0, 0.0), 0.25) == (0.0, 0.0, 0.0)


def test_room_point_is_averaged_with_the_track():
    from collections import deque
    t = Track(1, 0.0, 0.0, 0.0, deque(maxlen=8), 0.0)
    t.add_room((1.0, 0.0, 0.0))
    t.add_room((1.2, 0.0, 0.0))
    assert t.room[0] == pytest.approx(1.1) and t.n_room == 2


def test_tracks_in_one_frame_merge_sightings():
    tr = PosterTracker(0.5, need=5)
    ids = {tr.update((2.0 + 0.05 * k, 0.0, 0.0), POSTER, float(k))[0].id for k in range(6)}
    assert ids == {1}


def test_poster_on_the_near_half_of_a_side_wall_belongs_to_the_room():
    """2026-11-23: room (1, 0) entered from (0, 0) along +i; its left wall is the edge
    v = +0.5 between (1, 0) and (1, 1). A poster on the NEAR half of it (u = 0.6), seen from
    (0, 0): the ray pull-back lands in (0, 0); the wall rule puts it in (1, 0)."""
    c = MazeCells(1.02, (0.0, 0.0), 0.0)
    camera = (0.0, 0.0, 0.0)
    poster = (0.6 * 1.02, 0.51, 0.4)
    ray = [p - q for p, q in zip(poster, camera)]
    room = [q + v * (1 - 0.25 / math.dist(poster, camera)) for q, v in zip(camera, ray)]
    assert c.cell(room[:2]) == (0, 0)                      # the old placement: wrong cell
    assert c.room_cell(poster[:2], room[:2]) == (1, 0)
    # back wall seen head-on, and a rotated grid
    assert c.room_cell((1.53, 0.1), (1.2, 0.05)) == (1, 0)
    r = MazeCells(1.02, (0.3, -0.2), math.radians(90))
    assert r.room_cell(r.centre((0.6, 0.5))[:2], r.centre((0.3, 0.3))) == (1, 0)
