"""stairs_lidar.py: the odom height map, lidar-only stair detection, the camera check and the
fusion, on simulated Mid-360 scans with occlusion (stair_scene.lidar_scan)."""
import math

import numpy as np
import pytest

from maze_nav.stairs import StairEstimate, wrap
from maze_nav import stairs_lidar as L
from stair_scene import R_BASE_LIDAR, T_BASE_LIDAR, lidar_scan, maze_boxes, staircase

BASE = (0.0, 0.0, 0.35)
WALLS = [((0.0, 0.5, 0.0), (4.5, 0.55, 0.45)), ((0.0, -0.55, 0.0), (4.5, -0.5, 0.45))]


def build(boxes, base=BASE, yaw=0.0, scans=10, hm=None, seed=0):
    """Height map from `scans` simulated scans at one pose (points into odom as the node does)."""
    hm = hm or L.HeightMap()
    for k in range(scans):
        pl, r_ob = lidar_scan(boxes, base, yaw, seed=seed + k)
        hm.add(pl @ (r_ob @ R_BASE_LIDAR).T + (np.asarray(base) + r_ob @ T_BASE_LIDAR), base, yaw)
    return hm


def cam_est(edge_x, heading=0.0, centre=0.0, risers=3):
    e = StairEstimate(heading=heading, along=0.0, centre=centre, risers=risers, rise=0.12)
    e.along = float(np.dot([edge_x, 0.0], e.d()))
    return e


# ---------------------------------------------------------------- height map
def test_height_map_keeps_the_top_surface_and_drops_the_robot():
    hm = L.HeightMap()
    pts = np.array([[1.0, 0.0, 0.10], [1.0, 0.0, 0.11], [1.0, 0.0, 0.02],   # one cell
                    [0.1, 0.0, 0.0],                                        # own body box
                    [2.0, 0.0, 1.5]])                                       # far above
    hm.add(pts, BASE, 0.0)
    hm.add(pts, BASE, 0.0)
    xy, h = hm.cells((0.0, 0.0), 3.0)
    assert len(h) == 1 and h[0] == pytest.approx(0.11)


def test_floor_must_be_near_the_expected_height():
    """At 1.2 m the floor before the stairs is out of the lidar's view; the first tread (12 cm)
    is the lowest dense layer. That must not be taken for the floor (it made stairs look
    anchored)."""
    assert build(staircase(1.2) + WALLS).floor(BASE) is None
    assert build(staircase(2.0) + WALLS).floor(BASE) == pytest.approx(0.0, abs=0.02)


# ---------------------------------------------------------------- lidar-only detection
@pytest.mark.parametrize('dist', [1.6, 2.0, 2.5, 3.0, 3.5])
def test_lidar_detects_stairs_ahead(dist):
    hm = build(staircase(dist) + WALLS)
    ests = L.detect(hm, BASE, hm.floor(BASE))
    assert ests
    e = ests[0]
    assert abs(math.degrees(wrap(e.heading))) < 10
    assert e.distance(np.zeros(2)) == pytest.approx(dist, abs=0.10)
    # Beyond 2 m the cells are sparse: an early warning (the robot takes over at 1.5 m,
    # re-estimated by then), so fewer steps, a looser centre and no floor anchor (too few floor
    # cells right before the stairs in 1 s of scans) are accepted there.
    near = dist <= 2.0
    assert e.source == 'lidar' and (e.anchored or not near)
    assert e.risers >= (3 if near else 2)
    assert e.lateral(np.zeros(2)) == pytest.approx(0.0, abs=0.08 if near else 0.15)


@pytest.mark.parametrize('x,y,turn', [(0.6, 0.1, 20), (0.2, -0.15, -15), (0.6, 0.0, 0),
                                      (0.4, 0.2, 10)])
def test_lidar_centre_of_the_stairs(x, y, turn):
    """The stairs' sides from the riser itself: an end-to-end run with the centre taken from
    which 0.4 m bands found steps ended 0.42 m off the centre line, next to a wall."""
    base = (x, y, 0.35)
    hm = build(staircase(2.0, width=0.9) + WALLS, base=base, yaw=math.radians(turn))
    e = L.detect(hm, base, hm.floor(base))[0]
    assert e.lateral(np.array([x, y])) == pytest.approx(y, abs=0.08)


def test_lidar_detects_stairs_off_to_the_side():
    """Robot turned 40 deg away: the camera (90 deg wide, detector +-60 deg) is at its edge,
    the lidar sees all round."""
    hm = build(staircase(1.6, width=1.0), yaw=math.radians(40))
    e = L.detect(hm, BASE, hm.floor(BASE))[0]
    assert abs(math.degrees(wrap(e.heading))) < 10
    assert e.distance(np.zeros(2)) == pytest.approx(1.6, abs=0.10)


@pytest.mark.parametrize('name,boxes', [
    ('flat floor', []),
    ('corridor and end wall', WALLS + [((2.0, -0.5, 0), (2.05, 0.5, 0.45))]),
    ('one 12 cm step (mat)', staircase(1.5, risers=1, top=1.0, width=1.0)),
    ('ramp of 1 cm steps', staircase(1.5, risers=15, rise=0.01, tread=0.03, width=1.0)),
])
def test_lidar_not_stairs(name, boxes):
    hm = build(boxes)
    assert L.detect(hm, BASE, hm.floor(BASE)) == [], name


def test_lidar_quiet_in_the_example_maze():
    """Maze walls (0.45 m) hide the stairs from the lidar from the studded-mat cell, and from
    the cell in front of them they are inside its ~1.4 m blind zone: no lidar stairs, and
    nothing false either (mats, ramps, rails)."""
    mb = maze_boxes()
    for base, yaw in (((-2.0, 1.0, 0.55), math.radians(-135)), ((0.0, 0.0, 0.45), math.pi / 2),
                      ((-1.6, 0.0, 0.45), math.pi), ((-2.0, 1.0, 0.55), math.pi / 2)):
        hm = build(mb, base, yaw, scans=6)
        assert L.detect(hm, base, hm.floor(base)) == []


# ---------------------------------------------------------------- camera check
def test_check_confirms_camera_stairs():
    hm = build(staircase(2.0) + WALLS)
    chk = L.check(hm, cam_est(2.0), hm.floor(BASE), (0.0, 0.0))
    assert chk.kind == 'confirm' and chk.anchored
    assert chk.along == pytest.approx(2.0, abs=0.10)


def test_check_vetoes_camera_stairs_on_flat_floor():
    """Camera 'stairs' 2 m ahead where the lidar has mapped flat floor (a false hit)."""
    hm = build(WALLS)
    chk = L.check(hm, cam_est(2.0), hm.floor(BASE), (0.0, 0.0))
    assert chk.kind == 'veto'


def test_check_unknown_without_coverage():
    """Close up (0.8 m) the lidar sees neither the floor nor the stairs' bottom: no say."""
    hm = build(staircase(0.8) + WALLS)
    chk = L.check(hm, cam_est(0.8), hm.floor(BASE), (0.0, 0.0))
    assert chk.kind == 'unknown'


def test_map_remembers_the_bottom_riser():
    """Mapped from 2.0 m back, then the robot stands 0.5 m from the stairs and the camera
    reports the 2nd riser (bottom one below its view): the map has the nearer one."""
    hm = build(staircase(2.0) + WALLS)                      # stairs at x = 2.0 from (0, 0)
    near_base = (1.5, 0.0, 0.35)
    floor = hm.floor(BASE)
    hm = build(staircase(2.0) + WALLS, base=near_base, scans=4, hm=hm)
    cam = cam_est(2.15)                                    # one tread (0.15) late
    chk = L.check(hm, cam, floor, near_base[:2])
    assert chk.kind == 'confirm' and chk.anchored
    fused, why = L.fuse(cam, None, chk, near_base[:2])
    assert fused.source == 'both'
    assert fused.distance(np.array(near_base[:2])) == pytest.approx(0.5, abs=0.10)
    assert 'nearer' in why


# ---------------------------------------------------------------- fusion
def test_fuse_rules():
    xy = (0.0, 0.0)
    cam, lid = cam_est(1.5), cam_est(1.45)
    lid.source, lid.centre = 'lidar', 0.1
    out, _ = L.fuse(cam, lid, None, xy)                    # agree: the camera's centre and edge
    assert out.source == 'both' and out.centre == 0.0
    assert out.distance(np.zeros(2)) == pytest.approx(1.5)
    lid2 = cam_est(1.35)
    lid2.source, lid2.anchored = 'lidar', True
    out, _ = L.fuse(cam, lid2, None, xy)                   # anchored lidar a step nearer: lidar's
    assert out.distance(np.zeros(2)) == pytest.approx(1.35)
    out, why = L.fuse(cam, None, L.Check('veto', why='flat'), xy)
    assert out is None and 'vetoed' in why
    out, _ = L.fuse(None, lid, None, xy)                   # lidar only
    assert out is lid
    far = cam_est(3.0)
    far.source = 'lidar'
    out, why = L.fuse(cam, far, None, xy)                  # disagree: the nearer is taken
    assert out is cam and 'disagree' in why
    # A confirm that is only a few cm nearer (lidar position noise) keeps the camera's edge.
    out, _ = L.fuse(cam, None, L.Check('confirm', along=1.46, anchored=True), xy)
    assert out.distance(np.zeros(2)) == pytest.approx(1.5)
    # ... a step nearer, but not anchored on the floor: the camera's edge too.
    out, _ = L.fuse(cam, None, L.Check('confirm', along=1.35, anchored=False), xy)
    assert out.distance(np.zeros(2)) == pytest.approx(1.5)
