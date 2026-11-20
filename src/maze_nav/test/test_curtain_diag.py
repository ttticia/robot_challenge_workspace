"""curtains.diagnose (near-miss log line), CurtainTracker.votes, wall band 0.30-0.45."""
import numpy as np

from maze_nav.curtains import CurtainParams, CurtainTracker, detect_curtains, diagnose
from test_curtains import LIDAR_H, corridor, curtain_box, raycast


def run(boxes, p=None, xy=(0.0, 0.0)):
    pts = raycast(boxes, origin=(xy[0], xy[1], LIDAR_H))
    cells = []
    found = detect_curtains(pts, xy, 0.0, 0.0, p, cells_out=cells)
    return found, cells


def test_band_matches_wall_scan():
    assert CurtainParams().band == (0.30, 0.45)


def test_found_curtain_fills_cells_out():
    found, cells = run(corridor(curtain_box(x=1.6, gap=0.08)))
    assert len(found) == 1
    assert any(c['kind'] == 'curtain' for c in cells)


def test_20_cm_gap_is_reported_as_a_hurdle():
    """The lab curtain (~20 cm up) with the 15 cm spec: the log says why."""
    found, cells = run(corridor(curtain_box(x=1.6, gap=0.20)))
    assert found == []
    msg = diagnose(cells)
    assert msg is not None and 'hurdle' in msg and 'max_gap 15 cm' in msg


def test_20_cm_gap_is_a_curtain_with_max_gap_25():
    found, _ = run(corridor(curtain_box(x=1.6, gap=0.20)), CurtainParams(max_gap=0.25))
    assert len(found) == 1


def test_plain_wall_says_nothing():
    found, cells = run(corridor((1.6, -0.45, 0.0, 1.62, 0.45, 0.45)))
    assert found == [] and diagnose(cells) is None


def test_narrow_curtain_is_reported_as_too_narrow():
    boxes = corridor(curtain_box(x=1.6, gap=0.08, half=0.25))
    boxes += [(1.6, 0.25, 0.0, 1.62, 0.45, 0.45), (1.6, -0.45, 0.0, 1.62, -0.25, 0.45)]
    found, cells = run(boxes)
    assert found == []
    msg = diagnose(cells)
    assert msg is not None and ('wide' in msg or 'bins' in msg)


def test_tracker_votes():
    found, _ = run(corridor(curtain_box(x=1.6, gap=0.08)))
    t = CurtainTracker(window=3.0, need=3)
    assert t.votes(found[0]) == (0, False)
    t.add(0.0, found)
    t.add(0.5, found)
    assert t.votes(found[0]) == (2, False)
    t.add(1.0, found)
    assert t.votes(found[0]) == (3, True)
    assert np.isfinite(found[0].width)


# ------------------------------------------------------------------ near_miss_segment

def test_near_miss_segment_on_a_20_cm_gap():
    """The lab curtain with the 15 cm spec: no curtain, but a segment to inspect."""
    from maze_nav.curtains import near_miss_segment
    found, cells = run(corridor(curtain_box(x=1.6, gap=0.20)))
    assert found == []
    seg = near_miss_segment(cells)
    assert seg is not None
    p1, p2, gap = seg
    assert 0.15 < gap < 0.30
    assert abs((p1[0] + p2[0]) / 2 - 1.6) < 0.1      # on the curtain line
    assert np.linalg.norm(p2 - p1) > 0.4


def test_near_miss_segment_ignores_walls_and_high_gaps():
    from maze_nav.curtains import near_miss_segment
    _, cells = run(corridor((1.6, -0.45, 0.0, 1.62, 0.45, 0.45)))
    assert near_miss_segment(cells) is None                     # plain wall
    _, cells = run(corridor(curtain_box(x=1.6, gap=0.35)))
    assert near_miss_segment(cells, max_gap=0.30) is None       # a hurdle, not worth a look


# ------------------------------------------------------------------ lying down

def lying_points(boxes, h=0.20, xy=(0.0, 0.0)):
    """Lidar ~0.2 m above the floor (robot lying down), same FOV."""
    return raycast(boxes, origin=(xy[0], xy[1], h))


def test_gap_at_line_measures_the_hem_lying_down():
    from maze_nav.curtains import gap_at_line
    for gap in (0.12, 0.15, 0.20):
        pts = lying_points(corridor(curtain_box(x=0.7, gap=gap)))
        g, cov, n = gap_at_line(pts, (0.7, -0.45), (0.7, 0.45), (1.0, 0.0), 0.0)
        assert g is not None and abs(g - gap) < 0.03, (gap, g)
        assert cov >= 0.5


def test_gap_at_line_sees_no_gap_under_a_wall():
    from maze_nav.curtains import gap_at_line
    pts = lying_points(corridor((0.7, -0.45, 0.0, 0.72, 0.45, 0.45)))
    g, cov, _ = gap_at_line(pts, (0.7, -0.45), (0.7, 0.45), (1.0, 0.0), 0.0)
    assert g is None or cov < 0.5


def test_tracker_confirm_adds_or_promotes():
    from maze_nav.curtains import Curtain
    t = CurtainTracker()
    c = Curtain(np.array([1.0, -0.4]), np.array([1.0, 0.4]), np.array([1.0, 0.0]), 0.15)
    stored = t.confirm(c, 0.0)
    assert stored.id == 1 and t.confirmed == [stored]
    again = t.confirm(Curtain(np.array([1.02, -0.4]), np.array([1.02, 0.4]),
                              np.array([1.0, 0.0]), 0.16), 1.0)
    assert again is stored and len(t.confirmed) == 1
