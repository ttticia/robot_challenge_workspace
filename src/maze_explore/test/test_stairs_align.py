"""Lined up before the climb (after the 19:05 run, 2026-11-26): the robot squares up onto the
stairs' own centre line, measured by the depth stair check, within 3 cm and 2 deg, and a pose
jump during the stairs sequence no longer drops that sequence (it dropped the centring walk
and the AI leave sequence, and left the robot in the AI gait)."""
import inspect
import math
import types

import pytest

from maze_explore.grid import wrap
from test_qr_approach import Robot, explorer

E = explorer.Explorer


def make():
    R = type('R', (Robot,), dict(do_drive=E.do_drive, stair_line=E.stair_line,
                                 line_offset=E.line_offset))
    r = R()
    r.stair_align_tol, r.stair_align_deg = 0.03, 2.0
    r.stair_line_deg, r.stair_line_off = math.radians(10.0), 0.25
    r.stair_lines = {}
    return r


def test_the_measured_stairs_line_is_used_when_it_is_believable():
    r = make()
    r.stair_lines[((0, 0), 0)] = (0.9, -0.07, math.radians(2.0))     # 19:05: 7 cm right, +2 deg
    assert r.stair_line((0, 0), 0) == (0.9, -0.07, math.radians(2.0))


@pytest.mark.parametrize('line', [(0.9, -0.40, 0.0), (0.9, 0.0, math.radians(14.0))])
def test_a_measured_line_far_off_the_cell_is_not_used(line):
    r = make()
    r.stair_lines[((0, 0), 0)] = line
    assert r.stair_line((0, 0), 0) == (0.0, 0.0, 0.0)                 # the cell's centre line
    assert any('not used' in e for e in r.events)


def test_no_measurement_means_the_cell_centre_line():
    assert make().stair_line((0, 0), 0) == (0.0, 0.0, 0.0)


def test_lined_up_within_3_cm_and_2_deg_on_the_stairs_line():
    r = make()
    r.pose = [0.05, 0.03, math.radians(-4.0)]                         # 10 cm off the stairs line
    lx, ly, lh = 0.9, -0.07, math.radians(2.0)
    r.queue.extend([('drive', lx, ly, lh, 'align'), ('turn', lh, r.stair_align_deg),
                    ('drive', lx, ly, lh, 'align'), ('turn', lh, r.stair_align_deg)])
    assert r.run(60.0)
    assert abs(r.line_offset(lx, ly, lh)) < 0.03
    assert abs(math.degrees(wrap(lh - r.pose[2]))) < 2.0 + 0.5
    assert not r.backwards


def test_the_climb_is_lined_up_before_ai_mode():
    src = inspect.getsource(E.go)
    stairs = src[src.index("elif how == 'stairs':"):]
    assert stairs.index("'align'") < stairs.index('ai_enter_sequence')
    assert "setattr(self, 'ai_sequence', True)" in stairs


def test_the_stairs_line_comes_from_the_detector():
    src = inspect.getsource(E.do_stair_check)
    assert 'self.stair_lines[(cell, d)] = (px, py, hs)' in src


def test_a_pose_jump_in_the_stairs_sequence_drops_nothing():
    src = inspect.getsource(E.tick)
    keep = src.index("if jump and getattr(self, 'ai_sequence', False):")
    drop = src.index('self.queue.clear()')
    assert keep < drop                                   # checked before the dropping branch
    assert 'self.queue.clear()' not in src[keep:src.index('elif jump:')]


def test_halt_and_the_end_of_the_crossing_end_the_sequence():
    assert 'self.ai_sequence = False' in inspect.getsource(E.halt)
    src = inspect.getsource(E.go)
    assert 'self.ai_sequence = False' in src[src.index('def crossed'):]


def reanchor_robot(pose, walked=1.93):
    from maze_explore.grid import Grid
    R = type('R', (Robot,), dict(reanchor=E.reanchor))
    r = R()
    r.ai_reanchor, r.reanchor_min, r.ai_short_trust, r.cell_size = True, 0.15, True, 1.02
    r.ai_last_walked, r.pose = walked, list(pose)
    r.ai_reanchor_deg, r.ai_start = math.radians(45.0), None
    r.grid = Grid(0.0, 0.0, 0.0, 1.02)
    return r


def test_a_walk_that_came_up_short_keeps_the_grid():
    # 19:05 run: counted 1.93 m, the lidar put the robot about 0.6 m short, on the stair cell
    r = reanchor_robot([1.36, 0.0, 0.0])
    r.reanchor(0.0, 0.0, 0.0)
    assert (r.grid.x0, r.grid.y0, r.grid.yaw) == (0.0, 0.0, 0.0)
    assert any('short' in e for e in r.events)


def test_a_pose_ahead_of_the_counted_spot_is_re_anchored():
    r = reanchor_robot([2.40, 0.0, 0.0])                 # further than it can have walked
    r.reanchor(0.0, 0.0, 0.0)
    assert r.grid.x0 == pytest.approx(2.40 - 1.93)
    assert any('re-anchored' in e for e in r.events)
