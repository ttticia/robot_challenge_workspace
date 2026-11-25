"""The stop in the last cell (user, 2026-11-25): in its middle, facing out. The explorer's
own turn / drive actions run on the stand-in of test_qr_approach (perfect motion)."""
import math

import pytest

from test_qr_approach import Robot, explorer


def final_stop(robot, cell, d):
    """The queue plan() builds for the exit job (copied shape: turn, square, creep, square)."""
    h = robot.grid.heading(d)
    cx, cy = robot.grid.centre(cell)
    tx, ty = cx + robot.side_offset * math.cos(h), cy + robot.side_offset * math.sin(h)
    robot.queue.extend([('turn', h, 3.0), ('drive', tx, ty, h, 'square'),
                        ('drive', tx, ty, h, 'creep'), ('drive', tx, ty, h, 'square'),
                        ('turn', h, 2.0)])


@pytest.mark.parametrize('start', [(0.95, 0.12, 0.2), (1.02, -0.15, -0.3), (0.80, 0.05, 1.2)])
def test_ends_with_the_body_centred_in_the_last_cell_facing_out(start):
    R = type('R', (Robot,), dict(do_drive=explorer.Explorer.do_drive))
    r = R()
    r.v_max, r.ai_speed, r.curtain_speed, r.push_stop, r.step = 0.2, 0.15, 0.2, True, None
    r.pose = list(start)                       # somewhere in the last cell (1, 0), way out ahead
    final_stop(r, (1, 0), 0)
    assert r.run(90.0)
    x, y, yaw = r.pose
    mid = x - 0.075                            # middle of the body (0.27 ahead, 0.42 behind)
    assert abs(mid - 1.02) < 0.06 and abs(y) < 0.05 and abs(math.degrees(yaw)) < 4.0
    assert x + 0.27 < 1.53 and x - 0.42 > 0.51             # both ends inside the cell
    assert not r.backwards


def test_plan_builds_that_queue():
    import inspect
    src = inspect.getsource(explorer.Explorer.plan)
    assert "('drive', tx, ty, h, 'creep'), ('drive', tx, ty, h, 'square')" in src
