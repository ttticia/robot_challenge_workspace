"""Reorient on arrival (user, 2026-11-26 night): once a new cell's sides are known, the
robot turns on the spot to the nearest grid direction when more than reorient_deg off."""
import math

import pytest

from maze_explore.grid import wrap

explorer = pytest.importorskip('maze_explore.explorer')
from test_qr_approach import Robot as Base                      # noqa: E402

E = explorer.Explorer


class Robot(Base):
    do_turn = E.do_turn
    queue_reorient = E.queue_reorient

    def __init__(self, yaw_deg):
        super().__init__()
        self.pose = [0.0, 0.0, math.radians(yaw_deg)]
        self.reorient, self.reorient_deg, self.maze_rows = True, 5.0, 5
        self.turn_kick_after, self.turn_kick_gap, self.cmd_gap = 3.0, 60.0, 1.5
        self.ai_sequence, self.map_blind_until = False, 0.0


@pytest.mark.parametrize('yaw, target', [(12, 0), (-20, 0), (80, 90), (-100, -90), (172, 180)])
def test_turns_square_to_the_nearest_grid_direction(yaw, target):
    r = Robot(yaw)
    r.queue_reorient((1, 0))
    assert len(r.queue) == 1 and r.queue[0][0] == 'turn'
    assert abs(wrap(r.queue[0][1] - math.radians(target))) < 1e-9
    assert r.run(20.0)
    assert abs(wrap(r.pose[2] - math.radians(target))) < math.radians(3)


def test_nothing_when_already_square():
    r = Robot(4)
    r.queue_reorient((1, 0))
    assert not r.queue


@pytest.mark.parametrize('field, value, cell', [('reorient', False, (1, 0)),
                                                ('ai_sequence', True, (1, 0)),
                                                ('map_blind_until', 1e9, (1, 0)),
                                                (None, None, (5, 0))])          # end row
def test_not_when_off_untrusted_or_in_the_end_row(field, value, cell):
    r = Robot(20)
    if field:
        setattr(r, field, value)
    r.queue_reorient(cell)
    assert not r.queue
