"""cmd_vel_bridge.min_spin: slow turns on the spot are raised so the Lite3 turns at all."""
import math

from maze_nav.cmd_vel_bridge import min_spin


def test_slow_spin_raised_same_sign():
    assert min_spin(0.0, 0.05, 0.15) == 0.15         # RPP from standstill (bag s_maze1)
    assert min_spin(0.0, -0.04, 0.15) == -0.15
    assert min_spin(0.01, 0.1, 0.15) == 0.15         # creeping counts as on the spot


def test_left_alone():
    assert min_spin(0.0, 0.3, 0.15) == 0.3           # fast enough already
    assert min_spin(0.0, 0.005, 0.15) == 0.005       # noise / end of a turn
    assert min_spin(0.0, 0.0, 0.15) == 0.0           # stop stays a stop
    assert min_spin(0.1, 0.05, 0.15) == 0.05         # walking: arcs keep their rate
    assert min_spin(0.0, 0.05, 0.0) == 0.05          # off
    assert math.copysign(1, min_spin(0.0, -0.15, 0.15)) == -1
