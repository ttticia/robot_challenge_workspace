"""stair_check.StairCheck: turn to face possible stairs, hold, count hits, decide."""
import math

import numpy as np

from maze_nav.stair_check import CheckParams, StairCheck
from maze_nav.stairs import StairEstimate


def stairs_at(x, y, heading=0.0):
    """First riser through (x, y), stairs going up along heading, centre known."""
    d = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-math.sin(heading), math.cos(heading)])
    return StairEstimate(heading=heading, along=float(np.dot((x, y), d)),
                         centre=float(np.dot((x, y), n)), risers=2, rise=0.15)


def run(chk, xy, yaw, hits_per_s=0.0, source='camera', dt=0.05, t_max=40.0):
    t, next_hit = 0.0, 0.0
    phases = []
    while t < t_max:
        w, phase = chk.tick(t, xy, yaw)
        if not phases or phases[-1] != phase:
            phases.append(phase)
        if phase == 'DONE':
            break
        if hits_per_s and t >= next_hit:
            chk.add_hit(source)
            next_hit = t + 1.0 / hits_per_s
        yaw += w * dt
        t += dt
    return yaw, phases, t


def test_turns_to_face_stairs_off_to_the_side():
    # stairs 1.5 m away, 70 deg to the left (seen on the lidar / at the edge of the camera)
    est = stairs_at(1.5 * math.cos(math.radians(70)), 1.5 * math.sin(math.radians(70)),
                    heading=math.radians(70))
    chk = StairCheck(est, (0.0, 0.0), 0.0)
    yaw, phases, _ = run(chk, (0.0, 0.0), 0.0, hits_per_s=5)
    assert phases == ['TURN', 'HOLD', 'DONE']
    assert abs(math.degrees(yaw) - 70) <= 7
    assert chk.passed()


def test_hits_only_count_while_holding_still():
    chk = StairCheck(stairs_at(1.0, 0.0), (0.0, 0.0), 0.0)
    chk.add_hit('camera')                       # before the first tick: still TURN
    assert chk.cam_hits == 0
    run(chk, (0.0, 0.0), 0.0, hits_per_s=0.5)   # ~1-2 hits in 3 s
    assert not chk.passed()


def test_lidar_hits_count_half():
    chk = StairCheck(stairs_at(1.0, 0.0), (0.0, 0.0), 0.0, CheckParams(need=3, hold=3.0))
    run(chk, (0.0, 0.0), 0.0, hits_per_s=1.5, source='lidar')    # ~5 lidar hits = 2.5
    assert chk.lidar_hits >= 4 and chk.score < 3 and not chk.passed()
    chk = StairCheck(stairs_at(1.0, 0.0), (0.0, 0.0), 0.0, CheckParams(need=3, hold=3.0))
    run(chk, (0.0, 0.0), 0.0, hits_per_s=3.0, source='lidar')
    assert chk.passed()


def test_turn_gives_up_after_timeout_and_still_holds():
    chk = StairCheck(stairs_at(0.0, 1.0, math.pi / 2), (0.0, 0.0), 0.0,
                     CheckParams(turn_timeout=1.0))
    # a robot that cannot turn (walls): ticks without integrating w
    t, phase = 0.0, None
    while t < 10.0 and phase != 'DONE':
        _, phase = chk.tick(t, (0.0, 0.0), 0.0)
        t += 0.1
    assert phase == 'DONE' and t < 5.5          # 1 s turn timeout + 3 s hold
