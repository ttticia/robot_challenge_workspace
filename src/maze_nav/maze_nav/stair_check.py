"""Possible-stairs check: turn to face them, hold still, count riser detections (no ROS).

User 2026-11-20: the robot missed the stairs on the 06:38 run - stair_detector only looked
while maze_explorer was EXPLORING / TURNING / LOOKING / FACING_QR, never during the 38 s
junction look (when the robot faced each way), and it only acted on stairs confirmed by
3 of 6 frames within 1.5 m with the robot in front. Now anything stair-like - one camera
frame with a riser chain, a lidar height-map estimate, confirmed stairs too far or seen
from beside, or stairs seen during a junction / dead-end / start look - is a CANDIDATE.
Stair checks come after junction checking: once maze_explorer is back in its exploring
states, stair_detector hands over and runs this check:

  TURN   on the spot until the candidate's first riser (its middle, or the point on it
         straight ahead) is within `face_tol` of the heading: the camera (+-40 deg) then has
         the risers in view
  HOLD   still `hold` s: stair_detector keeps detecting; each camera frame whose riser chain
         (stairs.detect_stairs: >= min_risers discrete faces, each a rise apart in height and
         a tread apart in depth) lies near the candidate is a hit, so are lidar estimates
  DONE   `need` hits (or the tracker's own confirmation) -> stairs: line up as usual;
         else not stairs, and the spot is left alone for a while

StairCheck(...).tick(now, xy, yaw) -> (w, phase); add_hit(now, source) while holding.
"""
from dataclasses import dataclass
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class CheckParams:
    face_tol: float = math.radians(6)
    turn_rate: float = 0.3
    min_turn_rate: float = 0.15
    hold: float = 3.0              # s still, detecting
    need: int = 3                  # camera hits (lidar hits count half)
    turn_timeout: float = 15.0


class StairCheck:
    """One check of a candidate StairEstimate (odom)."""

    def __init__(self, est, xy, now, p=None):
        self.p = p or CheckParams()
        self.est = est
        self.target = np.asarray(est.edge_point(np.asarray(xy, float)), float)
        self.phase, self.since, self.t0 = 'TURN', now, now
        self.cam_hits = 0
        self.lidar_hits = 0

    def bearing(self, xy):
        d = self.target - np.asarray(xy, float)
        return math.atan2(d[1], d[0])

    def tick(self, now, xy, yaw):
        p = self.p
        if self.phase == 'TURN':
            err = wrap(self.bearing(xy) - yaw)
            if abs(err) > p.face_tol and now - self.since < p.turn_timeout:
                w = max(p.min_turn_rate, min(p.turn_rate, 1.5 * abs(err)))
                return math.copysign(w, err), 'TURN'
            self.phase, self.since = 'HOLD', now
        if self.phase == 'HOLD':
            if now - self.since < p.hold:
                return 0.0, 'HOLD'
            self.phase = 'DONE'
        return 0.0, 'DONE'

    def add_hit(self, source):
        """A detection near the candidate while holding still."""
        if self.phase != 'HOLD':
            return
        if source == 'lidar':
            self.lidar_hits += 1
        else:
            self.cam_hits += 1

    @property
    def score(self):
        return self.cam_hits + 0.5 * self.lidar_hits

    def passed(self):
        return self.score >= self.p.need
