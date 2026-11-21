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

Pose-mode pitch sweep (user 2026-11-21): with `pose` on and cmd_vel_bridge's Pose mode
available, HOLD is done in Pose mode (feet still) pitching the body through `pitches`
((pitch rad, + = nose down; s) steps: level, nose up for the upper steps, nose down for the
bottom riser), so the camera sees the whole flight. POSE_ENTER waits (<= `pose_wait` s) for
/maze/pose_status POSING; POSE_LEAVE waits (<= `pose_wait` s) for it to leave (back in Move
mode + gait: velocity is forwarded again, so the line-up can drive). Pose mode unavailable or
not reached in time: the plain `hold`.

StairCheck(...).tick(now, xy, yaw, pose_status) -> (w, phase, look); look: a float = keep the
Pose look going at that pitch (send /maze/pose_yaw 0 and /maze/pose_pitch), 'end' = end it
(NaN), None = nothing. add_hit(source) while holding.
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
    pose: bool = True              # HOLD in Pose mode with the pitch sweep (if available)
    pitches: tuple = ((0.0, 1.0), (-math.radians(10), 1.5), (math.radians(10), 1.5),
                      (0.0, 0.5))
    pose_wait: float = 6.0         # s for Pose mode to start / end before giving up on it


class StairCheck:
    """One check of a candidate StairEstimate (odom)."""

    def __init__(self, est, xy, now, p=None):
        self.p = p or CheckParams()
        self.est = est
        self.target = np.asarray(est.edge_point(np.asarray(xy, float)), float)
        self.phase, self.since, self.t0 = 'TURN', now, now
        self.cam_hits = 0
        self.lidar_hits = 0
        self.posing = False            # HOLD is in Pose mode (the pitch sweep)
        self.pose_note = ''            # why not, for the log

    def bearing(self, xy):
        d = self.target - np.asarray(xy, float)
        return math.atan2(d[1], d[0])

    def tick(self, now, xy, yaw, pose_status=None):
        p = self.p
        st = str(pose_status or '')
        if self.phase == 'TURN':
            err = wrap(self.bearing(xy) - yaw)
            if abs(err) > p.face_tol and now - self.since < p.turn_timeout:
                w = max(p.min_turn_rate, min(p.turn_rate, 1.5 * abs(err)))
                return math.copysign(w, err), 'TURN', None
            if p.pose and p.pitches and pose_status is not None and \
                    not st.startswith('UNAVAILABLE'):
                self.phase, self.since = 'POSE_ENTER', now
            else:
                self.pose_note = 'no Pose mode' if p.pose else ''
                self.phase, self.since = 'HOLD', now
        if self.phase == 'POSE_ENTER':
            if st.startswith('POSING'):
                self.phase, self.since, self.posing = 'HOLD', now, True
            elif st.startswith('UNAVAILABLE') or now - self.since > p.pose_wait:
                self.pose_note = f'Pose mode not reached ({st or "no status"})'
                self.phase, self.since = 'HOLD', now
                return 0.0, 'HOLD', 'end'
            else:
                return 0.0, 'POSE_ENTER', 0.0
        if self.phase == 'HOLD':
            if self.posing:
                t = now - self.since
                for pitch, dur in p.pitches:
                    if t < dur:
                        return 0.0, 'HOLD', pitch
                    t -= dur
                self.phase, self.since = 'POSE_LEAVE', now
            elif now - self.since < p.hold:
                return 0.0, 'HOLD', None
            else:
                self.phase = 'DONE'
        if self.phase == 'POSE_LEAVE':
            if (st.startswith('IDLE') or st.startswith('UNAVAILABLE')) and \
                    now - self.since > 0.3 or now - self.since > p.pose_wait:
                self.phase = 'DONE'
            else:
                return 0.0, 'POSE_LEAVE', 'end'
        return 0.0, 'DONE', None

    def hold_time(self):
        """s the HOLD lasts (the pitch sweep's length in Pose mode)."""
        return sum(d for _, d in self.p.pitches) if self.posing else self.p.hold

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
