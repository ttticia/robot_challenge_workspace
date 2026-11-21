"""Curtain inspection: go to the cell in front of a possible curtain, lie down, measure the gap.

User 2026-11-20: when a wall ahead shows a suitable gap at the bottom but isn't a confirmed
curtain yet, the robot drives to the middle of the cell in front of it, squares up, and LIES
DOWN: the lidar and camera then sit ~0.15-0.25 m above the floor, about level with the hem,
and see straight under it. curtain_detector measures the gap meanwhile
(curtains.gap_at_line); only a gap of 12-20 cm confirms the curtain. The robot stands up
again and then walks through. Earlier variant (look_mode 'pose', also the fallback): a
Lite3 Pose-mode look, pitched forward, yawing both ways.

Why there: the curtain hangs in an opening on a cell edge, so the middle of the cell in front
is `distance` (half a cell, 0.5 m) before its line, laterally at its centre: free space, and
square on every bin along the curtain sees under it at once.

InspectPlan(...).tick(now, xy, yaw, pose_status, lie_status) -> Step:
  GOTO      turn towards the look point (`distance` before the line, at the curtain's centre)
            and drive there (forward only: cmd_vel_bridge never reverses; closer than that
            already -> FACE where it is); skipped within `goto_tol`
  FACE      turn on the spot until square to the curtain line
  SETTLE    hold still `settle` s
  look_mode 'lie' (lie_status not UNAVAILABLE):
    LIE_DOWN  step.lie = True until lie_status is LYING (cmd_vel_bridge: stand toggle with the
              height checked); not LYING within `lie_wait` s -> stand up, then the Pose look
    LYING     hold `lie_hold` s, step.measuring = True: the caller measures the gap
    STAND_UP  step.lie = False until lie_status is IDLE (standing, back in auto mode);
              `stand_wait` s -> DONE with result 'STAND-UP NOT CONFIRMED' (caller holds)
  look_mode 'pose' / fallback: LOOK in Pose mode (body yaw + pitch per `looks`), else turning
            the whole robot (sim, drive:=false)
  DONE      zero
The caller sends (v, w) (after its wall check); while `pose` is not None it publishes it on
/maze/pose_yaw + /maze/pose_pitch at 10 Hz (NaN yaw when done); while `lie` is not None it
publishes it on /maze/lie (std_msgs/Bool) at 2 Hz.
"""
from dataclasses import dataclass, field
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class InspectParams:
    distance: float = 0.5          # m before the curtain line: middle of the cell in front
    goto_tol: float = 0.08         # m: closer than this to the look point = there
    speed: float = 0.08            # m/s for GOTO
    turn_rate: float = 0.3         # rad/s max for turns
    min_turn_rate: float = 0.12
    turn_tol: float = math.radians(4)
    settle: float = 0.8            # s still before the look
    look_mode: str = 'lie'         # 'lie' or 'pose'
    lie_wait: float = 8.0          # s for the robot to be down (LYING)
    lie_hold: float = 3.0          # s lying still while the gap is measured
    stand_wait: float = 15.0       # s for standing + Move mode + gait + auto mode (IDLE)
    pitch: float = math.radians(12)    # nose down during a Pose look
    look_yaw: float = math.radians(15)
    hold: float = 1.5              # s per Pose look once there
    pose_turn_time: float = 1.0    # s for the body to get there after POSING
    pose_wait: float = 4.0         # s for POSING before falling back to whole-robot turns
    timeout: float = 45.0          # s for the whole inspection (not cut while lying); was 75:
    #                                two 2026-11-21 sim inspections used all 75 s going nowhere
    stuck_time: float = 10.0       # s in GOTO without getting stuck_gain m closer: give up
    stuck_gain: float = 0.05
    looks: list = field(default_factory=lambda: None)   # [(yaw, pitch, hold)]; None = default

    def look_list(self):
        if self.looks is not None:
            return list(self.looks)
        p, y, h = self.pitch, self.look_yaw, self.hold
        return [(0.0, p, h + 0.5), (y, p, h), (-y, p, h), (0.0, p, h)]


@dataclass
class Step:
    v: float = 0.0
    w: float = 0.0
    pose: tuple = None             # (yaw, pitch) body target in Pose mode, else None
    lie: bool = None               # True = lie down / stay down, False = stand up, None = -
    measuring: bool = False        # lying still: measure the gap now
    phase: str = ''
    done: bool = False
    why: str = ''


class InspectPlan:
    """One inspection of a curtain candidate. centre / normal (2-vectors, normal pointing
    away from the robot's side) in the same fixed frame as the poses fed to tick()."""

    def __init__(self, centre, normal, now, p=None):
        self.p = p or InspectParams()
        self.c = np.asarray(centre, float)[:2]
        n = np.asarray(normal, float)[:2]
        self.n = n / max(np.linalg.norm(n), 1e-9)
        self.heading = math.atan2(self.n[1], self.n[0])
        self.target = self.c - self.p.distance * self.n
        self.t0 = now
        self.phase, self.since = 'GOTO', now
        self.looks = self.p.look_list()
        self.i = 0                 # current Pose look
        self.look_since = None
        self.pose_mode = None      # None = not tried yet, True = Pose mode, False = fallback
        self.lay = False           # it did lie down (measurements came from lying)
        self.goto_from = None
        self.goto_until = 0.0
        self.goto_best = (math.inf, now)   # (closest distance to the look point, when)
        self.after_stand = 'DONE'  # after STAND_UP: 'DONE', or 'LOOK' (couldn't lie down)
        self.result = ''

    # ------------------------------------------------------------------ helpers
    def dist(self, xy):
        """Distance in front of the curtain line (positive on the robot's side)."""
        return float(np.dot(self.c - np.asarray(xy, float)[:2], self.n))

    def turn(self, err):
        w = max(self.p.min_turn_rate, min(self.p.turn_rate, 1.5 * abs(err)))
        return math.copysign(w, err)

    def go(self, phase, now):
        self.phase, self.since = phase, now

    def done(self, why):
        self.result, self.phase = why, 'DONE'
        return Step(phase='DONE', done=True, why=why)

    # ------------------------------------------------------------------ tick
    def tick(self, now, xy, yaw, pose_status=None, lie_status=None):
        p = self.p
        xy = np.asarray(xy, float)[:2]
        if self.phase == 'DONE':
            return Step(phase='DONE', done=True, why=self.result)
        if now - self.t0 > p.timeout and self.phase not in ('LIE_DOWN', 'LYING', 'STAND_UP'):
            return self.done(f'inspection timed out after {p.timeout:.0f} s')
        if self.phase == 'GOTO':
            r = self.goto(now, xy, yaw)
            if r is not None:
                return r
        if self.phase == 'FACE':
            err = wrap(self.heading - yaw)
            if abs(err) > p.turn_tol:
                return Step(w=self.turn(err), phase='FACE')
            self.go('SETTLE', now)
        if self.phase == 'SETTLE':
            if now - self.since < p.settle:
                return Step(phase='SETTLE')
            lie_ok = p.look_mode == 'lie' and lie_status is not None and \
                not str(lie_status).startswith('UNAVAILABLE')
            self.go('LIE_DOWN' if lie_ok else 'LOOK', now)
            self.pose_mode = None if pose_status is not None and \
                not str(pose_status).startswith('UNAVAILABLE') else False
        if self.phase in ('LIE_DOWN', 'LYING', 'STAND_UP'):
            return self.lie(now, lie_status)
        if self.phase == 'LOOK':
            return self.look(now, yaw, pose_status)
        return Step(phase=self.phase)

    def goto(self, now, xy, yaw):
        """Drive to the look point; None once there (phase -> FACE)."""
        p = self.p
        off = self.target - xy
        d = float(np.linalg.norm(off))
        if self.goto_from is None:
            self.goto_from = xy.copy()
            self.goto_until = now + d / max(p.speed, 1e-3) + 10.0
        if d <= p.goto_tol or now > self.goto_until:
            self.go('FACE', now)
            return None
        if d < self.goto_best[0] - p.stuck_gain:
            self.goto_best = (d, now)
        elif now - self.goto_best[1] > p.stuck_time:
            # Not getting closer (a wall in the way, the robot pinned): looking from here is
            # pointless once well short of the look point.
            if d <= 3 * p.goto_tol:
                self.go('FACE', now)
                return None
            return self.done(f'no progress towards the look point for {p.stuck_time:.0f} s '
                             f'({d:.2f} m short): given up')
        # cmd_vel_bridge never reverses: a look point behind the robot (closer to the curtain
        # than half a cell already) -> square up where it is.
        if float(np.dot(off, np.array([math.cos(yaw), math.sin(yaw)]))) < 0 and \
                float(np.dot(off, self.n)) < 0:
            self.go('FACE', now)
            return None
        bearing = math.atan2(off[1], off[0])
        err = wrap(bearing - yaw)
        if abs(err) > math.radians(20):
            return Step(w=self.turn(err), phase='GOTO')
        return Step(v=p.speed * max(0.0, math.cos(err)), w=0.8 * err, phase='GOTO',
                    why=f'{d:.2f} m to the middle of the cell in front of the curtain')

    def lie(self, now, lie_status):
        p = self.p
        st = str(lie_status or '').split(':', 1)[0].strip()
        if self.phase == 'LIE_DOWN':
            if st == 'LYING':
                self.lay = True
                self.go('LYING', now)
            elif st == 'UNAVAILABLE' or now - self.since > p.lie_wait:
                self.result = f'could not lie down ({lie_status})'
                self.go('STAND_UP', now)              # make sure it is up, then the Pose look
                self.after_stand = 'LOOK'
                return Step(lie=False, phase='STAND_UP')
            else:
                return Step(lie=True, phase='LIE_DOWN')
        if self.phase == 'LYING':
            if now - self.since < p.lie_hold:
                return Step(lie=True, measuring=True, phase='LYING')
            self.go('STAND_UP', now)
            self.after_stand = 'DONE'
        # STAND_UP
        if st in ('IDLE', 'UNAVAILABLE') and now - self.since > 0.5:
            if self.after_stand == 'LOOK':
                self.go('LOOK', now)
                return Step(phase='LOOK')
            return self.done('measured lying down')
        if now - self.since > p.stand_wait:
            return self.done(f'STAND-UP NOT CONFIRMED after {p.stand_wait:.0f} s ({lie_status})')
        return Step(lie=False, phase='STAND_UP')

    def look(self, now, yaw, pose_status):
        p = self.p
        if self.i >= len(self.looks):
            if self.pose_mode is False:          # fallback: turn back square first
                err = wrap(self.heading - yaw)
                if abs(err) > p.turn_tol:
                    return Step(w=self.turn(err), phase='LOOK')
            return self.done('looked ' + (f'in Pose mode (pitch {math.degrees(p.pitch):.0f} '
                                          'deg down)' if self.pose_mode
                                          else 'by turning (no Pose mode)'))
        yaw_t, pitch_t, hold = self.looks[self.i]
        st = str(pose_status or '')
        if self.pose_mode is not False:
            if st.startswith('UNAVAILABLE') or \
                    (self.pose_mode is None and now - self.since > p.pose_wait):
                self.pose_mode, self.look_since = False, None     # fall back, same look
            else:
                if st == 'POSING':
                    if self.pose_mode is None:
                        self.pose_mode = True
                    if self.look_since is None:
                        self.look_since = now + p.pose_turn_time
                if self.look_since is not None and now >= self.look_since + hold:
                    self.i, self.look_since = self.i + 1, now
                    if self.i < len(self.looks):
                        self.look_since = now + p.pose_turn_time
                return Step(pose=(yaw_t, pitch_t), phase='LOOK')
        err = wrap(self.heading + yaw_t - yaw)
        if self.look_since is None:
            if abs(err) > p.turn_tol:
                return Step(w=self.turn(err), phase='LOOK')
            self.look_since = now
        if now - self.look_since >= hold:
            self.i, self.look_since = self.i + 1, None
        return Step(phase='LOOK')
