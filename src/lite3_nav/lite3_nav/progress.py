"""Goal timeout measured from the last progress, not from the goal's start (pure Python, no ROS,
unit-tested in test/test_progress.py).

A fixed goal_timeout_sec cut off long but healthy trips: in the 7 x 6 m dead-end maze (check C,
2026-11-16) goals across the maze timed out, were blacklisted, and the robot turned back and
forth between two far frontiers. With progress_m > 0 the timer restarts whenever the robot is
progress_m from where it last restarted, so only a robot that stops getting anywhere times out;
max_sec (if > 0) still caps a goal's total time (Nav2 oscillating along a path). progress_m 0
keeps the team's fixed timeout.
"""
import math


class GoalProgress:
    def __init__(self):
        self.started = self.last_progress = 0.
        self.anchor = None

    def start(self, xy, now):
        self.started = self.last_progress = now
        self.anchor = None if xy is None else (float(xy[0]), float(xy[1]))

    def update(self, xy, now, progress_m):
        if progress_m <= 0 or xy is None:
            return
        if self.anchor is None or math.hypot(xy[0]-self.anchor[0], xy[1]-self.anchor[1]) >= progress_m:
            self.anchor = (float(xy[0]), float(xy[1]))
            self.last_progress = now

    def timed_out(self, now, timeout, progress_m, max_sec=0.):
        since = self.last_progress if progress_m > 0 else self.started
        return now-since > timeout or (max_sec > 0 and now-self.started > max_sec)
