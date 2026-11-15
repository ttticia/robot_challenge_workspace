"""Tilt stop with a hold time (pure Python, no ROS, unit-tested in test/test_tilt.py).

The course has a 17.4° sloped pit and 0.12 m steps, whose climb gives short pitch spikes, so
the mission stops only once the tilt has stayed over the limit for hold_sec. An invalid IMU
reading (inf/NaN) always trips at once, and so does any over-limit reading when hold_sec is 0
(the mission passes 0 while disarmed, so arming still needs the robot level right now).
"""
import math


class TiltGuard:
    def __init__(self):
        self.over_since = None

    def update(self, tilt, now, limit):
        """Record one IMU reading (tilt and limit in radians, now in seconds)."""
        if tilt > limit or not math.isfinite(tilt):
            if self.over_since is None:
                self.over_since = now
        else:
            self.over_since = None

    def tripped(self, tilt, now, limit, hold):
        if not math.isfinite(tilt):
            return True
        if tilt <= limit:
            return False
        if hold <= 0.:
            return True
        since = now if self.over_since is None else self.over_since
        return now - since >= hold
