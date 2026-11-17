"""Auto stand-up on arming (no ROS here; mission.py wires it up, unit-tested).

The Lite3 has no stand command, only a stand/sit TOGGLE (0x21010202 on /simple_cmd), and the
bridge publishes no basic state (robot_info.md 6.3 / 6.4). Sent to a standing robot, the toggle
sits it down. So the body height above the floor, seen by the lidar, decides: base_link is about
0.35 m up when standing and 0.07-0.11 m when lying (robot_info.md 3). The toggle is sent at most
once per arming and only after several clouds in a row agree the robot is lying; then the robot
must read as standing within the timeout, else the mission disarms. Anything in between (0.18 to
0.27 m: getting up, sitting down, a step under the robot) is "unknown" and never triggers a send.
"""
import numpy as np

STAND_TOGGLE = 0x21010202
STANDING_MIN_M = 0.27
LYING_MAX_M = 0.18


def body_height(points_base, ring=(0.4, 2.5), min_points=100, layer_fraction=0.2):
    """base_link height above the floor (m) from cloud points in base_link (x forward, z up),
    or None. The floor is the lowest dense 1 cm layer between 0 and 0.5 m below base_link in a
    ring around the robot (same idea as wall_band.estimate_floor)."""
    p = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    r = np.hypot(p[:, 0], p[:, 1])
    z = p[(r > ring[0]) & (r < ring[1]) & (p[:, 2] < 0.) & (p[:, 2] > -.5), 2]
    if len(z) < min_points:
        return None
    hist, edges = np.histogram(z, bins=np.arange(-.5, .005, .01))
    dense = np.nonzero(hist >= max(layer_fraction*hist.max(), min_points/10))[0]
    if not len(dense):
        return None
    i = int(dense[0])
    return float(-np.median(z[(z >= edges[i]-.01) & (z <= edges[i+1]+.01)]))


def posture(height):
    if height is None:
        return 'unknown'
    return 'standing' if height >= STANDING_MIN_M else 'lying' if height <= LYING_MAX_M else 'unknown'


class StandUp:
    """One arming's stand-up. feed() each new cloud's body height; it returns one of
    'wait', 'send' (publish the toggle now; returned at most once), 'done' or 'fail:<REASON>'."""

    def __init__(self, now, timeout=15., unknown_timeout=4., agree=3):
        self.started, self.timeout, self.unknown_timeout, self.agree = now, timeout, unknown_timeout, agree
        self.sent_at = None
        self.lying = self.standing = 0

    def feed(self, height, now):
        state = posture(height)
        self.lying = self.lying+1 if state == 'lying' else 0
        self.standing = self.standing+1 if state == 'standing' else 0
        if self.standing >= self.agree:
            return 'done'
        if self.sent_at is None:
            if self.lying >= self.agree:
                self.sent_at = now
                return 'send'
            if now-self.started > self.unknown_timeout:
                return 'fail:STAND_STATE_UNKNOWN'
        elif now-self.sent_at > self.timeout:
            return 'fail:STAND_UP_FAILED'
        return 'wait'

    def expired(self, now):
        """No usable cloud at all: give up after the same limits."""
        limit = self.unknown_timeout if self.sent_at is None else self.timeout
        return now-(self.started if self.sent_at is None else self.sent_at) > limit


def quaternion_matrix(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
