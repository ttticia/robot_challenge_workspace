"""No height-map updates while the robot turns (2026-11-25 10:02: walls smeared into the
terrain cell ahead of the start made it an 'object' and set off scene understanding)."""
import math
from collections import deque
from types import SimpleNamespace as NS

import numpy as np
import pytest

MODULES = ('maze_explore.explorer', 'stairs_crossing.explorer_stairs')


def odom(t, yaw):
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=int((t % 1) * 1e9))),
              pose=NS(pose=NS(position=NS(x=0.0, y=0.0, z=0.0),
                              orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))


@pytest.mark.parametrize('module', MODULES)
def test_heights_only_while_not_turning(module):
    import importlib
    try:
        E = importlib.import_module(module).Explorer            # needs the ROS environment
    except ImportError as e:
        pytest.skip(f'{module}: {e}')
    added = []
    r = NS(feed_check=lambda stamp: None, t=100.0, pose=(0.0, 0.0, 0.0), blind_until=0.0, clouds=deque(), last_odom=None,
           last_yaw=None, turned_at=-1e9, heights_turn_max=0.2, heights_settle=0.3,
           odom_jumps=[], odom_grace_until=0.0, odom_hold_until=0.0, state='RUN',
           heights=NS(add=lambda x, y, h: added.append(len(h))))
    r.now = lambda: r.t
    cloud = NS(data=np.ones((5, 3), np.float32).tobytes())
    yaw = 0.0
    for i in range(30):                        # 3 s: still, turning at 0.5 rad/s, still again
        turning = 10 <= i < 20
        yaw += 0.05 if turning else 0.0
        r.t = 100.0 + 0.1 * i
        E.on_odom(r, odom(r.t, yaw))
        before = len(added)
        E.on_cloud(r, cloud)
        took = len(added) > before
        if i < 10 or i >= 24:
            assert took, i                     # standing: every cloud goes into the map
        if 10 <= i < 22:
            assert not took, i                 # turning, and 0.3 s after: none does
