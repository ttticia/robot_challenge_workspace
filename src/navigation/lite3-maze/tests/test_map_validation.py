"""Malformed upstream maps must not terminate the navigator."""
from types import SimpleNamespace

from lite3_maze.navigator import Navigator
from lite3_maze.navigation_core import GridMap
import numpy as np


def test_malformed_map_is_rejected_without_replacing_grid():
    errors = []
    logger = SimpleNamespace(error=lambda message, **kwargs: errors.append(message))
    node = SimpleNamespace(frame='map', get_logger=lambda: logger)
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='map'),
                          info=SimpleNamespace(width=2147483648, height=2147483648,
                                               resolution=.05), data=[])
    Navigator.external_map(node, msg)
    assert len(errors) == 1
    assert not hasattr(node, 'grid')


def test_live_scan_observes_unknown_without_erasing_slam_obstacles():
    grid = GridMap(resolution=.05, extent=2.)
    obstacle = grid.cell([.8, 0.])
    grid.log[obstacle[1], obstacle[0]] = 4.
    rotation = SimpleNamespace(x=0., y=0., z=0., w=1.)
    transform = SimpleNamespace(transform=SimpleNamespace(rotation=rotation,
                                translation=SimpleNamespace(x=0., y=0.)))
    node = SimpleNamespace(booting=False, frame='map', grid=grid, scan_time=-float('inf'),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
        get_parameter=lambda key: SimpleNamespace(value=.6),
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args: transform))
    from builtin_interfaces.msg import Time
    msg = SimpleNamespace(header=SimpleNamespace(stamp=Time(), frame_id='base_link'),
        ranges=[1., float('nan'), float('nan'), float('nan'), float('inf')], range_min=.4, range_max=20.,
        angle_min=0., angle_increment=np.pi/8)
    for _ in range(2):
        node.scan_time = -float('inf')
        Navigator.external_scan(node, msg)
    cell = grid.cell([.5, 0.])
    assert grid.log[cell[1], cell[0]] < -.3
    assert grid.log[obstacle[1], obstacle[0]] == 4.
    unknown = grid.cell([0., .5])
    assert grid.log[unknown[1], unknown[0]] == 0.
