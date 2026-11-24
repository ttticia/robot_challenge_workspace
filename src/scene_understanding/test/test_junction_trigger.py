"""Unit tests for the junction trigger rules (no ROS nodes needed)."""

import pytest

from scene_understanding.junction_trigger import TriggerLogic


def feed(logic, events):
    """Feed (detected, time) pairs; return the times that triggered."""
    return [t for detected, t in events if logic.update(detected, t)]


def test_rising_edge_triggers_once_per_junction():
    logic = TriggerLogic('rising_edge', cooldown_sec=2.0)
    # True held for a while (one junction), then False, then a new junction.
    events = [(False, 0.0), (True, 1.0), (True, 1.1), (True, 1.2),
              (False, 2.0), (True, 5.0), (True, 5.1)]
    assert feed(logic, events) == [1.0, 5.0]


def test_rising_edge_first_message_true():
    logic = TriggerLogic('rising_edge')
    assert feed(logic, [(True, 0.0), (True, 0.1)]) == [0.0]


def test_cooldown_filters_flicker():
    logic = TriggerLogic('rising_edge', cooldown_sec=2.0)
    # Detector flickers True/False within the cooldown period.
    events = [(True, 0.0), (False, 0.2), (True, 0.4), (False, 0.6),
              (True, 2.5)]
    assert feed(logic, events) == [0.0, 2.5]


def test_every_true_mode():
    logic = TriggerLogic('every_true', cooldown_sec=1.0)
    events = [(True, 0.0), (True, 0.5), (True, 1.5), (False, 2.0),
              (True, 3.0)]
    assert feed(logic, events) == [0.0, 1.5, 3.0]


def test_zero_cooldown():
    logic = TriggerLogic('every_true', cooldown_sec=0.0)
    assert feed(logic, [(True, 0.0), (True, 0.0)]) == [0.0, 0.0]


def test_false_never_triggers():
    logic = TriggerLogic('every_true')
    assert feed(logic, [(False, t) for t in range(5)]) == []


def test_invalid_mode():
    with pytest.raises(ValueError):
        TriggerLogic('sometimes')


def test_reported_pos_ahead_or_robot_cell():
    from scene_understanding.junction_trigger import JunctionTrigger
    node = JunctionTrigger.__new__(JunctionTrigger)
    node.cell_size = 1.02
    pose = {'x': 0.98, 'y': 0.05, 'yaw_deg': 91.0}
    node.report_cell = 'ahead'
    assert node._report_pos(pose) == ([1.02, 1.02], [1.02, 0.0])
    node.report_cell = 'robot'
    assert node._report_pos(pose) == ([1.02, 0.0], [1.02, 0.0])
    assert node._report_pos(None) == (None, None)


def test_scan_blocked_while_maze_explorer_runs():
    from types import SimpleNamespace
    from scene_understanding.junction_trigger import JunctionTrigger
    node = JunctionTrigger.__new__(JunctionTrigger)
    topic = {'value': '/maze/status'}
    publishers = {'/maze/status': 1}
    node.get_parameter = lambda name: SimpleNamespace(value=topic['value'])
    node.count_publishers = lambda t: publishers.get(t, 0)
    assert 'maze_explorer' in node._scan_blocked()
    publishers['/maze/status'] = 0
    assert node._scan_blocked() is None
    publishers['/maze/status'] = 1
    topic['value'] = ''
    assert node._scan_blocked() is None


def test_explorer_stitches_only_matching_views(monkeypatch):
    """An incomplete look cannot mix with the next cell's complete look."""
    from queue import Queue
    from types import SimpleNamespace
    from unittest.mock import Mock

    import numpy as np
    from scene_understanding import junction_trigger as module

    class Probe(module.JunctionTrigger):
        def __init__(self):
            self.explore_pending = self.explore_key = self.latest_receive_time = None
            self.explore_views = {}
            self.explore_work = Queue()
            self.trigger_count = 0

        def get_logger(self):
            return Mock()

        def _robot_position(self):
            return None, 'No TF'

    def image_callback(node, value):
        node.latest_receive_time = 10.2
        node.latest_stamp = SimpleNamespace(sec=value, nanosec=0)
        node.latest_frame = np.full((2, 3, 3), value, dtype=np.uint8)

    node = Probe()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)
    monkeypatch.setattr(module.ImageSubscriber, 'image_callback', image_callback)
    for cell, view, value in [(3, 'centre', 20), (4, 'left', 40), (3, 'right', 30)]:
        node.explore_event_callback(SimpleNamespace(data=f'room {view} cell ({cell}, 0) side 1'))
        node.image_callback(value)
    assert node.explore_work.empty()

    for view, value in [('centre', 2), ('left', 1), ('right', 3)]:
        node.explore_event_callback(SimpleNamespace(data=f'room {view} cell (4, 0) side 1'))
        node.image_callback(value)
    stitched, stamp, _, _, _, metadata, _, frames = node.explore_work.get_nowait()
    assert stitched[0, ::3, 0].tolist() == [1, 2, 3]
    assert stamp.sec == 2
    assert metadata['cell'] == [4, 0]
    assert len(frames) == 3
    node.image_callback(4)
    assert node.explore_work.empty()
