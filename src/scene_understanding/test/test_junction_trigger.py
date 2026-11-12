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
