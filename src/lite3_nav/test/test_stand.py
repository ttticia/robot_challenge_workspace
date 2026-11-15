"""Auto stand-up decisions (lite3_nav.stand); no ROS needed."""
import numpy as np

from lite3_nav.stand import StandUp, body_height, posture


def floor_cloud(height, n=2000, seed=0):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-2., 2., (n, 2))
    z = np.full((n, 1), -height) + rng.normal(0., .005, (n, 1))
    wall = np.column_stack([np.full(300, 1.), rng.uniform(-1., 1., 300), rng.uniform(-height, .2, 300)])
    return np.vstack([np.hstack([xy, z]), wall])


def test_body_height_standing_and_lying():
    assert abs(body_height(floor_cloud(.35)) - .35) < .02 and posture(body_height(floor_cloud(.35))) == 'standing'
    assert abs(body_height(floor_cloud(.09)) - .09) < .02 and posture(body_height(floor_cloud(.09))) == 'lying'
    assert body_height(np.zeros((0, 3))) is None and posture(None) == 'unknown'
    assert posture(.22) == 'unknown'  # getting up / sitting down: never a reason to send


def test_standing_robot_never_gets_the_toggle():
    s = StandUp(0.)
    assert [s.feed(.35, t*.1) for t in range(3)] == ['wait', 'wait', 'done']


def test_lying_robot_gets_one_toggle_then_stands():
    s = StandUp(0.)
    actions = [s.feed(.09, t*.1) for t in range(6)]
    assert actions == ['wait', 'wait', 'send', 'wait', 'wait', 'wait']
    actions = [s.feed(h, 1.+t*.1) for t, h in enumerate([.2, .3, .35, .35])]
    assert actions == ['wait', 'wait', 'wait', 'done']


def test_toggle_sent_once_and_fails_if_it_never_stands():
    s = StandUp(0., timeout=2.)
    actions = [s.feed(.09, t*.1) for t in range(40)]
    assert actions.count('send') == 1 and actions[-1] == 'fail:STAND_UP_FAILED'


def test_unknown_posture_fails_without_sending():
    s = StandUp(0., unknown_timeout=1.)
    actions = [s.feed(.22, t*.1) for t in range(15)]
    assert 'send' not in actions and actions[-1] == 'fail:STAND_STATE_UNKNOWN'
    assert StandUp(0., unknown_timeout=1.).expired(1.5)
