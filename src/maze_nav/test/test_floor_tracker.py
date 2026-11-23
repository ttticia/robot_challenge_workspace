"""stairs_lidar.FloorTracker: the floor under the feet carried between measurements by the
change of base_link height, only while standing (no ROS needed)."""
import math

import numpy as np
import pytest

from maze_nav.stairs_lidar import FloorTracker

STAND = 0.35


def test_nothing_before_a_measurement():
    t = FloorTracker()
    assert t.update(0.0, (0, 0, 0.45)) == (None, None)


def test_measured_then_tracked_down_a_step():
    """Start plate 0.10, then the robot steps 10 cm down into the curtain cell where no floor
    has been seen: the tracked floor follows base_z down (the start floor would stay 0.10).
    step_hold 0: the bare tracking (the bob filter has its own tests)."""
    t = FloorTracker(step_hold=0.0)
    assert t.update(0.0, (0, 0, 0.10 + STAND), measured=0.10) == (0.10, 'measured')
    z, src = t.update(1.0, (0.3, 0, 0.10 + STAND))
    assert src == 'tracked' and z == pytest.approx(0.10)
    z, src = t.update(2.0, (0.6, 0, 0.00 + STAND))
    assert src == 'tracked' and z == pytest.approx(0.0)


def test_frozen_while_lying_and_reanchored_after_standing_up():
    """Lying down drops base_link ~0.2 m: frozen, never read as a step. Standing up again at
    a slightly different height (gait) is not a floor change either."""
    t = FloorTracker(settle=1.5, step_hold=0.0)
    t.update(0.0, (0, 0, STAND), measured=0.0)
    assert t.update(1.0, (0, 0, STAND - 0.20), standing=False) == (0.0, 'frozen')
    assert t.update(5.0, (0, 0, STAND - 0.20), standing=False) == (0.0, 'frozen')
    assert t.update(6.0, (0, 0, STAND - 0.05)) == (0.0, 'frozen')      # still getting up
    z, src = t.update(8.0, (0, 0, STAND + 0.02))                       # settled, 2 cm higher
    assert src == 'tracked' and z == pytest.approx(0.0)
    z, src = t.update(9.0, (0.2, 0, STAND - 0.08))                     # then a step down
    assert src == 'tracked' and z == pytest.approx(-0.10)


def test_a_new_measurement_wins():
    t = FloorTracker()
    t.update(0.0, (0, 0, STAND), measured=0.0)
    t.update(1.0, (0.5, 0, STAND - 0.04))
    assert t.update(2.0, (0.6, 0, STAND - 0.04), measured=-0.05) == (-0.05, 'measured')


@pytest.mark.parametrize('when, xy, z', [
    (61.0, (0.1, 0.0), STAND),          # too old
    (5.0, (2.5, 0.0), STAND),           # walked too far
    (5.0, (0.1, 0.0), STAND + 0.7),     # base_z moved too much from the anchor
])
def test_expires(when, xy, z):
    t = FloorTracker(max_age=60.0, max_travel=2.0, max_rise=0.6)
    t.update(0.0, (0, 0, STAND), measured=0.0)
    assert t.update(when, (*xy, z)) == (None, None)
    assert t.update(when + 1, (*xy, z)) == (None, None)     # stays gone until measured


def test_travel_adds_up():
    t = FloorTracker(max_travel=1.0)
    t.update(0.0, (0, 0, STAND), measured=0.0)
    for k in range(1, 6):
        z, src = t.update(float(k), (0.18 * k, 0.0, STAND))
    assert src == 'tracked'                                 # 0.9 m
    assert t.update(6.0, (1.1, 0.0, STAND)) == (None, None)  # 1.1 m


# ------------------------------------------------------------------ gait bob

def walk(t, z_of, until, start=0.0, rate=10.0, update_every=5):
    """Feed observe() at `rate` Hz and update() every update_every samples; returns the
    (time, floor, source) of each update."""
    out, k = [], 0
    now = start
    while now < until:
        z = z_of(now)
        t.observe(now, z)
        if k % update_every == 0:
            out.append((now, *t.update(now, (0.2 * now, 0.0, z))))
        k += 1
        now = start + k / rate
    return out


def bob(amp=0.02, hz=2.5, noise=0.005, seed=0):
    rng = np.random.default_rng(seed)
    return lambda now: amp * math.sin(2 * math.pi * hz * now) + rng.normal(0.0, noise)


def test_bob_does_not_move_the_floor_but_a_step_does():
    """+-2 cm bob at 2.5 Hz + 5 mm noise, sampled 10 Hz, updated 2 Hz: on flat floor the
    tracked floor stays exactly at the measured one; a 10 cm step down at t = 5 s shows up
    in full within ~1.5 s (window 1 s + step_hold 0.5 s), is refined for one more window,
    then holds."""
    t = FloorTracker()
    b = bob()
    t.observe(0.0, STAND)
    t.update(0.0, (0, 0, STAND), measured=0.0)
    out = walk(t, lambda now: STAND + b(now) - (0.10 if now >= 5.0 else 0.0), 9.0, start=0.1)
    before = [f for now, f, src in out if now < 5.0]
    assert all(f == 0.0 for f in before)
    after = [(now, f) for now, f, src in out if now >= 7.1]   # accepted + refined
    assert after and all(f == pytest.approx(-0.10, abs=0.006) for _, f in after)
    assert len({f for _, f in after}) == 1                     # locked again: no jitter
    first = min(now for now, f, _ in out if f is not None and f < -0.05)
    assert first <= 6.6


def test_slow_drift_below_the_step_is_ignored():
    """Odom z creeping 3 cm over 30 s (Fast-LIO drift): the floor doesn't follow it.
    (max_travel raised: walk() covers 6 m here, which would expire the anchor first.)"""
    t = FloorTracker(max_travel=100.0)
    t.observe(0.0, STAND)
    t.update(0.0, (0, 0, STAND), measured=0.0)
    out = walk(t, lambda now: STAND + 0.001 * now + bob(seed=1)(now), 30.0, start=0.1)
    assert all(f == 0.0 for _, f, _ in out)


def test_a_short_blip_is_not_a_step():
    """base_z 6 cm low for 0.5 s (a stumble): shorter than window + step_hold, ignored."""
    t = FloorTracker()
    t.observe(0.0, STAND)
    t.update(0.0, (0, 0, STAND), measured=0.0)
    out = walk(t, lambda now: STAND - (0.06 if 3.0 <= now < 3.5 else 0.0), 6.0, start=0.1)
    assert all(f == 0.0 for _, f, _ in out)
