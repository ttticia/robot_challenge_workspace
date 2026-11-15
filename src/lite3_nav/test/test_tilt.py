import math

from lite3_nav.tilt import TiltGuard

LIMIT = math.radians(22.)


def test_short_spike_passes_and_long_tilt_trips():
    guard = TiltGuard()
    guard.update(math.radians(25.), 10., LIMIT)
    assert not guard.tripped(math.radians(25.), 10.3, LIMIT, .5)
    guard.update(math.radians(5.), 10.4, LIMIT)          # back under: the hold restarts
    guard.update(math.radians(25.), 10.6, LIMIT)
    assert not guard.tripped(math.radians(25.), 11.0, LIMIT, .5)
    assert guard.tripped(math.radians(25.), 11.1, LIMIT, .5)


def test_level_never_trips_and_invalid_always_trips():
    guard = TiltGuard()
    guard.update(math.radians(17.4), 0., LIMIT)           # the course's sloped pit
    assert not guard.tripped(math.radians(17.4), 60., LIMIT, .5)
    for bad in (math.inf, math.nan):
        guard.update(bad, 1., LIMIT)
        assert guard.tripped(bad, 1., LIMIT, .5)


def test_no_hold_trips_at_once():
    guard = TiltGuard()
    guard.update(math.radians(23.), 5., LIMIT)
    assert guard.tripped(math.radians(23.), 5., LIMIT, 0.)
    assert TiltGuard().tripped(math.radians(23.), 5., LIMIT, .5) is False  # no reading yet
