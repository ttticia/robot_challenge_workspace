from lite3_nav.progress import GoalProgress


def test_long_trip_with_progress_does_not_time_out():
    p = GoalProgress()
    p.start((0., 0.), 0.)
    for t in range(0, 200, 5):                  # 0.1 m/s for 200 s: 20 m, never stuck
        p.update((t * .1, 0.), float(t), .3)
        assert not p.timed_out(float(t), 45., .3, 600.)


def test_no_progress_times_out_after_timeout():
    p = GoalProgress()
    p.start((0., 0.), 0.)
    p.update((1., 0.), 10., .3)
    p.update((1.1, 0.), 50., .3)                # under 0.3 m since t=10: no progress
    assert not p.timed_out(50., 45., .3)
    assert p.timed_out(56., 45., .3)


def test_cap_and_fixed_timeout():
    p = GoalProgress()
    p.start((0., 0.), 0.)
    p.update((5., 0.), 299., .3)
    assert not p.timed_out(300., 45., .3, 300.) and p.timed_out(301., 45., .3, 300.)
    q = GoalProgress()                          # progress_m 0: the team's fixed timeout
    q.start((0., 0.), 0.)
    q.update((5., 0.), 30., 0.)
    assert not q.timed_out(45., 45., 0.) and q.timed_out(46., 45., 0.)
