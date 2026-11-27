"""The heat / load check of this computer before a run, and its logger (2026-11-26)."""
import os

from maze_explore import heat


def fake(monkeypatch, temps, throttled, programs=(), turbo=False):
    temps, throttled = list(temps), list(throttled)
    monkeypatch.setattr(heat, 'temperature', lambda: temps.pop(0) if len(temps) > 1 else temps[0])
    monkeypatch.setattr(heat, 'throttled_ms', lambda: throttled.pop(0) if len(throttled) > 1 else throttled[0])
    monkeypatch.setattr(heat, 'busy', lambda: list(programs))
    monkeypatch.setattr(heat, 'load', lambda: 0.8)
    monkeypatch.setattr(heat, 'turbo_on', lambda: turbo)
    monkeypatch.setattr(heat.time, 'sleep', lambda s: None)


def test_a_cool_idle_computer_is_a_go(monkeypatch):
    fake(monkeypatch, [58, 60], [1000, 1000])
    ok, lines = heat.check()
    assert ok and any('60 C' in l for l in lines)


def test_hot_throttled_or_rviz_running_is_a_no_go(monkeypatch):
    fake(monkeypatch, [83, 91], [1000, 1000])
    ok, lines = heat.check()
    assert not ok and any('NO-GO' in l and '91 C' in l for l in lines)
    fake(monkeypatch, [60, 62], [1000, 1900])                    # throttled 900 ms while we looked
    ok, lines = heat.check()
    assert not ok and any('throttled right now' in l for l in lines)
    fake(monkeypatch, [60, 60], [1000, 1000], programs=[('rviz2', 49.7)])
    ok, lines = heat.check()
    assert not ok and any('rviz2 is running' in l for l in lines)


def test_turbo_is_a_notice_not_a_no_go(monkeypatch):
    fake(monkeypatch, [60, 60], [1000, 1000], turbo=True)
    ok, lines = heat.check()
    assert ok and any('turbo boost is on' in l for l in lines)


def test_the_logger_writes_one_line_per_period(monkeypatch, tmp_path):
    fake(monkeypatch, [70], [1000, 1000, 1400, 1400])
    n = {'k': 0}

    def sleep(s):
        n['k'] += 1
        if n['k'] > 3:
            raise KeyboardInterrupt
    monkeypatch.setattr(heat.time, 'sleep', sleep)
    path = os.path.join(tmp_path, 'run', 'heat.csv')
    try:
        heat.log(path)
    except KeyboardInterrupt:
        pass
    rows = open(path).read().strip().split('\n')
    assert rows[0] == 'time,unix,temp_c,throttled_ms,load1,turbo' and len(rows) == 4
    assert [r.split(',')[3] for r in rows[1:]] == ['0', '400', '0']      # throttled ms per period
