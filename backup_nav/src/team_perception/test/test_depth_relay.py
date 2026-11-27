"""depth_relay's RateGate: every 3rd frame of a 30 Hz stream at 10 Hz, by stamp."""
from team_perception.depth_relay_node import RateGate


def frames(gate, stamps):
    return [t for t in stamps if gate(t)]


def test_30hz_to_10hz_with_jitter():
    stamps = [i / 30 + (0.003 if i % 2 else -0.003) for i in range(300)]   # 10 s, +-3 ms
    got = frames(RateGate(10.0), stamps)
    assert 99 <= len(got) <= 101
    gaps = [b - a for a, b in zip(got, got[1:])]
    assert min(gaps) > 0.09 and max(gaps) < 0.11


def test_dropped_frames_pass_the_next_one():
    stamps = [0.0, 1 / 30, 2 / 30, 0.5, 0.5 + 1 / 30]          # 0.43 s gap (lost frames)
    assert frames(RateGate(10.0), stamps) == [0.0, 0.5]


def test_rate_zero_passes_everything_and_clock_jump_resets():
    assert frames(RateGate(0.0), [0.0, 0.01, 0.02]) == [0.0, 0.01, 0.02]
    g = RateGate(10.0)
    assert frames(g, [100.0, 100.05, 5.0, 5.05, 5.1]) == [100.0, 5.0, 5.1]   # bag loop
