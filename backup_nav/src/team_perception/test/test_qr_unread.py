"""Codes located but not read: scan_qr's unread list and UnreadTracker's persistence."""
import numpy as np
import pytest

from team_perception import qr_utils
from team_perception.qr_utils import UnreadTracker, decode_qr, scan_qr
from test_qr_utils import make_qr


def _code(text='HINT'):
    return make_qr(text, size=300, border=60)


def test_decode_qr_still_returns_only_decoded():
    assert [t for t, _ in decode_qr(_code('A'))] == ['A']


@pytest.mark.skipif(qr_utils._zx is None, reason='unread codes need zxing-cpp')
def test_unreadable_code_is_reported_as_unread():
    img = _code('B')
    h, w = img.shape[:2]
    # Wipe a band through the data area (finder patterns intact): located, not decodable.
    bad = img.copy()
    bad[int(h * 0.45):int(h * 0.62), int(w * 0.30):int(w * 0.80)] = 0
    found, unread = scan_qr(bad, try_upscale=False, try_clahe=False)
    assert found == [] and len(unread) == 1
    u, v = unread[0].mean(axis=0)
    assert abs(u - w / 2) < w * 0.15 and abs(v - h / 2) < h * 0.15
    found, unread = scan_qr(img, try_upscale=False, try_clahe=False)
    assert [t for t, _ in found] == ['B'] and unread == []   # readable: not unread


def test_tracker_needs_persistence():
    t = UnreadTracker(need=3, window=5)
    assert t.update([(100, 100, 900)]) is None
    assert t.update([]) is None
    assert t.update([(104, 98, 900)]) is None
    assert t.update([(110, 101, 900)]) == pytest.approx((110, 101))   # 3 of last 4
    t.clear()
    assert t.update([(110, 101, 900)]) is None


def test_tracker_follows_one_code_and_restarts_on_a_jump():
    t = UnreadTracker(need=2, window=5, max_jump_px=80)
    t.update([(100, 100, 900), (500, 300, 100)])          # starts on the bigger outline
    assert t.update([(108, 102, 900), (505, 300, 100)]) == pytest.approx((108, 102))
    assert t.update([(400, 400, 900)]) is None              # far jump: start over
    assert t.update([(405, 402, 900)]) == pytest.approx((405, 402))


def test_scan_qr_drops_unread_outline_overlapping_a_read_code():
    a = np.float32([[10, 10], [50, 10], [50, 50], [10, 50]])
    b = a + 5
    assert qr_utils._overlaps(a, b) and not qr_utils._overlaps(a, a + 200)
