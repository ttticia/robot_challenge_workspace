"""User, 2026-11-26 night: a curtain box above 0.7 confidence at the edge under check is a
confirmed curtain at once; the lie-down gap check is not used (curtain_gap off)."""
import inspect
from collections import deque

import pytest

explorer = pytest.importorskip('maze_explore.explorer')
from test_curtain_confirm import EDGE, BACK, Stand     # noqa: E402

E = explorer.Explorer


class Robot(Stand):
    kind, go, do_cv_verify = E.kind, E.go, E.do_cv_verify

    def __init__(self):
        super().__init__()
        self.cv_sure, self.cv_verify_hits, self.cv_verify_time = 0.7, 5, 4.0
        self.curtain_gap = False

    def frames(self, confs, dt=0.3):
        self.act_t0 = self.t
        self.cv_frames = deque((self.t + dt * i, c >= 0.6, 0.0, c) for i, c in enumerate(confs))
        self.t += dt * len(confs)


def test_one_sure_box_confirms_at_once():
    r = Robot()
    r.frames([0.75])                                  # 0.3 s into the check, one frame
    assert r.do_cv_verify(*EDGE, True)
    assert EDGE in r.cv_curtains and BACK in r.cv_curtains
    assert not any('lying down' in e for e in r.events)
    assert not r.queue                                # no lie-down steps


def test_boxes_at_or_under_0_7_wait_for_the_usual_count():
    r = Robot()
    r.frames([0.69, 0.7])
    assert not r.do_cv_verify(*EDGE, True)            # still watching
    assert EDGE not in r.cv_curtains


def test_five_ordinary_hits_confirm_without_lying_down():
    r = Robot()
    r.frames([0.65] * 6, dt=0.7)
    assert r.do_cv_verify(*EDGE, True)
    assert EDGE in r.cv_curtains and not r.queue


def test_lie_down_gap_check_is_off_by_default():
    assert "p('curtain_gap', False)" in inspect.getsource(E.__init__)
    assert "p('cv_sure', 1.0)" in inspect.getsource(E.__init__)   # off since 2026-11-27
