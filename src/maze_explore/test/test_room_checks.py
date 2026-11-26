"""Room checks (user, 2026-11-26 night; 19:21 run: the stairs were in the one room whose
look was skipped once all missions were done, so they were never checked)."""
import math
from collections import deque

import pytest

from maze_explore.grid import Grid

explorer = pytest.importorskip('maze_explore.explorer')
E = explorer.Explorer
C, D = (3, 0), 0


class Robot:
    look_check = E.look_check
    skip_looks = E.skip_looks
    do_side_watch = E.do_side_watch
    look_late_stairs = E.look_late_stairs

    def __init__(self, kind='room', raised=False):
        self.grid = Grid(0.0, 0.0, 0.0, 1.02)
        self.kinds = {(C, D): kind}
        self.look_check_on, self.look_checked, self.look_depth = True, set(), True
        self.stair_check_on, self.stairs_checked, self.look_quick = True, {}, True
        self.room_stairs, self._raised = False, raised   # default: stairs only when seen
        self.cv_on, self.curtains_on = True, True
        self.cv_curtains, self.curtain_latched, self.cv_rejected = set(), set(), set()
        self.cv_pending, self.cv_checking, self.links = set(), set(), {}
        self.side_check_angle, self.side_check_hold = math.radians(45), 3.0
        self.looked, self.no_views, self.events = set(), set(), []
        self.empty_skip, self.empty_early, self.done = True, False, False
        self.queue = deque()

        self.low_walls = set()               # (cell, e) edges measured as ordinary walls
        self.tall_squares, self.empty_squares = 0, 4
        self.stairs_on, self.stairs_done = True, False

    def raised(self, c, d): return self._raised

    def sense(self):
        low, tall = self.low_walls, self.tall_squares
        return type('S', (), {'low_wall': staticmethod(lambda c, e: (c, e) in low),
                              'tall_inside': staticmethod(lambda c, e: (tall, True))})()
    def missions_done(self): return self.done
    def event(self, text): self.events.append(text)


@pytest.fixture(autouse=True)
def no_edge(monkeypatch):
    monkeypatch.setattr(explorer, 'edge_along', lambda *a: None)


def test_the_stair_check_runs_only_on_potential_stairs():
    # user, 2026-11-26 night: only when potential stairs are SEEN (raised / VLM)
    import inspect
    assert "p('room_stairs', False)" in inspect.getsource(E.__init__)
    r = Robot('room', raised=True)           # something stair-high: the check runs
    assert r.look_check(C, D, 0.0)
    acts = [a[0] for a in r.queue]
    assert 'stair_check' in acts and acts.count('side_watch') == 2
    turns = [a[1] for a in r.queue if a[0] == 'turn']
    assert any(abs(t - math.radians(45)) < 1e-6 for t in turns)
    assert any(abs(t + math.radians(45)) < 1e-6 for t in turns)
    assert not r.look_check(C, D, 0.0)       # once only
    r2 = Robot('room', raised=False)         # nothing stair-high: no stair check
    r2.look_check(C, D, 0.0)
    assert 'stair_check' not in [a[0] for a in r2.queue]
    assert [a[0] for a in r2.queue].count('side_watch') == 2   # the side scan stays


def test_an_object_cell_gets_no_stair_check_unless_stair_high():
    r = Robot('object', raised=False)
    r.look_check(C, D, 0.0)
    assert 'stair_check' not in [a[0] for a in r.queue]
    r2 = Robot('object', raised=True)        # stair-high, nothing over the walls
    r2.look_check(C, D, 0.0)
    assert 'stair_check' in [a[0] for a in r2.queue]


def test_a_skipped_look_still_gets_its_checks():
    r = Robot('room')
    r.done = True                            # all missions marked successful
    r.skip_looks()
    assert (C, D) not in r.looked and (C, D) in r.no_views
    r.look_check(C, D, 0.0)
    r.skip_looks()                           # checks made: now it counts as looked
    assert (C, D) in r.looked


def test_curtain_camera_watches_during_a_side_check():
    from maze_explore.curtain_node import watching
    assert watching('RUN cell=(3, 0) job=[look cell (3, 0) side 0 via 0 cells] act=side_watch x')
    assert not watching('RUN cell=(3, 0) job=[look cell (3, 0) side 0 via 0 cells] act=hold x')


def test_a_vlm_stairs_sighting_reopens_the_depth_check():
    import json
    from types import SimpleNamespace as NS

    class R:
        on_scene_result = E.on_scene_result

        def __init__(self):
            self.scene_results, self.events = {}, []
            self.stairs_on = self.stair_check_on = True
            self.stairs_done = False
            self.looked = {((2, 0), 0)}
            self.stairs_checked = {((2, 0), 0): False}    # checked before: not stairs
            self.vlm_stairs = set()

        def event(self, text): self.events.append(text)

    r = R()
    r.on_scene_result(NS(data=json.dumps({'cell': [3, 0], 'stairs': True})))
    assert ((2, 0), 0) in r.vlm_stairs and ((2, 0), 0) not in r.stairs_checked
    assert any('VLM sees stairs' in e for e in r.events)
    # a True verdict, or stairs already crossed, is left alone
    r2 = R()
    r2.stairs_checked[((2, 0), 0)] = True
    r2.on_scene_result(NS(data=json.dumps({'cell': [3, 0], 'stairs': True})))
    assert not r2.vlm_stairs
    r3 = R()
    r3.stairs_done = True
    r3.on_scene_result(NS(data=json.dumps({'cell': [3, 0], 'stairs': True})))
    assert not r3.vlm_stairs


def test_the_vlm_prompt_and_result_carry_stairs():
    from scene_understanding.scene_prompt import LOOK_PROMPT, extract_look_result
    assert '"stairs":false' in LOOK_PROMPT
    r = extract_look_result('{"scene_type":"unknown","objects":[],"hazard":"",'
                            '"survivor":false,"qr":false,"qr_view":"","stairs":true}')
    assert r['stairs'] is True


def test_no_side_checks_when_both_side_walls_are_measured_low():
    # user, 2026-11-26 night (19:42 run): the two 45-deg turns cost ~12 s and turning on a
    # late feed is when the odometry jumps; a low wall hides nothing
    from maze_explore.grid import nb
    b = nb(C, D)
    r = Robot('room')
    r.low_walls = {(b, 1), (b, 3)}           # both side edges measured as ordinary walls
    r.look_check(C, D, 0.0)
    assert 'side_watch' not in [a[0] for a in r.queue]
    r2 = Robot('room')
    r2.low_walls = {(b, 1)}                  # the other side unmeasured: checks stay
    r2.look_check(C, D, 0.0)
    assert [a[0] for a in r2.queue].count('side_watch') == 2


def test_confirmed_content_overrides_the_stair_check_and_the_side_scan():
    # user, 2026-11-26 night (20:12 run): stairs are 0.45 m, under wall height; a cell
    # with content above 0.65 m is a prop room: no stair check, no side scan
    for kind in ('room', 'object'):
        r = Robot(kind, raised=True)         # stair-high by the lidar: the props
        r.tall_squares = 7
        r.look_check(C, D, 0.0)
        acts = [a[0] for a in r.queue]
        assert 'stair_check' not in acts and 'side_watch' not in acts
    r2 = Robot('room', raised=True)          # nothing over the walls: both stay
    r2.look_check(C, D, 0.0)
    acts = [a[0] for a in r2.queue]
    assert 'stair_check' in acts and acts.count('side_watch') == 2


def test_an_empty_room_keeps_the_three_point_side_scan():
    # user, 2026-11-26 night: an empty dead end is where a curtain or the stairs can
    # hide on a side; the scan is face + 45 left + 45 right with the camera watching
    r = Robot('room', raised=False)
    r.tall_squares = 0
    r.look_check(C, D, 0.0)
    acts = [a[0] for a in r.queue]
    assert acts.count('side_watch') == 2 and acts.count('turn') >= 3


def test_stairs_seen_during_the_approach_get_a_late_check():
    # user, 2026-11-26 night: the look's stair check runs from the cell centre, before
    # the walk up to the room; stair-high seen after the approach still gets one check
    r = Robot('room', raised=True)
    r.look_late_stairs(C, D)
    acts = [a[0] for a in r.queue]
    assert acts[-1] == 'stair_check' and r.queue[-1][3] is True and r.queue[-1][4] is False
    r2 = Robot('room', raised=True)          # already checked: nothing queued
    r2.stairs_checked[(C, D)] = False
    r2.look_late_stairs(C, D)
    assert not r2.queue
    r3 = Robot('room', raised=True)          # confirmed content: a prop room, no check
    r3.tall_squares = 7
    r3.look_late_stairs(C, D)
    assert not r3.queue
    r4 = Robot('room', raised=False)         # nothing stair-high: no check
    r4.look_late_stairs(C, D)
    assert not r4.queue


def test_no_curtain_check_during_a_scene_check():
    # user, 2026-11-26 night: the look queues no cv_verify of its own (look_curtain
    # brings it back); candidates come from the side scan or the checks outside looks
    import inspect
    src = inspect.getsource(E.look_check)
    assert "getattr(self, 'look_curtain', False)" in src
    assert "p('look_curtain', False)" in inspect.getsource(E.__init__)
