"""Dead ends wait for scene understanding (user 2026-11-22): a dead end in the cell in front
is held out of goals, not blacklisted; room_scene's /scene/result decides (room ->
blacklisted, stairs / curtain -> candidate queue, else open). No answer 15 s after the last
capture -> re-check on the map (object -> blacklisted, not a dead end -> open), else one
retry; the 2nd try unanswered -> blacklisted. Explorer side, no ROS.

python3 -m pytest -q test/test_dead_end_scene_result.py"""
import json
import math
import time
from types import SimpleNamespace

import numpy as np

from maze_nav.core import Grid
from maze_nav.explorer import Explorer, scene_verdict
from maze_nav.scene_cells import CellGrid


class Fake:
    for _n in ('hold_dead_end', 'blacklist_dead_end', 'launch_cell', 'scene_candidate',
               'no_go_cell', 'on_scene_result', 'scene_timeout_tick', 'still_dead_end',
               'queue_candidate', 'deferred_candidates'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self):
        self.cfg = {'scene_result_timeout': 15.0, 'scene_max_attempts': 2,
                    'scene_look_grace': 30.0}
        self.logs = []
        self.log = SimpleNamespace(info=self.logs.append, warn=self.logs.append)
        self.cells = CellGrid()
        self.blacklist, self.dead_end_cells, self.scene_pending = [], set(), {}
        self.cand_queue, self.blocked_cells, self.exit_side = [], {}, None
        self.defers = []
        self.scene_results = True
        self.retries = []
        self.scene_retry_pub = SimpleNamespace(
            publish=lambda m: self.retries.append(json.loads(m.data)))
        self.grid = dead_end_grid()

    def publish_curtain_map(self):
        pass

    def get_logger(self):
        return self.log

    def grid_now(self):
        return self.grid

    def robot_pose(self):
        return (1.0, 0.0), math.pi


def dead_end_grid(open_far_side=False):
    """Free floor, walls around cell (0, 0) on all sides but +x (the way in from (1, 0)).
    open_far_side: its -x wall missing (not a dead end after all)."""
    res, origin = 0.05, (-2.0, -2.0)
    data = np.zeros((80, 80), np.int16)

    def wall(x0, y0, x1, y1):
        c0, r0 = int((x0 - origin[0]) / res), int((y0 - origin[1]) / res)
        c1, r1 = int((x1 - origin[0]) / res), int((y1 - origin[1]) / res)
        data[r0:r1 + 1, c0:c1 + 1] = 100
    wall(-0.55, 0.45, 0.55, 0.55)        # +y side
    wall(-0.55, -0.55, 0.55, -0.45)      # -y side
    if not open_far_side:
        wall(-0.55, -0.55, -0.45, 0.55)  # -x side (far end)
    return Grid(data, res, origin)


def held(e, cell=(0, 0), captured_ago=None, attempt=1):
    """Cell (0, 0) held, entered from (1, 0) facing -x; its last view captured_ago s ago."""
    e.hold_dead_end(cell, e.cells.centre(cell), math.pi, 'dead end')
    p = e.scene_pending[cell]
    p['attempt'] = attempt
    if captured_ago is not None:
        p['captured'] = time.monotonic() - captured_ago
    return p


def result(cell=(0, 0), **kw):
    r = {'cell': list(cell), 'cell_centre': [0.0, 0.0], 'attempt': 1, 'vlm': 'claude',
         'scene_type': 'dead_end', 'objects': [], 'hazards': []}
    r.update(kw)
    return SimpleNamespace(data=json.dumps(r))


# ---------------------------------------------------------------- verdicts
def test_verdicts():
    assert scene_verdict({'vlm': 'none', 'scene_type': 'room'}) is None     # not analysed
    assert scene_verdict({'vlm': 'x', 'scene_type': 'unknown'}) is None
    assert scene_verdict({'vlm': 'x', 'scene_type': 'room'}) == 'room'
    for st in ('market', 'Kitchen', 'workshop', 'clinic', 'cleaning room'):   # booklet types
        assert scene_verdict({'vlm': 'x', 'scene_type': st}) == 'room', st
    assert scene_verdict({'vlm': 'x', 'scene_type': 'dead_end',
                          'objects': ['toy kitchen']}) == 'room'
    assert scene_verdict({'vlm': 'x', 'scene_type': 'Stairs'}) == 'stairs'
    assert scene_verdict({'vlm': 'x', 'scene_type': 'unknown',
                          'hazards': ['curtain across the way']}) == 'curtain'
    assert scene_verdict({'vlm': 'x', 'scene_type': 'dead_end'}) == 'empty'


# ---------------------------------------------------------------- held, not blacklisted
def test_dead_end_is_held_not_blacklisted():
    e = Fake()
    held(e)
    assert (0, 0) not in e.dead_end_cells and e.blacklist == []
    assert e.no_go_cell((0, 0))


def test_without_scene_understanding_it_is_blacklisted_at_once():
    e = Fake()
    e.scene_results = False
    e.hold_dead_end((0, 0), (0.0, 0.0), math.pi, 'dead end')
    assert (0, 0) in e.dead_end_cells and e.scene_pending == {}


def test_launch_cell_is_never_held_or_blacklisted():
    e = Fake()
    e.exit_side = ((0, 0), (-1, 0))
    e.hold_dead_end((0, 0), (0.0, 0.0), math.pi, 'dead end')
    e.blacklist_dead_end((0, 0), (0.0, 0.0), 'dead end')
    assert e.scene_pending == {} and (0, 0) not in e.dead_end_cells


# ---------------------------------------------------------------- results
def test_room_is_blacklisted():
    e = Fake()
    held(e)
    e.on_scene_result(result(scene_type='room', objects=['chair']))
    assert (0, 0) in e.dead_end_cells and (0.0, 0.0) in e.blacklist
    assert e.scene_pending == {}


def test_stairs_are_queued_and_kept_out_of_until_checked():
    e = Fake()
    held(e)
    e.on_scene_result(result(scene_type='stairs'))
    assert e.scene_pending == {} and (0, 0) not in e.dead_end_cells
    c = e.cand_queue[0]
    assert c['kind'] == 'stairs' and c['id'] is None and c['cell'] == (0, 0)
    # target on the side shared with the cell before (+x side, x = 0.5), facing -x
    assert abs(c['target'][0] - 0.5) < 1e-9 and abs(c['heading'] - math.pi) < 1e-9
    assert e.no_go_cell((0, 0))                    # no goal into it before the check
    e.cand_queue.remove(c)                         # queue_goal released it: open again
    assert not e.no_go_cell((0, 0))


def test_curtain_is_queued():
    e = Fake()
    held(e)
    e.on_scene_result(result(scene_type='unknown', hazards=['curtain']))
    assert [c['kind'] for c in e.cand_queue] == ['curtain']


def test_empty_dead_end_is_opened_not_blacklisted():
    e = Fake()
    held(e)
    e.on_scene_result(result(scene_type='dead_end'))
    assert e.scene_pending == {} and not e.no_go_cell((0, 0))


def test_not_analysed_result_keeps_waiting():
    e = Fake()
    held(e)
    e.on_scene_result(result(vlm='none', scene_type='unknown'))
    assert (0, 0) in e.scene_pending


def test_result_for_a_cell_not_held_is_ignored():
    e = Fake()
    e.on_scene_result(result(scene_type='room', objects=['chair']))
    assert e.dead_end_cells == set()


# ---------------------------------------------------------------- no answer in time
def test_waits_15_s_after_the_last_capture():
    e = Fake()
    held(e, captured_ago=10.0)
    e.scene_timeout_tick()
    assert e.retries == [] and (0, 0) in e.scene_pending


def test_no_answer_still_a_dead_end_retries_once_then_blacklists():
    e = Fake()
    p = held(e, captured_ago=16.0)
    e.scene_timeout_tick()
    assert e.retries == [{'cell': [0, 0], 'attempt': 2}]
    assert p['attempt'] == 2 and (0, 0) not in e.dead_end_cells
    e.scene_timeout_tick()                         # the retry's 15 s haven't passed
    assert len(e.retries) == 1
    p['captured'] -= 16.0
    e.scene_timeout_tick()                         # 2nd try unanswered
    assert len(e.retries) == 1 and (0, 0) in e.dead_end_cells


def test_a_late_answer_to_the_first_try_still_counts():
    e = Fake()
    held(e, captured_ago=16.0)
    e.scene_timeout_tick()                         # retry sent
    e.on_scene_result(result(attempt=1, scene_type='dead_end'))
    assert e.scene_pending == {} and (0, 0) not in e.dead_end_cells


def test_no_answer_object_in_the_cell_blacklists_without_retry():
    e = Fake()
    held(e, captured_ago=16.0)
    e.blocked_cells = {(0, 0): (0.4, (0.0, 0.0))}
    e.scene_timeout_tick()
    assert e.retries == [] and (0, 0) in e.dead_end_cells


def test_no_answer_not_a_dead_end_any_more_opens_it():
    e = Fake()
    e.grid = dead_end_grid(open_far_side=True)
    held(e, captured_ago=16.0)
    e.scene_timeout_tick()
    assert e.retries == [] and e.scene_pending == {} and (0, 0) not in e.dead_end_cells


def test_never_captured_is_blacklisted_after_the_grace():
    e = Fake()
    p = held(e)
    p['since'] -= 31.0
    e.scene_timeout_tick()
    assert (0, 0) in e.dead_end_cells and e.retries == []


def test_still_dead_end_on_the_map():
    e = Fake()
    assert e.still_dead_end((0, 0), math.pi) is True
    e.grid = dead_end_grid(open_far_side=True)
    assert e.still_dead_end((0, 0), math.pi) is False
