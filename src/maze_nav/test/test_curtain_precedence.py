"""Curtains vs junctions and scans (user 2026-11-20), explorer side, no ROS.

Crossings: junctions and scans come first. Inspections (a possible curtain): the lie-down
check comes first, a junction look is paused for it. A curtain confirmed at a junction is
held until the junction's other ways are explored."""
import json
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from maze_nav.core import Frontier, Grid
from maze_nav.curtain_branch import CurtainHold, Deferral, branch_mask, in_mask, way_towards
from maze_nav.curtains import Curtain
from maze_nav.explorer import Explorer
from maze_nav.scene_cells import CellGrid


class Fake:
    for _n in ('on_curtain', 'on_junction', 'junction_pending', 'curtain_preemptable',
               'junction_age', 'preempt_curtain', 'set_state', 'take_for_curtain',
               'pause_junction', 'end_curtain', 'publish_hold', 'defer_curtain',
               'curtain_by_id', 'branch_frontiers', 'on_handover'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self, state, junction_req=None, curtain_status=None):
        self.state, self.junction_req, self.curtain_status = state, junction_req, curtain_status
        self.cfg = {'go': True, 'junction_timeout': 3.0, 'done_confirm': 2,
                    'curtain_approach': 0.9}
        self.logs, self.states, self.junction_tries, self.holds = [], [], 0, []
        self.log = SimpleNamespace(info=lambda m, **k: self.logs.append(m),
                                   warn=lambda m, **k: self.logs.append(m))
        self.status_pub = SimpleNamespace(publish=lambda m: self.states.append(
            m.data.split(':', 1)[0]))
        self.hold_pub = SimpleNamespace(publish=lambda m: self.holds.append(
            CurtainHold.from_json(m.data)))
        self.curtain_left_at = None
        self.spin_handle = self.goal_handle = self.current = self.goal_result = None
        self.look_turns = self.face = self.turn_creep = None
        self.junction = self.paused_junction = None
        self.inspecting, self.defers, self.hold, self.branch_cache = False, [], None, None
        self.queue_wait = None
        self.blacklist, self.failed, self.curtains, self.pose = [], [], [], None
        self.map = object()

    def get_logger(self):
        return self.log

    def stop_robot(self):
        pass

    def end_pose(self, why):
        pass

    def cancel_junction_goal(self):
        pass

    def maybe_junction(self):
        self.junction_tries += 1

    def begin_after_stairs(self, why):
        self.set_state('EXPLORING', why)

    def clear_local_costmap(self, why, global_too=False):
        pass

    def curtains_in_map(self):
        return list(self.curtains)

    def robot_pose(self):
        return self.pose

    def back_to_start(self, p):
        return False


def status(e, text):
    e.on_curtain(SimpleNamespace(data=text))


def handover(e):
    status(e, 'HANDOVER: curtain 1 1.8 m ahead, 0 cm off its centre line')


def inspect_handover(e):
    status(e, 'HANDOVER: possible curtain (wall with a gap under it): asking maze_explorer')


CELLS = CellGrid(1.0, (0.0, 0.0), 0.0)


def junction_rec(cell=(1, 0)):
    """Junction at cell (1, 0), come in from (0, 0): ways east (2, 0) and north (1, 1)."""
    return {'cell': cell, 'from_cell': (0, 0), 'cells': CELLS, 'half_width': 0.35,
            'paths': [('+i', 0.0), ('+j', math.pi / 2)],
            'cell_centre': CELLS.centre(cell)}


def junction_msg():
    return SimpleNamespace(data=json.dumps({
        'paths': [{'dir': 'left', 'heading': 1.57}, {'dir': 'right', 'heading': -1.57}],
        'cell_centre': [1.0, 2.0], 'cell': [1, 2], 'from_cell': [1, 1]}))


# ------------------------------------------------------------------ crossings: junction first

def test_crossing_handover_refused_during_looks_and_scans():
    for s in ('JUNCTION', 'DEAD_END', 'SCANNING'):
        e = Fake(s)
        handover(e)
        assert e.state == s and e.states == [], s
        assert 'comes first' in e.logs[-1]


def test_crossing_handover_refused_while_a_junction_waits():
    e = Fake('EXPLORING', junction_req=({'cell': (1, 2)}, time.monotonic()))
    handover(e)
    assert e.state == 'EXPLORING' and 'junction request' in e.logs[-1]
    e = Fake('EXPLORING', junction_req=({'cell': (1, 2)}, time.monotonic() - 10.0))
    assert not e.junction_pending()


def test_preemptable_only_a_crossing_before_the_fabric():
    for st, want in (('FACING', True), ('APPROACHING', True), ('HANDOVER', False),
                     ('THROUGH', False), ('WIGGLING', False), ('CHECKING', False)):
        assert Fake('CURTAIN', curtain_status=st).curtain_preemptable() is want, st
    for st in ('INSPECTING', 'LYING', 'FACING', 'APPROACHING'):   # inspection: never
        e = Fake('CURTAIN', curtain_status=st)
        e.inspecting = True
        assert not e.curtain_preemptable(), st
    assert not Fake('AT_CURTAIN', curtain_status='FACING').curtain_preemptable()


def test_junction_takes_the_robot_back_from_a_crossing_approach():
    e = Fake('CURTAIN', curtain_status='APPROACHING')
    e.on_junction(junction_msg())
    assert e.state == 'EXPLORING' and e.junction_tries == 1
    e = Fake('CURTAIN', curtain_status='THROUGH')          # in the fabric: crossing goes on
    e.on_junction(junction_msg())
    assert e.state == 'CURTAIN' and e.junction_tries == 0 and e.junction_req[0]['held']


def test_held_junction_ages_from_the_end_of_curtain():
    e = Fake('CURTAIN', curtain_status='THROUGH')
    e.on_junction(junction_msg())
    e.junction_req = (e.junction_req[0], time.monotonic() - 60.0)
    status(e, 'PASSED: through curtain 1')
    assert e.state == 'EXPLORING' and e.curtain_left_at is not None
    assert e.junction_pending()                  # counts from PASSED, not from 60 s ago
    e.curtain_left_at -= 10.0
    assert not e.junction_pending()
    e = Fake('EXPLORING')
    e.on_junction(junction_msg())
    assert not e.junction_req[0].get('held')


# ------------------------------------------------------------------ inspections: check first

def look(i=1, turned=True):
    return {'steps': [{'name': 'junction way east'}, {'name': 'junction way north'}], 'i': i,
            'turned': turned, 'hold_until': None, 'dead_end': None, 'rec': junction_rec()}


def test_inspection_pauses_the_junction_look_and_resumes_it():
    e = Fake('JUNCTION')
    e.junction = look(i=2)                       # turning to the 2nd way when interrupted
    inspect_handover(e)
    assert e.state == 'CURTAIN' and e.inspecting and e.junction is None
    assert e.paused_junction['i'] == 1 and not e.paused_junction['turned']   # turn again
    assert e.holds[-1].hold_new                  # at a junction: hold a confirmed curtain
    # LYING: a junction can't take it back, nor a crossing-style pre-empt
    status(e, 'LYING: lie_down: measuring the gap from low down')
    assert e.state == 'CURTAIN'
    # not a curtain: back to the look, nothing held
    status(e, 'IDLE: lying down: gap 30 cm, not 12-20 cm: not a curtain; left alone')
    assert e.state == 'JUNCTION' and e.junction['i'] == 1 and e.paused_junction is None
    assert not e.holds[-1].hold_new and e.holds[-1].ids == [] and e.defers == []


def test_inspection_first_even_with_a_junction_waiting():
    e = Fake('EXPLORING', junction_req=(junction_rec(), time.monotonic()))
    inspect_handover(e)
    assert e.state == 'CURTAIN' and e.junction_req[0]['held'] and e.holds[-1].hold_new
    e = Fake('DEAD_END')                         # dead-end scan still first
    inspect_handover(e)
    assert e.state == 'DEAD_END'


def test_inspection_away_from_junctions_doesnt_hold():
    e = Fake('EXPLORING')
    inspect_handover(e)
    assert e.state == 'CURTAIN' and not e.holds[-1].hold_new


def curtain_east(cid=3):
    """Curtain across the east way, 0.5 m into cell (2, 0) (x = 2.0), crossed along +x."""
    return Curtain(np.array([2.0, -0.45]), np.array([2.0, 0.45]), np.array([1.0, 0.0]),
                   0.15, cid)


def test_confirmed_at_junction_is_held_and_the_other_way_comes_first():
    e = Fake('JUNCTION')
    e.junction = look(i=1)
    inspect_handover(e)
    e.curtains = [curtain_east()]
    e.blacklist = [(2.6, 0.0), (5.0, 5.0)]       # the "dead end" behind it, and another
    status(e, 'CONFIRMED: curtain 3 confirmed (lying down: gap 15 cm): maze_explorer explores')
    assert e.state == 'JUNCTION' and e.junction is not None
    assert e.junction['rec']['skip_dirs'] == [(1, 0)]          # east = the curtain's way
    assert len(e.defers) == 1 and e.defers[0].ways == [(0, 1)]  # north first
    assert e.holds[-1].ids == [3] and not e.holds[-1].hold_new
    assert e.blacklist == [(5.0, 5.0)]


# ------------------------------------------------------------------ branch

def maze_grid():
    """Corridor y = -0.5..0.5 from x = -0.5 to 4, a north corridor at x = 0.5..1.5 up to
    y = 3, walls round both; the curtain at x = 2 (map occupancy: free 0, wall 100)."""
    res = 0.05
    w, h = int(5.0 / res), int(4.0 / res)
    data = np.full((h, w), 100, np.int16)
    origin = (-0.5, -1.0)

    def free(x0, x1, y0, y1):
        c0, c1 = int((x0 - origin[0]) / res), int((x1 - origin[0]) / res)
        r0, r1 = int((y0 - origin[1]) / res), int((y1 - origin[1]) / res)
        data[r0:r1, c0:c1] = 0
    free(-0.45, 3.95, -0.45, 0.45)
    free(0.55, 1.45, 0.45, 2.9)
    return Grid(data, res, origin)


def test_branch_mask_is_the_other_way_only():
    g = maze_grid()
    m = branch_mask(g, CELLS, (1, 0), [(0, 1)], [curtain_east()])
    assert in_mask(m, g, (1.0, 2.0))             # up the north way
    assert not in_mask(m, g, (1.0, 0.0))         # the junction cell itself
    assert not in_mask(m, g, (0.0, 0.0))         # the way in
    assert not in_mask(m, g, (1.7, 0.0))         # towards the curtain
    assert not in_mask(m, g, (3.0, 0.0))         # behind it
    assert way_towards(CELLS, (1, 0), [(1, 0), (0, 1)], (2.0, 0.0)) == (1, 0)


def frontier(xy):
    return Frontier(goal=xy, yaw=0.0, path_length=1.0, size=10, centroid=xy)


def test_branch_frontiers_then_release_and_go_to_the_curtain():
    e = Fake('EXPLORING')
    e.curtains = [curtain_east()]
    e.defers = [Deferral(3, (1, 0), CELLS, [(0, 1)])]
    g = maze_grid()
    north, behind, back = frontier((1.0, 2.5)), frontier((3.5, 0.0)), frontier((-0.3, 0.0))
    out = e.branch_frontiers(g, (1.0, 1.0), [behind, north, back])
    assert out == [north]                        # the other way only
    # north done: wait done_confirm replans, then release and head for the curtain
    assert e.branch_frontiers(g, (1.0, 2.5), [behind, back]) is None
    out = e.branch_frontiers(g, (1.0, 2.5), [behind, back])
    assert e.defers[0].released and e.holds[-1].ids == []
    assert len(out) == 1 and out[0].goal[0] > 2.0          # behind the curtain: redirected
    # in front of it: curtain_detector's turn
    assert e.branch_frontiers(g, (1.1, 0.0), [behind, back]) is None
    # crossed: back to normal
    e.curtains[0].passed.append(1)
    assert e.branch_frontiers(g, (2.5, 0.0), [behind, back]) == [behind, back]
    assert e.defers == []


# ------------------------------------------------------------------ curtain_detector side

from maze_nav.curtain_detector import CurtainDetector  # noqa: E402


class FakeDetector:
    control = CurtainDetector.control
    yield_to_explorer = CurtainDetector.yield_to_explorer

    def __init__(self, state, explorer):
        self.state, self.explorer, self.status_topic = state, explorer, '/maze/status'
        self.lock, self.lock_id, self.cooldown_until, self.aborted = object(), 1, 0.0, None

    def stop_robot(self):
        pass

    def set_state(self, state, why=''):
        self.state = state

    def abort(self, why):
        self.aborted = why
        self.state = 'ABORTED'


def test_detector_yields_before_the_fabric_only():
    for explorer in ('JUNCTION', 'DEAD_END', 'EXPLORING'):
        for st in ('FACING', 'APPROACHING'):
            d = FakeDetector(st, explorer)
            d.control()
            assert d.state == 'IDLE' and d.aborted is None and d.lock_id is None, (st, explorer)
            assert d.cooldown_until > time.monotonic()
    d = FakeDetector('THROUGH', 'JUNCTION')          # in the fabric: abort as before
    d.control()
    assert d.state == 'ABORTED'
    d = FakeDetector('APPROACHING', 'STOPPED')       # not a look: abort as before
    d.control()
    assert d.state == 'ABORTED'




def test_detector_holds_a_confirmed_curtain_when_asked():
    from maze_nav.curtain_detector import CurtainDetector

    class D:
        after_confirm = CurtainDetector.after_confirm

        def __init__(self, hold_new):
            self.hold = CurtainHold(hold_new, [])
            self.lock, self.lock_id, self.cooldown_until = object(), 4, 0.0
            self.state, self.why, self.walked = 'INSPECTING', '', False
            self.log = SimpleNamespace(info=lambda *a, **k: None)

        def get_logger(self):
            return self.log

        def start_manoeuvre(self):
            self.walked = True

        def set_state(self, state, why=''):
            self.state, self.why = state, why

    d = D(True)
    d.after_confirm(None, 'lying down: gap 15 cm')
    assert d.state == 'CONFIRMED' and 'curtain 4' in d.why and not d.walked
    assert d.lock is None and d.lock_id is None
    d = D(False)
    d.after_confirm(None, 'lying down: gap 15 cm')
    assert d.walked and d.state == 'INSPECTING'
    assert CurtainHold.from_json('junk').hold_new is False


# ------------------------------------------------------------------ crossing back (fix 1)

def test_needs_crossing():
    from maze_nav.curtain_branch import needs_crossing
    g = maze_grid()                               # curtain at x = 2 closes the east corridor
    c = curtain_east()
    assert needs_crossing(g, (1.0, 0.0), (3.5, 0.0), c)        # behind it: only through it
    assert needs_crossing(g, (3.5, 0.0), (1.0, 2.5), c)        # and back
    assert not needs_crossing(g, (1.0, 0.0), (1.0, 2.5), c)    # the north way: no
    assert not needs_crossing(g, (-0.3, 0.0), (1.0, 2.5), c)


class RedirectFake(Fake):
    curtain_redirect = Explorer.curtain_redirect

    def __init__(self, xy):
        super().__init__('EXPLORING')
        self.cfg.update(curtain_goal_radius=0.7, curtain_spot_wait=15.0)
        self.redirects, self.spot_wait, self.goals = {}, None, []
        self.xy = xy

    def grid_now(self):
        return maze_grid()

    def publish_goal(self, xy, yaw):
        self.goals.append(xy)


def test_crossed_curtain_goal_behind_goes_to_its_front_first():
    c = curtain_east()
    c.passed.append(1)                          # walked through along +x once
    e = RedirectFake((3.5, 0.0))
    e.curtains = [c]
    back_goal = frontier((1.0, 2.5))            # up the north way: only back through it
    out = e.curtain_redirect(back_goal, e.xy)
    assert math.hypot(out.goal[0] - 2.9, out.goal[1]) < 1e-6    # 0.9 m in front, far side
    assert e.redirects[out.goal] == ((1.0, 2.5), False)         # not blacklisted on arrival
    # at the spot: wait for curtain_detector, with the real goal on /maze/goal
    assert e.curtain_redirect(back_goal, (2.9, 0.0)) is None
    assert e.goals == [(1.0, 2.5)]
    e.spot_wait = (c.id, time.monotonic() - 20.0)               # it never came: Nav2 tries
    assert e.curtain_redirect(back_goal, (2.9, 0.0)) is back_goal
    # a goal on this side is left alone
    e = RedirectFake((3.5, 0.0))
    e.curtains = [c]
    here = frontier((3.8, 0.2))
    assert e.curtain_redirect(here, e.xy) is here and e.spot_wait is None


def test_uncrossed_curtain_far_goal_goes_to_its_front_too():
    e = RedirectFake((0.0, 0.0))
    e.curtains = [curtain_east()]
    out = e.curtain_redirect(frontier((3.8, 0.0)), e.xy)         # 1.8 m behind it
    assert math.hypot(out.goal[0] - 1.1, out.goal[1]) < 1e-6
    out = e.curtain_redirect(frontier((2.3, 0.0)), e.xy)         # at it: as before
    assert e.redirects[out.goal] == ((2.3, 0.0), True)


def test_startup_guard_logs_once_subscribed():
    class G:
        check_map_subscribers = Explorer.check_map_subscribers

        def __init__(self, n):
            self.first_map, self.map_sub_ok, self.logs = time.monotonic() - 30.0, False, []
            self.curtain_map_pub = SimpleNamespace(get_subscription_count=lambda: n)
            self.log = SimpleNamespace(info=lambda m, **k: self.logs.append(('info', m)),
                                       error=lambda m, **k: self.logs.append(('error', m)))

        def get_logger(self):
            return self.log

    g = G(0)
    g.check_map_subscribers()
    assert g.logs[-1][0] == 'error' and not g.map_sub_ok
    g = G(1)
    g.check_map_subscribers()
    assert g.map_sub_ok and g.logs[-1][0] == 'info'
    g.check_map_subscribers()
    assert len(g.logs) == 1
    g = G(0)
    g.first_map = time.monotonic()               # too early to tell
    g.check_map_subscribers()
    assert g.logs == []


def test_crossing_back_triggers_with_looser_limits():
    from maze_nav.curtain_detector import CurtainDetector

    class T:
        maybe_trigger = CurtainDetector.maybe_trigger

        def __init__(self, cur):
            self.cfg = {'trigger_distance': 2.0, 'trigger_max_offset': 0.35,
                        'trigger_max_bearing': math.radians(50), 'back_max_offset': 0.45,
                        'back_max_bearing': math.radians(80), 'regoal_radius': 1.0}
            self.cur, self.status_topic, self.hold, self.state = cur, '', CurtainHold(), 'IDLE'

        def curtains_odom(self):
            return [self.cur]

        def set_state(self, state, why=''):
            self.state = state

    # robot 1 m before the line, 0.4 m off its centre, facing 65 deg off its middle
    xy, yaw = (1.0, 0.4), math.atan2(-0.4, 1.0) + math.radians(65)
    first = curtain_east()
    t = T(first)
    t.maybe_trigger(xy, yaw)
    assert t.state == 'IDLE'                     # first crossing: strict limits
    back = curtain_east()
    back.passed.append(-1)                       # crossed the other way once
    t = T(back)
    t.maybe_trigger(xy, yaw)
    assert t.state == 'HANDOVER'


# ------------------------------------------------------------------ candidate queue

class QueueFake(Fake):
    for _n in ('to_map', 'queue_candidate', 'on_stairs_candidate', 'on_curtain_candidate',
               'release', 'start_queue', 'candidate_spot', 'queue_goal',
               'deferred_candidates', 'defer_side_curtains', 'workable', 'held_stair_cells'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self, state):
        super().__init__(state)
        self.cfg.update({'stairs_approach': 1.2, 'curtain_check_approach': 1.2,
                         'candidate_wait': 15.0, 'robot_radius': 0.25, 'turn_margin': 0.03})
        self.cand_queue, self.queue_active, self.queue_final = [], False, False
        self.released, self.spins = [], []
        self.cells = CellGrid()
        pub = lambda kind: SimpleNamespace(  # noqa: E731
            publish=lambda m: self.released.append((kind, json.loads(m.data)['id'])))
        self.stairs_release_pub, self.curtain_release_pub = pub('stairs'), pub('curtain')
        self.spin = SimpleNamespace(server_is_ready=lambda: True)
        self.box, self.pivot = (0.37, 0.27, 0.23), (0.0, 0.0)

    def grid_now(self):                          # no map: nothing held back beside a way
        return None

    def publish_curtain_map(self):
        self.map_pubs = getattr(self, 'map_pubs', 0) + 1

    def halt_for_look(self):
        pass

    def send_spin(self, angle, why=''):
        self.spins.append(angle)


def stairs_msg(sid=7, edge=(2.0, 0.0)):
    return SimpleNamespace(data=json.dumps({'id': sid, 'frame': 'map', 'edge': list(edge),
                                            'heading': 0.0}))


def curtain_msg(cid=2, centre=(1.0, 2.5)):
    """Possible curtain across the north way, normal pointing north (away from the robot)."""
    return SimpleNamespace(data=json.dumps({'id': cid, 'frame': 'map', 'centre': list(centre),
                                            'normal': [0.0, 1.0]}))


def test_candidates_are_queued_not_released():
    e = QueueFake('JUNCTION')
    e.on_stairs_candidate(stairs_msg())
    e.on_stairs_candidate(stairs_msg())                          # seen again: one entry
    e.on_curtain_candidate(curtain_msg())
    assert e.released == []
    assert [(c['kind'], c['id']) for c in e.cand_queue] == [('stairs', 7), ('curtain', 2)]
    assert not e.queue_active and e.start_queue('junction scanned') and e.queue_active


def test_queue_goes_to_the_nearest_spot_faces_it_and_releases_it():
    e = QueueFake('EXPLORING')
    e.on_stairs_candidate(stairs_msg(edge=(2.0, 0.0)))           # spot (0.8, 0.0)
    e.on_curtain_candidate(curtain_msg(centre=(1.0, 3.5)))       # spot (1.0, 2.3), further
    e.start_queue('nothing else to explore', final=True)       # stairs worked too
    g = maze_grid()
    out = e.queue_goal(g, (1.0, 1.0), 0.0)
    assert len(out) == 1 and out[0].goal == (0.8, 0.0) and out[0].yaw == 0.0
    # at the spot but facing north: turn to face the stairs first
    assert e.queue_goal(g, (0.8, 0.05), math.pi / 2) is None
    assert e.state == 'TURNING' and e.spins and e.spins[-1] < 0 and e.released == []
    # facing them: released, then wait for the detector's handover
    e.state = 'EXPLORING'
    assert e.queue_goal(g, (0.8, 0.05), 0.0) is None
    assert e.released == [('stairs', 7)]
    assert e.queue_goal(g, (0.8, 0.05), 0.0) is None              # waiting
    e.on_handover()                                               # STAIRS took over
    out = e.queue_goal(g, (0.8, 0.05), 0.0)
    assert len(out) == 1 and out[0].goal == pytest.approx((1.0, 2.3))
    e.cand_queue.clear()
    assert e.queue_goal(g, (1.0, 2.3), math.pi / 2) == [] and not e.queue_active


def test_stairs_wait_until_nothing_else_is_left():
    """User 2026-11-22: cross stairs only once every other way is exhausted."""
    e = QueueFake('EXPLORING')
    e.on_stairs_candidate(stairs_msg(edge=(2.0, 0.0)))           # nearer
    e.on_curtain_candidate(curtain_msg(centre=(1.0, 3.5)))
    assert e.map_pubs >= 1                                       # stairs cell walled for Nav2
    assert e.held_stair_cells() == {(3, 0)}
    assert e.start_queue('junction scanned')
    g = maze_grid()
    out = e.queue_goal(g, (1.0, 1.0), 0.0)
    assert out[0].goal == pytest.approx((1.0, 2.3))              # the curtain, not the stairs
    e.cand_queue = [c for c in e.cand_queue if c['kind'] == 'stairs']
    assert e.queue_goal(g, (1.0, 2.3), 0.0) == [] and not e.queue_active
    assert not e.start_queue('another junction scanned')        # stairs alone: not yet
    assert e.start_queue('nothing else to explore', final=True)
    assert e.held_stair_cells() == set()                         # open for Nav2 now
    out = e.queue_goal(g, (1.0, 1.0), 0.0)
    assert out[0].goal == (0.8, 0.0)


def test_candidate_without_a_detector_id_is_not_released():
    e = QueueFake('EXPLORING')
    e.queue_candidate('curtain', None, (1.0, 3.5), math.pi / 2, 'scene result')
    e.queue_candidate('curtain', None, (3.0, 0.0), 0.0, 'scene result')
    assert len(e.cand_queue) == 2
    e.release('curtain', None, 'in front')
    assert e.released == []
