"""The end stop (user, 2026-11-26): nose 0.10 m before the open far edge of the end cell,
turned 15 deg away from maze_side, then lying down; and the gait / curtain-turn rules of the
same day. The explorer's own plan / go / turn / drive code runs on the stand-in of
test_qr_approach (perfect motion)."""
import math

import pytest

from maze_explore import robot as codes
from maze_explore.grid import wrap
from test_qr_approach import Robot, explorer

E = explorer.Explorer


def make(**fields):
    R = type('R', (Robot,), dict(
        do_drive=E.do_drive, do_call=E.do_call, do_hold=E.do_hold, kind=E.kind,
        gait_for_step=E.gait_for_step, go=E.go, do_strafe=E.do_strafe,
        do_cmd=lambda self, code: self.sent.append(code) or True))
    r = R()
    r.v_max, r.ai_speed, r.curtain_speed, r.side_speed = 0.2, 0.15, 0.1, 0.1
    r.push_stop, r.push_sure, r.step, r.sent = True, False, None, []
    r.cell_size, r.end_gap, r.end_turn, r.lie_down, r.maze_rows = 1.02, 0.10, math.radians(15), True, 5
    r.halted = []
    r.halt = lambda state, why: r.halted.append(state)
    r.visited, r.exit_weak, r.exit_open = set(), {}, set()
    r.wall_share = lambda cell, d: 0.0
    for k, v in fields.items():
        setattr(r, k, v)
    return r


def exit_queue(r, cell, d):
    """Run the exit branch of the real plan() source on the stand-in."""
    import inspect, textwrap
    src = inspect.getsource(E.plan)
    a, b = src.index("elif j['job'] == 'exit':"), src.index("        else:\n            # Everything reachable is explored")
    body = textwrap.dedent(src[a:b].split('\n', 1)[1])
    r.exit_d = d
    exec(body, dict(explorer.__dict__, self=r, c=cell, q=r.queue))


@pytest.mark.parametrize('side, sign', [('L', -1.0), ('R', 1.0)])   # away from the maze
def test_end_stop_from_the_cell_before(side, sign):
    r = make(maze_side=side)
    r.pose = [4 * 1.02 + 0.05, 0.08, 0.3]              # in the last row, cell (4, 0)
    exit_queue(r, (4, 0), 0)
    assert r.run(120.0)
    x, y, yaw = r.pose
    edge = 5 * 1.02 + 0.51                             # the open far edge of the end cell
    assert x + 0.27 == pytest.approx(edge - 0.10, abs=0.04)        # nose 0.10 m before it
    assert math.degrees(yaw) == pytest.approx(sign * 15.0, abs=2.5)
    for cx, cy in ((0.27, 0.23), (0.27, -0.23), (-0.42, 0.23), (-0.42, -0.23)):   # all inside
        px = x + cx * math.cos(yaw) - cy * math.sin(yaw)
        py = y + cx * math.sin(yaw) + cy * math.cos(yaw)
        assert 5 * 1.02 - 0.51 < px < edge and abs(py) < 0.51
    assert r.sent == [codes.STAND_SIT] and r.halted == ['DONE'] and not r.backwards


def test_end_stop_not_reached_means_no_lying_down():
    r = make(maze_side='L')
    r.pose = [4 * 1.02, 0.0, 0.0]
    import numpy as np
    r.obstacles = np.stack([np.full(20, 5 * 1.02 - 0.3), np.linspace(-0.2, 0.2, 20)], 1)
    exit_queue(r, (4, 0), 0)
    r.halt = lambda state, why: (r.halted.append(state), r.queue.clear())
    r.stuck = lambda mode, along: True
    r.run(120.0)
    assert r.halted == ['STOPPED'] and r.sent == []


def test_way_into_the_row_after_the_last_is_the_exit_and_nothing_else_is():
    class S:
        def __init__(self, k): self.k = k
        def kind(self, cell, d, may_exit=True): return self.k
    r = make()
    r.walked = r.forced_wall = r.closed = r.cv_curtains = r.curtain_latched = r.cv_rejected = set()
    r.start, r.curtains_on, r.stairs_on, r.stair_check_on, r.stairs_checked = (0, 0), True, True, False, {}
    r.exit_cell = r.exit_d = None
    assert E.kind(r, S('free'), (4, 1), 0) == 'exit' and r.exit_cell == (4, 1)
    assert E.kind(r, S('wall'), (4, 0), 0) == 'wall'
    assert E.kind(r, S('free'), (4, 1), 1) == 'free'
    assert E.kind(r, S('exit'), (2, 2), 1) == 'wall'          # the map's exit elsewhere
    assert E.kind(r, S('free'), (5, 1), 1) == 'wall'          # nothing beside the end cell


def kinds_robot():
    r = make()
    r.walked, r.forced_wall, r.closed = set(), set(), set()
    r.cv_curtains = r.curtain_latched = r.cv_rejected = set()
    r.start, r.curtains_on, r.stairs_on, r.stair_check_on, r.stairs_checked = (0, 0), True, True, True, {}
    r.exit_cell = r.exit_d = None
    r.raised = lambda cell, d: False
    return r


class Reads:
    def __init__(self, k): self.k = k
    def kind(self, cell, d, may_exit=True): return self.k


@pytest.mark.parametrize('reading', ['blocked', 'object', 'stairs'])
def test_a_false_reading_on_the_way_into_the_end_cell_is_still_a_possible_exit(reading):
    r = kinds_robot()
    assert E.kind(r, Reads(reading), (4, 1), 0) == 'exit'
    assert (r.exit_cell, r.exit_d) == ((4, 1), 0) and ((4, 1), 0) in r.exit_weak
    assert E.kind(r, Reads(reading), (3, 1), 0) != 'exit'          # only in the last row
    assert E.kind(r, Reads(reading), (4, 1), 1) != 'exit'          # only its far side
    r.wall_share = lambda cell, d: 0.6                             # a wall is mapped there
    assert E.kind(r, Reads(reading), (4, 2), 0) != 'exit'


def test_unknown_after_facing_it_is_a_possible_exit_not_a_wall():
    r = kinds_robot()
    assert E.kind(r, Reads('unknown'), (4, 1), 0) == 'unknown'     # first it is faced
    r.exit_open.add(((4, 1), 0))                                   # do_resolve: still unknown
    assert E.kind(r, Reads('unknown'), (4, 1), 0) == 'exit'
    src = __import__('inspect').getsource(E.do_resolve)
    assert src.index('self.exit_open.add((cell, d))') < src.index('self.forced_wall.add((cell, d))')


def test_an_exit_seen_open_is_not_withdrawn_when_the_map_loses_it():
    r = kinds_robot()
    assert E.kind(r, Reads('free'), (4, 2), 0) == 'exit'
    assert E.kind(r, Reads('unknown'), (4, 2), 0) == 'exit'
    assert r.exit_cell == (4, 2) and not any('withdrawn' in e for e in r.events)
    r.wall_share = lambda cell, d: 0.7                              # a wall really is there now
    assert E.kind(r, Reads('wall'), (4, 2), 0) == 'wall' and r.exit_cell is None


def test_an_open_way_beats_a_possible_one_and_less_wall_beats_more():
    r = kinds_robot()
    r.wall_share = lambda cell, d: {(4, 0): 0.30, (4, 2): 0.10}.get(cell, 0.0)
    E.kind(r, Reads('blocked'), (4, 0), 0)
    assert r.exit_cell == (4, 0)
    E.kind(r, Reads('blocked'), (4, 2), 0)
    assert r.exit_cell == (4, 2)                                   # less of it walled
    E.kind(r, Reads('free'), (4, 1), 0)
    assert r.exit_cell == (4, 1)                                   # plainly open
    E.kind(r, Reads('blocked'), (4, 2), 0)
    assert r.exit_cell == (4, 1)


def test_a_possible_exit_that_stops_the_robot_becomes_a_wall():
    import numpy as np
    r = make(maze_side='L')
    r.exit_weak[((4, 0), 0)] = 0.1
    r.pose = [4 * 1.02, 0.0, 0.0]
    r.obstacles = np.stack([np.full(20, 5 * 1.02 - 0.3), np.linspace(-0.2, 0.2, 20)], 1)
    gave_up = []
    r.stuck = lambda mode, along: gave_up.append(r.step) or r.queue.clear() or True
    exit_queue(r, (4, 0), 0)
    r.run(120.0)
    assert gave_up == [((4, 0), 0, (5, 0))] and r.sent == [] and r.halted == []
    assert 'self.forced_wall.update({(a, d), (b, opp(d))})' in __import__('inspect').getsource(E.stuck)


def gait_steps(r, steps):
    """go() for each (a, d, uneven?) in turn; returns the gait codes sent."""
    r.gait, r.flat_gait, r.gait_switch, r.cmd_gap = 'hstep', 'flat_slow', True, 0.0
    r.cur_gait, r.rough_cells, r.stair_check_on, r.terrain_on, r.terrain_depth = 'flat_slow', set(), False, True, False
    r.walked, r.visited, r.arrive = set(), set(), lambda b: None
    r.side_blocked = lambda sign: False
    for a, d, uneven in steps:
        r.is_rough = lambda a_, d_, u=uneven: u
        r.pose = [*r.grid.centre(a), r.grid.heading(d)]
        r.go(a, d, 'walk')
        assert r.run(120.0)
    return r.sent


def test_gait_follows_the_floor():
    r = make()
    sent = gait_steps(r, [((0, 0), 0, False),      # plain -> plain: stays flat, nothing sent
                          ((1, 0), 0, True),       # plain -> uneven: H-Step
                          ((2, 0), 0, False),      # standing on uneven: no change
                          ((3, 0), 1, False),      # plain -> plain: flat again
                          ((3, 1), 1, False)])     # still flat: nothing sent
    assert sent == [codes.MOVE, codes.HSTEP, codes.AUTO, codes.MOVE, codes.FLAT_SLOW, codes.AUTO]
    assert r.rough_cells == {(2, 0)}


def test_the_optional_curtain_turns_are_180_right_then_50_left():
    import inspect
    src = inspect.getsource(E.go)
    assert 'wrap(body - self.curtain_right + self.curtain_left)' in src
    body, right, left = math.pi / 2, math.pi, math.radians(50)
    n = max(1, math.ceil(right / (math.pi / 2) - 1e-6))
    assert n == 2
    assert math.degrees(wrap(body - right + left)) == pytest.approx(-40.0)


def test_the_start_cell_has_walls_on_every_side_but_the_one_ahead():
    r = kinds_robot()                                  # user, 2026-11-26
    for d in (1, 2, 3):
        assert E.kind(r, Reads('free'), (0, 0), d) == 'wall'
        assert E.kind(r, Reads('curtain'), (0, 0), d) == 'wall'
    assert E.kind(r, Reads('free'), (0, 0), 0) == 'free'
    assert r.curtain_latched == set()                  # nothing latched on those sides


def curtain_robot(cv_on=True):
    r = kinds_robot()
    r.cv_on, r.cv_first, r.cv_seen, r.cv_pending, r.cv_checking = cv_on, True, {}, set(), set()
    r.cv_curtains, r.curtain_latched, r.cv_rejected = set(), set(), set()
    r.camera_pointed = E.camera_pointed.__get__(r)
    return r


def test_a_tall_wall_is_not_a_curtain_until_the_camera_has_pointed_at_it():
    r = curtain_robot()
    assert E.kind(r, Reads('curtain'), (1, 0), 1) == 'wall' and r.curtain_latched == set()
    r.cv_seen[((1, 1), 3)] = [100.0]                   # seen from the other side counts too
    assert E.kind(r, Reads('curtain'), (1, 0), 1) == 'curtain'
    assert ((1, 0), 1) in r.curtain_latched


def test_without_the_camera_the_lidar_still_decides_alone():
    r = curtain_robot(cv_on=False)
    assert E.kind(r, Reads('curtain'), (1, 0), 1) == 'curtain'


def lidar_view(front, top, clouds=50):
    from types import SimpleNamespace as NS
    me = NS(stair_agree=True, stair_lidar_top=0.38, stair_lidar_clouds=10, heights_n=clouds,
            sense=lambda: NS(interior=lambda cell, d: (False, front, top)))
    return E.stairs_not_in_lidar(me, (0, 0), 0)


def test_stairs_from_the_depth_camera_need_the_lidar_to_show_something_that_high():
    nan = float('nan')
    assert lidar_view(0.15, 0.45) == ''                          # real stairs: 0.45 m at the back
    assert 'highest 0.25' in lidar_view(0.12, 0.25)              # terrain / ramps: not believed
    assert 'nothing at all' in lidar_view(nan, nan)              # an empty cell
    assert lidar_view(nan, nan, clouds=3) == ''                  # height map just restarted: no veto


def test_no_exit_found_means_pause_and_lie_down_not_done():
    import inspect, textwrap
    src = inspect.getsource(E.plan)
    a = src.index("            # Everything reachable is explored and no exit is known")
    body = textwrap.dedent(src[a:])
    r = make()
    r.do_event = E.do_event.__get__(r)
    halted = []
    r.halt = lambda state, why: halted.append((state, why))
    exec(body, dict(explorer.__dict__, self=r, q=r.queue, cur=(3, 1)))
    assert r.run(20.0)
    assert r.sent == [codes.STAND_SIT] and r.lying
    assert halted and halted[0][0] == 'STOPPED' and 'no exit found' in halted[0][1]
    assert any('no exit found' in e for e in r.events)
    assert "if getattr(self, 'lying', False):" in inspect.getsource(E.begin)    # stands up on /nav/start


def test_re_anchor_after_a_jump_is_off_unless_asked_for():
    assert "p('jump_fix', False)" in __import__('inspect').getsource(E.__init__)
