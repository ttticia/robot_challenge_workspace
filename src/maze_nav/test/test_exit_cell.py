"""The robot stays in the 5 x 3 maze grid (1.02 m cells) and parks in the launch cell.

python3 -m pytest -q test/test_exit_cell.py"""
import json
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

import maze_nav.explorer as explorer_mod
from maze_nav.core import ExitDetector, Frontier
from maze_nav.explorer import Explorer
from maze_nav.scene_cells import CellGrid, boundary_sides, exit_cell, fit_grid
from test_scene_cells import _sim_maze_map

CELLS = CellGrid(1.02)
COLS, ROWS = 5, 3


# ------------------------------------------------------------------ grid geometry

@pytest.mark.parametrize('xy', [(0.0, 0.0), (4.08, 2.04), (2.04, 1.02), (4.08, 0.0),
                                (-0.5, 0.0), (4.08 + 0.6, 1.02)])
def test_inside_or_in_the_opening_is_not_an_exit(xy):
    """Cell centres (0, 0)..(4.08, 2.04), and points within the margin past the edge."""
    assert exit_cell(CELLS, xy, COLS, ROWS, margin=0.15) is None


@pytest.mark.parametrize('xy,cell', [((5.10, 1.02), (5, 1)),     # past the far end
                                     ((2.04, 3.06), (2, 3)),     # past the left side
                                     ((1.02, -1.02), (1, -1)),   # past the right side
                                     ((-1.02, 2.04), (-1, 2)),   # behind the start row
                                     ((4.08 + 0.51 + 0.2, 0.0), (5, 0))])
def test_outside_the_grid_is_an_exit_cell(xy, cell):
    assert exit_cell(CELLS, xy, COLS, ROWS, margin=0.15) == cell


def test_grid_follows_the_start_heading():
    """The grid's +i is the robot's heading at bringup, whatever the map frame says."""
    turned = CellGrid(1.02, (1.0, 1.0), math.pi / 2)
    assert exit_cell(turned, (1.0, 1.0 + 4.08), COLS, ROWS) is None
    assert exit_cell(turned, (1.0, 1.0 + 5.10), COLS, ROWS) == (5, 0)


def test_unknown_size_never_marks_an_exit():
    assert exit_cell(CELLS, (50.0, 50.0), 0, 3) is None
    assert exit_cell(CELLS, (50.0, 50.0), 5, 0) is None


def test_boundary_sides_cover_the_outline_once():
    sides = boundary_sides(COLS, ROWS)
    assert len(sides) == 2 * (COLS + ROWS) == len(set(sides))
    for (i, j), (di, dj) in sides:
        assert not (0 <= i + di < COLS and 0 <= j + dj < ROWS)


# ------------------------------------------------------------------ explorer behaviour

class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(explorer_mod.time, 'monotonic', c)
    return c


class Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class Handle:
    def __init__(self):
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


class Fake:
    for _n in ('grid_known', 'in_grid', 'update_exit', 'publish_launch', 'go_to_exit',
               'launch_tick', 'park', 'out_of_maze', 'send_launch_goal', 'face_exit',
               'seen_beyond', 'side_open', 'choose_exit', 'creep_to_centre',
               'launch_creep_tick', 'fit_cells', 'extents', 'maze_sides', 'decide_side'):
        locals()[_n] = getattr(Explorer, _n)

    def __init__(self, go=True, sign=1):
        self.maze_sign = sign                        # 1 left, -1 right, None auto
        self.cfg = {'maze_cols': COLS, 'maze_rows': ROWS, 'exit_cell_margin': 0.15,
                    'exit_confirm': 3, 'launch_tolerance': 0.25, 'launch_retries': 3,
                    'goal_max_time': 120.0, 'go': go,
                    'launch_yaw_tolerance': math.radians(10), 'turn_margin': 0.03,
                    'creep_margin': 0.05, 'creep_speed': 0.05,
                    'grid_fit': True, 'grid_fit_period': 5.0,
                    'grid_fit_max_yaw': math.radians(8), 'grid_fit_max_shift': 0.35,
                    'grid_fit_min_points': 150, 'grid_fit_min_score': 0.6}
        self.spin_busy, self.spins = False, []
        self.cmd_pub, self.pose = Pub(), None
        self.box, self.pivot = (0.37, 0.27, 0.23), (0.025, -0.04)   # nav2_params footprint
        self.cells = CellGrid(1.02)
        self.exit_side, self.exit_open, self.launch = None, {}, None
        self.launch_pub, self.goal_pub, self.grid_pub = Pub(), Pub(), Pub()
        self.cells0, self.grid_fit_at = CellGrid(1.02), -1e9
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.state, self.finished, self.sent, self.stops = 'EXPLORING', [], 0, 0
        self.g, self.logs = None, []
        self.log = SimpleNamespace(info=self.logs.append, warn=self.logs.append)
        self.nav = SimpleNamespace(send_goal_async=self.nav_goal)

    def get_logger(self):
        return self.log

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: PoseStamped().header.stamp))

    def nav_goal(self, goal):
        self.sent += 1
        return SimpleNamespace(add_done_callback=lambda cb: None)

    def robot_pose(self):
        return self.pose

    def send_spin(self, angle, why=''):
        self.spins.append(angle)
        self.spin_busy = True

    def set_state(self, state, why=''):
        self.state = state

    def finish(self, state, why):
        self.finished.append((state, why))
        self.state = state

    def stop_robot(self):
        self.stops += 1

    def grid_now(self):
        return self.g


EXITS = [('challenge_a.json', 1, (4, 2), (1, 0)),
         ('challenge_a_mirror.json', -1, (4, -2), (1, 0)),
         ('challenge_b.json', 1, (2, 2), (0, 1))]


@pytest.mark.parametrize('maze,sign,cell,d', EXITS)
def test_exit_chosen_at_the_end_after_exit_confirm_replans(maze, sign, cell, d):
    f = Fake(sign=sign)
    g = _sim_maze_map(maze)
    for _ in range(2):
        f.update_exit(g)
    assert f.exit_side is None                       # never chosen while exploring
    assert f.choose_exit(g) is None                  # 2 replans open < exit_confirm 3
    f.update_exit(g)
    assert f.choose_exit(g) == (cell, d) == f.exit_side
    msg = json.loads(f.launch_pub.msgs[-1].data)
    assert msg['state'] == 'found' and msg['launch_cell'] == list(cell)
    assert msg['exit_cell'] == [cell[0] + d[0], cell[1] + d[1]]


def seen_like_slam(g, name, exit_box):
    """Like a real map: nothing seen outside the maze's outline except through the exit
    (exit_box = x0, x1, y0, y1 in the maze frame). _sim_maze_map draws everything known."""
    d = json.load(open(os.path.join(os.path.dirname(__file__), name)))
    sx, sy, sth = d['start']
    res = g.resolution
    h, w = g.data.shape
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    ox, oy = g.origin[0] + (cols + 0.5) * res, g.origin[1] + (rows + 0.5) * res
    c, s = math.cos(sth), math.sin(sth)
    mx, my = sx + c * ox - s * oy, sy + s * ox + c * oy
    out = (mx < -0.54) | (mx > 4.62) | (my < -0.54) | (my > 2.58)
    x0, x1, y0, y1 = exit_box
    through = (mx >= x0) & (mx <= x1) & (my >= y0) & (my <= y1)
    g.data[out & ~through & (g.data == 0)] = -1
    return g


def paint(g, x0, x1, y0, y1, value):
    xs = g.origin[0] + (np.arange(g.data.shape[1]) + 0.5) * g.resolution
    ys = g.origin[1] + (np.arange(g.data.shape[0]) + 0.5) * g.resolution
    g.data[np.ix_((ys >= y0) & (ys <= y1), (xs >= x0) & (xs <= x1))] = value


def test_tilted_start_grid_fitted_then_the_real_exit_found(clock):
    """challenge_b_offset (placed 5 cm / 5 deg off): fit_cells lines the grid up with the
    walls first, then the real exit (+y of (2, 2)) is found, not a false one."""
    f = Fake()
    g = seen_like_slam(_sim_maze_map('challenge_b_offset.json'), 'challenge_b_offset.json',
                       (1.53, 2.55, 2.55, 5.0))
    f.fit_cells(g)
    assert math.degrees(f.cells.yaw) == pytest.approx(-5.0, abs=0.6)
    for _ in range(3):
        f.update_exit(g)
    assert f.choose_exit(g) == ((2, 2), (0, 1))


def test_walled_outline_has_no_exit():
    """challenge_a's exit walled over on the map: every outer side reads wall, no exit."""
    f = Fake()
    g = _sim_maze_map('challenge_a.json')
    paint(g, 4.55, 4.63, 1.51, 2.57, 100)
    for _ in range(5):
        f.update_exit(g)
    assert f.choose_exit(g) is None
    assert not f.go_to_exit('maze explored')


def test_start_cell_is_never_the_exit():
    """Sim 2026-11-22, every 5 x 3 run: the start cell's walls are too close to be mapped
    and beams passing just over them free the floor beyond, so its sides read open, even
    on the final map. The start area is never the final cell (booklet): not counted."""
    f = Fake()
    g = _sim_maze_map('challenge_a.json')
    paint(g, -0.56, 0.56, -0.56, -0.46, 0)           # start cell's -y wall never mapped
    for _ in range(20):
        f.update_exit(g)
    assert ((0, 0), (0, -1)) not in f.exit_open
    assert f.choose_exit(g) == ((4, 2), (1, 0))


def test_several_open_sides_take_the_most_often_open():
    f = Fake()
    g = _sim_maze_map('challenge_a.json')
    paint(g, 1.53, 2.55, -0.56, -0.46, 0)            # a second gap: -y of cell (2, 0)
    for _ in range(3):
        f.update_exit(g)
    for _ in range(4):                               # the real exit open for longer
        f.exit_open[((4, 2), (1, 0))] += 1
    assert f.choose_exit(g) == ((4, 2), (1, 0))
    assert any('several outer sides open' in m for m in f.logs)


def test_side_unknown_covers_both_arenas_until_a_cell_off_the_start_row():
    """maze_side auto (the arenas are mirrored): both grids count until the robot stands
    well inside a cell off the start row; that cell's side is the maze."""
    f = Fake(sign=None)
    assert f.in_grid((2.04, 2.04)) and f.in_grid((2.04, -2.04))
    f.decide_side((1.02, 0.0))                        # still on the start row
    f.decide_side((1.02, -0.65))                      # cell (1, -1) but at its edge
    assert f.maze_sign is None
    f.decide_side((1.02, -1.0))                       # well inside (1, -1)
    assert f.maze_sign == -1
    assert f.in_grid((2.04, -2.04)) and not f.in_grid((2.04, 1.5))
    assert f.extents() == [-2]


def test_mirrored_arena_exit_with_the_side_decided_on_the_way():
    f = Fake(sign=None)
    g = _sim_maze_map('challenge_a_mirror.json')
    for _ in range(3):
        f.update_exit(g)                              # both grids' sides counted
    f.decide_side((1.02, -1.02))
    assert f.choose_exit(g) == ((4, -2), (1, 0))
    assert json.loads(f.launch_pub.msgs[-1].data)['maze_side'] == 'right'


def test_frontiers_outside_the_grid_are_no_goals():
    f = Fake()
    assert f.in_grid((4.08 + 0.4, 1.02))             # in the last cell
    assert not f.in_grid((4.08 + 0.6, 1.02))         # past the outer side
    f.cfg.update(maze_cols=0)
    assert f.in_grid((40.0, 0.0))                    # size unknown: no filter


def found(f, cell=(4, 1), d=(1, 0)):
    f.exit_side = (cell, d)
    return f


def test_maze_explored_sends_the_robot_to_the_launch_cell(clock):
    f = found(Fake())
    old = Handle()
    f.goal_handle, f.current = old, Frontier((1.0, 1.0), 0.0, 1.0, 5, (1.0, 1.0))
    assert f.go_to_exit('maze explored')
    assert old.cancelled and f.current is None
    assert f.state == 'TO_EXIT' and f.sent == 1
    goal = f.goal_pub.msgs[-1].pose
    assert (goal.position.x, goal.position.y) == pytest.approx((4.08, 1.02))
    assert 2 * math.atan2(goal.orientation.z, goal.orientation.w) == pytest.approx(0.0)


def test_no_exit_found_means_no_trip():
    f = Fake()
    assert not f.go_to_exit('maze explored')
    assert f.state == 'EXPLORING'


def test_parks_when_reached_within_tolerance(clock):
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['result'] = GoalStatus.STATUS_SUCCEEDED
    f.launch_tick(((4.12, 1.00), 0.05))
    state, why = f.finished[-1]
    assert state == 'AT_EXIT' and 'launch cell [4, 1]' in why and 'exit cell [5, 1]' in why
    assert f.stops >= 1                              # stands still; nothing else is sent
    msg = json.loads(f.launch_pub.msgs[-1].data)
    assert msg['state'] == 'at' and msg['robot'] == [4.12, 1.0]


def test_turns_to_face_the_exit_before_parking(clock):
    """Nav2 ignores the goal heading (sim 2026-11-22: parked 85 deg off): turn, then park."""
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['result'] = GoalStatus.STATUS_SUCCEEDED
    f.launch_tick(((4.08, 1.02), math.radians(85)))
    assert f.spins == [pytest.approx(math.radians(-85))] and f.finished == []
    f.launch_tick(((4.08, 1.02), math.radians(85)))  # still turning: wait
    assert len(f.spins) == 1
    f.spin_busy = False
    f.launch_tick(((4.08, 1.02), math.radians(3)))
    assert f.finished[-1][0] == 'AT_EXIT' and '3 deg off' in f.finished[-1][1]


def test_creeps_to_the_centre_then_turns_then_parks(clock):
    """Sim 2026-11-22: Nav2 stopped 0.2 m short (heading -y into cell (4, 1)) and the turn
    was blocked both ways. Creep ahead to the centre first, then turn +90 deg."""
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['result'] = GoalStatus.STATUS_SUCCEEDED
    down = -math.pi / 2
    f.launch_tick(((4.08, 1.22), down))
    assert f.launch['creep']['target'] == pytest.approx(0.20) and f.spins == []
    f.pose = ((4.08, 1.12), down)
    f.launch_creep_tick()
    assert f.cmd_pub.msgs[-1].linear.x == pytest.approx(0.05)       # still creeping
    f.launch_tick(((4.08, 1.12), down))                              # waits for the creep
    assert f.spins == []
    f.pose = ((4.08, 1.02), down)
    f.launch_creep_tick()
    assert f.launch['creep'] is None and f.stops >= 1
    f.launch_tick(((4.08, 1.02), down))
    assert f.spins == [pytest.approx(math.pi / 2)]
    f.spin_busy = False
    f.launch_tick(((4.08, 1.02), 0.02))
    assert f.finished[-1][0] == 'AT_EXIT' and '1 deg off' in f.finished[-1][1]


def test_past_the_centre_never_creeps_back(clock):
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['result'] = GoalStatus.STATUS_SUCCEEDED
    f.launch_tick(((4.08, 0.90), -math.pi / 2))                      # 12 cm past it
    assert f.launch['creep'] is None and len(f.spins) == 1


def test_creep_stops_short_of_a_wall():
    """With a map: no further than the footprint stays creep_margin clear."""
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.g = _sim_maze_map('challenge_a.json')
    # cell (4, 0) facing +x, the "centre" pretended 0.54 m ahead: the outer wall's face at
    # x 4.565 leaves 4.565 - 0.05 - (4.08 + 0.27) = 0.165 m; the map draws the wall up to
    # a 5 cm cell thicker, so 0.11-0.165
    f.launch['xy'] = (4.62, 0.0)
    assert f.creep_to_centre((4.08, 0.0), 0.0)
    assert 0.10 <= f.launch['creep']['target'] <= 0.17
    f.launch['creep'] = None
    f.launch['xy'] = (4.08, 0.0)                                     # through the passage
    assert f.creep_to_centre((3.88, 0.0), 0.0)
    assert f.launch['creep']['target'] == pytest.approx(0.20, abs=0.01)


def test_gives_up_turning_after_two_tries(clock):
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['result'] = GoalStatus.STATUS_SUCCEEDED
    for _ in range(3):
        f.spin_busy = False
        f.launch_tick(((4.08, 1.02), math.radians(40)))   # the turns don't take
    assert len(f.spins) == 2
    assert f.finished[-1][0] == 'AT_EXIT' and '40 deg off' in f.finished[-1][1]


def test_retries_then_parks_inside_the_cell(clock):
    f = found(Fake())
    f.go_to_exit('maze explored')
    for _ in range(2):                               # 2 more goals: 3 in all
        f.launch['result'] = GoalStatus.STATUS_ABORTED
        f.launch_tick(((3.82, 1.30), 0.0))
        assert f.state == 'TO_EXIT'
    assert f.launch['tries'] == 3 and f.sent == 3
    f.launch['result'] = GoalStatus.STATUS_ABORTED
    f.launch_tick(((3.82, 1.30), 0.0))               # still in cell (4, 1): park anyway
    assert f.finished[-1][0] == 'AT_EXIT' and 'off centre after 3 goals' in f.finished[-1][1]


def test_reached_but_too_far_from_the_centre_tries_again(clock):
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['result'] = GoalStatus.STATUS_SUCCEEDED
    f.launch_tick(((3.72, 1.02), 0.0))               # 0.36 m short
    assert f.state == 'TO_EXIT' and f.launch['tries'] == 2


def test_gives_up_outside_the_launch_cell(clock):
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['tries'] = 3
    f.launch['result'] = GoalStatus.STATUS_ABORTED
    f.launch_tick(((1.0, 1.0), 0.0))
    assert f.finished[-1][0] == 'STOPPED'


def test_stuck_goal_times_out(clock):
    f = found(Fake())
    f.go_to_exit('maze explored')
    f.launch['handle'] = h = Handle()
    clock.t += 121.0
    f.launch_tick(((1.0, 1.0), 0.0))
    assert h.cancelled and f.launch['tries'] == 2    # sent again


def test_go_false_parks_without_driving(clock):
    f = found(Fake(go=False))
    f.go_to_exit('maze explored')
    f.launch_tick(((0.0, 0.0), 0.0))
    assert f.sent == 0 and f.finished[-1][0] == 'AT_EXIT'


def test_open_floor_next_to_the_exit_does_not_end_the_run(clock):
    """In the launch cell the open floor outside is within reach; only a robot outside the
    grid ends EXITED (fallback)."""
    f = found(Fake())
    f.exit = ExitDetector()
    f.exit.update = lambda grid, xy, now: True       # the detector says "open floor"
    assert not f.out_of_maze(((4.08, 1.02), 0.0))
    assert f.finished == []
    assert f.out_of_maze(((4.08 + 0.51 + 0.3, 1.02), 0.0))
    assert f.finished[-1][0] == 'EXITED'


# ------------------------------------------------------------------ grid from the walls

def wall_xy(g):
    r, c = np.nonzero(g.data >= 65)
    return np.c_[g.origin[0] + (c + 0.5) * g.resolution, g.origin[1] + (r + 0.5) * g.resolution]


def offset_truth():
    """challenge_b_offset: the real cell (0, 0) centre and +i heading in the start frame."""
    sx, sy, sth = 0.05, -0.04, math.radians(5.0)
    c, s = math.cos(-sth), math.sin(-sth)
    return (c * -sx - s * -sy, s * -sx + c * -sy), -sth


def near_maze(xy, cells=CellGrid(1.02), margin=0.4, upto=None):
    u, v = np.array([cells.local(p) for p in xy]).T
    m = margin / cells.size
    keep = (u >= -0.5 - m) & (u <= 4.5 + m) & (v >= -0.5 - m) & (v <= 2.5 + m)
    if upto is not None:
        keep &= u <= upto
    return xy[keep]


@pytest.mark.parametrize('upto', [None, 1.5])
def test_fit_grid_finds_the_tilted_cells(upto):
    """Placed 5 deg / 6 cm off at the start: found to the map's precision (~0.5 deg, a few
    cm), from the whole arena or only the first 1.5 cells (what the robot sees early on)."""
    (ox, oy), yaw = offset_truth()
    pts = near_maze(wall_xy(_sim_maze_map('challenge_b_offset.json')), upto=upto)
    fit, score = fit_grid(pts, CellGrid(1.02))
    assert math.degrees(fit.yaw) == pytest.approx(math.degrees(yaw), abs=0.6)
    assert fit.origin == pytest.approx((ox, oy), abs=0.03)
    assert score > 0.8


def test_fit_grid_leaves_an_aligned_grid_alone():
    pts = near_maze(wall_xy(_sim_maze_map('challenge_a.json')))
    fit, _ = fit_grid(pts, CellGrid(1.02))
    assert fit.yaw == pytest.approx(0.0, abs=math.radians(0.6))
    assert fit.origin == pytest.approx((0.0, 0.0), abs=0.03)


def test_fit_grid_without_points_is_none():
    assert fit_grid(np.zeros((0, 2)), CellGrid(1.02)) is None


def test_explorer_refits_its_grid_in_place_and_publishes_it(clock):
    f = Fake()
    held = f.cells                                   # scene_trigger-style holders follow
    g = _sim_maze_map('challenge_b_offset.json')
    f.fit_cells(g)
    (ox, oy), yaw = offset_truth()
    assert held is f.cells and held.yaw == pytest.approx(yaw, abs=math.radians(0.6))
    assert held.origin == pytest.approx((ox, oy), abs=0.03)
    msg = json.loads(f.grid_pub.msgs[-1].data)
    assert msg['size'] == 1.02 and msg['yaw'] == pytest.approx(yaw, abs=math.radians(0.6))
    n = len(f.grid_pub.msgs)
    clock.t += 1.0
    f.fit_cells(g)                                   # within grid_fit_period: nothing
    clock.t += 5.0
    f.fit_cells(g)                                   # same fit: nothing to publish
    assert len(f.grid_pub.msgs) == n


def test_explorer_ignores_a_fit_that_jumps_too_far(clock):
    """More than grid_fit_max_shift from the start-pose grid: kept as it was."""
    f = Fake()
    f.cfg['grid_fit_max_shift'] = 0.03                # the 6 cm offset is now "too far"
    f.fit_cells(_sim_maze_map('challenge_b_offset.json'))
    assert f.cells.origin == (0.0, 0.0) and f.cells.yaw == 0.0 and not f.grid_pub.msgs


def test_explorer_needs_enough_walls(clock):
    f = Fake()
    g = _sim_maze_map('challenge_b_offset.json')
    g.data[g.data >= 65] = 0                         # no walls mapped yet
    f.fit_cells(g)
    assert not f.grid_pub.msgs
