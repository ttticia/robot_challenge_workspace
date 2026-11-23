"""The maze grid (5 x 3 cells of 1.02 m), the exit and the launch cell the robot parks in.

python3 -m pytest -q test/test_exit_cell.py"""
import json
import math
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

COLS, ROWS = 5, 3
SUCCEEDED, ABORTED = GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED


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
    cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


class Fake:
    """The explorer's exit / launch / grid-fit methods on a stand-in node."""
    for _n in ('grid_known', 'in_grid', 'extents', 'maze_sides', 'decide_side',
               'update_exit', 'side_open', 'seen_beyond', 'choose_exit', 'publish_launch',
               'go_to_exit', 'send_launch_goal', 'launch_tick', 'creep_to_centre',
               'launch_creep_tick', 'face_exit', 'park', 'out_of_maze', 'fit_cells'):
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
        self.cells, self.cells0, self.grid_fit_at = CellGrid(1.02), CellGrid(1.02), -1e9
        self.exit_side, self.exit_open, self.launch = None, {}, None
        self.launch_pub, self.goal_pub, self.grid_pub, self.cmd_pub = Pub(), Pub(), Pub(), Pub()
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.box, self.pivot = (0.37, 0.27, 0.23), (0.025, -0.04)   # nav2_params footprint
        self.spin_busy, self.spins, self.pose, self.g = False, [], None, None
        self.state, self.finished, self.sent, self.stops, self.logs = 'EXPLORING', [], 0, 0, []
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


def paint(g, x0, x1, y0, y1, value):
    xs = g.origin[0] + (np.arange(g.data.shape[1]) + 0.5) * g.resolution
    ys = g.origin[1] + (np.arange(g.data.shape[0]) + 0.5) * g.resolution
    g.data[np.ix_((ys >= y0) & (ys <= y1), (xs >= x0) & (xs <= x1))] = value


def wall_xy(g):
    r, c = np.nonzero(g.data >= 65)
    return np.c_[g.origin[0] + (c + 0.5) * g.resolution, g.origin[1] + (r + 0.5) * g.resolution]


def offset_truth():
    """challenge_b_offset: the real cell (0, 0) centre and +i heading in the start frame."""
    sx, sy, sth = 0.05, -0.04, math.radians(5.0)
    c, s = math.cos(-sth), math.sin(-sth)
    return (c * -sx - s * -sy, s * -sx + c * -sy), -sth


# ------------------------------------------------------------------ grid geometry

def test_grid_geometry():
    cells = CellGrid(1.02)
    inside = [(0.0, 0.0), (4.08, 2.04), (-0.5, 0.0), (4.08 + 0.6, 1.02)]   # last: in the margin
    assert all(exit_cell(cells, xy, COLS, ROWS) is None for xy in inside)
    outside = {(5.10, 1.02): (5, 1), (2.04, 3.06): (2, 3), (1.02, -1.02): (1, -1),
               (-1.02, 2.04): (-1, 2)}
    assert all(exit_cell(cells, xy, COLS, ROWS) == c for xy, c in outside.items())
    turned = CellGrid(1.02, (1.0, 1.0), math.pi / 2)     # +i = the start heading, any frame
    assert exit_cell(turned, (1.0, 5.08), COLS, ROWS) is None
    assert exit_cell(turned, (1.0, 6.10), COLS, ROWS) == (5, 0)
    assert exit_cell(cells, (1.02, -2.04), COLS, ROWS, j0=-2) is None   # mirrored arena
    assert exit_cell(cells, (50.0, 50.0), 0, 3) is None                 # size unknown
    sides = boundary_sides(COLS, ROWS)
    assert len(sides) == 16 == len(set(sides))
    assert all(not (0 <= i + di < COLS and 0 <= j + dj < ROWS) for (i, j), (di, dj) in sides)


def test_maze_side_auto_covers_both_arenas_until_a_cell_off_the_start_row():
    """The two arenas are mirrored: until the robot stands well inside a cell off the start
    row, both grids count (goals, exit candidates)."""
    f = Fake(sign=None)
    assert f.in_grid((2.04, 2.04)) and f.in_grid((2.04, -2.04))
    f.decide_side((1.02, 0.0))                           # still on the start row
    f.decide_side((1.02, -0.65))                         # in (1, -1) but at its edge
    assert f.maze_sign is None
    f.decide_side((1.02, -1.0))
    assert f.maze_sign == -1 and f.extents() == [-2]
    assert f.in_grid((2.04, -2.04)) and not f.in_grid((2.04, 1.5))
    f.cfg['maze_cols'] = 0
    assert f.in_grid((40.0, 0.0))                        # size unknown: no filter


# ------------------------------------------------------------------ choosing the exit

@pytest.mark.parametrize('maze,sign,cell,d', [('challenge_a.json', 1, (4, 2), (1, 0)),
                                              ('challenge_a_mirror.json', None, (4, -2), (1, 0)),
                                              ('challenge_b.json', 1, (2, 2), (0, 1))])
def test_exit_chosen_at_the_end_after_exit_confirm_replans(maze, sign, cell, d):
    f = Fake(sign=sign)
    g = _sim_maze_map(maze)
    for _ in range(2):
        f.update_exit(g)
    assert f.exit_side is None and f.choose_exit(g) is None   # 2 replans < exit_confirm 3
    f.update_exit(g)
    if sign is None:
        f.decide_side((1.02, -1.02))                     # mirrored: found on the way
    assert f.choose_exit(g) == (cell, d) == f.exit_side
    msg = json.loads(f.launch_pub.msgs[-1].data)
    assert msg['state'] == 'found' and msg['launch_cell'] == list(cell)
    assert msg['exit_cell'] == [cell[0] + d[0], cell[1] + d[1]]
    assert msg['maze_side'] == ('right' if sign is None else 'left')


def test_false_exits_are_rejected():
    g = _sim_maze_map('challenge_a.json')
    # The start cell's walls are too close to map and beams passing over them free the
    # floor beyond (sim 2026-11-22: every run took its side): never the exit.
    f = Fake()
    paint(g, -0.56, 0.56, -0.56, -0.46, 0)
    for _ in range(20):
        f.update_exit(g)
    assert ((0, 0), (0, -1)) not in f.exit_open and f.choose_exit(g) == ((4, 2), (1, 0))
    # Two open sides: the most often open.
    f = Fake()
    paint(g, 1.53, 2.55, -0.56, -0.46, 0)                # a gap: -y of cell (2, 0)
    for _ in range(3):
        f.update_exit(g)
    f.exit_open[((4, 2), (1, 0))] += 4
    assert f.choose_exit(g) == ((4, 2), (1, 0))
    assert any('several outer sides open' in m for m in f.logs)
    # Every outer side walled: no exit, no trip.
    f = Fake()
    g = _sim_maze_map('challenge_a.json')
    paint(g, 4.55, 4.63, 1.51, 2.57, 100)
    for _ in range(5):
        f.update_exit(g)
    assert f.choose_exit(g) is None and not f.go_to_exit('maze explored')


def test_hand_placed_start_grid_fitted_then_the_real_exit_found(clock):
    """challenge_b_offset (5 cm / 5 deg off): fit_cells lines the grid up with the walls,
    then the exit is found where it really is. As on a real map, nothing outside the
    outline is seen except through the exit."""
    f = Fake()
    g = _sim_maze_map('challenge_b_offset.json')
    real = CellGrid(1.02, *offset_truth())
    rows, cols = np.nonzero(g.data == 0)
    for r, c in zip(rows, cols):
        u, v = real.local(g.world((c, r)))
        if (u < -0.53 or u > 4.53 or v < -0.53 or v > 2.53) and not (1.5 <= u <= 2.5 and v > 2.5):
            g.data[r, c] = -1
    f.fit_cells(g)
    assert math.degrees(f.cells.yaw) == pytest.approx(-5.0, abs=0.6)
    for _ in range(3):
        f.update_exit(g)
    assert f.choose_exit(g) == ((2, 2), (0, 1))


# ------------------------------------------------------------------ parking

def explored(f, cell=(4, 1), d=(1, 0)):
    f.exit_side = (cell, d)
    assert f.go_to_exit('maze explored')
    return f


def test_goes_to_the_launch_cell_creeps_to_its_centre_turns_and_parks(clock):
    """Sim 2026-11-22: Nav2 stopped 0.2 m short (its xy tolerance, heading -y into (4, 1))
    and a turn there was blocked by the walls; Nav2 also ignores the goal heading."""
    f = Fake()
    old = f.goal_handle = Handle()
    f.current = Frontier((1.0, 1.0), 0.0, 1.0, 5, (1.0, 1.0))
    explored(f)
    assert old.cancelled and f.current is None and f.state == 'TO_EXIT' and f.sent == 1
    goal = f.goal_pub.msgs[-1].pose
    assert (goal.position.x, goal.position.y) == pytest.approx((4.08, 1.02))
    f.launch['result'] = SUCCEEDED
    down = -math.pi / 2
    f.launch_tick(((4.08, 1.22), down))
    assert f.launch['creep']['target'] == pytest.approx(0.20) and not f.spins
    f.pose = ((4.08, 1.12), down)
    f.launch_creep_tick()
    assert f.cmd_pub.msgs[-1].linear.x == pytest.approx(0.05)
    f.launch_tick(((4.08, 1.12), down))                  # waits for the creep
    f.pose = ((4.08, 1.02), down)
    f.launch_creep_tick()
    assert f.launch['creep'] is None
    f.launch_tick(((4.08, 1.02), down))
    assert f.spins == [pytest.approx(math.pi / 2)]
    f.spin_busy = False
    f.launch_tick(((4.08, 1.02), 0.02))
    state, why = f.finished[-1]
    assert state == 'AT_EXIT' and 'launch cell [4, 1]' in why and '1 deg off' in why
    msg = json.loads(f.launch_pub.msgs[-1].data)
    assert msg['state'] == 'at' and msg['robot'] == [4.08, 1.02]


def test_parking_limits(clock):
    # past the centre: never creeps back; a turn that doesn't take: 2 tries, then parks
    f = explored(Fake())
    f.launch['result'] = SUCCEEDED
    for _ in range(3):
        f.spin_busy = False
        f.launch_tick(((4.08, 0.90), math.radians(-50)))
    assert f.launch['creep'] is None and len(f.spins) == 2
    assert f.finished[-1][0] == 'AT_EXIT' and '50 deg off' in f.finished[-1][1]
    # a creep never runs into a wall: (4, 0) facing the outer wall, 0.165 m of room
    f = explored(Fake())
    f.g = _sim_maze_map('challenge_a.json')
    f.launch['xy'] = (4.62, 0.0)
    assert f.creep_to_centre((4.08, 0.0), 0.0)
    assert 0.10 <= f.launch['creep']['target'] <= 0.17   # (map walls up to 5 cm thicker)
    # go:=false: parks where it is, nothing driven
    f = explored(Fake(go=False))
    f.launch_tick(((0.0, 0.0), 0.0))
    assert f.sent == 0 and f.finished[-1][0] == 'AT_EXIT'


def test_failed_launch_goals(clock):
    f = explored(Fake())
    f.launch['result'] = SUCCEEDED
    f.launch_tick(((3.72, 1.02), 0.0))                   # "reached", 0.36 m short: again
    assert f.state == 'TO_EXIT' and f.launch['tries'] == 2
    f.launch['handle'] = h = Handle()
    clock.t += 121.0
    f.launch_tick(((3.82, 1.30), 0.0))                   # stuck goal: cancelled, again
    assert h.cancelled and f.launch['tries'] == 3
    f.launch['result'] = ABORTED
    f.launch_tick(((3.82, 1.30), 0.0))                   # out of tries, inside (4, 1): parks
    assert f.finished[-1][0] == 'AT_EXIT' and 'off centre after 3 goals' in f.finished[-1][1]
    f = explored(Fake())
    f.launch['tries'], f.launch['result'] = 3, ABORTED
    f.launch_tick(((1.0, 1.0), 0.0))                     # out of tries, elsewhere
    assert f.finished[-1][0] == 'STOPPED'


def test_open_floor_next_to_the_exit_does_not_end_the_run(clock):
    """In the launch cell the open floor outside is within reach; only a robot outside the
    grid ends EXITED (fallback)."""
    f = Fake()
    f.exit = ExitDetector()
    f.exit.update = lambda grid, xy, now: True           # "open floor"
    assert not f.out_of_maze(((4.08, 1.02), 0.0)) and not f.finished
    assert f.out_of_maze(((4.08 + 0.51 + 0.3, 1.02), 0.0))
    assert f.finished[-1][0] == 'EXITED'


# ------------------------------------------------------------------ grid from the walls

def test_fit_grid():
    """To the map's precision (5 cm cells: ~0.5 deg, a few cm), from the whole arena or
    only the first 1.5 cells; an aligned grid stays; no walls, no fit."""
    (ox, oy), yaw = offset_truth()
    pts = wall_xy(_sim_maze_map('challenge_b_offset.json'))
    near = pts[(pts[:, 0] > -0.9) & (pts[:, 0] < 5.0) & (pts[:, 1] > -0.9) & (pts[:, 1] < 3.0)]
    for p in (near, near[near[:, 0] < 1.5 * 1.02]):
        fit, score = fit_grid(p, CellGrid(1.02))
        assert math.degrees(fit.yaw) == pytest.approx(math.degrees(yaw), abs=0.6)
        assert fit.origin == pytest.approx((ox, oy), abs=0.03) and score > 0.8
    pts = wall_xy(_sim_maze_map('challenge_a.json'))
    fit, _ = fit_grid(pts[(pts[:, 0] > -0.9) & (pts[:, 0] < 5.0) & (pts[:, 1] > -0.9)
                          & (pts[:, 1] < 3.0)], CellGrid(1.02))
    assert fit.yaw == pytest.approx(0.0, abs=math.radians(0.6))
    assert fit.origin == pytest.approx((0.0, 0.0), abs=0.03)
    assert fit_grid(np.zeros((0, 2)), CellGrid(1.02)) is None


def test_explorer_refits_its_grid(clock):
    f = Fake()
    held = f.cells                                       # scene_trigger-style holders follow
    g = _sim_maze_map('challenge_b_offset.json')
    f.fit_cells(g)
    (ox, oy), yaw = offset_truth()
    assert held is f.cells and held.yaw == pytest.approx(yaw, abs=math.radians(0.6))
    assert held.origin == pytest.approx((ox, oy), abs=0.03)
    assert json.loads(f.grid_pub.msgs[-1].data)['size'] == 1.02
    n = len(f.grid_pub.msgs)
    clock.t += 1.0
    f.fit_cells(g)                                       # within grid_fit_period
    clock.t += 5.0
    f.fit_cells(g)                                       # same fit: nothing new
    assert len(f.grid_pub.msgs) == n
    f = Fake()
    f.cfg['grid_fit_max_shift'] = 0.03                   # the fit would move it too far
    f.fit_cells(g)
    assert f.cells.yaw == 0.0 and not f.grid_pub.msgs
    f = Fake()
    g.data[g.data >= 65] = 0                             # no walls mapped yet
    f.fit_cells(g)
    assert not f.grid_pub.msgs


def test_scene_trigger_follows_the_explorers_grid():
    """/maze/grid from the explorer moves scene_trigger's grid in place (its trigger and
    detectors hold the same object); another cell size or a bad message changes nothing."""
    from std_msgs.msg import String
    from maze_nav.scene_trigger import SceneTrigger
    logs = []
    st = SimpleNamespace(cells=CellGrid(1.02), get_logger=lambda: SimpleNamespace(
        info=logs.append, warn=logs.append))
    held = st.cells
    SceneTrigger.on_grid(st, String(data=json.dumps(
        {'size': 1.02, 'origin': [0.04, -0.03], 'yaw': -0.087})))
    assert held is st.cells and held.origin == (0.04, -0.03) and held.yaw == -0.087
    SceneTrigger.on_grid(st, String(data=json.dumps(
        {'size': 1.0, 'origin': [9.0, 9.0], 'yaw': 1.0})))
    SceneTrigger.on_grid(st, String(data='not json'))
    assert held.origin == (0.04, -0.03) and any('bad /maze/grid' in m for m in logs)
