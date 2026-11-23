"""An ~18 cm rolling pin across the middle of a corridor cell (user 2026-11-22): exploration
walks over it with the H-Step gait (the default, cmd_vel_bridge gait:=hstep).

It must not stop or reroute the robot: not a wall (wall band 0.30-0.45 m), not stairs (one
rise, not a chain), not a drop, not a curtain (nothing in the wall band), and the floor under
the feet stays the floor. It IS terrain 'obstacle' ahead, so cmd_vel_bridge caps the speed at
terrain_obstacle_speed (0.10 m/s) while stepping over.

Two shapes, both full corridor width: a solid bar lying on the floor (worst case for the
stair / terrain detectors) and a roller on end stands (8 cm roller, 10-18 cm up, free under).
The sim's version is challenge_a.json's "pin" in cell (2, 0) (make_challenge_mazes.py).
Simulated Mid-360 scans and depth frames with occlusion (stair_scene). The body keeps its
standing height over the pin (the legs step over it; the body doesn't ride on it)."""
import json
import math

import numpy as np
import pytest

from maze_nav import stairs_lidar as L
from maze_nav.cmd_vel_bridge import terrain_cap
from maze_nav.core import wall_ranges
from maze_nav.curtains import detect_curtains
from maze_nav.stairs import detect_stairs, level_transform
from stair_scene import (K, R_BASE_CAM, R_BASE_LIDAR, STANDING, T_BASE_CAM, T_BASE_LIDAR,
                         lidar_scan, render)

PIN_X, PIN_TOP, HALF = 2.0, 0.18, 0.45      # corridor along +x, inner walls at y = +-0.45
WALLS = [((-1.0, HALF, 0.0), (5.0, HALF + 0.05, 0.45)),
         ((-1.0, -HALF - 0.05, 0.0), (5.0, -HALF, 0.45))]
SHAPES = {
    'bar on the floor': [((PIN_X - 0.04, -HALF, 0.0), (PIN_X + 0.04, HALF, PIN_TOP))],
    'roller on stands': [((PIN_X - 0.04, -HALF + 0.05, PIN_TOP - 0.08), (PIN_X + 0.04, HALF - 0.05, PIN_TOP)),
                         ((PIN_X - 0.04, -HALF, 0.0), (PIN_X + 0.04, -HALF + 0.05, PIN_TOP)),
                         ((PIN_X - 0.04, HALF - 0.05, 0.0), (PIN_X + 0.04, HALF, PIN_TOP))],
}
BAND = (0.30, 0.45)                         # wall_scan band_low / band_high
N_BEAMS = 720


def odom_points(boxes, base, yaw, seed):
    pl, r_ob = lidar_scan(boxes, base, yaw, seed=seed, n=12000)
    return pl @ (r_ob @ R_BASE_LIDAR).T + (np.asarray(base) + r_ob @ T_BASE_LIDAR)


def walk(boxes, x0=0.0, x1=3.0, step=0.1):
    """Walk along the corridor centre over the pin, facing +x; yields (base, lidar odom
    points, lidar height map, depth floor map, depth frame, r_ob) at each step."""
    hm, dm = L.HeightMap(), L.HeightMap()
    n = int(round((x1 - x0) / step))
    for k in range(n + 1):
        base = (x0 + k * step, 0.0, STANDING)
        pts = np.concatenate([odom_points(boxes, base, 0.0, 10 * k + j) for j in range(2)])
        hm.add(pts, base, 0.0)
        depth, r_ob = render(boxes, base, 0.0, seed=k)
        dm.add(L.depth_floor_points(depth, K, r_ob, R_BASE_CAM, T_BASE_CAM, base), base, 0.0)
        yield base, pts, hm, dm, depth, r_ob


@pytest.fixture(scope='module', params=list(SHAPES))
def run(request):
    return request.param, list(walk(WALLS + SHAPES[request.param]))


def test_pin_is_not_a_wall(run):
    """wall_scan: no beam stops at the pin, so neither SLAM nor the Nav2 costmaps see it."""
    name, steps = run
    inc = 2 * math.pi / N_BEAMS
    for base, pts, *_ in steps:
        r = wall_ranges(pts, base[:2], 0.0, 0.0, BAND, -math.pi, inc, N_BEAMS, 0.25, 12.0)
        a = -math.pi + inc * np.arange(N_BEAMS)
        ok = np.isfinite(r)
        hx, hy = base[0] + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        on_pin = (np.abs(hx - PIN_X) < 0.15) & (np.abs(hy) < HALF - 0.08)
        assert not on_pin.any(), f'{name} at x={base[0]:.1f}: pin in the wall scan'


def test_pin_is_not_stairs_or_a_drop(run):
    """stairs_lidar (rising and descending) and the depth camera's riser chain: no stairs,
    so stair_detector neither hands over nor starts a stair check."""
    name, steps = run
    for base, pts, hm, dm, depth, r_ob in steps:
        floor = hm.floor(base)
        if floor is not None:
            ups = [e for e in L.detect(hm, base, floor) if e.risers >= 2]
            assert ups == [], f'{name} at x={base[0]:.1f}: lidar stairs {ups}'
        assert L.detect_down(hm, base, hm.floor_under(base, also=(dm,)) or 0.0) == [], \
            f'{name} at x={base[0]:.1f}: descending stairs'
        r, t, _ = level_transform(r_ob, R_BASE_CAM, T_BASE_CAM)
        det = detect_stairs(depth, K, r, t, floor_z=-STANDING)
        assert det is None, f'{name} at x={base[0]:.1f}: camera stairs {det}'


def test_pin_is_not_a_curtain(run):
    name, steps = run
    for base, pts, *_ in steps:
        lidar = np.asarray(base) + T_BASE_LIDAR
        assert detect_curtains(pts, lidar[:2], 0.0, 0.0) == [], \
            f'{name} at x={base[0]:.1f}: curtain'


def test_floor_under_the_feet_stays_the_floor(run):
    """Over the pin the floor under the feet is still ~0 (not the pin top): the lying height
    and the terrain reference don't jump 20 cm."""
    name, steps = run
    seen = [hm.floor_under(base, also=(dm,)) for base, pts, hm, dm, *_ in steps
            if abs(base[0] - PIN_X) <= 0.4]
    seen = [u for u in seen if u is not None]
    assert seen, f'{name}: floor never seen near the pin'
    assert max(abs(u) for u in seen) < 0.04, f'{name}: floor under {seen}'


def test_pin_ahead_slows_to_obstacle_speed(run):
    """0.25-1.2 m before the pin the strip ahead reads 'obstacle' (or 'rough' when the lidar
    only sees the roller's ends): cmd_vel_bridge caps forward speed at 0.10 (0.15) m/s.
    Never 'drop' or 'unknown' all the way (no cap at all)."""
    name, steps = run
    levels = []
    for base, pts, hm, dm, *_ in steps:
        if 0.3 <= PIN_X - base[0] <= 1.1:
            floor = hm.floor_under(base, also=(dm,)) or 0.0
            levels.append(L.terrain(hm, base, 0.0, floor)['level'])
    assert 'obstacle' in levels, f'{name}: {levels}'
    assert 'drop' not in levels, f'{name}: {levels}'
    assert terrain_cap('obstacle', 0.1, 0.25) == pytest.approx(0.10)


# ---------------------------------------------------------------- find_pin (the pin push)
def test_pin_found_ahead_for_the_push(run):
    """stairs_lidar.find_pin (/maze/terrain 'pin'): found from <= 1.0 m out right up to the
    bar (cmd_vel_bridge starts its push at <= 0.35 m), within 5 cm of where it is."""
    name, steps = run
    for base, pts, hm, dm, *_ in steps:
        ahead = PIN_X - base[0]
        if not 0.1 <= ahead <= 1.0:
            continue
        floor = hm.floor_under(base, also=(dm,)) or 0.0
        p = L.find_pin(hm, base, 0.0, floor)
        assert p is not None, f'{name} at x={base[0]:.1f}: no pin'
        assert abs(p['x'] - PIN_X) < 0.05 and abs(p['at'] - ahead) < 0.05, f'{name}: {p}'
        assert p['height'] > 0.15
    base, pts, hm, dm, *_ = steps[len(steps) // 2]
    t = L.terrain(hm, base, 0.0, 0.0)
    assert t['pin'] is not None and t['level'] in ('obstacle', 'rough', 'even'), t
    json.dumps(t)                                       # stair_detector publishes it as JSON


def lidar_walk(boxes, yaw=0.0, x1=PIN_X - 0.1, step=0.1):
    """Lidar height map only (as stair_detector's terrain), walking along +x facing yaw."""
    hm = L.HeightMap()
    for k in range(int(round(x1 / step)) + 1):
        base = (k * step, 0.0, STANDING)
        pts = np.concatenate([odom_points(boxes, base, yaw, 10 * k + j) for j in range(2)])
        hm.add(pts, base, yaw)
        yield base, hm


def _slats(top=0.12, width=0.05, gap=0.17, plate=0.0, n=5):
    """maze_info.md: slats 5 cm thick, 12 cm tall, 17 cm apart (optionally on a plate)."""
    out = [((PIN_X + k * (width + gap) - width / 2, -HALF, plate),
            (PIN_X + k * (width + gap) + width / 2, HALF, plate + top)) for k in range(n)]
    if plate:
        out.append(((PIN_X - 0.1, -HALF, 0.0), (PIN_X + n * (width + gap), HALF, plate)))
    return out


def _rubble(seed=3):
    """maze_info.md: block rubble up to 15 cm."""
    rng = np.random.default_rng(seed)
    return [((x, y, 0.0), (x + 0.2, min(y + 0.2, HALF),
                           float(rng.choice([0.03, 0.06, 0.09, 0.12, 0.15]))))
            for x in np.arange(PIN_X, PIN_X + 1.0, 0.2) for y in np.arange(-HALF, HALF, 0.2)]


NOT_PINS = {
    'slats': _slats(),
    'slats on a plate': _slats(plate=0.10),
    'rubble': _rubble(),
    'terrain plate 10 cm': [((PIN_X, -HALF, 0.0), (PIN_X + 1.0, HALF, 0.10))],
    'step 15 cm': [((PIN_X, -HALF, 0.0), (PIN_X + 1.0, HALF, 0.15))],
}


@pytest.mark.parametrize('name', list(NOT_PINS))
def test_other_terrain_is_not_a_pin(name):
    """Only the pin gets the push: slats, rubble, plate edges and a step never read as one
    (they are still terrain 'obstacle' / 'rough' for the speed cap)."""
    for base, hm in lidar_walk(WALLS + NOT_PINS[name], x1=PIN_X + 0.3):
        p = L.find_pin(hm, base, 0.0, 0.0)
        assert p is None, f'{name} at x={base[0]:.1f}: {p}'


def test_pin_found_crossed_off_square():
    """Crossed 20 deg off square (the bar smears along the heading): still found, in place."""
    yaw = math.radians(20)
    found = []
    for base, hm in lidar_walk(WALLS + SHAPES['bar on the floor'], yaw=yaw):
        p = L.find_pin(hm, base, yaw, 0.0)
        if PIN_X - base[0] <= 0.8:
            assert p is not None, f'x={base[0]:.1f}: no pin'
            assert abs(p['x'] - PIN_X) < 0.06, p
            found.append(p)
    assert found
