"""The two presentation mazes (resource/Presentation, 2026-11-21; test/maze_curtains.json,
test/maze_stairs.json from make_presentation_mazes.py): descending stairs, terrain, the floor
under the feet, object footprints, the terrain speed cap and the sim's "steps" solid.

Lidar tests use stair_scene's simulated Mid-360 scans with occlusion, on boxes built from
the maze JSON (heights in it are guesses from the photos, not measurements)."""
import importlib.util
import json
import math
import os

import numpy as np
import pytest

from maze_nav import stairs_lidar as L
from maze_nav.cmd_vel_bridge import terrain_cap
from maze_nav.scene_understanding.scene_cells import (CellGrid, block_boxes, cell_objects,
                                                      grow_box, in_boxes, object_box)
from stair_scene import R_BASE_LIDAR, STANDING, T_BASE_LIDAR, lidar_scan

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name):
    with open(os.path.join(HERE, name)) as f:
        return json.load(f)


def steps_boxes(s):
    """A "steps" solid as one box per tread (fake_maze_lite3.steps_height)."""
    lo, hi, n = s['min'], s['max'], int(s.get('treads', 2))
    axis = 0 if s['down'][1] == 'x' else 1
    length = (hi[axis] - lo[axis]) / n
    out = []
    for i in range(n):
        a0 = lo[axis] + i * length if s['down'][0] == '+' else hi[axis] - (i + 1) * length
        b0, b1 = list(lo), list(hi)
        b0[axis], b1[axis] = a0, a0 + length
        b1[2] = hi[2] - i * (hi[2] - lo[2]) / n
        b0[2] = 0.0
        out.append((tuple(b0), tuple(b1)))
    return out


def boxes_of(maze):
    """stair_scene boxes ((x0, y0, z0), (x1, y1, z1)) of a maze JSON (wedges as blocks at
    their mean height; curtains left out: fabric isn't terrain)."""
    out = []
    for s in maze['solids']:
        lo, hi = s['min'], s['max']
        if s['kind'] in ('tile', 'mat', 'block', 'wall', 'rail'):
            out.append((tuple(lo), tuple(hi)))
        elif s['kind'] == 'wedge':
            out.append(((lo[0], lo[1], 0.0), (hi[0], hi[1], (lo[2] + hi[2]) / 2)))
        elif s['kind'] == 'steps':
            out += steps_boxes(s)
    return out


def ground(boxes, x, y):
    """Height of the walkable surface at (x, y): highest non-wall box top there (or 0)."""
    h = 0.0
    for lo, hi in boxes:
        if hi[2] <= 0.30 and lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]:
            h = max(h, hi[2])
    return h


def scan_into(hm, boxes, xy, yaw, scans=2, seed=0):
    base = (xy[0], xy[1], ground(boxes, *xy) + STANDING)
    for k in range(scans):
        pl, r_ob = lidar_scan(boxes, base, yaw, seed=seed + k, n=12000)
        hm.add(pl @ (r_ob @ R_BASE_LIDAR).T + (np.asarray(base) + r_ob @ T_BASE_LIDAR), base, yaw)
    return base


def walk(boxes, path, yaw, step=0.1):
    """Height map built walking a straight path; yields (base, hm) at each step."""
    hm = L.HeightMap()
    (x0, y0), (x1, y1) = path
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    for k in range(n + 1):
        xy = (x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
        base = scan_into(hm, boxes, xy, yaw, seed=10 * k)
        yield base, hm


# ---------------------------------------------------------------- descending stairs
def test_stairs_maze_descending_stairs_found_from_the_landing():
    """maze_stairs: from the normal cell down onto the landing, facing the exit (-y): the
    drop at the landing's edge (maze y = -0.5) is found, heading down (-y), before the robot
    is at the edge; it is lidar-only, anchored, down=True."""
    boxes = boxes_of(load('maze_stairs.json'))
    hits = []
    for base, hm in walk(boxes, ((0.0, 1.0), (0.0, 0.0)), -math.pi / 2):
        for e in L.detect_down(hm, base, hm.floor_under(base)):
            hits.append((base[1], e))
    assert hits, 'descending stairs never found'
    y, e = hits[-1]
    assert e.down and e.anchored and e.source == 'lidar'
    assert abs(math.degrees(L.wrap(e.heading + math.pi / 2))) < 10
    edge_y = e.edge_point(np.array([0.0, y]))[1]
    assert edge_y == pytest.approx(-0.5, abs=0.10)
    assert e.risers >= 2 and e.rise > 0.08                   # 22 cm: two steps of > 8 cm
    assert max(h[0] for h in hits) >= 0.2                     # seen well before the edge


@pytest.mark.parametrize('name,path,yaw', [
    ('stairs maze: start -> slats', ((2.0, 0.0), (2.0, 1.0)), math.pi / 2),
    ('stairs maze: slats -> block field', ((2.0, 1.0), (1.2, 1.0)), math.pi),
    ('stairs maze: block field, looking into the room (10 cm down)', ((1.0, 1.3), (1.0, 0.8)),
     -math.pi / 2),
    ('curtain maze: start -> raised box', ((0.0, 1.0), (0.0, 0.2)), -math.pi / 2),
    ('curtain maze: slats, looking at the curtain cell (10 cm down)', ((1.0, -0.2), (1.0, 0.3)),
     math.pi / 2),
])
def test_presentation_mazes_no_false_stairs(name, path, yaw):
    """Plate edges (10 cm), slats, the block field and the raised box: neither descending
    nor rising stairs."""
    maze = 'maze_stairs.json' if name.startswith('stairs') else 'maze_curtains.json'
    boxes = boxes_of(load(maze))
    for base, hm in walk(boxes, path, yaw, step=0.25):
        under = hm.floor_under(base)
        assert L.detect_down(hm, base, under) == [], f'{name}: false descending stairs'
        ups = [e for e in L.detect(hm, base, hm.floor(base))
               if e.risers >= 2 and e.rise > 0.08 and e.width >= 0.6]
        assert ups == [], f'{name}: false rising stairs {ups}'


# ---------------------------------------------------------------- floor under the feet
PLATE = 0.10                      # start floor (wall_scan) in both mazes


def test_local_floor_follows_the_lower_curtain_cell():
    """maze_curtains: the start plate is 10 cm above the curtain cell's lab floor. Walking
    from the slats into the curtain cell to where the curtain is inspected (mid-cell), the
    floor under the feet is the lab floor (~0.0), not the start floor (0.10): the gap
    measured lying down and the lying height would be 10 cm off with the start floor.
    From the plate the lidar never sees that floor (first seen ~1.8 m ahead), so it comes
    from the standing height (LocalFloor 'body'), learned on the start floor."""
    boxes = boxes_of(load('maze_curtains.json'))
    lf = L.LocalFloor()
    lf.update(PLATE + STANDING, None, start_floor=PLATE)    # standing at the start
    out = []
    for base, hm in walk(boxes, ((1.0, -0.3), (1.0, 1.0)), math.pi / 2):
        out.append((base[1],) + lf.update(base[2], hm.floor_under(base), PLATE))
    y, floor, src = out[-1]
    assert floor == pytest.approx(0.0, abs=0.02) and src in ('body', 'seen')
    # On the slats the body rides 6 cm higher: the floor follows (what the feet are on).
    assert out[0][1] == pytest.approx(PLATE + 0.06, abs=0.02)


def test_local_floor_rules():
    lf = L.LocalFloor()
    assert lf.update(0.45) == (None, None)                       # nothing known yet
    assert lf.update(0.45, None, start_floor=0.10)[0] == pytest.approx(0.10)
    assert lf.update(0.35)[0] == pytest.approx(0.0)              # stepped down 10 cm
    assert lf.update(0.20, lying=True) == (None, None)           # lying: nothing
    assert lf.stand_h == pytest.approx(0.35)                     # ... and not learned
    f, src = lf.update(0.30, seen=0.0)                            # lidar: base 0.30 over it
    assert src == 'seen' and f == 0.0 and lf.stand_h < 0.35
    lf2 = L.LocalFloor()
    lf2.update(0.12, None, start_floor=0.0)                       # lying at the start
    assert lf2.stand_h is None


def test_floor_under_is_none_lying_down():
    """Seen floor under the feet after a 2 m walk on a plate; lying down (base 0.2 m lower)
    it is out of the window: None."""
    boxes = [((-3.0, -3.0, 0.0), (3.0, 3.0, PLATE))]
    for base, hm in walk(boxes, ((0.0, -2.0), (0.0, 0.0)), math.pi / 2):
        pass
    assert hm.floor_under(base) == pytest.approx(PLATE, abs=0.02)
    assert hm.floor_under(base) is not None
    lying = (base[0], base[1], base[2] - 0.20)
    assert hm.floor_under(lying) is None


# ---------------------------------------------------------------- terrain
def test_terrain_levels_in_the_stairs_maze():
    """Facing the block field from the slats cell: obstacle; facing the slats from the
    start cell: rough or obstacle (the lidar smears 4 cm slats into a raised surface).
    Short walks: no floor under the feet yet, so the start floor (as stair_detector)."""
    boxes = boxes_of(load('maze_stairs.json'))
    hm = L.HeightMap()
    for base, hm in walk(boxes, ((2.0, 1.2), (1.75, 1.2)), math.pi, step=0.05):
        pass
    t = L.terrain(hm, base, math.pi, hm.floor_under(base) or PLATE)
    assert t['level'] == 'obstacle', t
    hm = L.HeightMap()
    for base, hm in walk(boxes, ((2.0, -0.3), (2.0, 0.2)), math.pi / 2, step=0.05):
        pass
    t = L.terrain(hm, base, math.pi / 2, hm.floor_under(base) or PLATE)
    assert t['level'] in ('rough', 'obstacle'), t


def test_terrain_on_rough_ground_and_drop():
    """Standing on the block field: on_rough (Pose looks skipped there); at the landing
    edge facing down: drop."""
    boxes = boxes_of(load('maze_stairs.json'))
    hm = L.HeightMap()
    for base, hm in walk(boxes, ((2.2, 1.0), (1.0, 1.0)), math.pi, step=0.1):
        pass
    t = L.terrain(hm, base, math.pi, hm.floor_under(base) or PLATE)
    assert t['on_rough'], t
    hm = L.HeightMap()
    for base, hm in walk(boxes, ((0.0, 1.0), (0.0, 0.0)), -math.pi / 2, step=0.1):
        pass
    t = L.terrain(hm, base, -math.pi / 2, hm.floor_under(base) or PLATE)
    assert t['level'] == 'drop', t


def test_terrain_flat_plate_not_rough():
    """The start plate with no terrain: even ahead, not on_rough (2 cm lidar noise)."""
    boxes = [((-3.0, -3.0, 0.0), (3.0, 3.0, PLATE))]
    for base, hm in walk(boxes, ((0.0, -2.0), (0.0, 0.0)), math.pi / 2):
        t = L.terrain(hm, base, math.pi / 2, hm.floor_under(base) or PLATE)
        assert t['level'] in ('even', 'unknown') and not t['on_rough'], t


def test_terrain_unknown_without_floor():
    assert L.terrain(L.HeightMap(), (0.0, 0.0, 0.35), 0.0, None)['level'] == 'unknown'


def test_terrain_cap():
    assert terrain_cap('even', 0.1, 0.25) == 0.25
    assert terrain_cap('rough', 0.1, 0.25) == 0.15
    assert terrain_cap('obstacle', 0.1, 0.25) == 0.10
    assert terrain_cap('drop', 0.1, 0.25) == 0.10
    assert terrain_cap('rough', 3.0, 0.25) == 0.25                  # stale
    assert terrain_cap(None, None, 0.25) == 0.25
    assert terrain_cap('obstacle', 0.1, 0.08) == 0.08               # never raises


# ---------------------------------------------------------------- object footprints
def _maze_map(name):
    from test_scene_cells import _sim_maze_map
    return _sim_maze_map(name)


@pytest.mark.parametrize('name', ['maze_curtains.json', 'maze_stairs.json'])
def test_room_object_footprint_not_whole_cell(name):
    """The room's object is found, and its footprint box covers well under half the cell:
    the rest of the room stays free once only the box (+ 0.25 m) is blocked."""
    g = _maze_map(name)
    cells = CellGrid()
    found = cell_objects(g, cells)
    assert len(found) == 1, found
    cell = next(iter(found))
    box = grow_box(None, object_box(g, cells, cell))
    assert box is not None
    area = (box[2] - box[0]) * (box[3] - box[1])
    assert 0.09 <= area <= 0.25, box
    blocked = block_boxes(g, [box], margin=0.25)
    centre = cells.centre(cell)
    # Points of the room cell 0.35 m from its centre, away from the object, stay free.
    ox, oy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    away = np.array(centre) - np.array([ox, oy])
    away = away / max(np.linalg.norm(away), 1e-6)
    p = np.array(centre) + 0.35 * away
    gc = blocked.cell(tuple(p))
    assert blocked.data[gc[1], gc[0]] < 50, 'free half of the room blocked'
    assert in_boxes((ox, oy), [box])


def test_grow_box_min_size_and_union():
    b = grow_box(None, (1.0, 1.0, 1.0, 1.05))                       # a single face: a line
    assert b[2] - b[0] == pytest.approx(0.3) and b[3] - b[1] == pytest.approx(0.3)
    b2 = grow_box(b, (1.2, 0.9, 1.4, 1.0))
    assert b2[0] == pytest.approx(b[0]) and b2[2] == pytest.approx(1.4)
    assert grow_box(None, None) is None


def test_keepout_boxes_mask():
    explorer = pytest.importorskip('maze_nav.explorer')
    g = explorer.keepout_boxes_mask([(0.0, 0.0, 0.3, 0.3)])
    assert g.data[g.cell((0.15, 0.15))[1], g.cell((0.15, 0.15))[0]] == 100
    assert g.data[g.cell((0.0, -0.08))[1], g.cell((0.0, -0.08))[0]] == 0
    assert explorer.keepout_boxes_mask([]).data.shape == (2, 2)


# ---------------------------------------------------------------- sim geometry
def _sim_module():
    pytest.importorskip('rclpy')
    spec = importlib.util.spec_from_file_location('fake_maze_lite3',
                                                  os.path.join(HERE, 'fake_maze_lite3.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_sim_steps_and_floors():
    fm = _sim_module()
    m = fm.Maze(os.path.join(HERE, 'maze_stairs.json'))
    assert m.height(2.0, 0.0) == pytest.approx(0.10)                # start plate
    assert m.height(1.0, 0.25) == pytest.approx(0.0)                # room: lab floor
    assert m.height(0.0, 0.0) == pytest.approx(0.22)                # landing
    assert m.height(0.0, -0.65) == pytest.approx(0.11)              # the step outside
    assert m.height(0.0, -0.95) == pytest.approx(0.0)               # lab floor: out
    assert m.is_out(0.0, -0.95) and not m.is_out(2.0, 0.0)
    m = fm.Maze(os.path.join(HERE, 'maze_curtains.json'))
    assert m.height(0.0, 1.0) == pytest.approx(0.10)                # start plate
    assert m.height(1.0, 1.0) == pytest.approx(0.0)                 # curtain cell
    assert m.is_out(1.0, 2.1) and not m.is_out(1.0, 1.0)
    assert len(m.curtains) == 1
    pts, occ = m.points()
    assert len(pts) > 1000 and occ.any()


def test_maze_files_are_current():
    """The JSON files are what make_presentation_mazes.py writes (edit the script, rerun)."""
    spec = importlib.util.spec_from_file_location(
        'make_presentation_mazes', os.path.join(HERE, 'make_presentation_mazes.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert json.loads(json.dumps(mod.maze_curtains())) == load('maze_curtains.json')
    assert json.loads(json.dumps(mod.maze_stairs())) == load('maze_stairs.json')
