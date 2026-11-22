"""Explore the maze on the SLAM map with Nav2 until the robot is out.

Start: once /map, TF and Nav2 are up, the robot holds still (status PAUSED) until the
operator publishes on /maze/start (std_msgs/Empty):
    ros2 topic pub --once /maze/start std_msgs/msg/Empty
so there is time to start a rosbag recording. Messages sent before PAUSED are ignored.
wait_for_start:=false skips the wait (simulator). Then it creeps `creep_distance` straight
ahead (status CREEPING), with no steering and no turn on the spot: a sim run that steered
the creep 28 deg towards the frontier swung the rear corner into a side wall 0.15 m away,
which the lidar can't see that close (vertical FOV down to -7 deg: no wall band within
~0.5 m beside the robot, so it is missing from the map and from every costmap). The lidar sees no walls behind the robot, so the SLAM map ends at about the
start pose and the robot can sit a few cm off it (seen live: base_link 2-3 cm behind the map
edge, "Robot is out of bounds of the costmap"). The creep also takes the robot off the start
cell's back wall: placed with its rear ~8 cm from it, turning on the spot hit it (spin
"Collision Ahead", RPP "detected collision ahead"). Humble's Smac planner crashes when asked
to plan from outside the global costmap; that is now ruled out by the rolling global
costmap in nav2_params.yaml, not by the creep. Then a scan (status SCANNING, Nav2's /spin
behavior, `start_scan`) turns `scan_angle` left, twice that right and `scan_angle` left again,
back to the heading it had before the scan, so SLAM sees the start area. Both are checked
against walls on the SLAM map with the footprint box, so the robot can start with its body
0.15 m from a wall: the creep is shortened to keep `creep_margin` to a wall ahead (or
skipped), and each scan side is cut to what its sweep clears with `turn_margin` (e.g. ~20 deg
left / ~32 deg right with a wall 0.15 m off the right side), still ending on the start heading;
both sides < 5 deg -> no scan. A failed
turn is logged and the run moves on; it never blocks exploring. Creep and scan are skipped
with go:=false.

Loop (period `replan_period`):
  1. read /map (slam_toolbox) and the robot pose (TF map -> base_link)
  2. exit check: was in corridors, has moved >= min_displacement, and a disk of seen-free
     floor >= exit_open_radius lies within exit_near_radius of it (and away from the
     start) for `dwell` s -> DONE. With the maze size known (`maze_cols` x `maze_rows`
     cells of `cell_size` m, start cell (0, 0) in its corner) the robot stays inside
     instead: no frontier goals outside the grid; once nothing inside is left to explore,
     an outer side open on the final map (and on >= `exit_confirm` replans) is the exit;
     the robot drives to the cell inside it, faces out and stands still (TO_EXIT ->
     AT_EXIT; /maze/launch_cell, JSON: the drone launches from there). The open-floor check
     then only ends the run if the robot got outside the grid anyway
  3. frontier = free cell next to unknown; pick the nearest along free space, keep the
     current one unless another is clearly closer (no flip-flopping)
  4. send it to Nav2 /navigate_to_pose; a reached goal is blacklisted. A failed one is only
     set aside (other frontiers first), and nothing if it failed while Nav2 was down
     (lifecycle manager is_active = false). A "reached" goal more than `false_success_dist`
     from the robot counts as failed: Humble's controller reports success when its TF is too
     old (robot 2026-11-19 11:37). No frontier found while `scan_topic` (/wall_scan) is older
     than `scan_stale` s doesn't count towards done_confirm: stale scans, stale map.
     Start gate: after /maze/start (or straight away with wait_for_start:=false) the run
     only begins once `scan_topic` has come steadily (no gap > `start_gap` s) for
     `start_steady` s; until then the log says `start held: <why>` every 2 s
  5. recovery: `max_failures` failures in a row, or only set-aside frontiers left, while open
     frontiers remain -> clear both costmaps, forget the set-aside goals and try again, up to
     `max_recoveries` times. Seen live: a phantom wall (the robot's own leg) put the robot
     "in lethal space" for the planner, the goal failed, was blacklisted, and the run ended
     NO_EXIT with a 2 m wide frontier still open.
Turning near a wall (`pre_turn`): before each new goal, if the way there leaves the robot
more than `pre_turn_min_angle` off its heading (Nav2's path follower would turn on the spot
first, always the short way) and that turn's sweep would bring the footprint within
`turn_margin` of a wall on the SLAM map, the explorer stops (status TURNING) and turns the
robot itself the long way round, if that sweep is clear, then goes on (core.plan_turn).
Blocked both ways (the usual case in a 0.9 m corridor: the rear corners turn on a 0.437 m
circle): it creeps straight ahead, no steering, up to `pre_turn_max_creep` m, to the spot
where the turn is clear with the most room to the nearest wall, then turns there
(core.plan_creep_turn: `turn_margin`, else 1 cm, which is all a 0.9 m corridor corner
leaves). No such spot: left to Nav2. Clear short turns are left to Nav2 too. Once per goal.
Every turn check rotates the footprint about the robot's real pivot (`pivot_x`, `pivot_y`,
base_link frame; measured ~2.5 cm ahead, ~4 cm right), not base_link: about it the rear
corners sweep ~0.48 m, about base_link 0.44 m (core.pivot_pose). Live 2026-11-19 04:39:
turns planned about base_link "fitted" and then hit (RPP "collision ahead").
Sim 2026-11-16: an 80 deg short-way turn at a cell edge swung the rear corner into the wall. Walls within ~0.5 m
beside the robot that the lidar never saw from further away aren't on the map, so this only
covers walls it has seen.
Stairs (stair_detector.py): while EXPLORING, TURNING or FACING_QR, stair_detector's /stairs/status
HANDOVER means stairs are close ahead. The explorer cancels the Nav2 goal (or turn), sends a
zero Twist and goes to STAIRS; only then does stair_detector drive (face the stairs, square up,
stop in front), so it and Nav2 never both command nav_cmd_vel. Its AT_STAIRS -> AT_STAIRS here:
lined up, cmd_vel_bridge walks over them in AI mode and leaves AI mode by itself
(/maze/ai_status DONE, user 2026-11-21) -> explore on; NOT LINED UP or a walk that didn't
finish: hold still until /maze/start. Its ABORTED or IDLE, or `stairs_timeout` s in STAIRS ->
back to EXPLORING. stairs_topic:='' = ignore it.
Candidate queue (user 2026-11-21): possible stairs and possible curtains seen in a junction
or dead-end look are held by their detector (/stairs/candidate, /curtain/candidate) and
queued here. Once a junction scan is done (or nothing else is left to explore, before
NO_EXIT) the explorer works the queue, nearest first along free space: drive to the spot
`stairs_approach` / `curtain_check_approach` m in front of it, face it (TURNING), release it
(/maze/stairs_release, /maze/curtain_release), and wait `candidate_wait` s for the detector's
HANDOVER (stair check / lie-down inspection); then the next. Queue empty: explore on.
Curtains (curtain_detector.py, `curtain_topic` / `curtains_topic`; '' = ignore): confirmed
curtains (/curtain/curtains, map frame) are cleared on the explorer's copy of /map (a wall with a
low gap is passable), so frontiers behind one count. A goal within `curtain_goal_radius` of a
curtain not yet walked through from this side is sent to the spot `curtain_approach` m in
front of its middle instead, facing it. curtain_detector's HANDOVER: the explorer cancels Nav2, stops and goes to CURTAIN;
curtain_detector inspects (a possible curtain: lie down, measure the gap) or walks the robot
through (a confirmed one). Order (user 2026-11-20):
  - Inspection HANDOVER ("HANDOVER: possible curtain ...", now only for a released queued
    candidate) while EXPLORING, TURNING, FACING_QR or JUNCTION, junction request pending or
    not: the lie-down check
    comes first. A JUNCTION look (or its follow-up) is paused and resumed after it, a
    pending junction request is held. Not a curtain: the wall stays, the look goes on (the
    way then ends in a dead end there and is scanned like one).
  - A curtain confirmed at a junction (look paused, or junction request held) is held:
    /maze/curtain_hold hold_new makes curtain_detector report CONFIRMED instead of walking
    through. The junction's other ways (not the way in, not the curtain's, not back to the
    start cell) are explored fully first: only frontiers in curtain_branch.branch_mask (down
    those ways, not through the junction cell or the curtain) are goals, until none are left
    for `done_confirm` replans. Then the curtain is released (its id leaves the hold ids) and
    the explorer drives to the spot in front of it; curtain_detector's crossing HANDOVER
    takes it from there. The look's follow-up skips the curtain's way. Several nest: the
    newest first.
  - Crossing HANDOVER (a confirmed curtain) during JUNCTION, DEAD_END or SCANNING, or with a
    junction request pending, is ignored (curtain_detector times out, cools down and hands
    over again later). A junction request or a dead end while CURTAIN and the crossing is
    still FACING or APPROACHING (the front not at the fabric yet) takes the robot back:
    JUNCTION / DEAD_END as from EXPLORING, and curtain_detector yields (IDLE, the curtain
    not marked passed, so it is crossed later).
  - During an inspection (INSPECTING / LYING) and the rest of a crossing a junction request
    is held (it doesn't expire) until CURTAIN ends (`junction_timeout` counts from then;
    `junction_radius` still applies); a dead end there is only blacklisted (a curtain looks
    like one; a confirmed curtain's dead end behind it is un-blacklisted).
Confirmed curtains are drawn as free on /maze/map (OccupancyGrid, latched: /map with the
curtains cleared, republished with every /map) and in the map saved at the end. Nav2's global
costmap takes its static layer from /maze/map (nav2_params.yaml), so it can plan through a
curtain once wall_scan drops it (walked through); one not crossed yet stays in the obstacle
layer. On PASSED both costmaps are cleared: the curtain's old marks never clear on their own.
Its PASSED, IDLE or ABORTED, or `curtain_timeout` s -> EXPLORING. Its STUCK (sensors still
covered) -> AT_CURTAIN: hold still until /maze/start. On PASSED, and on /maze/start from
AT_CURTAIN, the local costmap is cleared: fabric the robot pushed or dragged was marked right
around it (wall_scan's dropped curtain bins read +inf, which never clear a cell).
Unreadable QR code (`qr_unread_topic`, from qr_scanner: located in the image but not read,
e.g. too far round, blurred or at a steep angle): while EXPLORING, the explorer cancels the
goal and turns to face it (FACING_QR; cut short where the body would touch a wall), then
holds still while qr_scanner runs its slow, thorough decode on every frame.
It goes back to EXPLORING when the code is read (`qr_read_topic`), when it hasn't been seen
for `face_lost` s, or after `face_timeout` s (that spot is then ignored for `face_cooldown`
s, so the robot can't get stuck on it). Re-aims up to `face_max_turns` times if it drifts.
Stairs (HANDOVER) interrupt it.
Junction (`junction_topic`, from scene_trigger: a new cell with 2+ open ways on besides the
way in, robot near its centre; user 2026-11-21): while EXPLORING, the explorer cancels the
goal and, without leaving the junction cell, scans each adjacent cell not visited yet,
left-most first (JUNCTION): turn on the spot to face it (cut short where the body would
touch a wall), move sideways (`junction_centre`: linear.y, `centre_speed`) onto the line
through the junction cell's centre along the robot's heading, then hold `junction_hold` s
with perception on and one /scene/capture (kind junction). Possible stairs / curtains seen
are queued by their detectors; an object room scanned this way is marked explored (no goal
into it, no room look-in later). Then (`junction_follow`): queued candidates -> work the
queue (see Candidate queue); none -> trace each way on the map (scene_cells.trace_way, up to
`junction_trace_cells` cells), blacklist the ones ending in a dead end, and drive one cell
into the nearest way to an unexplored cell, then explore on. Status stays JUNCTION
throughout. Requests older than `junction_timeout` s or more than `junction_radius` m from
the cell centre are dropped.
Dead end (`dead_end_topic`: scene_trigger's /scene/trigger, kind dead_end, fired from the
cell before it while facing it): the dead-end cell is held out of goals (not blacklisted)
until scene understanding answers for it (user 2026-11-22, `scene_result_topic`): room ->
blacklisted; stairs / curtain -> the candidate queue, checked after the junctions; anything
else -> open again. No answer `scene_result_timeout` s after its last view: an object in it
on the map -> blacklisted, not a dead end any more -> open, else /scene/retry (room_scene
resends its views; `scene_max_attempts` tries); the last unanswered -> blacklisted. Without
room_scene (no capture topic) it is blacklisted at once.
While EXPLORING the explorer also cancels the goal and holds still `dead_end_hold`
s (DEAD_END) so scene understanding gets a steady view, then, still in the cell before it,
turns the camera to the middle of the dead end's left wall, then its right wall (the
record's side_walls; at most `dead_end_look_angle` off the dead end's heading; cut short
where the body would touch a wall), holding `dead_end_side_hold` s at each so the QR
detector sees the side walls (candidates seen there are queued, user 2026-11-21), then
explores on. During JUNCTION it holds
`dead_end_hold` s at that way and scans the side walls the same way before the next way.
Stairs interrupt both; curtains don't (see Curtains).
Pose-mode looks (`pose_looks`, user's call 2026-11-19): the dead-end side-wall looks don't
turn the robot (no Nav2 /spin). The start scan is NOT a Pose-mode look any more: it always
turns the robot with Nav2 /spin, 30 deg left, 60 right, 30 left (user 2026-11-20). In a
look the feet stay put and only the body yaws in Lite3 Pose mode: the explorer publishes the body yaw it wants (map-frame offset from
the stance heading, + = left, at most `pose_max_yaw` * `pose_axis_limit`, cut where the body
box + `pose_margin` would touch a wall over the sweep plus `pose_overshoot`; none when the box
+ `pose_margin` already touches a wall at the stance: motor buzzing, run 2026-11-21 10:56) on `pose_yaw_topic` at 10 Hz, and cmd_vel_bridge switches modes and sends the yaw axis
command (see its doc). The hold counts from the bridge's POSING (/maze/pose_status) plus
`pose_turn_time` s for the body to get there. Dead end: left wall, right wall, `dead_end_side_hold` s
each. No POSING within `pose_wait` s: that look is skipped. While /maze/pose_status says
UNAVAILABLE (sim, drive:=false, no transfer_interfaces), or with pose_looks:=false, the old
Nav2 spins are used. Also while stair_detector's /maze/terrain says the robot stands on rough
ground (on_rough: a block field; presentation maze 2026-11-21), so no body twist is held there.
Objects (`object_keepout`, 2026-11-21): 'footprint' keeps only the object's box (+ margin) out
of the explorer's map, /maze/map and the Nav2 keepout mask, so the free half of a room can be
entered; 'cell' blocks the whole cell (never entered).
Back of the start cell (user 2026-11-20): the pose at /maze/start is kept. No frontier goal,
junction look or junction follow-up goes into the start cell (within 0.5 m of that pose) or
behind it (`start_back_depth` m back, `start_back_width` m either side; the lidar never saw
there, so it reads as a frontier, but it is the start cell's back wall). 0 = off.
Nav2 down for `nav_down_timeout` s (a server crashed and the lifecycle manager deactivated
the rest; Humble doesn't bring it back) -> STOPPED, not NO_EXIT.
Nothing left to explore and no exit -> DONE (no exit found): the running goal is cancelled
at once, a zero Twist goes to nav_cmd_vel and "nav stopped" is printed. Either way Nav2 is
cancelled, the map is saved and /maze/status says why. Ctrl+C / SIGTERM (also from
ros2 launch) ends the run as STOPPED the same way: rclpy's own signal handler is off so
the context is still valid for the cancel and the final status.

Status on /maze/status (std_msgs/String, latched): STARTING, PAUSED, CREEPING, SCANNING, EXPLORING, TURNING,
FACING_QR, JUNCTION, DEAD_END, STAIRS, AT_STAIRS, CURTAIN, AT_CURTAIN, TO_EXIT, AT_EXIT, EXITED, NO_EXIT, STOPPED. Goals on /maze/goal (PoseStamped) for RViz. `go`
parameter false = plan and publish goals but never send them to Nav2 (dry run on the robot).
"""
import array
import collections
import dataclasses
import json
import math
import os
import re
import signal
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.curtain_branch import (CurtainHold, Deferral, branch_mask, in_mask,
                                    needs_crossing, way_towards)
from maze_nav.curtains import clear_curtains, curtains_from_json
from maze_nav.scene_cells import (DIRECTIONS, CellGrid, block_boxes, block_cells,
                                             boundary_sides, cell_objects, exit_cell,
                                             fit_grid, grow_box, in_boxes, object_box, opening,
                                             side_heading, side_walls, trace_way)
from touble_shooting.live_params import attach as attach_live_params
from maze_nav.core import (FREE_MAX, OCCUPIED_MIN, ExitDetector, Frontier, Grid, choose_frontier,
                           clearance_map, find_frontiers, geodesic_distance,
                           footprint_hits, free_distance_ahead, heading_towards, max_turn,
                           plan_creep_turn,
                           plan_turn)

START_CMD = 'ros2 topic pub --once /maze/start std_msgs/msg/Empty'

CURTAIN_STATES = ('CURTAIN', 'AT_CURTAIN')
END_STATES = ('EXITED', 'NO_EXIT', 'STOPPED', 'AT_EXIT')
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_pitch_roll(q):
    """(pitch, roll) rad of a quaternion (x, y, z, w); pitch + = nose down (REP 103)."""
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    return math.asin(sinp), math.atan2(2 * (q.w * q.x + q.y * q.z),
                                       1 - 2 * (q.x * q.x + q.y * q.y))


def dead_end_scene(steps, cell, centre, kind='dead_end'):
    """Tag a dead end's (or room's) view steps (facing into it first, then its side walls,
    each with 'view') for /scene/capture: the cell (scene_trigger grid), its centre in map,
    kind, and 'last' on the final view. Returns steps."""
    for k, s in enumerate(steps):
        s['scene'] = {'cell': [int(cell[0]), int(cell[1])],
                      'cell_centre': [round(float(v), 3) for v in centre], 'kind': kind,
                      'view': s.get('view', 'centre'), 'last': k == len(steps) - 1}
    return steps


# The scoring server's scene_type vocabulary (challenge booklet, section F): each is a room.
ROOM_SCENE_TYPES = ('kitchen', 'market', 'workshop', 'clinic', 'cleaning_room', 'room')


def scene_verdict(result):
    """room_scene's /scene/result -> 'room', 'stairs', 'curtain', 'empty', or None (not
    analysed: no VLM, or scene_type unknown with nothing found). scene_type names it first
    (a room type from ROOM_SCENE_TYPES, stairs, a curtain); else hazards mentioning stairs /
    a curtain; else any object = a room (user 2026-11-22: a room is a cell with an object)."""
    if result.get('vlm') in (None, '', 'none'):
        return None
    st = str(result.get('scene_type') or '').strip().lower().replace(' ', '_')
    for kind, words in (('room', ROOM_SCENE_TYPES), ('stairs', ('stair',)),
                        ('curtain', ('curtain',))):
        if any(w in st for w in words):
            return kind
    hazards = ' '.join(str(h) for h in result.get('hazards') or []).lower()
    if 'stair' in hazards:
        return 'stairs'
    if 'curtain' in hazards:
        return 'curtain'
    if result.get('objects'):
        return 'room'
    if st in ('', 'unknown'):
        return None
    return 'empty'


def path_field(grid, xy, robot_radius=0.25):
    """(m) shortest path length from xy over free cells at least robot_radius from walls,
    per map cell (inf = not reachable), or None (robot nowhere near passable space). Same
    rules as core.find_frontiers."""
    passable = grid.free & (clearance_map(grid) >= robot_radius)
    start = grid.cell(xy)
    if not grid.contains(start) or not passable[start[1], start[0]]:
        idx = np.argwhere(passable)
        if len(idx) == 0:
            return None
        d = np.hypot(idx[:, 1] - start[0], idx[:, 0] - start[1])
        if d.min() * grid.resolution > 0.6:
            return None
        start = (int(idx[d.argmin(), 1]), int(idx[d.argmin(), 0]))
    return geodesic_distance(passable, start) * grid.resolution


def room_look_spot(raw, grid, cells, cell, field, skip=(), radius=3.0):
    """(neighbour, d, m): the cell to look into room `cell` from, d the direction from the
    room to it, m the path length there. Its side shared with the room is open on `raw` (the
    map without blocked cells drawn in), it isn't in `skip`, and its centre is free on
    `grid` and reachable within `radius` m along `field` (path_field: a straight line through
    a wall doesn't count, robot 2026-11-21 10:28). The nearest such; None if none (yet)."""
    best = None
    for d in DIRECTIONS.values():
        n = (cell[0] + d[0], cell[1] + d[1])
        if n in skip or opening(raw, cells, cell, d) != 'open':
            continue
        g = grid.cell(cells.centre(n))
        if not grid.contains(g) or not 0 <= grid.data[g[1], g[0]] <= FREE_MAX:
            continue
        dist = float(field[g[1], g[0]])
        if dist <= radius and (best is None or dist < best[2]):
            best = (n, d, dist)
    return best


def keepout_mask(cells, blocked, resolution=0.05, margin=0.1):
    """Grid (core.Grid, map frame) over the given maze cells: 100 inside them, 0 around
    (Nav2 KeepoutFilter: lethal / free). No cells: a 2 x 2 free patch at the map origin, so
    the filter has a mask (and is active) from the start."""
    if not blocked:
        return Grid(np.zeros((2, 2), np.int16), resolution, (0.0, 0.0))
    corners = np.array([cells.world((c[0] + a, c[1] + b)) for c in blocked
                        for a in (-0.5, 0.5) for b in (-0.5, 0.5)])
    lo, hi = corners.min(axis=0) - margin, corners.max(axis=0) + margin
    w, h = np.ceil((hi - lo) / resolution).astype(int)
    g = Grid(np.zeros((int(h), int(w)), np.int16), resolution, (float(lo[0]), float(lo[1])))
    return block_cells(g, cells, blocked)


def keepout_boxes_mask(boxes, resolution=0.05, margin=0.1):
    """Keepout mask (core.Grid, map frame) over the given map boxes: 100 inside, 0 around.
    No boxes: a 2 x 2 free patch at the map origin (the filter keeps a mask)."""
    if not boxes:
        return Grid(np.zeros((2, 2), np.int16), resolution, (0.0, 0.0))
    b = np.array(boxes, float)
    lo, hi = b[:, :2].min(axis=0) - margin, b[:, 2:].max(axis=0) + margin
    w, h = np.ceil((hi - lo) / resolution).astype(int)
    g = Grid(np.zeros((int(h), int(w)), np.int16), resolution, (float(lo[0]), float(lo[1])))
    return block_boxes(g, boxes)


class Explorer(Node):

    def __init__(self):
        super().__init__('maze_explorer')
        p = self.declare_parameter
        p('go', True)
        p('start_delay', 5.0)          # s after the first map before moving
        p('wait_for_start', True)      # hold still until /maze/start (start rosbag recording)
        p('replan_period', 1.0)
        p('robot_radius', 0.25)        # half width incl. feet + a little
        p('goal_clearance', 0.33)      # goals at least this far from walls
        p('min_frontier_size', 5)      # cells
        p('robot_width', 0.46)         # m, footprint width (nav2_params.yaml)
        p('min_frontier_width_ratio', 0.8)   # frontiers narrower than this x robot_width: ignored
        # A goal is cancelled after goal_timeout s without the robot moving goal_progress m
        # (was a flat 90 s per goal: pinned on a wall, Nav2 retried 4 x 20 s in the 2026-11-21
        # sim), or after goal_max_time s in all (long goals at 0.15 m/s still get there).
        p('goal_timeout', 45.0)
        p('goal_progress', 0.2)
        p('goal_max_time', 120.0)
        p('max_failures', 3)           # failed goals in a row before NO_EXIT
        p('done_confirm', 5)           # replans in a row with no frontier before NO_EXIT
        p('max_recoveries', 3)         # costmap clear + retry rounds while frontiers are open
        p('exit_open_radius', 1.1)     # m of seen-free floor around a point = outside
        p('exit_near_radius', 0.8)     # that point within this distance of the robot
        p('exit_min_displacement', 1.5)
        p('exit_dwell', 2.0)
        # No junction / dead-end looks once this much seen-free floor is next to the robot
        # (x exit_open_radius; maze corridors give <= ~0.45 m): outside the maze every cell
        # looked like a junction and the exit check never got its dwell (sim 2026-11-21).
        p('open_space_no_looks', 0.8)
        p('max_runtime', 900.0)        # s; hard stop
        p('creep_distance', 0.10)      # m straight ahead after start, onto the map and
        #                                off the back wall so the scan turns clear it; 0 = off
        p('creep_speed', 0.05)         # m/s
        p('start_scan', False)         # after the creep: scan_angle left, 2x right, left again
        # rad; 30 deg (user 2026-11-20: physical turns 30 left, 60 right, 30 left); less on a
        # side where the body would hit a wall
        p('scan_angle', math.pi / 6)
        # Footprint box (nav2_params.yaml: x -0.37..0.27, y +-0.23) for the start creep and scan
        # wall checks on the SLAM map, so the robot can start with its body 0.15 m from a wall.
        p('footprint_back', 0.37)
        p('footprint_front', 0.27)
        p('footprint_half_width', 0.23)
        p('turn_margin', 0.03)         # m kept clear of walls while turning on the spot
        # Point (base_link frame) the robot really turns about: median of 6 in-place turns in
        # the 11-15 bags (maze2, run1), fit rms 0.5-2.5 cm. All turn checks rotate the
        # footprint about it (core.pivot_pose). 0, 0 = about base_link (the old model).
        p('pivot_x', 0.025)            # m ahead of base_link
        p('pivot_y', -0.04)            # m left of base_link (negative = right)
        p('pre_turn', True)            # turn the robot ourselves when Nav2's turn would hit a wall
        p('pre_turn_min_angle', 0.6)   # rad; = RPP rotate_to_heading_min_angle (nav2_params.yaml)
        p('pre_turn_max_creep', 0.6)   # m straight ahead to find room when blocked both ways; 0 = off
        p('creep_margin', 0.05)        # m kept clear of walls ahead at the end of the creep
        p('goal_max_offset', 7.5)      # m in x and y from the robot; global costmap is 16 x 16 rolling
        p('goal_grace', 3)             # replans the current goal's frontier may be missing
        #                                before switching goals (a blink isn't a switch)
        p('nav_down_timeout', 5.0)     # s of Nav2 reported inactive before giving up (STOPPED)
        p('stairs_topic', '/stairs/status')   # stair_detector handshake; '' = ignore stairs
        p('stairs_timeout', 60.0)      # s in STAIRS (stair_detector driving) before exploring on
        # ...restarted once stair_detector reaches APPROACHING, with this many s: checking and
        # aligning used 34 of the 60 s in the 2026-11-21 sim and the approach was cut off.
        p('stairs_approach_timeout', 60.0)
        # Back of the start cell (user 2026-11-20): no frontier goal, junction look or
        # junction follow-up into the start cell or behind the start pose, up to
        # start_back_depth m back and start_back_width m either side of its line. 0 = off.
        p('start_back_depth', 1.5)
        p('start_back_width', 0.6)
        # Candidate queue (user 2026-11-21): possible stairs / curtains seen in a junction or
        # dead-end look are held by their detector (/stairs/candidate, /curtain/candidate) and
        # queued here. Once a junction scan is done (or nothing else is left to explore) the
        # robot goes to the nearest, *_approach m in front of it, faces it and releases it
        # (/maze/stairs_release, /maze/curtain_release) for the check; then the next. '' = off.
        p('stairs_candidate_topic', '/stairs/candidate')
        p('stairs_release_topic', '/maze/stairs_release')
        p('stairs_approach', 1.2)
        p('curtain_candidate_topic', '/curtain/candidate')
        p('curtain_release_topic', '/maze/curtain_release')
        p('curtain_check_approach', 1.2)
        p('candidate_wait', 15.0)      # s after a release for the detector to take over
        # Occupied cells (user 2026-11-21): rooms may hold objects (footprint >= 0.3 x 0.3 m);
        # the robot doesn't enter a cell with one. A free-standing blob of map walls inside a
        # cell (all of it more than object_border m from its edges, not joined to any wall:
        # walls off the grid lines are not objects; >= object_min_extent m across) on
        # object_confirm maps in a row blocks the cell for good: drawn as wall on the
        # explorer's map and /maze/map (Nav2), so no goal or path goes into it. Cells: the
        # scene_trigger grid (cell_size m, cell (0, 0) on the map origin, along its x axis).
        p('block_objects', True)
        p('cell_size', 1.02)
        # Maze size (user 2026-11-22): maze_cols cells along the start heading (+i) by
        # maze_rows to its left (+j), start cell (0, 0) in the corner. The robot never leaves
        # it: no frontier goal outside it; once everything inside is explored, an outer side
        # open on the final map (and on >= exit_confirm replans) is the exit, and the robot parks in the cell inside
        # the exit, facing out, standing (AT_EXIT: the drone's launch cell, /maze/launch_cell).
        # Within launch_tolerance m of the cell centre, else up to launch_retries goals.
        # The open-floor exit check only ends the run if the robot is still more than
        # exit_cell_margin m outside the grid. 0 = size unknown: old exit behaviour.
        p('maze_cols', 5)
        p('maze_rows', 3)
        p('exit_cell_margin', 0.15)
        p('exit_confirm', 3)
        p('launch_tolerance', 0.25)
        p('launch_retries', 3)
        # Nav2 ignores the goal heading (yaw_goal_tolerance 6.28): turn to face the exit
        # ourselves, up to 2 turns, until within this.
        p('launch_yaw_tolerance', math.radians(10))
        # Grid from the walls (user 2026-11-22): the start pose only roughly lines the grid
        # up with the real cells (5 deg off = 0.4 m at the far end of 5 cells). Every
        # grid_fit_period s the grid is refitted to the wall pixels near the maze
        # (scene_cells.fit_grid): at most grid_fit_max_yaw / grid_fit_max_shift from the
        # start-pose grid, only with >= grid_fit_min_points pixels lining up to a score of
        # >= grid_fit_min_score (0..2). Published on /maze/grid for scene_trigger.
        p('grid_fit', True)
        p('grid_fit_period', 5.0)
        p('grid_fit_max_yaw', math.radians(8.0))
        p('grid_fit_max_shift', 0.35)
        p('grid_fit_min_points', 150)
        p('grid_fit_min_score', 0.6)
        p('object_border', 0.12)
        p('object_min_extent', 0.2)
        p('object_min_pixels', 4)
        p('object_confirm', 3)
        # Room look-in (user 2026-11-21): an object cell is a room (the object, e.g. a toy
        # kitchen, marks its type; survivor posters and QR codes hang on its walls). From the
        # nearest reached neighbour with an open side into it: drive to that cell's centre,
        # square up facing in, hold room_look_hold s, then its left and right walls (Pose
        # mode), perception on, one /scene/capture per view - never going in. Started from
        # EXPLORING within room_look_radius m; at most room_look_max per run; none within
        # room_stair_radius m of stairs (/stairs/pose: stairs show up as objects too).
        p('room_looks', True)
        p('room_look_radius', 3.0)
        p('room_look_hold', 3.0)
        p('room_look_max', 6)
        p('room_stair_radius', 2.0)
        # Nav2 KeepoutFilter (nav2_params.yaml, both costmaps): the painted object cells as a
        # mask, so the controller and spin / wait recoveries treat them as lethal too, not only
        # the planner (/maze/map). '' = off.
        p('keepout_mask_topic', '/maze/keepout_mask')
        p('keepout_info_topic', '/maze/costmap_filter_info')
        # What a confirmed object blocks (presentation mazes 2026-11-21: the object fills about
        # half a room, and the posters on the room's side walls are at a steep angle from the
        # doorway). 'footprint': only the object's box (occupied map cells inside the cell,
        # union over maps, >= 0.3 m a side) grown by object_keepout_margin, on the explorer's
        # map, /maze/map and the keepout mask: the robot may step into the free half (and a
        # dead end there gets the usual dead-end look from inside). 'cell': the whole cell,
        # never entered (the 2026-11-21 morning rule; set this if entering is penalised).
        p('object_keepout', 'footprint')
        p('object_keepout_margin', 0.25)   # m: Nav2 doesn't inflate keepout-filter cells
        # No Pose-mode looks while standing on rough ground (stair_detector /maze/terrain
        # on_rough, fresh within 2 s): a held body twist on a block field (presentation maze
        # 2026-11-21: the room is looked into from one); Nav2 spins instead. '' = off.
        p('terrain_topic', '/maze/terrain')
        # QR codes seen but unreadable (qr_scanner /qr/unread): turn to face them until read
        # (/qr/read). '' = ignore; maze_nav.launch.py blanks both when perception:=false.
        p('qr_unread_topic', '/qr/unread')
        p('qr_read_topic', '/qr/read')
        p('face_timeout', 10.0)        # s facing a code without a read before giving up on it
        p('face_lost', 4.0)            # s with no unread sighting: code gone (or was noise)
        p('face_min_angle', math.radians(8))   # rad: closer than this = already facing it
        p('face_max_turns', 3)         # re-aims while it drifts off-centre
        p('face_cooldown', 60.0)       # s: ignore a code given up on (within 0.75 m)
        # scene_trigger: junctions (face each way, left to right) and dead ends (stop, never
        # go in). '' = ignore.
        p('junction_topic', '/scene/junction')
        p('junction_hold', 1.0)        # s facing (scanning) each adjacent unexplored cell
        # Junction centring (user 2026-11-21): after turning to face a way, move sideways onto
        # the junction cell's centre line along it (linear.y on nav_cmd_vel), then scan.
        p('junction_centre', True)
        p('centre_speed', 0.06)        # m/s sideways (cmd_vel_bridge max_lateral 0.10)
        p('centre_tol', 0.03)          # m off the centre line = centred
        p('centre_max', 0.35)          # m: never strafe further than this
        p('centre_timeout', 10.0)      # s per way
        p('junction_timeout', 3.0)     # s: drop a junction request not started by then
        p('junction_radius', 0.35)     # m from the cell centre to still do the look
        p('false_success_dist', 0.5)   # m: a "reached" goal this far from the robot = failed
        p('scan_topic', '/wall_scan')  # freshness only: no NO_EXIT while it is stale; '' = off
        p('scan_stale', 1.0)           # s without a scan = stale
        # Start gate: the run (creep / scan / explore) starts only once scan_topic has come
        # without a gap > start_gap s for start_steady s. wall_scan publishes only when the
        # odometry is current, so this also waits out late lidar / odom from the bridge
        # (2026-11-19: /maze/start sent while odom was ~1 s late -> Nav2 TF errors). 0 = off.
        p('start_steady', 3.0)
        p('start_gap', 0.5)
        p('junction_follow', True)     # after the scan: queued candidates, else the nearest way
        p('junction_dead_end_hold', 3.0)   # s facing into a dead end from the cell before
        p('junction_trace_cells', 10)  # cells followed along each way looking for its end
        p('junction_align_tol', math.radians(3))   # rad: squared up to the walls within
        p('dead_end_topic', '/scene/trigger')
        p('dead_end_hold', 0.5)        # s holding still facing a dead end
        p('dead_end_side_hold', 1.0)   # s holding at each of its side walls
        p('dead_end_look_angle', math.radians(60))   # rad: side-wall turn at most this off
        # curtain_detector: map override + walk-through handshake. '' = ignore curtains.
        p('curtain_topic', '/curtain/status')
        p('curtains_topic', '/curtain/curtains')
        p('curtain_timeout', 150.0)    # s in CURTAIN (curtain_detector driving) before going on
        p('curtain_approach', 0.9)     # m in front of a curtain's middle for goals behind it
        p('curtain_goal_radius', 0.7)  # m: goals this close to a curtain go to its front first
        p('curtain_spot_wait', 15.0)   # s in front of a curtain for curtain_detector, then Nav2
        # Pose-mode looks (body yaw only, feet still) for the start scan and the dead-end
        # side walls; cmd_vel_bridge sends the mode + axis commands. False = Nav2 spins.
        p('pose_looks', True)
        p('pose_yaw_topic', '/maze/pose_yaw')
        p('pose_status_topic', '/maze/pose_status')
        p('pose_max_yaw', math.radians(16))   # rad; = cmd_vel_bridge pose_max_yaw
        # = cmd_vel_bridge pose_axis_limit: the bridge never sends more than this fraction of
        # the full yaw axis, so the most body yaw asked is pose_max_yaw * pose_axis_limit
        # (motor buzzing at full axis in a tight cell, run 2026-11-21 10:56).
        p('pose_axis_limit', 0.8)
        # m kept clear of walls by the body box around a Pose-mode yaw (vs turn_margin 0.03
        # for a whole-robot turn): the feet are planted, so a body that touches a wall can't
        # slip away and the motors stall against it. Also checked at the stance first.
        p('pose_margin', 0.08)
        p('pose_overshoot', math.radians(3))  # rad of extra sweep checked past the yaw asked
        p('pose_turn_time', 1.0)       # s after POSING for the body to reach the yaw
        p('pose_wait', 6.0)            # s for the bridge to reach POSING before skipping
        p('scan_hold', 1.5)            # s per side of a pose-mode start scan (unused: start scan spins)
        p('pose_pitch_topic', '/maze/pose_pitch')   # '' = no tilt compensation
        p('pose_max_pitch', math.radians(20))   # rad; = cmd_vel_bridge pose_max_pitch (NOT measured)
        # Tilt compensation at dead ends (user 2026-11-21): body pitched more than
        # tilt_threshold away from its pitch at /maze/start (uneven floor) -> each dead-end
        # view is a Pose-mode look with the body pitched back by that much, clamped to
        # pose_max_pitch. Pitch from TF odom -> base_link (Fast-LIO), averaged tilt_window s.
        p('tilt_comp', True)
        p('tilt_threshold', math.radians(30))
        p('tilt_window', 0.5)
        p('camera_frame', 'camera_link')   # its pitch in odom goes into /scene/capture
        # One message per dead-end view (end of its hold) for room_scene; '' = off.
        p('scene_capture_topic', '/scene/capture')
        # Dead ends wait for scene understanding (user 2026-11-22): a dead end seen in the cell
        # in front is held out of goals, not blacklisted, until room_scene's /scene/result
        # for it: room (object) -> blacklisted; stairs / curtain -> candidate queue (checked
        # after the junctions); anything else -> open again. No answer scene_result_timeout s
        # after its last capture -> re-check on the map (object in it -> blacklisted; not a
        # dead end any more -> open), else /scene/retry once (scene_max_attempts tries in
        # all); the last try unanswered -> blacklisted. Never captured within
        # scene_look_grace s -> blacklisted. '' (result topic) = blacklist at once (old way).
        p('scene_result_topic', '/scene/result')
        p('scene_retry_topic', '/scene/retry')
        p('scene_result_timeout', 15.0)
        p('scene_max_attempts', 2)
        p('scene_look_grace', 30.0)
        p('map_dir', os.path.expanduser('~/challenge_ws/maps'))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in ('go', 'start_delay', 'wait_for_start', 'robot_radius', 'goal_clearance',
                                      'min_frontier_size', 'robot_width', 'min_frontier_width_ratio', 'goal_timeout', 'goal_progress', 'goal_max_time', 'max_failures', 'done_confirm', 'max_recoveries',
                                      'max_runtime', 'map_dir', 'creep_distance', 'creep_speed',
                                      'start_scan', 'scan_angle', 'footprint_back',
                                      'footprint_front', 'footprint_half_width', 'turn_margin',
                                      'pivot_x', 'pivot_y',
                                      'pre_turn', 'pre_turn_min_angle', 'pre_turn_max_creep',
                                      'creep_margin', 'goal_max_offset', 'goal_grace',
                                      'nav_down_timeout', 'stairs_timeout', 'stairs_approach_timeout',
                                      'open_space_no_looks', 'face_timeout', 'face_lost',
                                      'face_min_angle', 'face_max_turns', 'face_cooldown',
                                      'junction_hold', 'junction_timeout', 'junction_radius',
                                      'junction_centre', 'centre_speed', 'centre_tol',
                                      'centre_max', 'centre_timeout',
                                      'curtain_check_approach', 'candidate_wait',
                                      'false_success_dist', 'scan_stale', 'start_steady',
                                      'start_gap',
                                      'junction_follow', 'junction_dead_end_hold',
                                      'junction_trace_cells', 'junction_align_tol',
                                      'dead_end_hold', 'dead_end_side_hold',
                                      'dead_end_look_angle', 'curtain_timeout', 'curtain_approach',
                                      'curtain_goal_radius', 'curtain_spot_wait', 'pose_looks', 'pose_max_yaw',
                                      'pose_axis_limit', 'pose_margin', 'pose_overshoot',
                                      'start_back_depth', 'start_back_width',
                                      'stairs_approach',
                                      'block_objects', 'cell_size', 'object_border',
                                      'maze_cols', 'maze_rows', 'exit_cell_margin',
                                      'exit_confirm', 'launch_tolerance', 'launch_retries',
                                      'launch_yaw_tolerance', 'grid_fit', 'grid_fit_period',
                                      'grid_fit_max_yaw', 'grid_fit_max_shift',
                                      'grid_fit_min_points', 'grid_fit_min_score',
                                      'object_min_extent', 'object_min_pixels', 'object_confirm',
                                      'object_keepout', 'object_keepout_margin',
                                      'room_looks', 'room_look_radius', 'room_look_hold',
                                      'room_look_max', 'room_stair_radius',
                                      'pose_turn_time', 'pose_wait', 'scan_hold',
                                      'pose_max_pitch', 'tilt_comp', 'tilt_threshold',
                                      'tilt_window', 'camera_frame',
                                      'scene_result_timeout', 'scene_max_attempts',
                                      'scene_look_grace')}
        self.exit = ExitDetector(open_radius=g('exit_open_radius'),
                                 near_radius=g('exit_near_radius'),
                                 min_displacement=g('exit_min_displacement'),
                                 dwell=g('exit_dwell'))

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        # When a Nav2 server dies, the lifecycle manager deactivates the rest and every goal
        # then fails at once. Those failures say nothing about the maze, so they must not
        # blacklist frontiers (which ended a run as NO_EXIT 6 s after planner_server crashed).
        self.nav_check = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.clear_costmaps = [
            self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')]
        self.status_pub = self.create_publisher(String, '/maze/status', LATCHED)
        # Perception window (perceive_tick): QR / survivor checks only at dead ends.
        self.perceive_pub = self.create_publisher(Bool, '/maze/perceive', LATCHED)
        self.perceive = None
        self.goal_pub = self.create_publisher(PoseStamped, '/maze/goal', LATCHED)
        self.hold_pub = self.create_publisher(String, '/maze/curtain_hold', LATCHED)
        self.launch_pub = self.create_publisher(String, '/maze/launch_cell', LATCHED)
        self.grid_pub = self.create_publisher(String, '/maze/grid', LATCHED)
        self.curtain_map_pub = self.create_publisher(OccupancyGrid, '/maze/map', LATCHED)
        self.keepout_pub, self.keepout_cells = None, None
        if g('keepout_mask_topic') and g('keepout_info_topic'):
            self.keepout_pub = self.create_publisher(OccupancyGrid, g('keepout_mask_topic'),
                                                     LATCHED)
            info = CostmapFilterInfo(type=0, filter_mask_topic=g('keepout_mask_topic'),
                                     base=0.0, multiplier=1.0)
            info.header.frame_id = 'map'
            self.keepout_info_pub = self.create_publisher(CostmapFilterInfo,
                                                          g('keepout_info_topic'), LATCHED)
            self.keepout_info_pub.publish(info)
        # Same input as Nav2's velocity_smoother output, so the stop goes through
        # cmd_vel_bridge (clamps, drive:=false preview) like any other command.
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd_vel', 10)
        self.scene_capture_pub = (self.create_publisher(String, g('scene_capture_topic'), 10)
                                  if g('scene_capture_topic') else None)
        self.scene_results = bool(g('scene_result_topic')) and self.scene_capture_pub is not None
        self.scene_retry_pub = None
        if self.scene_results:
            self.create_subscription(String, g('scene_result_topic'), self.on_scene_result, 10)
            if g('scene_retry_topic'):
                self.scene_retry_pub = self.create_publisher(String, g('scene_retry_topic'), 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        # Volatile on purpose: a start sent before this run is never replayed.
        self.create_subscription(Empty, '/maze/start', self.on_start, 10)
        self.stair_spots = []         # (x, y) odom: first risers of confirmed stairs
        if g('stairs_topic'):
            self.create_subscription(String, g('stairs_topic'), self.on_stairs, LATCHED)
        self.stairs_release_pub = self.create_publisher(
            String, g('stairs_release_topic'), 10) if g('stairs_release_topic') else None
        if g('stairs_candidate_topic') and g('stairs_topic'):
            self.create_subscription(String, g('stairs_candidate_topic'),
                                     self.on_stairs_candidate, 10)
            self.create_subscription(PoseStamped, '/stairs/pose', self.on_stair_pose, LATCHED)
        self.curtain_release_pub = self.create_publisher(
            String, g('curtain_release_topic'), 10) if g('curtain_release_topic') else None
        if g('curtain_candidate_topic') and g('curtain_topic'):
            self.create_subscription(String, g('curtain_candidate_topic'),
                                     self.on_curtain_candidate, 10)
        self.ai_done_at = -1e9       # monotonic time cmd_vel_bridge last said AI mode DONE
        self.create_subscription(String, '/maze/ai_status', self.on_ai_status, LATCHED)
        if g('qr_unread_topic'):
            self.create_subscription(PointStamped, g('qr_unread_topic'), self.on_qr_unread, 10)
            if g('qr_read_topic'):
                self.create_subscription(String, g('qr_read_topic'), self.on_qr_read, 10)
        if g('curtain_topic'):
            self.create_subscription(String, g('curtain_topic'), self.on_curtain, LATCHED)
        if g('curtains_topic'):
            self.create_subscription(String, g('curtains_topic'), self.on_curtains, LATCHED)
        if g('junction_topic'):
            self.create_subscription(String, g('junction_topic'), self.on_junction, 10)
        self.scan_topic, self.scan_at = g('scan_topic'), None
        self.scan_steady_since = None   # first scan of the current run without big gaps
        if self.scan_topic:
            # raw: only "a scan arrived" is needed, not its ranges
            self.create_subscription(LaserScan, self.scan_topic, self.on_scan_tick, 10,
                                     raw=True)
        if g('dead_end_topic'):
            self.create_subscription(String, g('dead_end_topic'), self.on_dead_end, 10)
        self.create_timer(g('replan_period'), self.step)
        self.create_timer(0.1, self.scene_tick)   # junction holds are 0.5 s: faster than step
        self.create_timer(0.1, self.creep_tick)   # faster than cmd_vel_bridge's idle_timeout
        self.create_timer(0.2, self.perceive_tick)
        self.create_timer(1.0, self.scene_timeout_tick)
        self.create_timer(5.0, self.check_map_subscribers)
        self.pose = None             # running pose look {'yaw', 'hold', 'sent', 'since'}
        self.pose_live = False       # a pose target was sent and not released (NaN) yet
        self.pose_status, self.pose_status_at = None, -1e9
        self.on_rough, self.terrain_at = False, -1e9    # stair_detector /maze/terrain
        if g('terrain_topic'):
            self.create_subscription(String, g('terrain_topic'), self.on_terrain, 10)
        self.pose_pub = None
        self.pose_pitch_pub = None   # dead-end tilt compensation (cmd_vel_bridge pitch axis)
        if g('pose_looks') and g('pose_yaw_topic') and g('pose_status_topic'):
            self.pose_pub = self.create_publisher(Float32, g('pose_yaw_topic'), 10)
            if g('pose_pitch_topic'):
                self.pose_pitch_pub = self.create_publisher(Float32, g('pose_pitch_topic'), 10)
            self.create_subscription(String, g('pose_status_topic'), self.on_pose_status,
                                     LATCHED)
            self.create_timer(0.1, self.pose_tick)
        self.tilt_samples = collections.deque(maxlen=50)   # (monotonic, pitch, roll) rad
        self.tilt_tf_stamp, self.tilt_tf_at = None, 0.0
        self.tilt_ref, self.tilt_ref_pending = None, False   # body pitch at /maze/start

        self.map = None
        self.first_map = None
        self.started = None
        self.start_requested = False
        self.state = None
        self.current = None          # Frontier being driven to
        self.current_missing = 0     # replans in a row its frontier wasn't found
        self.goal_handle = None
        self.goal_sent_at = None
        self.goal_moved = None       # (xy, monotonic) of the current goal's last progress
        self.goal_result = None      # None while running, else GoalStatus code
        self.blacklist = []          # reached goals: never again
        self.failed = []             # failed goals: set aside until the next recovery
        self.recoveries = 0
        self.failures = 0
        self.empty_replans = 0       # replans in a row that found no frontier
        self.box = (self.cfg['footprint_back'], self.cfg['footprint_front'],
                    self.cfg['footprint_half_width'])
        self.pivot = (float(self.cfg['pivot_x']), float(self.cfg['pivot_y']))
        self.scan_turns = None       # planned at the start of SCANNING, from the map
        self.scan_pose = False       # scan_turns are pose-mode body yaws, not spins
        self.creep_target = None     # m the creep actually drives (shortened near a wall)
        self.spin_busy = False       # scan turn sent, result not back yet
        self.spin_handle = None
        self.pre_turned = None       # goal (x, y) the last pre-turn was for: once per goal
        self.turn_creep = None       # TURNING: creeping ahead to room for the turn {goal, ...}
        self.creep_from = None       # (x, y) in map where the creep started
        self.start_pose = None       # ((x, y), yaw) in map at /maze/start: back_to_start()
        self.cells = CellGrid(float(self.cfg['cell_size']))
        self.cells0 = CellGrid(self.cells.size)   # from the start pose: fit_cells' limits
        self.grid_fit_at = -1e9
        self.object_seen = {}        # cell -> maps in a row with an object in it
        self.blocked_cells = {}      # cell -> (extent, (x, y)): objects, never entered
        self.paint_cells = set()     # blocked cells drawn as wall (not the one the robot is in)
        self.object_boxes = {}       # blocked cell -> (x0, y0, x1, y1) map box of its object
        self.objects_map = None      # /map message the objects were last looked for on
        self.dead_end_cells = set()  # checked dead ends (self.cells): never a goal into them
        # dead ends waiting for scene understanding: cell -> {'centre', 'heading', 'attempt',
        # 'since', 'captured'} (monotonic times); no goal into them meanwhile
        self.scene_pending = {}
        self.exit_side = None        # (cell inside, d out): confirmed exit on the grid's edge
        self.exit_open = {}          # outer side -> maps in a row it read open
        self.launch = None           # TO_EXIT: the launch cell goal (go_to_exit)
        self.room_looks = {}         # object cell -> (x, y) of the object: look in from outside
        self.rooms_done = set()      # object cells looked into (or given up)
        self.room_looks_started, self.room_look_at = 0, -1e9
        self.creep_until = None      # monotonic deadline for the creep
        self.goal_result_at = None   # monotonic time goal_result arrived
        self.nav_query = None        # (future, sent_at) of the pending is_active call
        self.nav_reply_at = -1.0     # sent_at of the newest is_active answer
        self.nav_active = True
        self.nav_down_since = None
        self.stairs_since = None     # monotonic time STAIRS (or its approach) began
        self.stairs_approaching = False   # stair_detector reached APPROACHING this STAIRS
        self.resume_requested = False   # /maze/start while AT_STAIRS
        self.at_stairs_since = None  # monotonic time AT_STAIRS began
        self.cand_queue = []         # candidates to check: {'kind', 'id', 'target', 'heading'}
        self.queue_active = False    # working the queue (after a junction scan / at the end)
        self.queue_wait = None       # (candidate, monotonic time) released, detector's turn
        self.visited = set()         # self.cells cells the robot has stood in
        self.scanned_cells = set()   # cells viewed from a junction scan (scene understanding)
        self.explored_cells = set()  # object rooms seen in a junction scan: no goal into them
        self.unread = None           # newest /qr/unread: (PointStamped in odom, monotonic time)
        self.qr_read_at = -1e9       # monotonic time of the newest /qr/read
        self.face = None             # FACING_QR: {'since', 'turns'}
        self.face_given_up = []      # [((x, y) in map, monotonic time)]: codes not read in time
        self.junction_req = None     # newest /scene/junction: (record, monotonic time)
        self.junction = None         # JUNCTION / DEAD_END look: {'steps', 'i', 'turned', ...}
        self.junction_goal = None    # Nav2 goal handle of a junction follow-up drive
        self.side_walls_warned = False
        self.curtains = []           # curtains.Curtain in curtains_frame (curtain_detector)
        self.curtains_frame = 'odom'
        self.curtain_since = None    # monotonic time CURTAIN began
        self.curtain_status = None   # curtain_detector's state (/curtain/status)
        self.curtain_left_at = None  # monotonic time CURTAIN / AT_CURTAIN last ended
        self.redirects = {}          # curtain approach goal -> the frontier goal behind it
        self.inspecting = False      # CURTAIN for an inspection (not a crossing)
        self.paused_junction = None  # JUNCTION look paused for an inspection
        self.defers = []             # curtain_branch.Deferral: other junction ways first
        self.hold = None             # last CurtainHold published
        self.branch_cache = None     # (map msg id, curtain id, mask)
        self.spot_wait = None        # (curtain id, monotonic time) waiting in front of one
        self.map_sub_ok = False      # /maze/map has a subscriber (startup guard)
        self.attach_live_params()
        self.set_state('STARTING', 'waiting for /map, TF map->base_link and Nav2')

    def attach_live_params(self):
        """`ros2 param set /maze_explorer <name> <value>` while running (touble_shooting).
        cfg entries apply at once; the few copied at startup get setters below."""
        c = self.cfg

        def box(_):
            self.box = (c['footprint_back'], c['footprint_front'], c['footprint_half_width'])

        def pivot(_):
            self.pivot = (float(c['pivot_x']), float(c['pivot_y']))

        def exit_attr(attr):
            return lambda v: setattr(self.exit, attr, float(v))
        setters = {'footprint_back': box, 'footprint_front': box, 'footprint_half_width': box,
                   'pivot_x': pivot, 'pivot_y': pivot,
                   'exit_open_radius': exit_attr('open_radius'),
                   'exit_near_radius': exit_attr('near_radius'),
                   'exit_min_displacement': exit_attr('min_displacement'),
                   'exit_dwell': exit_attr('dwell')}
        # cell_size: the object / dead-end cell grid is built once; map_dir: files open
        attach_live_params(self, cfgs=[c], setters=setters, locked=('cell_size', 'map_dir'))

    # ------------------------------------------------------------------ helpers
    def set_state(self, state, why=''):
        if state != self.state:
            if self.state in CURTAIN_STATES and state not in CURTAIN_STATES:
                self.curtain_left_at = time.monotonic()   # held junction requests age now
            self.state = state
            self.get_logger().info(f'[{state}] {why}')
            self.status_pub.publish(String(data=f'{state}: {why}' if why else state))

    def on_map(self, msg):
        self.map = msg
        self.publish_curtain_map()
        if self.first_map is None:
            self.first_map = time.monotonic()

    def on_start(self, _msg):
        if self.state == 'PAUSED':
            self.start_requested = True
            self.get_logger().info('start received')
            if self.cfg['tilt_comp']:
                self.set_tilt_ref()
        elif self.state == 'AT_STAIRS':
            self.resume_requested = True
            self.get_logger().info('start received: exploring on from the stairs')
        elif self.state == 'AT_CURTAIN':
            self.resume_requested = True
            self.clear_local_costmap('start at the curtain')
            self.get_logger().info('start received: exploring on from the curtain')
        elif self.state == 'STARTING':
            self.get_logger().warn('start ignored: not ready yet (wait for PAUSED)')

    def on_stairs(self, msg):
        """stair_detector's /stairs/status (see the module doc)."""
        stairs = msg.data.split(':', 1)[0].strip()
        if stairs == 'HANDOVER' and \
                self.state in ('EXPLORING', 'TURNING', 'FACING_QR', 'JUNCTION', 'DEAD_END') \
                and self.cfg['go']:
            if self.spin_handle is not None:          # a turn, scan or look-around
                self.spin_handle.cancel_goal_async()
                self.spin_handle = None
            self.end_pose('stairs ahead')
            self.face = None                          # facing a QR code ends here
            self.junction = None                      # and a junction / dead-end look
            self.cancel_junction_goal()
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_handle, self.current, self.goal_result = None, None, None
            self.stop_robot()
            self.stairs_since = time.monotonic()
            self.stairs_approaching = False
            self.on_handover()
            self.set_state('STAIRS', 'stairs ahead: Nav2 stopped, stair_detector lines the '
                           'robot up')
        elif self.state == 'STAIRS':
            if stairs == 'APPROACHING' and not self.stairs_approaching:
                self.stairs_approaching = True          # its own timeout from here
                self.stairs_since = time.monotonic()
            if stairs == 'AT_STAIRS':
                self.resume_requested = False
                self.at_stairs_since = time.monotonic()
                why = msg.data.split(':', 1)[-1].strip()
                self.set_state('AT_STAIRS', f'{why}; ' + (
                    f'line it up by hand, then: {START_CMD}' if 'NOT LINED UP' in why else
                    'cmd_vel_bridge walks over them in AI mode, then exploring on by itself'))
            elif stairs in ('IDLE', 'ABORTED'):
                self.begin_after_stairs(f'stair_detector {msg.data}')

    def on_curtain(self, msg):
        """curtain_detector's /curtain/status (see the module doc)."""
        cur = msg.data.split(':', 1)[0].strip()
        self.curtain_status = cur
        inspect = cur == 'HANDOVER' and 'possible curtain' in msg.data
        if self.curtain_preemptable() and self.junction_pending() and self.cfg['go']:
            rec = self.junction_req[0]               # held through the crossing's start
            return self.preempt_curtain(f'junction at cell {rec.get("cell")}')
        if inspect and self.cfg['go'] and \
                self.state in ('EXPLORING', 'TURNING', 'FACING_QR', 'JUNCTION'):
            self.take_for_curtain(True, 'possible curtain ahead: Nav2 stopped, curtain_detector '
                                  'inspects it first' + (' (junction look paused)'
                                                         if self.state == 'JUNCTION' else ''))
        elif cur == 'HANDOVER' and not inspect and self.cfg['go'] and (
                self.state in ('JUNCTION', 'DEAD_END', 'SCANNING') or self.junction_pending()):
            first = 'a junction request' if self.junction_pending() else self.state
            self.get_logger().info(f'curtain ahead, but {first} comes first: curtain later')
        elif cur == 'HANDOVER' and not inspect and self.cfg['go'] and \
                self.state in ('EXPLORING', 'TURNING', 'FACING_QR'):
            self.take_for_curtain(False, 'curtain ahead: Nav2 stopped, curtain_detector walks '
                                  'the robot through')
        elif self.state == 'CURTAIN':
            if cur == 'STUCK':
                self.resume_requested = False
                self.set_state('AT_CURTAIN', f'{msg.data.split(":", 1)[-1].strip()}; holding '
                               f'still until: {START_CMD}')
            elif cur == 'CONFIRMED':
                self.defer_curtain(msg.data)
                self.end_curtain(f'curtain_detector {msg.data}')
            elif cur in ('PASSED', 'IDLE', 'ABORTED'):
                if cur == 'PASSED':
                    self.clear_local_costmap('through a curtain', global_too=True)
                self.end_curtain(f'curtain_detector {msg.data}')

    def take_for_curtain(self, inspect, why):
        """Stop everything for curtain_detector: an inspection (pauses a junction look) or a
        crossing."""
        if self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()
            self.spin_handle = None
        self.end_pose('curtain ahead')
        self.face = None
        if inspect and self.state == 'JUNCTION' and self.junction is not None:
            self.pause_junction()
        self.junction = None
        self.cancel_junction_goal()
        self.turn_creep = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.stop_robot()
        self.inspecting = inspect
        self.on_handover()
        if inspect and self.junction_pending():
            self.junction_req[0]['held'] = True     # after the inspection
        self.curtain_since = time.monotonic()
        self.set_state('CURTAIN', why)
        self.publish_hold()

    def pause_junction(self):
        """Keep the JUNCTION look for after the inspection: the step being turned to (or
        held at) is done again, a follow-up action is sent again."""
        j = self.junction
        if j.get('turned') and j['i'] > 0 and j['i'] <= len(j['steps']):
            j['i'] -= 1
        j['turned'], j['hold_until'] = False, None
        j.pop('posed', None)
        if j.get('todo'):
            for k in ('sent', 'status', 'until'):
                j['todo'][0].pop(k, None)
        self.paused_junction = j

    def end_curtain(self, why):
        """CURTAIN / AT_CURTAIN over: back to a paused junction look, else EXPLORING."""
        self.inspecting = False
        j, self.paused_junction = self.paused_junction, None
        if j is not None:
            self.junction = j
            self.set_state('JUNCTION', f'{why}; back to the junction look')
        else:
            self.begin_after_stairs(why)
        self.publish_hold()

    def publish_hold(self):
        """/maze/curtain_hold: hold new curtains while inspecting at a junction; the
        deferred curtains' ids until their junction's other ways are done."""
        hold = CurtainHold(
            hold_new=self.state in CURTAIN_STATES and self.inspecting and
            (self.paused_junction is not None or self.junction_pending()),
            ids=[d.curtain_id for d in self.defers if not d.released])
        if self.hold is None or hold.to_json() != self.hold.to_json():
            self.hold = hold
            self.hold_pub.publish(String(data=hold.to_json()))

    def curtain_by_id(self, cid):
        for c in self.curtains_in_map():
            if c.id == cid:
                return c
        return None

    def defer_curtain(self, text):
        """CONFIRMED at a junction: hold the curtain, explore the junction's other ways first."""
        m = re.search(r'curtain (\d+)', text)
        if m is None:
            self.get_logger().warn(f'CONFIRMED without a curtain id: {text[:120]}')
            return
        cid = int(m.group(1))
        j = self.paused_junction
        rec = j.get('rec') if j is not None else None
        if rec is None and self.junction_req is not None:
            rec = self.junction_req[0]
        if rec is None or rec.get('cells') is None:
            self.get_logger().info(f'curtain {cid}: no junction to explore first; crossing it')
            return
        cells, cell = rec['cells'], rec['cell']
        cur = self.curtain_by_id(cid)
        pose = self.robot_pose()
        if cur is not None:
            target = (float(cur.centre[0]), float(cur.centre[1]))
        elif pose is not None:                       # squared up facing it
            (x, y), yaw = pose
            target = (x + math.cos(yaw), y + math.sin(yaw))
        else:
            return
        dirs = [DIRECTIONS[n] for n, _ in rec['paths'] if n in DIRECTIONS]
        if not dirs:
            return
        cway = way_towards(cells, cell, dirs, target)
        others = [d for d in dirs if d != cway and
                  not self.back_to_start(cells.centre((cell[0] + d[0], cell[1] + d[1])))]
        rec['skip_dirs'] = list(rec.get('skip_dirs') or []) + [cway]
        if cur is not None:                          # its "dead end" is the way on
            self.blacklist = [b for b in self.blacklist if math.hypot(
                b[0] - cur.centre[0], b[1] - cur.centre[1]) > 0.9]
        if not others:
            self.get_logger().info(f'curtain {cid}: junction {list(cell)} has no other way; '
                                   'crossing it')
            return
        self.defers.append(Deferral(cid, tuple(cell), cells, others))
        self.get_logger().info(
            f'curtain {cid} at junction {list(cell)} (way {cway}): exploring its other '
            f'way(s) {others} fully first, then crossing it')
        self.publish_hold()

    def to_map(self, xy, heading, frame):
        """(x, y), heading from a fixed frame into map (latest TF), or None."""
        if frame == 'map':
            return (float(xy[0]), float(xy[1])), float(heading)
        try:
            tf = self.tf.lookup_transform('map', frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return ((t.x + math.cos(th) * xy[0] - math.sin(th) * xy[1],
                 t.y + math.sin(th) * xy[0] + math.cos(th) * xy[1]), float(heading) + th)

    def queue_candidate(self, kind, cid, target, heading, what):
        if cid is not None and any(c['kind'] == kind and c['id'] == cid
                                   for c in self.cand_queue):
            return
        self.cand_queue.append({'kind': kind, 'id': cid, 'target': target,
                                'heading': heading})
        self.get_logger().info(
            f'possible {kind} {cid} at ({target[0]:.2f}, {target[1]:.2f}) ({what}): queued; '
            f'checked after the junction scan, nearest first ({len(self.cand_queue)} queued)')

    def on_stairs_candidate(self, msg):
        """stair_detector holds possible stairs seen in a junction / dead-end look: queue
        them (user 2026-11-21)."""
        try:
            d = json.loads(msg.data)
            sid, frame = int(d['id']), str(d.get('frame', 'odom'))
            edge, hd = (float(d['edge'][0]), float(d['edge'][1])), float(d['heading'])
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /stairs/candidate ({e}): {msg.data[:160]}')
            return
        m = self.to_map(edge, hd, frame)
        if m is None:
            return self.release('stairs', sid, f'no TF map -> {frame}; checking now')
        self.queue_candidate('stairs', sid, m[0], m[1], d.get('what', 'stairs'))

    def on_curtain_candidate(self, msg):
        """curtain_detector holds a possible curtain seen in a look: queue it."""
        try:
            d = json.loads(msg.data)
            cid, frame = int(d['id']), str(d.get('frame', 'odom'))
            centre = (float(d['centre'][0]), float(d['centre'][1]))
            hd = math.atan2(float(d['normal'][1]), float(d['normal'][0]))
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /curtain/candidate ({e}): {msg.data[:160]}')
            return
        m = self.to_map(centre, hd, frame)
        if m is None:
            return self.release('curtain', cid, f'no TF map -> {frame}; inspecting now')
        self.queue_candidate('curtain', cid, m[0], m[1], d.get('what', 'curtain'))

    def release(self, kind, cid, why):
        if cid is None:                      # not held by a detector (e.g. from /scene/result)
            self.get_logger().info(f'{kind} candidate: in front of it ({why})')
            return
        pub = self.stairs_release_pub if kind == 'stairs' else self.curtain_release_pub
        if pub is not None:
            pub.publish(String(data=json.dumps({'id': int(cid)})))
        self.get_logger().info(f'{kind} {cid}: released for the check ({why})')

    def on_ai_status(self, msg):
        if msg.data.startswith('DONE'):
            self.ai_done_at = time.monotonic()

    def start_queue(self, why):
        """Work the candidate queue from now on (nearest first), if there is one."""
        if not self.cand_queue or self.queue_active:
            return bool(self.cand_queue)
        self.queue_active = True
        self.get_logger().info(f'{why}: checking {len(self.cand_queue)} queued candidate(s), '
                               'nearest first')
        return True

    def candidate_spot(self, c):
        a = self.cfg['stairs_approach' if c['kind'] == 'stairs' else 'curtain_check_approach']
        h = c['heading']
        return (c['target'][0] - a * math.cos(h), c['target'][1] - a * math.sin(h))

    def queue_goal(self, grid, xy, yaw):
        """Queue active: the goal to the nearest candidate's spot ([Frontier]), None = wait
        (released, facing it, or no path field), [] = queue done."""
        now = time.monotonic()
        if self.queue_wait is not None:
            c, t = self.queue_wait
            if now - t < self.cfg['candidate_wait']:
                return None
            self.get_logger().warn(f'{c["kind"]} {c["id"]}: no check {self.cfg["candidate_wait"]:.0f}'
                                   ' s after the release; next candidate')
            self.queue_wait = None
        if not self.cand_queue:
            self.queue_active = False
            self.get_logger().info('candidate queue done: exploring on')
            return []
        field = path_field(grid, xy, self.cfg['robot_radius'])

        def dist(c):
            sx, sy = self.candidate_spot(c)
            if field is not None:
                g = grid.cell((sx, sy))
                if grid.contains(g) and math.isfinite(field[g[1], g[0]]):
                    return float(field[g[1], g[0]])
            return math.hypot(sx - xy[0], sy - xy[1]) + 100.0      # no path: last
        c = min(self.cand_queue, key=dist)
        spot = self.candidate_spot(c)
        if any(math.hypot(spot[0] - f[0], spot[1] - f[1]) < 0.4 for f in self.failed):
            self.cand_queue.remove(c)
            self.release(c['kind'], c['id'], 'the spot in front can\'t be reached; checking '
                         'from here if it is in range')
            self.queue_wait = (c, now)
            return None
        if math.hypot(spot[0] - xy[0], spot[1] - xy[1]) > 0.35:
            return [Frontier(goal=spot, yaw=c['heading'], path_length=0.0, size=1,
                             centroid=spot)]
        want = wrap(c['heading'] - yaw)
        if abs(want) > math.radians(10) and c.get('turned', 0) < 2 and \
                self.spin.server_is_ready():
            c['turned'] = c.get('turned', 0) + 1
            sign = 1 if want > 0 else -1
            ok = max_turn(grid, xy, yaw, self.box, sign, abs(want), self.cfg['turn_margin'],
                          pivot=self.pivot)
            if ok > math.radians(5):
                self.halt_for_look()
                self.set_state('TURNING', f'facing {c["kind"]} {c["id"]} to check it')
                self.send_spin(sign * ok, f'face {c["kind"]} {c["id"]}')
                return None
        self.halt_for_look()
        self.cand_queue.remove(c)
        a = self.cfg['stairs_approach' if c['kind'] == 'stairs' else 'curtain_check_approach']
        self.release(c['kind'], c['id'], f'{a:.1f} m in front, facing it')
        self.queue_wait = (c, now)
        return None

    def on_handover(self):
        """A detector took over (STAIRS / CURTAIN): the released candidate is in its hands."""
        self.queue_wait = None

    def branch_frontiers(self, grid, xy, frontiers):
        """With a deferred curtain: only frontiers down its junction's other ways; none left
        (done_confirm replans) -> release it and go to the spot in front of it. Returns the
        frontiers to choose from, or None to wait this replan."""
        def gone(d):
            c = self.curtain_by_id(d.curtain_id)
            return c is None or bool(c.passed)
        before = len(self.defers)
        self.defers = [d for d in self.defers if not gone(d)]
        if len(self.defers) != before:
            self.get_logger().info('deferred curtain crossed (or gone): exploring on')
            self.publish_hold()
        if not self.defers:
            return frontiers
        d = self.defers[-1]
        cur = self.curtain_by_id(d.curtain_id)
        name = f'curtain {d.curtain_id}'
        if not d.released:
            key = (id(self.map), name)
            if self.branch_cache is None or self.branch_cache[0] != key:
                mask = branch_mask(grid, d.cells, d.junction_cell, d.ways,
                                   [cur] if cur is not None else [])
                self.branch_cache = (key, mask)
            mask = self.branch_cache[1]
            inb = [f for f in frontiers if in_mask(mask, grid, f.goal)]
            if inb:
                d.empty = 0
                return inb
            d.empty += 1
            if d.empty < self.cfg['done_confirm']:
                self.get_logger().info(f'{name}: no frontier left down junction '
                                       f'{list(d.junction_cell)}\'s other ways ({d.empty}/'
                                       f'{self.cfg["done_confirm"]}); checking again')
                return None
            d.released = True
            self.get_logger().info(f'{name}: junction {list(d.junction_cell)}\'s other ways '
                                   'explored; going back to '
                                   + 'cross it')
            self.publish_hold()
        side = cur.side_of(xy)
        spot = cur.centre - side * cur.n * self.cfg['curtain_approach']
        if any(math.hypot(spot[0] - g[0], spot[1] - g[1]) < 0.4 for g in self.failed):
            self.get_logger().warn(f'curtain {d.curtain_id}: the spot in front of it can\'t '
                                   'be reached; exploring on')
            self.defers.remove(d)
            return frontiers
        if math.hypot(spot[0] - xy[0], spot[1] - xy[1]) < 0.35:
            now = time.monotonic()
            d.at_spot = d.at_spot or now
            if now - d.at_spot > 20.0:
                self.get_logger().warn(f'curtain {d.curtain_id}: no crossing after 20 s in '
                                       'front of it; exploring on')
                self.defers.remove(d)
                return frontiers
            return None                              # curtain_detector's turn
        far = cur.centre + side * cur.n * 0.6
        heading = math.atan2(side * cur.n[1], side * cur.n[0])
        return [Frontier(goal=(float(far[0]), float(far[1])), yaw=heading, path_length=0.0,
                         size=1, centroid=(float(far[0]), float(far[1])))]

    def clear_local_costmap(self, why, global_too=False):
        """Fabric pushed or dragged through a curtain is marked around the robot; nothing
        clears it (no returns there now), and Nav2 would start in collision. global_too: the
        global costmap too, whose obstacle layer still has the curtain itself (marked before
        it was crossed); its static layer refills from /maze/map, curtain free."""
        for c, name in ((self.clear_costmaps[1], 'local'),) + (
                ((self.clear_costmaps[0], 'global'),) if global_too else ()):
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
                self.get_logger().info(f'{why}: {name} costmap cleared')
            else:
                self.get_logger().warn(f'{why}: {name} costmap clear service not ready')

    def junction_age(self):
        """s since the junction request came in. One that came in during CURTAIN /
        AT_CURTAIN (held) doesn't age until that ends: then it counts from there."""
        rec, at = self.junction_req
        if rec.get('held'):
            if self.state in CURTAIN_STATES:
                return 0.0
            if self.curtain_left_at is not None:
                at = max(at, self.curtain_left_at)
        return time.monotonic() - at

    def junction_pending(self):
        """A junction request is in and not expired: it comes before a curtain."""
        return self.junction_req is not None and \
            self.junction_age() <= self.cfg['junction_timeout']

    def curtain_preemptable(self):
        """CURTAIN for a crossing that hasn't reached the fabric yet: a junction or dead end
        may take the robot back (curtain_detector then yields, not marking it passed). Never
        an inspection: at a junction the lie-down check comes first (user 2026-11-20)."""
        return self.state == 'CURTAIN' and not self.inspecting and \
            self.curtain_status in ('FACING', 'APPROACHING')

    def on_curtains(self, msg):
        frame, curtains = curtains_from_json(msg.data)
        if frame:
            self.curtains_frame, self.curtains = frame, curtains
            self.publish_curtain_map()

    def curtain_map(self):
        """/map with the confirmed curtains drawn as free and the occupied cells as walls
        (OccupancyGrid), or None."""
        if self.map is None:
            return None
        cs = self.curtains_in_map()
        if not cs and not self.paint_cells:
            return self.map
        g = self.block_objects(clear_curtains(Grid.from_msg(self.map), cs))
        out = OccupancyGrid()
        out.header, out.info = self.map.header, self.map.info
        out.data = array.array('b', np.asarray(g.data, np.int8).tobytes())
        return out

    def publish_curtain_map(self):
        m = self.curtain_map()
        if m is not None:
            self.curtain_map_pub.publish(m)
        self.publish_keepout()

    def footprint_keepout(self):
        return self.cfg.get('object_keepout', 'cell') == 'footprint'

    def painted_boxes(self):
        """Map boxes of the painted objects, grown by object_keepout_margin (footprint mode)."""
        m = self.cfg['object_keepout_margin']
        return [(b[0] - m, b[1] - m, b[2] + m, b[3] + m) for b in
                (self.object_boxes.get(c) for c in sorted(self.paint_cells)) if b is not None]

    def block_objects(self, grid):
        """grid with the painted objects drawn as wall: their boxes, or whole cells."""
        if self.footprint_keepout():
            return block_boxes(grid, self.painted_boxes())
        return block_cells(grid, self.cells, self.paint_cells)

    def in_object(self, xy):
        """Is xy where the robot must not go: in a blocked cell ('cell'), or within
        object_keepout_margin of an object's box ('footprint')?"""
        if self.footprint_keepout():
            m = self.cfg['object_keepout_margin']
            boxes = getattr(self, 'object_boxes', {}).values()
            return in_boxes(xy, [b for b in boxes if b is not None], m)
        return self.cells.cell(xy) in self.blocked_cells

    def publish_keepout(self):
        """The painted objects as Nav2's keepout mask (map frame), on change."""
        cells = frozenset(self.paint_cells)
        key = (cells, tuple(round(v, 2) for b in self.painted_boxes() for v in b)) \
            if self.footprint_keepout() else cells
        if self.keepout_pub is None or key == self.keepout_cells:
            return
        self.keepout_cells = key
        g = keepout_boxes_mask(self.painted_boxes()) if self.footprint_keepout() else \
            keepout_mask(self.cells, cells)
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(g.resolution)
        msg.info.height, msg.info.width = (int(v) for v in g.data.shape)
        msg.info.origin.position.x, msg.info.origin.position.y = g.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', np.asarray(g.data, np.int8).tobytes())
        self.keepout_pub.publish(msg)
        if cells:
            what = 'object footprint(s)' if self.footprint_keepout() else 'object cell(s)'
            self.get_logger().info(f'keepout mask: {len(cells)} {what} lethal for Nav2 '
                                   f'({", ".join(str(list(c)) for c in sorted(cells))})')

    def curtains_in_map(self):
        """The known curtains in the map frame (latest TF), [] if none or no TF."""
        if not self.curtains:
            return []
        if self.curtains_frame == 'map':        # curtain_detector's default
            return list(self.curtains)
        try:
            tf = self.tf.lookup_transform('map', self.curtains_frame, Time())
        except TransformException:
            return []
        t, q = tf.transform.translation, tf.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return [c.transformed(t.x, t.y, th) for c in self.curtains]

    def grid_now(self):
        """/map as a Grid, curtains cleared (they are passable), occupied cells as walls."""
        return self.block_objects(clear_curtains(Grid.from_msg(self.map),
                                                 self.curtains_in_map()))

    def update_objects(self, xy):
        """Look for objects in the cells on a new /map; block cells that keep showing one."""
        if not self.cfg['block_objects'] or self.map is None:
            return
        if self.map is not self.objects_map:
            self.objects_map = self.map
            raw = clear_curtains(Grid.from_msg(self.map), self.curtains_in_map())
            found = cell_objects(raw, self.cells, self.cfg['object_border'],
                                 self.cfg['object_min_extent'], self.cfg['object_min_pixels'])
            here = self.cells.cell(xy)
            self.object_seen = {c: self.object_seen.get(c, 0) + 1 for c in found
                                if c != here and c not in self.blocked_cells}
            for c, n in self.object_seen.items():
                if n >= self.cfg['object_confirm']:
                    self.blocked_cells[c] = found[c]
                    ext, (ox, oy) = found[c]
                    self.get_logger().warn(
                        f'cell {list(c)}: object {ext:.2f} m across at ({ox:.2f}, {oy:.2f}) - '
                        + ('its footprint kept out' if self.footprint_keepout() else
                           'occupied, not entering it'))
                    self.queue_room_look(c, (ox, oy))
            # Footprints grow as more of each object is seen (the lidar sees faces only).
            boxes_before = dict(self.object_boxes)
            for c in self.blocked_cells:
                self.object_boxes[c] = grow_box(
                    self.object_boxes.get(c),
                    object_box(raw, self.cells, c, self.cfg['object_border']))
            if self.object_boxes != boxes_before and self.paint_cells:
                self.publish_curtain_map()
        # Not drawn while the robot's body may still be in it (Nav2: "in lethal space").
        reach_m = self.cfg['footprint_back'] + 0.08
        if self.footprint_keepout():
            m = self.cfg['object_keepout_margin'] + reach_m
            paint = {c for c in self.blocked_cells if self.object_boxes.get(c) is not None
                     and not in_boxes(xy, [self.object_boxes[c]], m)}
        else:
            reach = 0.5 + reach_m / self.cells.size
            u, v = self.cells.local(xy)
            paint = {c for c in self.blocked_cells
                     if max(abs(u - c[0]), abs(v - c[1])) > reach}
        if paint != self.paint_cells:
            self.paint_cells = paint
            self.publish_curtain_map()
        cg = self.current.goal if self.current is not None else None
        if cg is not None and self.goal_handle is not None and \
                self.in_object(cg) and cg not in self.blacklist:
            self.get_logger().warn(f'goal ({cg[0]:.2f}, {cg[1]:.2f}) is in an occupied '
                                   + ('object footprint' if self.footprint_keepout() else 'cell')
                                   + ': cancelled')
            self.blacklist.append(cg)
            self.goal_handle.cancel_goal_async()

    # ------------------------------------------------------------ room look-in
    def queue_room_look(self, cell, obj_xy):
        """A newly confirmed object cell: look into it from outside later (maybe_room_look)."""
        if cell in self.scanned_cells:
            return self.mark_explored(cell, 'object confirmed after a junction scan of it')
        if not self.cfg['room_looks'] or cell in self.rooms_done or cell in self.room_looks:
            return
        if cell in self.dead_end_cells:
            self.get_logger().info(f'cell {list(cell)}: object in a dead end already scanned; '
                                   'no room look')
            return
        self.room_looks[cell] = obj_xy
        self.get_logger().info(f'cell {list(cell)}: room (object at ({obj_xy[0]:.2f}, '
                               f'{obj_xy[1]:.2f})): will look in from a neighbouring cell')

    def on_stair_pose(self, msg):
        """stair_detector's /stairs/pose (odom, latched): stairs read as object cells too."""
        p = (msg.pose.position.x, msg.pose.position.y)
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.5 for q in self.stair_spots):
            self.stair_spots.append(p)

    def near_stairs(self, xy):
        """xy (map) within room_stair_radius m of a confirmed stair's first riser."""
        if not self.stair_spots:
            return False
        pt = PointStamped()
        pt.header.frame_id = 'odom'
        r = self.cfg['room_stair_radius']
        for q in self.stair_spots:
            pt.point.x, pt.point.y = q
            m = self.odom_to_map(pt)
            if m is not None and math.hypot(xy[0] - m[0], xy[1] - m[1]) <= r:
                return True
        return False

    def maybe_room_look(self):
        """EXPLORING: a queued room within reach -> stop and look in from its neighbour."""
        if not self.room_looks or not self.cfg['go'] or self.junction_pending():
            return
        now = time.monotonic()
        if now - self.room_look_at < 1.0:             # map copies + TF: 1 Hz is plenty
            return
        self.room_look_at = now
        if self.spin_busy or self.turn_creep is not None or not self.nav.server_is_ready():
            return
        pose = self.robot_pose()
        if pose is None or self.map is None:
            return
        xy = pose[0]
        for cell in list(self.room_looks):
            why = None
            if self.room_looks_started >= self.cfg['room_look_max']:
                why = f'{self.cfg["room_look_max"]} room looks done this run'
            elif self.near_stairs(self.cells.centre(cell)):
                why = 'next to the stairs (stairs read as objects)'
            if why:
                del self.room_looks[cell]
                self.rooms_done.add(cell)
                self.get_logger().info(f'cell {list(cell)}: no room look: {why}')
        if not self.room_looks:
            return
        raw = clear_curtains(Grid.from_msg(self.map), self.curtains_in_map())  # rooms not drawn
        grid = self.grid_now()
        field = path_field(grid, xy, self.cfg['robot_radius'])
        if field is None:
            return
        skip = set(self.blocked_cells) | self.dead_end_cells
        best = None
        for cell in self.room_looks:
            spot = room_look_spot(raw, grid, self.cells, cell, field, skip,
                                  self.cfg['room_look_radius'])
            if spot is not None and (best is None or spot[2] < best[3]):
                best = (cell, *spot)
        if best is not None:
            self.start_room_look(*best[:3])

    def start_room_look(self, cell, n, d):
        """Junction-follow-up style actions: to n's centre, square up facing into `cell`, hold
        (the centre view), then the room's left and right walls (posters, QR codes)."""
        obj = self.room_looks.pop(cell)
        self.rooms_done.add(cell)
        self.room_looks_started += 1
        into = (-d[0], -d[1])
        face = self.cells.heading(into)
        group = f'room {list(cell)}'
        hold = {'do': 'hold', 's': float(self.cfg['room_look_hold']), 'group': group,
                'name': f'{group}: facing in', 'view': 'centre'}
        look = {'do': 'look', 'group': group, 'name': f'{group}: walls',
                'steps': [{'name': f'{group} {k} wall', 'point': p, 'facing': face, 'view': k,
                           'hold': self.cfg['dead_end_side_hold'], 'short': True}
                          for k, p in side_walls(self.cells, cell, into).items()]}
        dead_end_scene([hold] + look['steps'], cell, self.cells.centre(cell), kind='room')
        todo = [{'do': 'goto', 'xy': self.cells.centre(n), 'yaw': face, 'group': group,
                 'name': f'{group}: to cell {list(n)} next to it'},
                {'do': 'align', 'heading': face, 'group': group, 'tries': 0,
                 'name': f'{group}: squaring up facing in'},
                hold, look]
        self.halt_for_look()
        self.junction = self.new_look([])
        self.junction['rec'] = {}                 # follow-up only: no junction ways
        self.junction['todo'] = todo
        self.set_state('JUNCTION', f'{group}: object at ({obj[0]:.2f}, {obj[1]:.2f}); looking '
                       f'in from cell {list(n)} (reachable along free space), not entering')

    def keep_out_of_dead_ends(self):
        """Checked dead ends are never entered (user 2026-11-21): a confirmed curtain on
        one's edge makes it a way on again (drop it); a running goal into one is cancelled.
        Dead ends still waiting for scene understanding (or queued as a scene candidate):
        a running goal into one is cancelled too, but not blacklisted (it may open again)."""
        cg = self.current.goal if self.current is not None else None
        if cg is not None and self.goal_handle is not None and \
                self.cells.cell(cg) not in self.dead_end_cells and \
                self.no_go_cell(self.cells.cell(cg)):
            self.get_logger().warn(f'goal ({cg[0]:.2f}, {cg[1]:.2f}) is in a dead end waiting '
                                   'for scene understanding: cancelled')
            self.goal_handle.cancel_goal_async()
            self.goal_handle, self.current, self.goal_result = None, None, None
            return
        if not self.dead_end_cells:
            return
        for c in self.curtains_in_map():
            mid = (np.asarray(c.p1) + np.asarray(c.p2)) / 2
            for side in (-0.25, 0.25):
                cell = self.cells.cell(mid + side * np.asarray(c.n))
                if cell in self.dead_end_cells:
                    self.dead_end_cells.discard(cell)
                    self.get_logger().info(f'cell {list(cell)}: curtain {c.id} on its edge; '
                                           'no longer a dead end')
        cg = self.current.goal if self.current is not None else None
        if cg is not None and self.goal_handle is not None and \
                self.cells.cell(cg) in self.dead_end_cells and cg not in self.blacklist:
            self.get_logger().warn(f'goal ({cg[0]:.2f}, {cg[1]:.2f}) is in a checked dead end: '
                                   'cancelled')
            self.blacklist.append(cg)
            self.goal_handle.cancel_goal_async()

    def curtain_redirect(self, f, xy):
        """Goal f at a curtain not yet walked through from this side (within
        curtain_goal_radius), or only reachable through any curtain, walked through or not
        (curtain_branch.needs_crossing on the explorer's map) -> a copy of f aimed at the
        spot in front of the curtain's middle, facing it. curtain_detector does every
        crossing from there; Nav2 never pushes into the fabric itself (its global costmap
        has a walked-through curtain as free). At the spot: None (wait) for up to
        `curtain_spot_wait` s, with f published on /maze/goal (curtain_detector crosses back
        only for a goal beyond); then f itself (Nav2 tries)."""
        grid = None
        for c in self.curtains_in_map():
            side = c.side_of(xy)
            near = side not in c.passed and \
                c.distance(f.goal) <= self.cfg['curtain_goal_radius']
            through = False
            if not near and c.distance(xy) >= 0.3 and c.distance(f.goal) >= 0.3:
                grid = grid if grid is not None else self.grid_now()
                through = needs_crossing(grid, xy, f.goal, c)
            if not (near or through):
                continue
            spot = c.centre - side * c.n * self.cfg['curtain_approach']
            if math.hypot(spot[0] - xy[0], spot[1] - xy[1]) < 0.35:
                now = time.monotonic()
                if self.spot_wait is None or self.spot_wait[0] != c.id:
                    self.spot_wait = (c.id, now)
                    self.publish_goal(f.goal, math.atan2(side * c.n[1], side * c.n[0]))
                    self.get_logger().info(f'in front of curtain {c.id}: waiting for '
                                           'curtain_detector to walk the robot through')
                if now - self.spot_wait[1] < self.cfg['curtain_spot_wait']:
                    return None
                self.get_logger().warn(f'in front of curtain {c.id}, no crossing after '
                                       f'{self.cfg["curtain_spot_wait"]:.0f} s: Nav2 tries',
                                       throttle_duration_sec=30.0)
                return f
            goal = (float(spot[0]), float(spot[1]))
            yaw = math.atan2(side * c.n[1], side * c.n[0])
            self.redirects[goal] = (f.goal, near)
            self.get_logger().info(f'goal ({f.goal[0]:.2f}, {f.goal[1]:.2f}) is '
                                   f'{"at" if near else "through"} curtain {c.id}: going to '
                                   f'({goal[0]:.2f}, {goal[1]:.2f}) in front of it first',
                                   throttle_duration_sec=10.0)
            return dataclasses.replace(f, goal=goal, yaw=yaw)
        self.spot_wait = None
        return f

    def publish_goal(self, xy, yaw):
        """/maze/goal (map) without sending it to Nav2."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(xy[0]), float(xy[1])
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(msg)

    def check_map_subscribers(self):
        """Startup guard: Nav2's global costmap takes its static layer from /maze/map
        (nav2_params.yaml). Nobody on it 20 s after the first /map = that setting didn't
        take (or Nav2 isn't up): the global costmap has no map walls."""
        if self.first_map is None or self.map_sub_ok or \
                time.monotonic() - self.first_map < 20.0:
            return
        n = self.curtain_map_pub.get_subscription_count()
        if n > 0:
            self.map_sub_ok = True
            self.get_logger().info(f'/maze/map: {n} subscriber(s) (Nav2 global costmap)')
        else:
            self.get_logger().error(
                '/maze/map has NO subscriber 20 s after the first /map: Nav2\'s global '
                'costmap static layer is not reading it (nav2_params.yaml global_costmap '
                'static_layer map_topic), so it has no map walls. Check: ros2 param get '
                '/global_costmap/global_costmap static_layer.map_topic',
                throttle_duration_sec=30.0)

    def begin_after_stairs(self, why):
        self.pre_turned = None
        self.set_state('EXPLORING', why)

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())   # latest, no waiting
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y), yaw

    def min_frontier_width(self):
        return self.cfg['min_frontier_width_ratio'] * self.cfg['robot_width']

    def poll_nav(self):
        """Ask the lifecycle manager whether Nav2 is active; non-blocking, one call at a time."""
        now = time.monotonic()
        if self.nav_query is not None:
            future, sent = self.nav_query
            if future.done():
                active = future.result() is not None and future.result().success
            elif now - sent > 3.0:
                # No answer: the manager is stuck resetting (it blocks on the dead server).
                self.nav_check.remove_pending_request(future)
                active = False
            else:
                return
            self.nav_query = None
            self.nav_reply_at = sent
            if active != self.nav_active:
                # Separate call sites: rclpy raises "Logger severity cannot be changed between
                # calls" if one call site logs at two severities.
                if active:
                    self.get_logger().info('Nav2 is active again')
                else:
                    self.get_logger().error('Nav2 is NOT active (a server died? see the Nav2 '
                                            'log above)')
            self.nav_active = active
            self.nav_down_since = None if active else (self.nav_down_since or now)
        if self.nav_check.service_is_ready():
            self.nav_query = (self.nav_check.call_async(Trigger.Request()), now)

    def nav_known_after(self, t):
        """True once an is_active answer to a call sent after t is in (or none can come)."""
        return self.nav_reply_at >= t or not self.nav_check.service_is_ready()

    # ------------------------------------------------------------------ main loop
    def step(self):
        if self.state in END_STATES:
            return
        pose = self.robot_pose()
        if self.map is None or pose is None:
            return
        if self.state == 'STARTING':
            if not self.nav.server_is_ready():
                self.get_logger().info('waiting for Nav2 navigate_to_pose',
                                       throttle_duration_sec=5.0)
                return
            if time.monotonic() - self.first_map < self.cfg['start_delay']:
                return
            if self.cfg['wait_for_start']:
                self.set_state('PAUSED', 'ready; holding still - start the rosbag recording, '
                               f'then: {START_CMD}')
        if self.state in ('STARTING', 'PAUSED'):        # STARTING here = ready, no wait
            if self.state == 'PAUSED' and not self.start_requested:
                self.get_logger().info(f'waiting for start: {START_CMD}',
                                       throttle_duration_sec=15.0)
                return
            held = self.start_held()
            if held:
                self.get_logger().warn(f'start held: {held}', throttle_duration_sec=2.0)
                return
            self.started = time.monotonic()
            if not self.cfg['go']:
                self.begin()
            else:
                self.creep_or_scan(pose)
        # Out of the maze? Checked in EVERY moving state, before the per-state returns: on
        # 2026-11-21 the sim robot left the maze and did junction looks back to back (0.1 s of
        # EXPLORING between them), so a check only in EXPLORING never held for exit_dwell s
        # and the run timed out outside. finish() ends any look, spin, goal or handover.
        if self.state not in ('STARTING', 'PAUSED', 'CREEPING') and self.out_of_maze(pose):
            return
        if self.state == 'CREEPING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return                                  # creep_tick drives, then moves on
        if self.state == 'TO_EXIT':                 # driving to the drone's launch cell
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return self.launch_tick(pose)
        if self.state in ('CURTAIN', 'AT_CURTAIN'):   # curtain_detector drives / holding still
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.state == 'CURTAIN' and \
                    time.monotonic() - self.curtain_since > self.cfg['curtain_timeout']:
                self.get_logger().warn(f'curtain_detector still busy after '
                                       f'{self.cfg["curtain_timeout"]:.0f} s')
                self.end_curtain('curtain timeout; exploring on')
            elif self.state == 'AT_CURTAIN' and self.resume_requested:
                self.resume_requested = False
                self.end_curtain('resumed at the curtain')
            return
        if self.state in ('STAIRS', 'AT_STAIRS'):  # stair_detector drives / holding still
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            limit = self.cfg['stairs_approach_timeout' if self.stairs_approaching
                             else 'stairs_timeout']
            if self.state == 'STAIRS' and time.monotonic() - self.stairs_since > limit:
                self.get_logger().warn(f'stair_detector still busy after {limit:.0f} s'
                                       + (' of approach' if self.stairs_approaching else ''))
                self.begin_after_stairs('stairs timeout; exploring on')
            elif self.state == 'AT_STAIRS' and self.resume_requested:
                self.resume_requested = False
                self.begin_after_stairs('resumed at the stairs')
            elif self.state == 'AT_STAIRS' and self.at_stairs_since is not None and \
                    self.ai_done_at > self.at_stairs_since:
                self.begin_after_stairs('over the stairs (AI mode done, auto mode sent): '
                                        'exploring on')
            return
        if self.state == 'TURNING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy or self.turn_creep is not None:
                return                              # spinning, or creep_tick creeping
            self.set_state('EXPLORING', 'turned; going on')   # replan from the new heading
            return
        if self.state == 'FACING_QR':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return self.face_tick(pose)
        if self.state in ('JUNCTION', 'DEAD_END'):  # scene_tick turns / holds
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return
        if self.state == 'SCANNING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy:
                return
            if self.scan_turns is None:
                self.plan_scan(pose)
            if not self.scan_turns:
                self.release_pose()
                self.begin()
            elif self.scan_pose:
                self.send_pose(self.scan_turns.pop(0), self.cfg['scan_hold'], 'start scan')
                return
            elif not self.spin.server_is_ready():
                if time.monotonic() - self.started < 10.0:   # action discovery takes a moment
                    return
                self.get_logger().warn('Nav2 /spin not available; skipping start scan')
                self.scan_turns = []
                self.begin()
            else:
                self.send_spin(self.scan_turns.pop(0))
                return
        if self.cfg['go']:
            self.poll_nav()
            if self.nav_down_since is not None and \
                    time.monotonic() - self.nav_down_since >= self.cfg['nav_down_timeout']:
                self.finish('STOPPED', 'Nav2 is down (lifecycle manager inactive); '
                            'check the log for a crashed Nav2 server')
                self.stop_robot()
                return
        xy, yaw = pose
        self.visited.add(self.cells.cell(xy))
        self.update_objects(xy)
        self.keep_out_of_dead_ends()
        grid = self.grid_now()
        self.fit_cells(grid)
        self.update_exit(grid)
        if time.monotonic() - self.started > self.cfg['max_runtime']:
            return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')

        if self.maybe_face(grid, xy, yaw):
            return

        # Goal bookkeeping
        if self.goal_result is not None:
            ok = self.goal_result == GoalStatus.STATUS_SUCCEEDED
            if ok and self.current is not None and self.false_success(self.current.goal, xy):
                ok = False                          # set aside below, not blacklisted
            if not ok and not self.nav_known_after(self.goal_result_at):
                return                              # was Nav2 still up? ask first
            if not ok and not self.nav_active:
                self.get_logger().warn('goal failed while Nav2 is down; not blacklisting it')
                self.goal_result, self.goal_handle, self.current = None, None, None
                return
            if self.current is not None and ok:
                # Reached goals are blacklisted: a frontier still there after the robot
                # stood on it can't be cleared from there (e.g. unknown seen through a slit
                # at a wall corner, which the example maze has next to the start cell).
                self.blacklist.append(self.current.goal)
                if self.current.goal in self.redirects:   # in front of a curtain
                    orig, near = self.redirects.pop(self.current.goal)
                    if near:                    # the curtain face itself: done with it
                        self.blacklist.append(orig)
            if not ok:
                self.failures += 1
                if self.current is not None:
                    self.failed.append(self.current.goal)
                    self.get_logger().warn(
                        f'goal ({self.current.goal[0]:.2f}, {self.current.goal[1]:.2f}) failed '
                        f'(status {self.goal_result}); set aside, other frontiers first')
            else:
                self.failures = 0
            self.goal_result, self.goal_handle, self.current = None, None, None
        elif self.goal_handle is not None and self.goal_stalled(xy):
            self.goal_handle.cancel_goal_async()
            return

        if self.failures >= self.cfg['max_failures']:
            return self.recover_or_finish(grid, xy, f'{self.failures} goals failed in a row')

        if not self.nav_active:
            return                                  # no goals into a dead Nav2
        if self.queue_active:
            q = self.queue_goal(grid, xy, yaw)
            if q is None:
                return                              # facing / released: the detector's turn
            if q:
                nxt = q[0]
                if self.current is not None and self.goal_handle is not None and math.hypot(
                        nxt.goal[0] - self.current.goal[0],
                        nxt.goal[1] - self.current.goal[1]) < 0.4:
                    return                          # driving there
                if self.pre_turn(grid, xy, yaw, nxt):
                    return
                self.send(nxt, 1)
                return
        frontiers = self.find(grid, xy, self.blacklist + self.failed)
        # Smac (Humble) doesn't check the goal is inside the rolling global costmap either.
        reach = self.in_reach(frontiers, xy)
        off = self.cfg['goal_max_offset']
        if frontiers and not reach:
            self.get_logger().warn(f'{len(frontiers)} frontiers, all > {off} m away '
                                   '(outside the global costmap); waiting',
                                   throttle_duration_sec=10.0)
            return
        frontiers = self.branch_frontiers(grid, xy, reach)
        if frontiers is None:
            return                                  # waiting (deferred curtain)
        if not frontiers:
            # One empty result can be a map still filling in (start-up, SLAM update);
            # only a few in a row mean everything is mapped.
            stale = self.scan_stale()
            if stale is not None:
                # No fresh scans = no map updates: an empty frontier list means nothing
                # (robot 2026-11-19 11:37: odometry ~1 s late, wall_scan dropped every
                # cloud, NO_EXIT 5 s later with the maze unexplored).
                self.get_logger().warn(f'no frontier, but {stale}: not counting it; waiting '
                                       'for the map to update', throttle_duration_sec=5.0)
                return
            self.empty_replans += 1
            if self.empty_replans < self.cfg['done_confirm']:
                self.get_logger().info(f'no frontier ({self.empty_replans}/'
                                       f'{self.cfg["done_confirm"]}); checking again')
                return
            if self.failed:
                return self.recover_or_finish(grid, xy, 'only failed frontiers left')
            if self.start_queue('nothing else to explore'):
                self.empty_replans = 0
                return
            if self.go_to_exit('maze explored'):
                return
            # Everything mapped: stop now, even mid-goal (finish cancels it).
            self.finish('NO_EXIT', 'all frontiers mapped, no open space found')
            self.stop_robot()
            print('nav stopped', flush=True)
            return
        self.empty_replans = 0
        nxt = self.curtain_redirect(choose_frontier(frontiers, self.current, yaw, xy), xy)
        if nxt is None:
            return                                  # in front of a curtain: its turn
        if self.current is not None and self.goal_handle is not None:
            cg = self.current.goal
            if math.hypot(nxt.goal[0] - cg[0], nxt.goal[1] - cg[1]) < 0.4:
                self.current_missing = 0
                return                              # same goal, keep driving
            # The current goal's frontier can blink out for one map update and come back
            # (live run 2026-11-15 09:39: two goals swapped every 1-3 s, each swap a Nav2
            # preempt). Keep driving it for goal_grace replans before switching.
            if not any(math.hypot(f.goal[0] - cg[0], f.goal[1] - cg[1]) < 0.4
                       for f in frontiers):
                self.current_missing += 1
                if self.current_missing < self.cfg['goal_grace']:
                    return
        self.current_missing = 0
        if self.pre_turn(grid, xy, yaw, nxt):
            return
        self.send(nxt, len(frontiers))

    def pre_turn(self, grid, xy, yaw, f):
        """Turn the robot ourselves before goal f if Nav2's own turn would hit a wall."""
        if not (self.cfg['go'] and self.cfg['pre_turn']) or not self.spin.server_is_ready():
            return False
        if self.pre_turned is not None and \
                math.hypot(f.goal[0] - self.pre_turned[0], f.goal[1] - self.pre_turned[1]) < 0.4:
            return False                            # turned for this goal already
        heading = heading_towards(grid, xy, f.goal, self.cfg['robot_radius'])
        angle, why = plan_turn(grid, xy, yaw, heading, self.box, self.cfg['turn_margin'],
                               self.cfg['pre_turn_min_angle'], pivot=self.pivot)
        short = math.degrees(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        where = f'goal ({f.goal[0]:.2f}, {f.goal[1]:.2f})'
        if angle is None and why == 'blocked both ways' and self.cfg['pre_turn_max_creep'] > 0:
            # Find room straight ahead (core.plan_creep_turn), creep there, turn there.
            d, margin, why2 = plan_creep_turn(
                grid, xy, yaw, f.goal, self.box,
                margins=tuple(sorted({self.cfg['turn_margin'], 0.01}, reverse=True)),
                min_angle=self.cfg['pre_turn_min_angle'],
                max_creep=self.cfg['pre_turn_max_creep'],
                creep_margin=self.cfg['creep_margin'], robot_radius=self.cfg['robot_radius'],
                pivot=self.pivot)
            if d is None:
                self.get_logger().warn(f'{where} needs a {short:+.0f} deg turn, blocked by walls '
                                       f'both ways and {why2}; leaving it to Nav2')
                return False
            self.halt_for_turn(f)
            self.turn_creep = {'goal': f.goal, 'from': xy, 'target': d, 'margin': margin,
                               'until': time.monotonic() + d / self.cfg['creep_speed'] + 3.0}
            self.set_state('TURNING', f'{short:+.0f} deg to {where} blocked by walls both ways: '
                           f'creeping {d * 100:.0f} cm ahead to turn there ({why2})')
            return True
        if angle is None:
            return False
        self.halt_for_turn(f)
        self.set_state('TURNING', f'{short:+.0f} deg to {where} would hit a wall: turning '
                       f'{math.degrees(angle):+.0f} deg instead')
        self.send_spin(angle, 'turn before goal')
        return True

    def halt_for_turn(self, f):
        """Stop driving (cancel the Nav2 goal) before turning for goal f, once per goal."""
        self.pre_turned = f.goal
        if self.goal_handle is not None:            # stop where we are
            self.goal_handle.cancel_goal_async()
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.stop_robot()

    def on_qr_unread(self, msg):
        """qr_scanner: a QR code in view that it can't read (position in odom)."""
        self.unread = (msg, time.monotonic())

    def on_qr_read(self, _msg):
        self.qr_read_at = time.monotonic()

    def odom_to_map(self, msg):
        """(x, y) in map of a PointStamped in another fixed frame (latest TF), or None."""
        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x + math.cos(th) * msg.point.x - math.sin(th) * msg.point.y,
                t.y + math.sin(th) * msg.point.x + math.cos(th) * msg.point.y)

    def face_turn(self, grid, xy, yaw, target):
        """Turn (rad, + = left) towards target, cut to what the walls allow (max_turn)."""
        want = wrap(math.atan2(target[1] - xy[1], target[0] - xy[0]) - yaw)
        if abs(want) < self.cfg['face_min_angle']:
            return 0.0, want
        sign = 1 if want > 0 else -1
        ok = max_turn(grid, xy, yaw, self.box, sign, abs(want), self.cfg['turn_margin'],
                      pivot=self.pivot)
        return (sign * ok if ok > math.radians(5) else 0.0), want

    def maybe_face(self, grid, xy, yaw):
        """
        EXPLORING: a QR code is in view but unreadable: stop and turn to face it (FACING_QR)
        so the camera sees it squarely, until qr_scanner reads it. Returns True if started.
        """
        now = time.monotonic()
        self.face_given_up = [(p, t) for p, t in self.face_given_up
                              if now - t < self.cfg['face_cooldown']]
        if self.unread is None or now - self.unread[1] > 1.5 or not self.cfg['go'] or \
                self.state != 'EXPLORING' or self.spin_busy or self.turn_creep is not None \
                or now - self.qr_read_at < 1.0 or not self.spin.server_is_ready():
            return False
        target = self.odom_to_map(self.unread[0])
        if target is None or any(math.hypot(target[0] - p[0], target[1] - p[1]) < 0.75
                                 for p, _ in self.face_given_up):
            return False
        turn, want = self.face_turn(grid, xy, yaw, target)
        self.halt_for_look()
        self.face = {'since': now, 'turns': 0, 'target': target}
        what = (f'turning {math.degrees(turn):+.0f} deg' if turn else
                'already facing it' if abs(want) < self.cfg['face_min_angle'] else
                f'walls block the {math.degrees(want):+.0f} deg turn, holding')
        self.set_state('FACING_QR', f'QR code at ({target[0]:.2f}, {target[1]:.2f}) in view '
                       f'but unreadable: {what} until it is read')
        if turn:
            self.face['turns'] = 1
            self.send_spin(turn, 'face QR code')
        return True

    def face_tick(self, pose):
        """FACING_QR: wait for the read; re-aim if it drifts; give up after face_timeout."""
        now, f = time.monotonic(), self.face
        if f is None:
            return self.set_state('EXPLORING', 'facing QR code interrupted; going on')
        if self.qr_read_at > f['since']:
            self.face = None
            return self.set_state('EXPLORING', 'QR code read; going on')
        if self.spin_busy:
            return
        if now - f['since'] > self.cfg['face_timeout']:
            self.face_given_up.append((f['target'], now))
            self.face = None
            self.get_logger().warn(f'QR code at ({f["target"][0]:.2f}, {f["target"][1]:.2f}) '
                                   f'still unreadable after {self.cfg["face_timeout"]:.0f} s; '
                                   f'ignoring it for {self.cfg["face_cooldown"]:.0f} s')
            return self.set_state('EXPLORING', 'QR code not read in time; going on')
        if self.unread is None or now - self.unread[1] > self.cfg['face_lost']:
            self.face = None
            return self.set_state('EXPLORING', 'unreadable QR code no longer in view; going on')
        target = self.odom_to_map(self.unread[0])
        if target is None or f['turns'] >= self.cfg['face_max_turns']:
            return
        xy, yaw = pose
        turn, want = self.face_turn(self.grid_now(), xy, yaw, target)
        if turn and abs(want) > math.radians(15):
            f['turns'] += 1
            f['target'] = target
            self.send_spin(turn, 'face QR code again')

    def false_success(self, goal, xy):
        """Nav2 said SUCCEEDED but the robot is far from the goal. Humble's controller does
        that when it can't transform the goal (TF too old): it checks the robot against a
        default (0, 0) pose instead, and a robot near the start "reaches" any goal (robot
        2026-11-19 11:37: "Reached the goal!" 1.5 m short, the only frontier blacklisted)."""
        d = math.hypot(goal[0] - xy[0], goal[1] - xy[1])
        if d <= self.cfg['false_success_dist']:
            return False
        self.get_logger().warn(f'Nav2 says goal ({goal[0]:.2f}, {goal[1]:.2f}) reached, but the '
                               f'robot is {d:.2f} m from it (TF error?): counting it as failed')
        return True

    def on_scan_tick(self, _raw):
        now = time.monotonic()
        if self.scan_at is None or now - self.scan_at > self.cfg['start_gap']:
            self.scan_steady_since = now
        self.scan_at = now

    def start_held(self):
        """None when the run may start, else why not (start gate, see start_steady)."""
        need = self.cfg['start_steady']
        if not self.scan_topic or need <= 0:
            return None
        if self.scan_at is None:
            return f'no {self.scan_topic} yet (lidar / odometry not through)'
        now = time.monotonic()
        age = now - self.scan_at
        if age > self.cfg['start_gap']:
            return (f'{self.scan_topic} {age:.1f} s old: lidar or odometry late (wall_scan '
                    '"no TF within ..."); waiting for it to recover')
        steady = now - self.scan_steady_since
        if steady < need:
            return f'{self.scan_topic} steady for {steady:.1f} of {need:.0f} s'
        return None

    def scan_stale(self):
        """None while /wall_scan is fresh (or not watched), else why it is stale."""
        if not self.scan_topic:
            return None
        if self.scan_at is None:
            return f'no {self.scan_topic} yet'
        age = time.monotonic() - self.scan_at
        if age > self.cfg['scan_stale']:
            return f'{self.scan_topic} {age:.1f} s old'
        return None

    # ------------------------------------------------------------ junctions, dead ends
    def on_junction(self, msg):
        """scene_trigger: near the centre of a new cell with 2+ ways on."""
        try:
            rec = json.loads(msg.data)
            rec['paths'] = [(str(w['dir']), float(w['heading'])) for w in rec['paths']]
            rec['cell_centre'] = (float(rec['cell_centre'][0]), float(rec['cell_centre'][1]))
            rec['cell'] = (int(rec['cell'][0]), int(rec['cell'][1]))
            rec['from_cell'] = (int(rec['from_cell'][0]), int(rec['from_cell'][1]))
            gr = rec.get('grid')
            rec['cells'] = None if gr is None else CellGrid(
                float(gr['size']), (float(gr['origin'][0]), float(gr['origin'][1])),
                float(gr['yaw']))
            rec['half_width'] = float((gr or {}).get('opening_half_width', 0.35))
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /scene/junction message ({e}): {msg.data[:200]}')
            return
        self.junction_req = (rec, time.monotonic())
        if self.curtain_preemptable() and self.cfg['go']:
            self.preempt_curtain(f'junction at cell {rec["cell"]}')
        elif self.state in CURTAIN_STATES:
            rec['held'] = True                       # kept until the curtain step ends
            self.get_logger().info(f'junction at cell {rec["cell"]}: held until the curtain '
                                   f'step ends (curtain_detector {self.curtain_status})')
            self.publish_hold()                      # inspecting at a junction now

    def preempt_curtain(self, why):
        """Junction first: leave CURTAIN (curtain_detector yields) and start the look."""
        self.stop_robot()
        self.set_state('EXPLORING', f'{why} first; the curtain after it')
        self.maybe_junction()

    def on_dead_end(self, msg):
        """scene_trigger's /scene/trigger: facing a dead end from the cell before it."""
        try:
            rec = json.loads(msg.data)
            if rec.get('kind') != 'dead_end':
                return
            centre = (float(rec['cell_centre'][0]), float(rec['cell_centre'][1]))
            facing = float(rec['heading']) if rec.get('heading') is not None else None
            sides = rec.get('side_walls') or {}
            walls = [(k, (float(sides[k][0]), float(sides[k][1])))
                     for k in ('left', 'right') if k in sides]
        except (ValueError, KeyError, TypeError, IndexError, AttributeError) as e:
            self.get_logger().warn(f'bad /scene/trigger message ({e}): {msg.data[:200]}')
            return
        where = f'dead end at cell {rec.get("cell")} ({centre[0]:.2f}, {centre[1]:.2f})'
        if rec.get('manual'):
            # touble_shooting/trigger_scene.py: the look only, from EXPLORING only (a look
            # ends in EXPLORING: from PAUSED it would start the run), cell not blacklisted.
            if self.state != 'EXPLORING' or not self.cfg['go']:
                self.get_logger().warn(f'manual {where}: ignored, only while EXPLORING with '
                                       f'go (now {self.state}, go {self.cfg["go"]})')
                return
            self.get_logger().warn(f'manual {where}: dead-end look (cell not blacklisted)')
        cell = self.cells.cell(centre)
        manual = bool(rec.get('manual'))
        if not manual and cell in self.rooms_done:
            self.blacklist_dead_end(cell, centre, f'{where}: a room already looked into')
            return
        if not manual and not self.cfg['go']:
            self.blacklist_dead_end(cell, centre, f'{where}: no look (go:=false)')
            return
        if self.in_open_space():
            if not manual:
                self.blacklist_dead_end(cell, centre, f'{where}: no look')
            self.get_logger().info(f'{where}: open space here ({self.exit.open_nearby:.2f} m '
                                   'of free floor), probably outside the maze; no look')
            return
        if not manual:
            self.hold_dead_end(cell, centre, facing, where)
        if not walls and not self.side_walls_warned:
            self.side_walls_warned = True
            self.get_logger().warn('/scene/trigger has no side_walls (old scene_trigger?): '
                                   'dead ends get no side-wall scan')
        # Left wall first, then right: one sweep, like the junction look.
        sides = [{'name': f'dead end {k} wall', 'point': xy, 'facing': facing, 'view': k,
                  'hold': self.cfg['dead_end_side_hold'], 'short': True} for k, xy in walls]
        steps = dead_end_scene([{'name': 'dead end', 'hold': self.cfg['dead_end_hold']}]
                               + sides, rec.get('cell') or self.cells.cell(centre), centre)
        if self.state == 'EXPLORING' or self.curtain_preemptable():
            if self.state == 'CURTAIN':
                self.get_logger().info(f'{where}: before the curtain')
            self.halt_for_look()
            self.junction = self.new_look(steps)
            self.set_state('DEAD_END', f'{where}: holding still '
                           f'{self.cfg["dead_end_hold"]:.1f} s, then scanning its '
                           f'{" and ".join(k for k, _ in walls) or "no"} side walls; '
                           'not going in')
        elif self.state == 'JUNCTION' and self.junction is not None and \
                self.junction.get('rec') is not None and self.cfg['junction_follow']:
            self.get_logger().info(f'{where}: junction scan views it')
        elif self.state in ('JUNCTION', 'DEAD_END') and self.junction is not None:
            self.junction['dead_end'] = steps       # hold at this way, then scan its sides
            self.get_logger().info(f'{where}: holding at this way, then scanning its side '
                                   'walls')
        else:
            self.get_logger().info(f'{where}: no look now ({self.state})')

    # ------------------------------------------------------------ dead ends vs scene results
    def launch_cell(self):
        return self.exit_side[0] if self.exit_side is not None else None

    def blacklist_dead_end(self, cell, centre, why):
        """Never a goal into this dead end again."""
        self.scene_pending.pop(cell, None)
        if cell == self.launch_cell():
            self.get_logger().info(f'{why}; the launch cell: not blacklisted')
            return
        if centre not in self.blacklist:
            self.blacklist.append(centre)           # never send a goal into it
        self.dead_end_cells.add(cell)               # nor anywhere else in it
        self.get_logger().info(f'{why}: blacklisted')

    def hold_dead_end(self, cell, centre, heading, why, captured=False):
        """A dead end in the cell in front: no goal into it until scene understanding has
        answered for it (on_scene_result / scene_timeout_tick). captured: its view went out
        already (junction scan), the result timeout runs from now. Without scene results
        (room_scene off or no capture topic): blacklisted at once."""
        if cell in self.dead_end_cells or cell in self.scene_pending or \
                cell == self.launch_cell() or self.scene_candidate(cell):
            return
        if not self.scene_results:
            return self.blacklist_dead_end(cell, centre, f'{why} (no scene understanding)')
        now = time.monotonic()
        self.scene_pending[cell] = {'centre': (float(centre[0]), float(centre[1])),
                                    'heading': heading, 'attempt': 1,
                                    'since': now, 'captured': now if captured else None}
        self.get_logger().info(f'{why}: held (no goals into it) until scene understanding '
                               'answers; not blacklisted yet')

    def scene_candidate(self, cell):
        """A queued stairs / curtain candidate from scene understanding is in cell."""
        return any(c.get('cell') == cell for c in self.cand_queue)

    def no_go_cell(self, cell):
        """No goal into cell: a blacklisted dead end, one waiting for scene understanding,
        or a scene candidate still queued (checked after the junctions)."""
        return cell in self.dead_end_cells or cell in self.scene_pending or \
            self.scene_candidate(cell)

    def on_scene_result(self, msg):
        """room_scene's /scene/result for a held dead end: room -> blacklist; stairs /
        curtain -> candidate queue; anything else -> open. 'not analysed' (vlm none,
        scene_type unknown) is no answer: scene_timeout_tick decides."""
        try:
            r = json.loads(msg.data)
            cell = (int(r['cell'][0]), int(r['cell'][1]))
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /scene/result ({e}): {msg.data[:200]}')
            return
        p = self.scene_pending.get(cell)
        if p is None:
            return
        verdict = scene_verdict(r)
        where = f'dead end {list(cell)}'
        if verdict is None:
            self.get_logger().info(f'{where}: scene result (attempt {r.get("attempt", 1)}) '
                                   f'not analysed (vlm {r.get("vlm")!r}, scene_type '
                                   f'{r.get("scene_type")!r}); waiting')
            return
        if verdict == 'room':
            return self.blacklist_dead_end(cell, p['centre'], f'{where}: scene understanding '
                                           f'says room (objects {r.get("objects") or []})')
        self.scene_pending.pop(cell, None)
        if verdict in ('stairs', 'curtain'):
            h = p['heading']
            if h is None:
                pose = self.robot_pose()
                h = math.atan2(p['centre'][1] - pose[0][1], p['centre'][0] - pose[0][0]) \
                    if pose is not None else 0.0
            half = self.cells.size / 2
            target = (p['centre'][0] - half * math.cos(h), p['centre'][1] - half * math.sin(h))
            self.queue_candidate(verdict, None, target, h, f'scene understanding, cell '
                                 f'{list(cell)}')
            self.cand_queue[-1]['cell'] = cell     # no goals into it until it is checked
            return
        self.get_logger().info(f'{where}: scene understanding says '
                               f'{r.get("scene_type")!r}, no room / stairs / curtain: open '
                               '(not blacklisted)')

    def scene_timeout_tick(self):
        """1 Hz: held dead ends with no answer in time. Never captured within
        scene_look_grace s -> blacklisted. scene_result_timeout s after the last capture
        (or retry): object in the cell -> blacklisted (room); not a dead end on the map any
        more -> open; else retry (up to scene_max_attempts tries); the last unanswered ->
        blacklisted."""
        if not self.scene_pending:
            return
        now = time.monotonic()
        for cell, p in list(self.scene_pending.items()):
            where = f'dead end {list(cell)}'
            if p['captured'] is None:
                if now - p['since'] >= self.cfg['scene_look_grace']:
                    self.blacklist_dead_end(cell, p['centre'], f'{where}: never captured for '
                                            'scene understanding')
                continue
            if now - p['captured'] < self.cfg['scene_result_timeout']:
                continue
            waited = f'no scene result {self.cfg["scene_result_timeout"]:.0f} s after ' \
                     f'attempt {p["attempt"]}'
            if cell in self.blocked_cells:
                self.blacklist_dead_end(cell, p['centre'], f'{where}: {waited}; an object in '
                                        'it on the map (room)')
                continue
            if self.still_dead_end(cell, p['heading']) is False:
                self.scene_pending.pop(cell, None)
                self.get_logger().info(f'{where}: {waited}; not a dead end on the map any '
                                       'more: open (not blacklisted)')
                continue
            if p['attempt'] >= self.cfg['scene_max_attempts'] or self.scene_retry_pub is None:
                self.blacklist_dead_end(cell, p['centre'], f'{where}: {waited} (last try)')
                continue
            p['attempt'] += 1
            p['captured'] = now
            self.scene_retry_pub.publish(String(data=json.dumps(
                {'cell': [int(cell[0]), int(cell[1])], 'attempt': p['attempt']})))
            self.get_logger().info(f'{where}: {waited}; still a dead end, no object on the '
                                   f'map: asking scene understanding again (attempt '
                                   f'{p["attempt"]})')

    def still_dead_end(self, cell, heading):
        """True / False: cell's three sides other than the way in (heading: from the cell
        before into it) are walls / one is open on the map now. None: can't tell."""
        grid = self.grid_now()
        if grid is None:
            return None
        if heading is None:
            back = None
        else:
            back = min(DIRECTIONS.values(), key=lambda d: abs(wrap(
                self.cells.heading(d) - (heading + math.pi))))
        sides = [opening(grid, self.cells, cell, d) for d in DIRECTIONS.values() if d != back]
        if any(s == 'open' for s in sides) and (back is not None or
                                               sum(s == 'open' for s in sides) > 1):
            return False
        return True

    @staticmethod
    def new_look(steps):
        """A JUNCTION / DEAD_END look: each step {'name', 'hold', 'short', and 'heading' (map
        yaw) or 'point' (+ 'facing'): turn towards it; neither: no turn}, then hold 'hold' s.
        'short': turn the short way; otherwise (later junction ways) keep turning right."""
        return {'steps': steps, 'i': 0, 'turned': False, 'hold_until': None,
                'dead_end': None}

    def scene_tick(self):
        """10 Hz: start a junction look, and run a junction / dead-end look."""
        if self.cfg['tilt_comp'] and self.pose_pitch_pub is not None:
            self.sample_tilt()
        if self.state in ('JUNCTION', 'DEAD_END'):
            self.look_tick()
        elif self.state == 'EXPLORING':
            self.maybe_junction()
            if self.state == 'EXPLORING':
                self.maybe_room_look()

    def maybe_junction(self):
        """EXPLORING: a junction request is in: stop and face each way, left to right."""
        if self.junction_req is None or not self.cfg['go']:
            return
        rec = self.junction_req[0]
        if self.junction_age() > self.cfg['junction_timeout']:
            self.junction_req = None
            self.get_logger().info(f'junction at {rec.get("cell")}: request expired')
            return
        if self.spin_busy or self.turn_creep is not None or not self.spin.server_is_ready():
            return
        pose = self.robot_pose()
        if pose is None or self.map is None:
            return
        self.junction_req = None
        (x, y), yaw = pose
        c = rec['cell_centre']
        if self.in_open_space():
            self.get_logger().info(f'junction at {rec.get("cell")}: open space here '
                                   f'({self.exit.open_nearby:.2f} m of free floor), probably '
                                   'outside the maze; skipping the look')
            return
        if math.hypot(x - c[0], y - c[1]) > self.cfg['junction_radius']:
            self.get_logger().info(f'junction at {rec.get("cell")}: robot already '
                                   f'{math.hypot(x - c[0], y - c[1]):.2f} m from its centre; '
                                   'skipping the look')
            return
        # Left-most first: the largest left turn (relative to the heading now) first.
        ways = sorted(rec['paths'], key=lambda w: wrap(w[1] - yaw), reverse=True)
        # Don't turn to face the way back into the start cell (1 m cells).
        back = [n for n, h in ways
                if self.back_to_start((c[0] + math.cos(h), c[1] + math.sin(h)))]
        if back:
            self.get_logger().info(f'junction at {rec.get("cell")}: not facing way(s) '
                                   f'{", ".join(map(str, back))} (back to the start cell)')
            ways = [w for w in ways if w[0] not in back]
            if not ways:
                return
        # Only adjacent cells not stood in yet (user 2026-11-21: scan each adjacent
        # unexplored cell).
        size = rec['cells'].size if rec.get('cells') is not None else self.cells.size

        def nb(h):
            return (c[0] + size * math.cos(h), c[1] + size * math.sin(h))
        seen = [n for n, h in ways if self.cells.cell(nb(h)) in self.visited]
        if seen:
            self.get_logger().info(f'junction at {rec.get("cell")}: way(s) '
                                   f'{", ".join(map(str, seen))} lead to cells already '
                                   'visited: not scanned')
            ways = [w for w in ways if w[0] not in seen]
            if not ways:
                return
        self.halt_for_look()
        steps = []
        for k, (n, h) in enumerate(ways):
            cell = self.cells.cell(nb(h))
            steps.append({
                'name': f'junction way {n}', 'heading': h, 'short': k == 0,
                'hold': self.cfg['junction_hold'], 'scan': True, 'cell': cell,
                'centre': c if self.cfg['junction_centre'] else None,
                'scene': {'cell': [int(cell[0]), int(cell[1])],
                          'cell_centre': [round(float(v), 3) for v in nb(h)],
                          # one view per cell: each is its cell's last (room_scene sends it
                          # to the VLM at once, not after its 20 s room_timeout)
                          'kind': 'junction', 'view': str(n), 'last': True}})
        self.junction = self.new_look(steps)
        self.junction['rec'] = rec                  # look_tick follows it up afterwards
        self.set_state('JUNCTION', f'cell {rec.get("cell")}: {len(ways)} adjacent unexplored '
                       'cell(s) ('
                       + ', '.join(f'{n} {math.degrees(wrap(h - yaw)):+.0f} deg'
                                   for n, h in ways) + '); facing each left to right, '
                       'centring sideways, scanning; not leaving the cell until done')

    def at_dead_end(self):
        """True while the robot is looking at a dead end or scanning a junction way (user
        2026-11-19 / 11-21: QR checks only there): DEAD_END (scene_trigger's /scene/trigger:
        hold, then the side walls); in a JUNCTION look, holding at a way ('scan'), a step
        named 'dead end ...' or the hold facing one; in a follow-up, squaring up to /
        holding at a room or dead end. FACING_QR too, so a code seen there is followed up."""
        if self.state in ('DEAD_END', 'FACING_QR'):
            return True
        j = self.junction
        if self.state != 'JUNCTION' or not j:
            return False
        if j.get('dead_end'):
            return True
        st, k = j.get('steps') or [], j.get('i', 0)
        if 0 < k <= len(st) and st[k - 1].get('scan') and j.get('hold_until') is not None:
            return True
        if 'todo' in j:
            act = j['todo'][0] if j['todo'] else None
            return bool(act) and act.get('do') in ('align', 'hold', 'look') and \
                str(act.get('group', '')).startswith(('dead end', 'room'))
        steps, i = j.get('steps') or [], j.get('i', 0)
        if 0 < i <= len(steps) and 'dead end' in str(steps[i - 1].get('name', '')):
            return True
        # holding facing a dead end met mid-look: its side-wall steps come next
        return j.get('hold_until') is not None and i < len(steps) and \
            'dead end' in str(steps[i].get('name', ''))

    def perceive_tick(self):
        """5 Hz: /maze/perceive (Bool, latched) = at_dead_end(), on change.
        team_perception's qr_scanner / human_detector run only while it is True."""
        on = self.at_dead_end()
        if on != self.perceive:
            self.perceive = on
            self.perceive_pub.publish(Bool(data=on))
            self.get_logger().info(f'perception {"ON" if on else "off"} ({self.state})')

    def look_tick(self):
        """JUNCTION / DEAD_END: turn to the next step, hold its 'hold' s, ...; then EXPLORING.
        A dead end seen meanwhile (j['dead_end']: its side-wall steps): hold
        dead_end_hold s facing it, then scan its side walls before the next step."""
        j, now = self.junction, time.monotonic()
        what = 'junction' if self.state == 'JUNCTION' else 'dead end'
        if j is None:
            return self.set_state('EXPLORING', f'{what} look interrupted; going on')
        if self.spin_busy:
            return
        if j['turned'] and j['hold_until'] is None:          # a turn (or none) just ended
            st = j['steps'][j['i'] - 1]
            if st.get('centre') is not None and not st.get('centred') and \
                    not self.centre_tick(st, now):
                return                                       # moving sideways
            # a pose look held inside the pose already (the body swings back after it)
            j['hold_until'] = now + (0.0 if j.pop('posed', False) else st['hold'])
        if j['dead_end'] is not None:        # facing a dead end: its hold + side walls next
            j['steps'][j['i']:j['i']] = j['dead_end']
            j['dead_end'] = None
        if j['hold_until'] is not None and now < j['hold_until']:
            return
        if j['hold_until'] is not None and 0 < j['i'] <= len(j['steps']):
            held = j['steps'][j['i'] - 1]
            self.capture_view(held.get('scene'), held['hold'])   # posed: sent in pose_tick
            if held.get('scan'):
                self.note_scanned(held['cell'])
        j['turned'], j['hold_until'] = False, None
        if j['i'] >= len(j['steps']):
            self.release_pose()
            if j.get('rec') is not None and (self.cfg['junction_follow'] or j.get('todo')):
                if 'todo' not in j:
                    j['todo'] = self.plan_after_junction(j['rec'])
                if j['todo']:
                    return self.after_junction_tick(j, now)
            self.junction = None
            return self.set_state('EXPLORING', f'{what} looked at; going on')
        pose = self.robot_pose()
        if pose is None or self.map is None:
            return
        xy, yaw = pose
        step = j['steps'][j['i']]
        j['i'] += 1
        j['turned'] = True                                   # hold here even if no turn
        if 'heading' in step:
            h = step['heading']
        elif 'point' in step:
            if self.use_pose():
                return self.pose_side_wall(j, step, xy, yaw)
            h = side_heading(xy, step['point'], step.get('facing'),
                             self.cfg['dead_end_look_angle'])
        else:                                                # hold where it is
            if step.get('scene') is not None:                # facing into a dead end
                j['pitch'] = self.tilt_pitch(step['name'])
                if j['pitch'] is not None:
                    j['stance'], j['posed'] = yaw, True
                    self.send_pose(0.0, step['hold'], step['name'], pitch=j['pitch'],
                                   scene=step['scene'])
            return
        self.release_pose()                                  # a real turn: feet move
        want = wrap(h - yaw)
        if not step.get('short') and want > math.radians(20):
            want -= 2 * math.pi                              # later ways: keep turning right
        if abs(want) < math.radians(5):
            return
        sign = 1 if want > 0 else -1
        ok = max_turn(self.grid_now(), xy, yaw, self.box, sign, abs(want),
                      self.cfg['turn_margin'], pivot=self.pivot)
        if ok < math.radians(5):
            self.get_logger().warn(f'{step["name"]}: {math.degrees(want):+.0f} deg turn '
                                   'blocked by walls; holding here')
            return
        if ok < abs(want) - math.radians(2):
            self.get_logger().info(f'{step["name"]}: walls cut the {math.degrees(want):+.0f} '
                                   f'deg turn to {math.degrees(sign * ok):+.0f}')
        self.send_spin(sign * ok, step['name'])

    def centre_tick(self, st, now):
        """Junction centring (user 2026-11-21): move sideways (linear.y, feet keep the
        heading) onto the line through the junction cell's centre along the robot's heading.
        True once centred, given up, or blocked by a mapped wall."""
        pose = self.robot_pose()
        if pose is None or self.map is None:
            return False
        (x, y), yaw = pose
        cx, cy = st['centre']
        lat = -(x - cx) * math.sin(yaw) + (y - cy) * math.cos(yaw)   # + = left of the line
        tol, sf = self.cfg['centre_tol'], st.get('strafe')
        if sf is None:
            if abs(lat) <= tol:
                st['centred'] = True
                return True
            # Only along the cell's axis: moving sideways towards the centre line then moves
            # away from the nearer side wall, which the lidar can't see this close (sim
            # 2026-11-22: a turn cut short 40 deg, the sideways move ran into a wall).
            off = abs(wrap(yaw - st['heading'])) if 'heading' in st else 0.0
            if off > math.radians(10):
                self.get_logger().warn(f'{st["name"]}: {math.degrees(off):.0f} deg off the '
                                       'way after the turn: not moving sideways; scanning '
                                       'from here')
                st['centred'] = True
                return True
            move = max(-self.cfg['centre_max'], min(self.cfg['centre_max'], -lat))
            grid = self.grid_now()
            for f in (0.5, 1.0):
                pt = (x - f * move * math.sin(yaw), y + f * move * math.cos(yaw))
                if footprint_hits(grid, pt, yaw, self.box, 0.01):
                    self.get_logger().warn(f'{st["name"]}: {abs(move) * 100:.0f} cm sideways '
                                           'would touch a mapped wall; scanning from here')
                    st['centred'] = True
                    return True
            st['strafe'] = {'since': now, 'from': lat, 'best': abs(lat), 'best_at': now}
            self.get_logger().info(f'{st["name"]}: {abs(lat) * 100:.0f} cm off the cell\'s '
                                   f'centre line; moving {"right" if lat > 0 else "left"}')
            return False
        if abs(lat) < sf['best'] - 0.01:
            sf['best'], sf['best_at'] = abs(lat), now
        stuck = now - sf['best_at'] > 2.0                    # against something unmapped
        if abs(lat) <= tol or lat * sf['from'] < 0 or stuck or \
                now - sf['since'] > self.cfg['centre_timeout']:
            self.stop_robot()
            st['centred'] = True
            self.get_logger().info(f'{st["name"]}: centred ({lat * 100:+.0f} cm)'
                                   if abs(lat) <= max(tol, 0.05) else
                                   f'{st["name"]}: still {lat * 100:+.0f} cm off after '
                                   f'{now - sf["since"]:.0f} s; scanning from here')
            return True
        v = max(-self.cfg['centre_speed'], min(self.cfg['centre_speed'], -1.5 * lat))
        if abs(v) < 0.03:
            v = math.copysign(0.03, v)
        cmd = Twist()
        cmd.linear.y = float(v)
        self.cmd_pub.publish(cmd)
        return False

    def note_scanned(self, cell):
        """A junction scan looked into cell: an object room there needs no look-in later."""
        self.scanned_cells.add(cell)
        if cell in self.blocked_cells:
            self.mark_explored(cell, 'object seen in the junction scan')

    def mark_explored(self, cell, why):
        """Object room viewed (scene understanding triggered): explored, no goal into it, no
        room look-in (user 2026-11-21)."""
        if cell in self.explored_cells:
            return
        self.explored_cells.add(cell)
        self.rooms_done.add(cell)
        self.room_looks.pop(cell, None)
        self.get_logger().info(f'cell {list(cell)}: room marked explored ({why})')

    def capture_view(self, scene, hold):
        """End of a dead-end view's hold (still facing it): one /scene/capture for
        room_scene, which takes the camera frame stamped inside [hold_start, stamp]. Once per
        view (scene['sent']); the explorer never waits for scene understanding."""
        if not scene or scene.get('sent') or self.scene_capture_pub is None:
            return
        scene['sent'] = True
        held = self.scene_pending.get((int(scene['cell'][0]), int(scene['cell'][1])))
        if held is not None:                 # its result timeout runs from the last view
            held['captured'] = time.monotonic()
            scene['attempt'] = held['attempt']
        pose = self.robot_pose()
        now = self.ros_ns()
        tilt = self.body_tilt()
        cam = self.tf_pitch(self.cfg['camera_frame'])
        pitch = (self.junction or {}).get('pitch')
        rec = dict(scene)
        rec.pop('sent', None)
        rec.update({
            'stamp': f'{now // 10**9}.{now % 10**9:09d}',
            'hold_start': f'{(now - int(hold * 1e9)) // 10**9}.'
                          f'{(now - int(hold * 1e9)) % 10**9:09d}',
            'robot': None if pose is None else
            {'x': round(pose[0][0], 3), 'y': round(pose[0][1], 3),
             'yaw_deg': round(math.degrees(pose[1]), 1)},
            'frame': 'map',
            'tilt_deg': None if tilt is None else round(math.degrees(tilt[0]), 1),
            'roll_deg': None if tilt is None else round(math.degrees(tilt[1]), 1),
            'pitch_cmd_deg': None if pitch is None else round(math.degrees(pitch), 1),
            'camera_pitch_deg': None if cam is None else round(math.degrees(cam), 1)})
        self.scene_capture_pub.publish(String(data=json.dumps(rec)))
        self.get_logger().info(f'scene capture: {scene.get("kind", "dead_end").replace("_", " ")} '
                               f'{scene["cell"]} {scene["view"]}'
                               + (' (last view)' if scene['last'] else ''))

    def ros_ns(self):
        return self.get_clock().now().nanoseconds

    # ------------------------------------------------------------ tilt (dead-end looks)
    def sample_tilt(self):
        """10 Hz: body pitch / roll from TF odom -> base_link (Fast-LIO, local; /imu/data
        would be one more copy over the robot bridge). Not sampled while the transform's
        stamp hasn't changed for 0.5 s (stale; no clock comparison between machines)."""
        try:
            t = self.tf.lookup_transform('odom', 'base_link', Time())   # latest, no waiting
        except TransformException:
            return
        now = time.monotonic()
        st = (t.header.stamp.sec, t.header.stamp.nanosec)
        if st != self.tilt_tf_stamp:
            self.tilt_tf_stamp, self.tilt_tf_at = st, now
        elif now - self.tilt_tf_at > 0.5:
            return
        pitch, roll = quat_pitch_roll(t.transform.rotation)
        self.tilt_samples.append((now, pitch, roll))
        if self.tilt_ref_pending:
            self.set_tilt_ref()

    def body_tilt(self):
        """(pitch, roll) rad averaged over the last tilt_window s, pitch relative to the
        pitch at /maze/start (+ = nose down); None with fewer than 3 fresh samples."""
        now = time.monotonic()
        s = [(p, r) for t, p, r in self.tilt_samples if now - t <= self.cfg['tilt_window']]
        if len(s) < 3:
            return None
        return (sum(x[0] for x in s) / len(s) - (self.tilt_ref or 0.0),
                sum(x[1] for x in s) / len(s))

    def set_tilt_ref(self):
        """/maze/start: the robot stands still on the (flat) start cell. Its pitch now is
        the zero for the tilt (stance pitch, odom not exactly gravity-aligned)."""
        self.tilt_ref, self.tilt_ref_pending = None, True
        t = self.body_tilt()
        if t is None:
            return                                   # sample_tilt sets it once TF comes
        self.tilt_ref, self.tilt_ref_pending = t[0], False
        (self.get_logger().warn if abs(t[0]) > math.radians(5) else self.get_logger().info)(
            f'tilt zero: body pitch {math.degrees(t[0]):+.1f} deg, roll '
            f'{math.degrees(t[1]):+.1f} deg at the start'
            + (' (more than 5 deg: start cell not level, or odom not gravity-aligned)'
               if abs(t[0]) > math.radians(5) else ''))

    def tf_pitch(self, frame):
        """Pitch (rad, + = nose down) of `frame` in odom now, or None."""
        try:
            t = self.tf.lookup_transform('odom', frame, Time())
        except TransformException:
            return None
        return quat_pitch_roll(t.transform.rotation)[0]

    def tilt_pitch(self, why):
        """Pose-mode pitch (rad, + = nose down) for a dead end's views, or None: only when
        the body is pitched more than tilt_threshold (uneven floor) and Pose mode works.
        Pitches the body back by the tilt, clamped to pose_max_pitch. Roll is logged only
        (Pose mode has no roll axis here)."""
        if self.pose_pitch_pub is None or not self.cfg['tilt_comp']:
            return None
        t = self.body_tilt()
        if t is None:
            self.get_logger().warn(f'{why}: no fresh odom TF, tilt unknown: no pitch '
                                   'compensation')
            return None
        pitch, roll = t
        if abs(pitch) <= self.cfg['tilt_threshold']:
            return None
        lean = f'body {abs(math.degrees(pitch)):.0f} deg nose {"down" if pitch > 0 else "up"}'
        if not self.use_pose():
            self.get_logger().warn(f'{why}: {lean}, but Pose mode is unavailable '
                                   f'({self.pose_status}): no pitch compensation')
            return None
        lim = abs(self.cfg['pose_max_pitch'])
        comp = max(-lim, min(lim, -pitch))
        self.get_logger().warn(f'{why}: {lean} (roll {math.degrees(roll):+.0f} deg): Pose-mode '
                               f'pitch {math.degrees(comp):+.0f} deg'
                               + (' (pose_max_pitch limit)' if abs(pitch) > lim else ''))
        return comp

    def pose_side_wall(self, j, step, xy, yaw):
        """A dead-end side-wall step in Pose mode: yaw the body (feet still) towards it, as
        far as pose_max_yaw / the walls allow, and hold dead_end_side_hold s there."""
        if not self.pose_live or 'stance' not in j:
            j['stance'] = yaw                    # body centred: this is the feet's heading
        stance = j['stance']
        h = side_heading(xy, step['point'], step.get('facing'),
                         self.cfg['dead_end_look_angle'])
        want = wrap(h - stance)
        pitch, body_yaw = j.get('pitch'), None     # tilt compensation of this dead end
        grid, margin = self.grid_now(), self.cfg['pose_margin']
        if abs(want) >= math.radians(5) and footprint_hits(grid, xy, stance, self.box, margin):
            # Already within pose_margin of a wall: any body yaw would push into it (motor
            # buzzing, run 2026-11-21 10:56: squaring up was blocked, then 2 x full axis).
            self.get_logger().warn(f'{step["name"]}: body within {margin * 100:.0f} cm of a '
                                   'wall here: no Pose-mode yaw, holding as it is')
            want = 0.0
        if abs(want) >= math.radians(5):
            sign = 1 if want > 0 else -1
            lim = min(abs(want), abs(self.cfg['pose_max_yaw']) * self.cfg['pose_axis_limit'])
            over = abs(self.cfg['pose_overshoot'])
            ok = max_turn(grid, xy, stance, self.box, sign, lim + over, margin,
                          pivot=self.pivot) - over      # the body may swing a little past
            ok = min(ok, lim)
            if ok < math.radians(5):
                self.get_logger().warn(f'{step["name"]}: body yaw '
                                       f'{math.degrees(sign * lim):+.0f} deg blocked by walls; '
                                       'holding here')
            else:
                if ok < abs(want) - math.radians(2):
                    self.get_logger().info(f'{step["name"]}: wall is {math.degrees(want):+.0f} '
                                           f'deg off; body yaw {math.degrees(sign * ok):+.0f} '
                                           'deg (Pose mode limit / walls)')
                body_yaw = sign * ok
        if body_yaw is None and pitch is None:
            return                                   # plain hold (captured in look_tick)
        j['posed'] = True
        self.send_pose(body_yaw or 0.0, step['hold'], step['name'], pitch=pitch,
                       scene=step.get('scene'))

    # ------------------------------------------------------------ pose-mode looks
    def on_pose_status(self, msg):
        self.pose_status, self.pose_status_at = msg.data, time.monotonic()

    def on_terrain(self, msg):
        try:
            rough = bool(json.loads(msg.data).get('on_rough'))
        except (ValueError, AttributeError):
            return
        if rough != self.on_rough:
            self.get_logger().info('standing on rough ground: Nav2 spins instead of Pose looks'
                                   if rough else 'on even ground again: Pose looks')
        self.on_rough, self.terrain_at = rough, time.monotonic()

    def use_pose(self):
        """Pose-mode looks on and cmd_vel_bridge able to send them (else Nav2 spins), and not
        standing on rough ground (/maze/terrain, fresh within 2 s)."""
        if getattr(self, 'on_rough', False) and time.monotonic() - self.terrain_at <= 2.0:
            return False
        return self.pose_pub is not None and self.pose_status is not None and \
            not self.pose_status.startswith('UNAVAILABLE')

    def send_pose(self, yaw, hold, why, pitch=None, scene=None):
        """Body yaw (rad from the stance heading, + = left), held `hold` s once there;
        pitch (rad, + = nose down) with it, or None. cmd_vel_bridge sends the pitch axis
        only inside a yaw look, so both go out together at 10 Hz (pose_tick). scene: the
        dead-end view captured at the end of the hold, still posed."""
        self.spin_busy, self.spin_why = True, why
        self.pose_live = True
        self.pose = {'yaw': float(yaw), 'hold': float(hold), 'sent': time.monotonic(),
                     'since': None, 'pitch': pitch, 'scene': scene}
        self.get_logger().info(f'{why}: Pose mode body yaw {math.degrees(yaw):+.0f} deg'
                               + (f', pitch {math.degrees(pitch):+.0f} deg'
                                  if pitch is not None else '') + f', hold {hold:.1f} s')
        self.pose_pub.publish(Float32(data=float(yaw)))
        if pitch is not None and self.pose_pitch_pub is not None:
            self.pose_pitch_pub.publish(Float32(data=float(pitch)))

    def pose_tick(self):
        """10 Hz: keep the running pose look's target fresh; end it after its hold."""
        p = self.pose
        if p is None:
            return
        now = time.monotonic()
        st = self.pose_status or ''
        if st.startswith('UNAVAILABLE') and self.pose_status_at >= p['sent']:
            return self.end_pose(f'cmd_vel_bridge: {st}; look skipped (Nav2 spins from now on)',
                                 warn=True)
        if p['since'] is None and st == 'POSING':
            p['since'] = now
        if p['since'] is not None and now >= p['since'] + self.cfg['pose_turn_time'] + p['hold']:
            self.capture_view(p.get('scene'), p['hold'])     # before the body swings back
            self.pose, self.spin_busy = None, False          # next step, or release_pose()
            return
        if p['since'] is None and now - p['sent'] > self.cfg['pose_wait']:
            return self.end_pose(f'cmd_vel_bridge not in Pose mode after '
                                 f'{self.cfg["pose_wait"]:.0f} s ({st}); look skipped', warn=True)
        self.pose_pub.publish(Float32(data=p['yaw']))
        if p.get('pitch') is not None and self.pose_pitch_pub is not None:
            self.pose_pitch_pub.publish(Float32(data=float(p['pitch'])))

    def end_pose(self, why, warn=False):
        """Stop a pose look now (interrupted / failed) and release Pose mode."""
        if self.pose is not None:
            (self.get_logger().warn if warn else self.get_logger().info)(
                f'{self.spin_why}: pose look ended ({why})')
            self.pose, self.spin_busy = None, False
        self.release_pose()

    def release_pose(self):
        """Tell cmd_vel_bridge the pose looks are done (NaN): leave Pose mode now."""
        if self.pose_live and self.pose_pub is not None:
            self.pose_pub.publish(Float32(data=float('nan')))
        self.pose_live = False

    # ------------------------------------------------------------ after a junction look
    def plan_after_junction(self, rec):
        """After scanning every adjacent unexplored cell from the junction cell (user
        2026-11-21): candidates queued (stairs / curtains, from this scan or earlier dead
        ends) -> check them, nearest first (start_queue; step() drives), and explore on
        from there. None -> drive one cell into the nearest way that leads to an unexplored
        cell and isn't a dead end, and explore on. Each way is traced on the map
        (scene_cells.trace_way): one that ends in a dead end is blacklisted, not visited.
        Returns the list of actions for after_junction_tick."""
        cells, cell = rec.get('cells'), rec['cell']
        for st in (self.junction or {}).get('steps') or []:
            if st.get('scan') and st['cell'] in self.blocked_cells:
                self.mark_explored(st['cell'], 'object seen in the junction scan')
        if self.start_queue(f'junction {list(cell)} scanned'):
            return []
        if cells is None:
            self.get_logger().warn('/scene/junction has no grid (old scene_trigger?): '
                                   'exploring on')
            return []
        grid = self.grid_now()
        if grid is None:
            return []
        hw = rec['half_width']

        def openings(c, d):
            return opening(grid, cells, c, d, half_width=hw)

        pose = self.robot_pose()
        xy, yaw = pose if pose is not None else (cells.centre(cell), 0.0)
        ways = []
        for name, heading in rec['paths']:
            d = DIRECTIONS.get(name)
            if d is None:
                continue
            nxt = (cell[0] + d[0], cell[1] + d[1])
            centre = cells.centre(nxt)
            t = trace_way(openings, cell, d, int(self.cfg['junction_trace_cells']))
            why = None
            if self.back_to_start(centre):
                why = 'back to the start cell'
            elif d in (rec.get('skip_dirs') or []):
                why = 'a held curtain'
            elif self.cells.cell(centre) in self.visited:
                why = 'visited already'
            elif self.cells.cell(centre) in self.explored_cells | set(self.blocked_cells):
                why = 'an object room'
            elif t['kind'] == 'dead_end':
                # Not blacklisted here (user 2026-11-22): the adjacent one was just viewed
                # by this scan, so it waits for scene understanding; one further down is
                # viewed when the robot faces it from the cell before (/scene/trigger).
                dc = self.cells.cell(cells.centre(t['cell']))
                if tuple(t['cell']) == nxt and dc in self.scanned_cells:
                    self.hold_dead_end(dc, cells.centre(t['cell']), heading,
                                       f'junction {list(cell)} way {name}: dead end '
                                       f'{list(t["cell"])}', captured=True)
                why = f'a dead end at cell {list(t["cell"])}'
            if why:
                self.get_logger().info(f'junction {list(cell)} way {name}: {why}; not taken')
                continue
            dist = math.hypot(centre[0] - xy[0], centre[1] - xy[1])
            ways.append((dist, abs(wrap(heading - yaw)), name, nxt, centre, heading))
        if not ways:
            self.get_logger().info(f'junction {list(cell)}: no unexplored way to take; '
                                   'exploring on')
            return []
        _, _, name, nxt, centre, heading = min(ways)
        self.get_logger().info(f'junction {list(cell)}: leaving by the nearest unexplored way '
                               f'{name} (cell {list(nxt)})')
        return [{'do': 'goto', 'xy': centre, 'yaw': heading, 'group': 'leave',
                 'name': f'nearest unexplored way {name}: to cell {list(nxt)}'}]

    def after_junction_tick(self, j, now):
        """Run the junction follow-up's first action: goto (Nav2), align (spin), hold, look
        (side-wall steps, run by look_tick; the action stays first while they run so
        at_dead_end() keeps perception on, and pause_junction() resumes them)."""
        act = j['todo'][0]
        if act['do'] == 'look':
            if not act.get('steps_added'):
                act['steps_added'] = True
                j.pop('stance', None)            # body centred on the squared-up heading
                j['steps'].extend(act['steps'])
                self.set_state('JUNCTION', act['name'])
            else:                                # look_tick ran them all
                self.next_after_junction(j)
            return
        if act['do'] == 'hold':
            if 'until' not in act:
                j['pitch'] = self.tilt_pitch(act['name']) if act.get('scene') else None
                self.set_state('JUNCTION', f'{act["name"]}: holding {act["s"]:.1f} s')
                if j['pitch'] is not None:       # Pose-mode hold: done when the pose ends
                    act['until'] = now
                    j.pop('stance', None)
                    return self.send_pose(0.0, act['s'], act['name'], pitch=j['pitch'],
                                          scene=act['scene'])
                act['until'] = now + act['s']
            if now >= act['until']:
                self.capture_view(act.get('scene'), act['s'])
                self.next_after_junction(j)
            return
        if act['do'] == 'goto':
            if 'sent' not in act:
                act['sent'], act['status'] = now, None
                self.set_state('JUNCTION', act['name'])
                self.send_junction_goal(act)
                return
            if act['status'] is None:
                if now - act['sent'] > self.cfg['goal_timeout']:
                    self.get_logger().warn(f'{act["name"]}: timed out; cancelling')
                    self.cancel_junction_goal()
                    self.next_after_junction(j, failed=True)
                return
            pose = self.robot_pose()
            if act['status'] == GoalStatus.STATUS_SUCCEEDED and pose is not None and \
                    self.false_success(act['xy'], pose[0]):
                act['status'] = GoalStatus.STATUS_ABORTED
            if act['status'] != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'{act["name"]}: failed (status {act["status"]})')
                self.next_after_junction(j, failed=True)
            else:
                self.next_after_junction(j)
            return
        # align: turn on the spot until the heading is within junction_align_tol
        pose = self.robot_pose()
        if pose is None or self.map is None:
            return
        xy, yaw = pose
        want = wrap(act['heading'] - yaw)
        tol = self.cfg['junction_align_tol']
        if abs(want) <= tol or act['tries'] >= 3:
            self.get_logger().info(f'{act["name"]}: {math.degrees(want):+.1f} deg off the '
                                   'walls' + (' (gave up squaring up)' if abs(want) > tol else ''))
            self.next_after_junction(j)
            return
        act['tries'] += 1
        grid = self.grid_now()
        sign, other = (1 if want > 0 else -1), 2 * math.pi - abs(want)
        for margin in (self.cfg['turn_margin'], 0.01):
            if max_turn(grid, xy, yaw, self.box, sign, abs(want), margin,
                        pivot=self.pivot) >= abs(want) - 1e-6:
                return self.send_spin(sign * abs(want), act['name'])
            if max_turn(grid, xy, yaw, self.box, -sign, other, margin,
                        pivot=self.pivot) >= other - 1e-6:
                return self.send_spin(-sign * other, act['name'] + ' (the long way)')
        self.get_logger().warn(f'{act["name"]}: {math.degrees(want):+.0f} deg turn blocked by '
                               'walls both ways; holding as it is')
        self.next_after_junction(j)

    def next_after_junction(self, j, failed=False):
        """Done with the first action; failed = skip the rest of its group too."""
        done = j['todo'].pop(0)
        if failed:
            j['todo'] = [a for a in j['todo'] if a['group'] != done['group']]

    def send_junction_goal(self, act):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = act['xy']
        msg.pose.orientation.z = math.sin(act['yaw'] / 2)
        msg.pose.orientation.w = math.cos(act['yaw'] / 2)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'{act["name"]}: goal ({act["xy"][0]:.2f}, {act["xy"][1]:.2f})')
        self.cancel_junction_goal()
        future = self.nav.send_goal_async(NavigateToPose.Goal(pose=msg))
        future.add_done_callback(lambda fut, a=act: self.on_junction_goal_accepted(fut, a))

    def on_junction_goal_accepted(self, future, act):
        handle = future.result()
        j = self.junction
        if j is None or not j.get('todo') or j['todo'][0] is not act:   # interrupted meanwhile
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            act['status'] = GoalStatus.STATUS_ABORTED
            return
        self.junction_goal = handle
        handle.get_result_async().add_done_callback(
            lambda fut, a=act, h=handle: self.on_junction_goal_result(fut, a, h))

    def on_junction_goal_result(self, future, act, handle):
        if self.junction_goal is handle:
            self.junction_goal = None
        act['status'] = future.result().status

    def cancel_junction_goal(self):
        if self.junction_goal is not None:
            self.junction_goal.cancel_goal_async()
            self.junction_goal = None

    def halt_for_look(self):
        """Stop driving (cancel the Nav2 goal) before a look-around."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.stop_robot()

    def end_turn_creep(self, why):
        """Crept ahead for a blocked turn: turn now, if it is clear from here."""
        tc, self.turn_creep = self.turn_creep, None
        self.stop_robot()
        self.get_logger().info(f'turn creep: {why}')
        pose = self.robot_pose()
        if pose is None or self.map is None:
            return                                  # step() moves on to EXPLORING
        xy, yaw = pose
        grid = self.grid_now()
        heading = heading_towards(grid, xy, tc['goal'], self.cfg['robot_radius'])
        want = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        if abs(want) < math.radians(5):
            return
        m, s = tc['margin'], (1.0 if want >= 0 else -1.0)
        other = 2 * math.pi - abs(want)
        if max_turn(grid, xy, yaw, self.box, s, abs(want), m, pivot=self.pivot) >= abs(want) - 1e-6:
            angle = want
        elif max_turn(grid, xy, yaw, self.box, -s, other, m, pivot=self.pivot) >= other - 1e-6:
            angle = -s * other
        else:
            self.get_logger().warn(f'turn creep: {math.degrees(want):+.0f} deg still blocked '
                                   'here; leaving it to Nav2')
            return
        self.send_spin(angle, 'turn after creep')

    def begin(self):
        self.set_state('EXPLORING', 'go' if self.cfg['go'] else 'DRY RUN (go:=false)')

    def find(self, grid, xy, exclude):
        fs = find_frontiers(grid, xy, self.cfg['robot_radius'], self.cfg['goal_clearance'],
                            self.cfg['min_frontier_size'], exclude,
                            min_width=self.min_frontier_width())
        # The lidar never saw behind the start pose: that patch reads as a frontier, but it
        # is the start cell's back wall (run 2026-11-20 01:28: goal (-0.73, 0.44), +67 deg).
        return [f for f in fs if not self.back_to_start(f.goal)
                and self.in_grid(f.goal)
                and not self.in_object(f.goal)
                and not self.no_go_cell(self.cells.cell(f.goal))
                and self.cells.cell(f.goal) not in self.explored_cells]

    def in_reach(self, frontiers, xy):
        off = self.cfg['goal_max_offset']
        return [f for f in frontiers
                if abs(f.goal[0] - xy[0]) < off and abs(f.goal[1] - xy[1]) < off]

    def recover_or_finish(self, grid, xy, why):
        """Goals keep failing: retry while open frontiers remain, else NO_EXIT."""
        open_ = self.in_reach(self.find(grid, xy, self.blacklist), xy)
        if (not open_ or self.recoveries >= self.cfg['max_recoveries']) and \
                self.start_queue(f'{why}; nothing else to explore'):
            self.failed, self.failures, self.empty_replans = [], 0, 0
            return
        if not open_:
            if self.go_to_exit(f'{why}; nothing else reachable to explore'):
                return
            self.finish('NO_EXIT', f'{why}; no open frontier left')
            self.stop_robot()
            return
        if self.recoveries >= self.cfg['max_recoveries']:
            if self.go_to_exit(f'{why}; {len(open_)} frontiers unreachable after '
                               f'{self.recoveries} recoveries'):
                return
            self.finish('NO_EXIT', f'{why}; {len(open_)} frontiers still open but unreachable '
                        f'after {self.recoveries} recoveries')
            self.stop_robot()
            return
        self.recoveries += 1
        self.get_logger().warn(
            f'{why}, but {len(open_)} frontiers are still open: recovery '
            f'{self.recoveries}/{self.cfg["max_recoveries"]} - clearing costmaps, retrying them')
        for c in self.clear_costmaps:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        self.failed, self.failures, self.empty_replans = [], 0, 0

    def scan_or_begin(self):
        if self.cfg['start_scan']:
            self.set_state('SCANNING', 'turning left and right to see the start area')
        else:
            self.begin()

    def plan_scan(self, pose):
        """Left, right, back: each side as far as scan_angle or until the body would touch a
        wall on the map, so it ends on the heading it started with."""
        xy, yaw = pose
        grid, a = self.grid_now(), abs(self.cfg['scan_angle'])
        # The start scan always turns the robot (Nav2 /spin), never a Pose-mode body yaw
        # (user 2026-11-20); pose_looks only applies to the dead-end side-wall looks.
        self.scan_pose = False
        m = self.cfg['turn_margin']
        left = max_turn(grid, xy, yaw, self.box, +1, a, m, pivot=self.pivot)
        right = max_turn(grid, xy, yaw, self.box, -1, a, m, pivot=self.pivot)
        dl, dr = math.degrees(left), math.degrees(right)
        if left < math.radians(5) and right < math.radians(5):
            self.get_logger().warn(f'start scan skipped: walls too close to turn '
                                   f'({dl:.0f} deg left, {dr:.0f} deg right possible)')
            self.scan_turns = []
            return
        if left < a or right < a:
            self.get_logger().warn(f'start scan limited by walls: {dl:.0f} deg left, '
                                   f'{dr:.0f} deg right')
        if self.scan_pose:
            self.get_logger().info(f'start scan in Pose mode (feet still): body {dl:.0f} deg '
                                   f'left, then {dr:.0f} deg right, then centre')
            self.scan_turns = [t for t in (left, -right) if abs(t) > math.radians(5)]
            return
        self.get_logger().info(f'start scan: {dl:.0f} deg left, {dl + dr:.0f} right, '
                               f'{dr:.0f} left')
        self.scan_turns = [t for t in (left, -(left + right), right) if abs(t) > 1e-3]

    def back_to_start(self, p):
        """Is map point p in the start cell (within 0.5 m of the start pose) or behind the
        start pose (start_back_depth m back, start_back_width m either side)? Goals, junction
        looks and junction follow-ups there would turn the robot to the start cell's back."""
        depth = self.cfg['start_back_depth']
        if self.start_pose is None or depth <= 0:
            return False
        (sx, sy), syaw = self.start_pose
        dx, dy = p[0] - sx, p[1] - sy
        if math.hypot(dx, dy) < 0.5:
            return True
        along = dx * math.cos(syaw) + dy * math.sin(syaw)
        side = -dx * math.sin(syaw) + dy * math.cos(syaw)
        return -depth <= along < -0.1 and abs(side) <= self.cfg['start_back_width']

    def creep_or_scan(self, pose):
        if self.start_pose is None:
            self.start_pose = (tuple(pose[0]), pose[1])
            self.get_logger().info(f'start pose ({pose[0][0]:.2f}, {pose[0][1]:.2f}), heading '
                                   f'{math.degrees(pose[1]):.0f} deg: no goals or turns '
                                   'into the start cell or behind it')
        if self.cfg['creep_distance'] > 0:
            self.begin_creep(pose)
        else:
            self.scan_or_begin()

    def begin_creep(self, pose):
        xy, yaw = pose
        d = self.cfg['creep_distance']
        grid = self.grid_now()
        self.creep_from = xy
        # Shorter if a wall ahead is close (e.g. the front 0.15 m from a wall): end with
        # creep_margin to spare.
        free = free_distance_ahead(grid, xy, yaw, self.box, d,
                                   self.cfg['creep_margin'])
        if free < d:
            self.get_logger().warn(f'creep shortened to {free * 100:.0f} cm: wall ahead')
        if free < 0.02:
            self.creep_target = 0.0
            self.set_state('CREEPING', 'skipped: wall right ahead')
            return self.end_creep('creep skipped: wall right ahead')
        self.creep_target = free
        self.creep_until = time.monotonic() + free / self.cfg['creep_speed'] + 3.0
        self.set_state('CREEPING', f'{free * 100:.0f} cm straight ahead, no steering')

    def creep_tick(self):
        if self.state == 'TO_EXIT' and self.launch is not None and self.launch['creep']:
            return self.launch_creep_tick()
        tc = self.turn_creep
        if tc is not None and self.state != 'TURNING':
            self.turn_creep = None                  # taken over (STAIRS) or finished
            self.stop_robot()
            return
        if self.state != 'CREEPING' and tc is None:
            return
        start, target, until = ((tc['from'], tc['target'], tc['until']) if tc is not None
                                else (self.creep_from, self.creep_target, self.creep_until))
        pose = self.robot_pose()
        done, moved = None, 0.0
        if pose is not None:
            moved = math.hypot(pose[0][0] - start[0], pose[0][1] - start[1])
            if moved >= target:
                done = f'crept {moved * 100:.0f} cm'
        if done is None and time.monotonic() > until:
            done = f'creep timed out after {moved * 100:.0f} cm'
        if done is None:
            msg = Twist()
            msg.linear.x = float(self.cfg['creep_speed'])   # angular stays 0: no steering
            self.cmd_pub.publish(msg)
            return
        if tc is not None:
            return self.end_turn_creep(done)
        self.end_creep(done)

    def end_creep(self, why):
        self.stop_robot()
        self.get_logger().info(why)
        # The lidar never saw behind the start pose, so that patch reads as a big frontier
        # (seen in the sim even after a 360 deg turn). It is the start cell's back, not a way
        # on: an earlier run drove back into the dead-end start cell and got stuck turning.
        self.blacklist.append(self.creep_from)
        self.scan_or_begin()

    def send_spin(self, angle, why='start scan'):
        self.spin_busy = True
        self.spin_why = why
        # Spins run at max_rotational_vel 0.4 rad/s, slowing to min 0.1 only at the end: half
        # the max plus 5 s covers that, and a robot that doesn't move is still caught quickly
        allowance = abs(angle) / 0.2 + 5.0
        goal = Spin.Goal(target_yaw=float(angle),
                         time_allowance=Duration(sec=int(math.ceil(allowance))))
        self.get_logger().info(f'{why}: turn {math.degrees(angle):+.0f} deg')
        self.spin.send_goal_async(goal).add_done_callback(self.on_spin_accepted)

    def on_spin_accepted(self, future):
        handle = future.result()
        if self.state not in ('SCANNING', 'TURNING', 'FACING_QR',
                              'JUNCTION', 'DEAD_END', 'TO_EXIT'):   # stopped meanwhile
            if handle.accepted:
                handle.cancel_goal_async()
            self.spin_busy = False
            return
        if not handle.accepted:
            self.get_logger().warn(f'{self.spin_why}: turn rejected; continuing')
            self.spin_busy = False
            return
        self.spin_handle = handle
        handle.get_result_async().add_done_callback(self.on_spin_result)

    def on_spin_result(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'{self.spin_why}: turn ended with status {status}; continuing')
        self.spin_handle = None
        self.spin_busy = False

    def send(self, f, n):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = f.goal
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(f.yaw / 2), math.cos(f.yaw / 2)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'frontier goal ({f.goal[0]:.2f}, {f.goal[1]:.2f}) '
                               f'{f.path_length:.1f} m away, {f.width:.2f} m wide, {f.size} cells; '
                               f'{n} frontiers')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.current = f
        if not self.cfg['go']:
            return
        self.goal_sent_at = time.monotonic()
        self.goal_moved = None          # (xy, monotonic) where the robot last made progress
        self.goal_result = None
        future = self.nav.send_goal_async(NavigateToPose.Goal(pose=msg))
        future.add_done_callback(lambda fut, f=f: self.on_accepted(fut, f))

    def on_accepted(self, future, f):
        handle = future.result()
        if f is not self.current:                   # superseded while being accepted
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.goal_result = GoalStatus.STATUS_ABORTED
            self.goal_result_at = time.monotonic()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fut, f=f: self.on_result(fut, f))

    def on_result(self, future, f):
        if f is self.current:
            self.goal_result = future.result().status
            self.goal_result_at = time.monotonic()

    def goal_stalled(self, xy):
        """The current goal is stuck: no goal_progress m of movement for goal_timeout s, or
        goal_max_time s since it was sent (logs why)."""
        now = time.monotonic()
        if self.goal_moved is None or \
                math.hypot(xy[0] - self.goal_moved[0][0], xy[1] - self.goal_moved[0][1]) >= \
                self.cfg['goal_progress']:
            self.goal_moved = ((xy[0], xy[1]), now)
        if now - self.goal_moved[1] > self.cfg['goal_timeout']:
            self.get_logger().warn(f'goal timed out: robot moved < {self.cfg["goal_progress"]:.2f}'
                                   f' m in {self.cfg["goal_timeout"]:.0f} s; cancelling')
            return True
        if now - self.goal_sent_at > self.cfg['goal_max_time']:
            self.get_logger().warn(f'goal timed out after {self.cfg["goal_max_time"]:.0f} s; '
                                   'cancelling')
            return True
        return False

    def in_open_space(self):
        """Open floor next to the robot (ExitDetector's last update): no looks here."""
        f = self.cfg['open_space_no_looks']
        return f > 0 and self.exit.open_nearby >= f * self.exit.open_radius

    def out_of_maze(self, pose):
        """ExitDetector on the current map; finishes EXITED (and returns True) once out.
        With the maze size known, only once the robot itself is more than exit_cell_margin m
        outside the grid (a fallback: it is meant to stay in): in the launch cell next to
        the exit the open floor outside is within reach too."""
        xy = pose[0]
        if not self.exit.update(self.grid_now(), xy, time.monotonic()):
            return False
        if self.grid_known() and self.in_grid(xy, self.cfg['exit_cell_margin']):
            return False
        self.finish('EXITED', f'open space around ({xy[0]:.2f}, {xy[1]:.2f}), '
                    f'{self.exit.open_nearby:.2f} m of open floor next to it'
                    + (f' (ended {self.state})' if self.state != 'EXPLORING' else ''))
        return True

    # ------------------------------------------------------------------ exit / launch cell
    def grid_known(self):
        return self.cfg['maze_cols'] > 0 and self.cfg['maze_rows'] > 0

    def in_grid(self, xy, margin=0.0):
        """xy inside the maze_cols x maze_rows grid (+ margin m); True with the size unknown."""
        return exit_cell(self.cells, xy, self.cfg['maze_cols'], self.cfg['maze_rows'],
                         margin) is None

    def fit_cells(self, grid):
        """Refit the cell grid to the walls on the map (every grid_fit_period s); changes
        self.cells in place, so everything holding it follows. Cell indices stay the same
        (at most grid_fit_max_shift m and grid_fit_max_yaw from the start-pose grid)."""
        cfg = self.cfg
        now = time.monotonic()
        if not (cfg['grid_fit'] and self.grid_known()) or grid is None or \
                now - self.grid_fit_at < cfg['grid_fit_period']:
            return
        self.grid_fit_at = now
        rows, cols_ = np.nonzero(grid.data >= OCCUPIED_MIN)
        if len(rows) < cfg['grid_fit_min_points']:
            return
        xy = np.c_[grid.origin[0] + (cols_ + 0.5) * grid.resolution,
                   grid.origin[1] + (rows + 0.5) * grid.resolution]
        c0, s0 = math.cos(self.cells.yaw), math.sin(self.cells.yaw)
        d = xy - np.asarray(self.cells.origin)
        u = (c0 * d[:, 0] + s0 * d[:, 1]) / self.cells.size
        v = (-s0 * d[:, 0] + c0 * d[:, 1]) / self.cells.size
        m = 0.4 / self.cells.size                   # walls on the outline + a little
        near = (u >= -0.5 - m) & (u <= cfg['maze_cols'] - 0.5 + m) & \
            (v >= -0.5 - m) & (v <= cfg['maze_rows'] - 0.5 + m)
        if near.sum() < cfg['grid_fit_min_points']:
            return
        res = fit_grid(xy[near], self.cells0, max_yaw=cfg['grid_fit_max_yaw'])
        if res is None or res[1] < cfg['grid_fit_min_score']:
            return
        fit, score = res
        shift = math.hypot(fit.origin[0] - self.cells0.origin[0],
                           fit.origin[1] - self.cells0.origin[1])
        if shift > cfg['grid_fit_max_shift']:
            return
        moved = math.hypot(fit.origin[0] - self.cells.origin[0],
                           fit.origin[1] - self.cells.origin[1])
        turned = abs(wrap(fit.yaw - self.cells.yaw))
        if moved < 0.015 and turned < math.radians(0.3):
            return
        self.cells.origin, self.cells.yaw = fit.origin, fit.yaw
        self.get_logger().info(
            f'grid fitted to the walls ({int(near.sum())} px, score {score:.2f}): yaw '
            f'{math.degrees(fit.yaw):+.1f} deg, cell (0, 0) at ({fit.origin[0]:+.2f}, '
            f'{fit.origin[1]:+.2f}); moved {moved * 100:.0f} cm, turned '
            f'{math.degrees(turned):.1f} deg')
        self.grid_pub.publish(String(data=json.dumps(
            {'size': self.cells.size, 'origin': [round(v_, 4) for v_ in fit.origin],
             'yaw': round(fit.yaw, 5), 'score': round(score, 3), 'frame': 'map'})))
        if self.launch is not None and self.exit_side is not None:   # follow it
            cell, dd = self.exit_side
            self.launch['xy'], self.launch['yaw'] = (self.cells.centre(cell),
                                                     self.cells.heading(dd))

    def update_exit(self, grid):
        """Count, every replan, which outer sides of the grid read open (side_open). The exit
        is only chosen once the maze is explored (choose_exit): early on, the start cell's
        walls are too close for the lidar's wall band, and beams passing just over them mark
        the floor beyond free, so its sides read open (sim 2026-11-22: every run took cell
        [0, 0]'s side as the exit after 3 replans)."""
        if self.exit_side is not None or not self.grid_known() or grid is None:
            return
        for side in boundary_sides(self.cfg['maze_cols'], self.cfg['maze_rows']):
            if self.side_open(grid, side):
                self.exit_open[side] = self.exit_open.get(side, 0) + 1

    def choose_exit(self, grid):
        """Maze explored: the exit is an outer side open on the final map that also read
        open on >= exit_confirm replans (the most often open if several). None = no exit."""
        if self.exit_side is not None:
            return self.exit_side
        if not self.grid_known() or grid is None:
            return None
        cands = [side for side in boundary_sides(self.cfg['maze_cols'], self.cfg['maze_rows'])
                 if self.exit_open.get(side, 0) >= self.cfg['exit_confirm']
                 and self.side_open(grid, side)]
        if not cands:
            self.get_logger().warn('no open outer side on the map: no exit found')
            return None
        side = max(cands, key=lambda c: self.exit_open[c])
        if len(cands) > 1:
            self.get_logger().warn('several outer sides open: ' + ', '.join(
                f'{list(c)} -> {list(d)} ({self.exit_open[(c, d)]}x)' for c, d in cands)
                + '; taking the most often open')
        self.exit_side = side
        self.publish_launch('found')
        cell, d = side
        self.get_logger().info(f'exit: cell {list(cell)} is open towards {list(d)} '
                               f'({self.exit_open[side]} replans); launch cell {list(cell)}')
        return side

    def side_open(self, grid, side, shifts=(0.0, 0.15, -0.15, 0.3, -0.3)):
        """The outer side reads open, with free floor seen past it, at its middle or slid up
        to 0.3 m along it. The grid comes from the start pose: 5 deg off at the start puts
        the far end of a 5-cell maze ~0.4 m off the real cells (sim 2026-11-22), so the real
        opening is no longer under the side's middle."""
        cell, d = side
        c, s = math.cos(self.cells.yaw), math.sin(self.cells.yaw)
        along = (c * -d[1] - s * d[0], s * -d[1] + c * d[0])   # map direction along the side
        for k in shifts:
            cells = dataclasses.replace(self.cells, origin=(self.cells.origin[0] + k * along[0],
                                                            self.cells.origin[1] + k * along[1]))
            if opening(grid, cells, cell, d) == 'open' and self.seen_beyond(grid, side, cells):
                return True
        return False

    def seen_beyond(self, grid, side, cells=None, depth=0.45, spread=0.2):
        """Free floor seen `depth` m past the side (2 of 3 points `spread` m apart). A grid
        a few degrees off the real cells (hand-placed start) puts its outer line inside the
        maze, where the strip reads open; past it is then the real wall or unseen space."""
        cells = cells or self.cells
        cell, d = side
        n = 0
        for s in (-spread, 0.0, spread):
            u = cell[0] + d[0] * (0.5 + depth / cells.size) + d[1] * s / cells.size
            v = cell[1] + d[1] * (0.5 + depth / cells.size) + d[0] * s / cells.size
            c = grid.cell(cells.world((u, v)))
            n += bool(grid.contains(c) and 0 <= grid.data[c[1], c[0]] <= FREE_MAX)
        return n >= 2

    def publish_launch(self, state, xy=None):
        """/maze/launch_cell (latched JSON): 'found' when the exit is seen, 'at' when parked."""
        cell, d = self.exit_side
        cx, cy = self.cells.centre(cell)
        msg = {'state': state, 'launch_cell': list(cell),
               'exit_cell': [cell[0] + d[0], cell[1] + d[1]], 'exit_direction': list(d),
               'centre': [round(cx, 3), round(cy, 3)],
               'heading': round(self.cells.heading(d), 3), 'frame': 'map'}
        if xy is not None:
            msg['robot'] = [round(xy[0], 3), round(xy[1], 3)]
        self.launch_pub.publish(String(data=json.dumps(msg)))

    def go_to_exit(self, why):
        """Everything inside explored: drive to the launch cell (the one inside the exit),
        facing out (TO_EXIT). False if no exit was found."""
        if self.choose_exit(self.grid_now()) is None:
            return False
        cell, d = self.exit_side
        self.launch = {'cell': cell, 'xy': self.cells.centre(cell),
                       'yaw': self.cells.heading(d), 'tries': 0, 'handle': None,
                       'result': None, 'sent_at': 0.0, 'turns': 0, 'centred': False,
                       'creep': None}
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.current, self.goal_result = None, None
        self.set_state('TO_EXIT', f'{why}: to launch cell {list(cell)}, facing the exit')
        self.send_launch_goal()
        return True

    def send_launch_goal(self):
        L = self.launch
        L['tries'] += 1
        L['result'], L['handle'], L['sent_at'] = None, None, time.monotonic()
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = L['xy']
        msg.pose.orientation.z = math.sin(L['yaw'] / 2)
        msg.pose.orientation.w = math.cos(L['yaw'] / 2)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'launch cell {list(L["cell"])}: goal ({L["xy"][0]:.2f}, '
                               f'{L["xy"][1]:.2f}) facing {math.degrees(L["yaw"]):.0f} deg '
                               f'(try {L["tries"]})')
        if not self.cfg['go']:
            return
        future = self.nav.send_goal_async(NavigateToPose.Goal(pose=msg))
        future.add_done_callback(lambda fut, L=L: self.on_launch_accepted(fut, L))

    def on_launch_accepted(self, future, L):
        handle = future.result()
        if L is not self.launch or self.state != 'TO_EXIT':
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            L['result'] = GoalStatus.STATUS_ABORTED
            return
        L['handle'] = handle
        handle.get_result_async().add_done_callback(
            lambda fut, h=handle, L=L: self.on_launch_result(fut, h, L))

    def on_launch_result(self, future, handle, L):
        if L is self.launch and L['handle'] is handle:
            L['result'] = future.result().status

    def launch_tick(self, pose):
        """TO_EXIT: park once the goal is reached within launch_tolerance; else try again,
        up to launch_retries goals; then park anyway if inside the launch cell, else STOPPED."""
        xy, yaw = pose
        L = self.launch
        if not self.cfg['go']:
            return self.park(xy, yaw, 'go:=false, not driven')
        if self.spin_busy or L['creep']:
            return                                  # turning / creeping to the centre
        if L['result'] is None:
            if time.monotonic() - L['sent_at'] <= self.cfg['goal_max_time']:
                return
            self.get_logger().warn(f'launch cell goal still running after '
                                   f'{self.cfg["goal_max_time"]:.0f} s; cancelling')
            if L['handle'] is not None:
                L['handle'].cancel_goal_async()
            L['result'] = GoalStatus.STATUS_ABORTED
        off = math.hypot(xy[0] - L['xy'][0], xy[1] - L['xy'][1])
        if L['result'] == GoalStatus.STATUS_SUCCEEDED and off <= self.cfg['launch_tolerance']:
            if not L['centred']:
                L['centred'] = True
                if self.creep_to_centre(xy, yaw):
                    return
            if L['turns'] < 2 and self.face_exit(xy, yaw):
                return
            return self.park(xy, yaw)
        if L['tries'] < self.cfg['launch_retries']:
            self.get_logger().warn(f'launch cell goal ended (status {L["result"]}) {off:.2f} m '
                                   'from the cell centre; sending it again')
            return self.send_launch_goal()
        if self.cells.cell(xy) == L['cell']:
            return self.park(xy, yaw, f'{off:.2f} m off centre after {L["tries"]} goals')
        self.finish('STOPPED', f'could not reach launch cell {list(L["cell"])} after '
                    f'{L["tries"]} goals ({off:.2f} m from its centre)')
        self.stop_robot()

    def creep_to_centre(self, xy, yaw):
        """Nav2 stops up to xy_goal_tolerance (0.2 m) short of the cell centre, where a turn
        on the spot hits the walls (sim 2026-11-22: blocked both ways 0.2 m off). Creep
        straight ahead (no steering, never back) to the point on the heading nearest the
        centre, as far as the footprint stays creep_margin m clear. False = nothing to do."""
        L = self.launch
        c, s = math.cos(yaw), math.sin(yaw)
        d = (L['xy'][0] - xy[0]) * c + (L['xy'][1] - xy[1]) * s
        grid = self.grid_now()
        if grid is not None and d > 0:
            d = free_distance_ahead(grid, xy, yaw, self.box, d, self.cfg['creep_margin'])
        if d < 0.03:
            return False
        L['creep'] = {'from': xy, 'target': d,
                      'until': time.monotonic() + d / self.cfg['creep_speed'] + 3.0}
        self.get_logger().info(f'launch cell: creeping {d * 100:.0f} cm ahead to the centre')
        return True

    def launch_creep_tick(self):
        """10 Hz while creeping to the launch cell's centre (creep_tick's timer)."""
        cr = self.launch['creep']
        pose = self.robot_pose()
        moved = 0.0 if pose is None else math.hypot(pose[0][0] - cr['from'][0],
                                                    pose[0][1] - cr['from'][1])
        if moved < cr['target'] and time.monotonic() <= cr['until']:
            msg = Twist()
            msg.linear.x = float(self.cfg['creep_speed'])
            self.cmd_pub.publish(msg)
            return
        self.stop_robot()
        self.launch['creep'] = None
        self.get_logger().info(f'launch cell: crept {moved * 100:.0f} cm')

    def face_exit(self, xy, yaw):
        """Turn on the spot to face the exit if more than launch_yaw_tolerance off: the short
        way if the footprint clears the walls, else the long way. False = facing it already,
        or no clear turn either way (park as it is)."""
        L = self.launch
        want = wrap(L['yaw'] - yaw)
        if abs(want) <= self.cfg['launch_yaw_tolerance']:
            return False
        L['turns'] += 1
        sign = 1.0 if want > 0 else -1.0
        other = 2 * math.pi - abs(want)
        grid, margin = self.grid_now(), self.cfg['turn_margin']
        if grid is None or max_turn(grid, xy, yaw, self.box, sign, abs(want), margin,
                                    pivot=self.pivot) >= abs(want) - 1e-6:
            self.send_spin(want, 'launch cell: face the exit')
            return True
        if max_turn(grid, xy, yaw, self.box, -sign, other, margin,
                    pivot=self.pivot) >= other - 1e-6:
            self.send_spin(-sign * other, 'launch cell: face the exit (the long way)')
            return True
        self.get_logger().warn(f'launch cell: {math.degrees(want):+.0f} deg turn to face the '
                               'exit blocked by walls both ways; parking as it is')
        return False

    def park(self, xy, yaw, note=None):
        """Hold still, standing, in the launch cell: the end of the run (AT_EXIT)."""
        L = self.launch
        L['handle'] = None
        off = math.hypot(xy[0] - L['xy'][0], xy[1] - L['xy'][1])
        turn = math.degrees(abs(wrap(yaw - L['yaw'])))
        self.stop_robot()
        self.publish_launch('at', xy)
        cell, d = self.exit_side
        self.finish('AT_EXIT', f'in launch cell {list(cell)}: {off:.2f} m from its centre, '
                    f'{turn:.0f} deg off facing the exit (exit cell '
                    f'{[cell[0] + d[0], cell[1] + d[1]]}); standing still for the drone'
                    + (f' ({note})' if note else ''))

    def finish(self, state, why):
        if self.state == 'CREEPING' or self.turn_creep is not None:
            self.stop_robot()
        self.turn_creep = None
        if self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()
            self.spin_handle = None
        self.end_pose(state)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        if self.launch is not None and self.launch['handle'] is not None:
            self.launch['handle'].cancel_goal_async()
            self.launch['handle'] = None
        self.current = None
        self.junction = None
        self.cancel_junction_goal()
        self.save_map()
        self.set_state(state, why)

    def stop_robot(self):
        if self.cfg['go']:
            for _ in range(3):                      # a few in case one is dropped
                self.cmd_pub.publish(Twist())

    def save_map(self):
        if self.map is None:
            return
        try:
            from maze_nav.map_io import save_map
            path = save_map(self.curtain_map(), self.cfg['map_dir'])   # curtains: free
            self.get_logger().info(f'map saved: {path}')
        except Exception as e:                      # never let saving stop the shutdown
            self.get_logger().error(f'map not saved: {e}')


def main():
    # rclpy's default SIGINT handler shuts the context down before we get control back
    # (rclpy.ok() is already False), which would skip the goal cancel and the map save.
    # Same pattern as cmd_vel_bridge: catch the signal ourselves and stop the loop.
    stop = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = Explorer()
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node.state not in END_STATES and rclpy.ok():
            node.finish('STOPPED', 'shutdown')
            end = time.monotonic() + 0.3     # let the cancel requests and status go out
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
