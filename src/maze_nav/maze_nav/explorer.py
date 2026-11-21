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
     start) for `dwell` s -> DONE
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
Stairs (stair_detector.py): while EXPLORING, TURNING or LOOKING, stair_detector's /stairs/status
HANDOVER means stairs are close ahead. The explorer cancels the Nav2 goal (or turn), sends a
zero Twist and goes to STAIRS; only then does stair_detector drive (face the stairs, square up,
stop in front), so it and Nav2 never both command nav_cmd_vel. Its AT_STAIRS -> AT_STAIRS here:
hold still until /maze/start (switch gait for the stairs first), then explore on. Its ABORTED
or IDLE, or `stairs_timeout` s in STAIRS -> back to EXPLORING. stairs_topic:='' = ignore it.
Possible stairs seen in a junction look (user 2026-11-21): stair_detector holds them
(/stairs/candidate) instead of checking them after the look; the junction's other ways are
explored first (curtain_branch.Deferral with `stairs`, like a curtain there), then the robot
goes `stairs_approach` m in front of the first riser, facing it, and releases them
(/maze/stairs_release): stair_detector hands over for the check. No other way: released now.
Curtains (curtain_detector.py, `curtain_topic` / `curtains_topic`; '' = ignore): confirmed
curtains (/curtain/curtains, map frame) are cleared on the explorer's copy of /map (a wall with a
low gap is passable), so frontiers behind one count. A goal within `curtain_goal_radius` of a
curtain not yet walked through from this side is sent to the spot `curtain_approach` m in
front of its middle instead, facing it. curtain_detector's HANDOVER: the explorer cancels Nav2, stops and goes to CURTAIN;
curtain_detector inspects (a possible curtain: lie down, measure the gap) or walks the robot
through (a confirmed one). Order (user 2026-11-20):
  - Inspection HANDOVER ("HANDOVER: possible curtain ...") while EXPLORING, TURNING,
    LOOKING, FACING_QR or JUNCTION, junction request pending or not: the lie-down check
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
Look-around (`look_topic`, from poster_detector): a QR room with no survivor poster confirmed
yet (QR codes only appear in rooms with survivors). While EXPLORING within `look_radius` of the
requested point, the explorer cancels the goal and sweeps left, right and back (LOOKING), each
way as far as `look_angle` or until the body would touch a wall (like the start scan), so
the camera sees the room; then EXPLORING again. Requests never reached expire after
`look_timeout` s (poster_detector then reports that room's survivor from the QR alone).
Unreadable QR code (`qr_unread_topic`, from qr_scanner: located in the image but not read,
e.g. too far round, blurred or at a steep angle): while EXPLORING, the explorer cancels the
goal and turns to face it (FACING_QR; cut short where the body would touch a wall, like
LOOKING), then holds still while qr_scanner runs its slow, thorough decode on every frame.
It goes back to EXPLORING when the code is read (`qr_read_topic`), when it hasn't been seen
for `face_lost` s, or after `face_timeout` s (that spot is then ignored for `face_cooldown`
s, so the robot can't get stuck on it). Re-aims up to `face_max_turns` times if it drifts.
Stairs (HANDOVER) interrupt it like LOOKING.
Junction (`junction_topic`, from scene_trigger: a new cell with 2+ open ways on besides the
way in, robot near its centre): while EXPLORING, the explorer cancels the goal and faces
each way, left-most first, holding `junction_hold` s at each but the last (JUNCTION; each
turn cut short where the body would touch a wall). Then (`junction_follow`, the user's spec
2026-11-19) it traces each way on the map (scene_cells.trace_way, up to
`junction_trace_cells` cells along corridors and bends): every way that ends in a dead end,
nearest first, it drives to the cell before the dead end (Nav2, cell centre), squares up
to the walls facing into it (turns on the spot until within `junction_align_tol`) and holds
`junction_dead_end_hold` s, then scans its left and right side walls like a DEAD_END
(Pose-mode body yaw with `pose_looks`, else spins; `dead_end_side_hold` s each, perception
on); then it drives one cell into the rightmost way (seen from the
way in) that doesn't end in a dead end, and explores on. A way whose end isn't mapped yet
counts as not a dead end. Status stays JUNCTION throughout. Requests older than
`junction_timeout` s or more than `junction_radius` m from the cell centre are dropped.
Dead end (`dead_end_topic`: scene_trigger's /scene/trigger, kind dead_end, fired from the
cell before it while facing it): the dead-end cell's centre is blacklisted so no goal goes
into it; while EXPLORING the explorer also cancels the goal and holds still `dead_end_hold`
s (DEAD_END) so scene understanding gets a steady view, then, still in the cell before it,
turns the camera to the middle of the dead end's left wall, then its right wall (the
record's side_walls; at most `dead_end_look_angle` off the dead end's heading; cut short
where the body would touch a wall), holding `dead_end_side_hold` s at each so the poster
and QR detectors see the side walls, then explores on. During JUNCTION it holds
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
LOOKING, FACING_QR, JUNCTION, DEAD_END, STAIRS, AT_STAIRS, CURTAIN, AT_CURTAIN, EXITED, NO_EXIT, STOPPED. Goals on /maze/goal (PoseStamped) for RViz. `go`
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
from maze_nav.scene_understanding.scene_cells import (DIRECTIONS, CellGrid, block_boxes,
                                                      block_cells, cell_objects, grow_box,
                                                      in_boxes, object_box, opening,
                                                      side_heading, side_walls, trace_way)
from maze_nav.core import (FREE_MAX, ExitDetector, Frontier, Grid, choose_frontier,
                           clearance_map, find_frontiers, geodesic_distance,
                           footprint_hits, free_distance_ahead, heading_towards, max_turn,
                           plan_creep_turn,
                           plan_turn)

START_CMD = 'ros2 topic pub --once /maze/start std_msgs/msg/Empty'

CURTAIN_STATES = ('CURTAIN', 'AT_CURTAIN')
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
        p('start_scan', True)          # after the creep: scan_angle left, 2x right, left again
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
        # Look-around requests from poster_detector: a QR room (QR codes only appear in rooms
        # with survivors) where no poster has been confirmed yet. '' = ignore them.
        p('look_topic', '/perception/look_request')
        p('look_radius', 1.5)          # m: look around once the robot is this close
        p('look_angle', math.pi)       # rad each way (less where the body would hit a wall)
        p('look_timeout', 120.0)       # s: drop a request the robot never got close to
        # Back of the start cell (user 2026-11-20): no frontier goal, junction look or
        # junction follow-up into the start cell or behind the start pose, up to
        # start_back_depth m back and start_back_width m either side of its line. 0 = off.
        p('start_back_depth', 1.5)
        p('start_back_width', 0.6)
        # Possible stairs at a junction (user 2026-11-21): stair_detector holds them
        # (/stairs/candidate); the junction's other ways are explored first (like a curtain
        # there), then the robot goes stairs_approach m in front of the first riser, facing it,
        # and releases them (/maze/stairs_release) for the stair check. '' = off.
        p('stairs_candidate_topic', '/stairs/candidate')
        p('stairs_release_topic', '/maze/stairs_release')
        p('stairs_approach', 1.2)
        # Occupied cells (user 2026-11-21): rooms may hold objects (footprint >= 0.3 x 0.3 m);
        # the robot doesn't enter a cell with one. A free-standing blob of map walls inside a
        # cell (all of it more than object_border m from its edges, not joined to any wall:
        # walls off the grid lines are not objects; >= object_min_extent m across) on
        # object_confirm maps in a row blocks the cell for good: drawn as wall on the
        # explorer's map and /maze/map (Nav2), so no goal or path goes into it. Cells: the
        # scene_trigger grid (cell_size m, cell (0, 0) on the map origin, along its x axis).
        p('block_objects', True)
        p('cell_size', 1.0)
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
        p('junction_hold', 0.5)        # s facing each way but the last
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
        p('junction_follow', True)     # after the look: visit dead ends, else go rightmost
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
                                      'open_space_no_looks', 'look_radius',
                                      'look_angle', 'look_timeout', 'face_timeout', 'face_lost',
                                      'face_min_angle', 'face_max_turns', 'face_cooldown',
                                      'junction_hold', 'junction_timeout', 'junction_radius',
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
                                      'object_min_extent', 'object_min_pixels', 'object_confirm',
                                      'object_keepout', 'object_keepout_margin',
                                      'room_looks', 'room_look_radius', 'room_look_hold',
                                      'room_look_max', 'room_stair_radius',
                                      'pose_turn_time', 'pose_wait', 'scan_hold',
                                      'pose_max_pitch', 'tilt_comp', 'tilt_threshold',
                                      'tilt_window', 'camera_frame')}
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
        if g('look_topic'):
            self.create_subscription(PointStamped, g('look_topic'), self.on_look_request, 10)
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
        self.object_seen = {}        # cell -> maps in a row with an object in it
        self.blocked_cells = {}      # cell -> (extent, (x, y)): objects, never entered
        self.paint_cells = set()     # blocked cells drawn as wall (not the one the robot is in)
        self.object_boxes = {}       # blocked cell -> (x0, y0, x1, y1) map box of its object
        self.objects_map = None      # /map message the objects were last looked for on
        self.dead_end_cells = set()  # checked dead ends (self.cells): never a goal into them
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
        self.looks = []              # pending look-around requests: (PointStamped, received)
        self.look_turns = None       # LOOKING: turns still to make
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
        self.set_state('STARTING', 'waiting for /map, TF map->base_link and Nav2')

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
                self.state in ('EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR', 'JUNCTION',
                               'DEAD_END') \
                and self.cfg['go']:
            if self.spin_handle is not None:          # a turn, scan or look-around
                self.spin_handle.cancel_goal_async()
                self.spin_handle = None
            self.end_pose('stairs ahead')
            self.look_turns = None                    # a look-around ends here
            self.face = None                          # so does facing a QR code
            self.junction = None                      # and a junction / dead-end look
            self.cancel_junction_goal()
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_handle, self.current, self.goal_result = None, None, None
            self.stop_robot()
            self.stairs_since = time.monotonic()
            self.stairs_approaching = False
            self.set_state('STAIRS', 'stairs ahead: Nav2 stopped, stair_detector lines the '
                           'robot up')
        elif self.state == 'STAIRS':
            if stairs == 'APPROACHING' and not self.stairs_approaching:
                self.stairs_approaching = True          # its own timeout from here
                self.stairs_since = time.monotonic()
            if stairs == 'AT_STAIRS':
                self.resume_requested = False
                self.set_state('AT_STAIRS', f'{msg.data.split(":", 1)[-1].strip()}; holding '
                               f'still - switch gait, then: {START_CMD}')
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
                self.state in ('EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR', 'JUNCTION'):
            self.take_for_curtain(True, 'possible curtain ahead: Nav2 stopped, curtain_detector '
                                  'inspects it first' + (' (junction look paused)'
                                                         if self.state == 'JUNCTION' else ''))
        elif cur == 'HANDOVER' and not inspect and self.cfg['go'] and (
                self.state in ('JUNCTION', 'DEAD_END', 'SCANNING') or self.junction_pending()):
            first = 'a junction request' if self.junction_pending() else self.state
            self.get_logger().info(f'curtain ahead, but {first} comes first: curtain later')
        elif cur == 'HANDOVER' and not inspect and self.cfg['go'] and \
                self.state in ('EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR'):
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
        self.look_turns, self.face = None, None
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
            ids=[d.curtain_id for d in self.defers if not d.released and d.stairs is None])
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

    def release_stairs(self, sid, why):
        if self.stairs_release_pub is not None:
            self.stairs_release_pub.publish(String(data=json.dumps({'id': int(sid)})))
        self.get_logger().info(f'stairs {sid}: released for the stair check ({why})')

    def on_stairs_candidate(self, msg):
        """stair_detector holds possible stairs seen at a junction: explore the junction's
        other ways first, then go to them and release them (user 2026-11-21)."""
        try:
            d = json.loads(msg.data)
            sid, frame = int(d['id']), str(d.get('frame', 'odom'))
            ex, ey = float(d['edge'][0]), float(d['edge'][1])
            hd = float(d['heading'])
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /stairs/candidate ({e}): {msg.data[:160]}')
            return
        for old in self.defers:
            if old.stairs is not None and old.stairs['id'] == sid:
                return                               # already held
        edge, heading = (ex, ey), hd
        if frame != 'map':
            try:
                tf = self.tf.lookup_transform('map', frame, Time())
            except TransformException:
                return self.release_stairs(sid, f'no TF map -> {frame}')
            t, q = tf.transform.translation, tf.transform.rotation
            th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            edge = (t.x + math.cos(th) * ex - math.sin(th) * ey,
                    t.y + math.sin(th) * ex + math.cos(th) * ey)
            heading = hd + th
        j = self.junction if self.state == 'JUNCTION' else self.paused_junction
        rec = j.get('rec') if j is not None else None
        if not rec or rec.get('cells') is None or not rec.get('paths'):
            return self.release_stairs(sid, 'not at a junction')
        cells, cell = rec['cells'], rec['cell']
        dirs = [DIRECTIONS[n] for n, _ in rec['paths'] if n in DIRECTIONS]
        sway = way_towards(cells, cell, dirs, edge)
        others = [w for w in dirs if w != sway and
                  not self.back_to_start(cells.centre((cell[0] + w[0], cell[1] + w[1])))]
        rec['skip_dirs'] = list(rec.get('skip_dirs') or []) + [sway]
        if not others:
            return self.release_stairs(sid, f'junction {list(cell)} has no other way')
        self.defers.append(Deferral(None, tuple(cell), cells, others, stairs={
            'id': sid, 'edge': edge, 'heading': heading, 'done': False}))
        self.get_logger().info(
            f'possible stairs {sid} at junction {list(cell)} (way {sway}): exploring its other '
            f'way(s) {others} fully first, then checking them')

    def branch_frontiers(self, grid, xy, frontiers):
        """With a deferred curtain: only frontiers down its junction's other ways; none left
        (done_confirm replans) -> release it and go to the spot in front of it. Returns the
        frontiers to choose from, or None to wait this replan."""
        def gone(d):
            if d.stairs is not None:
                return d.stairs['done']
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
        cur = self.curtain_by_id(d.curtain_id) if d.stairs is None else None
        name = f'curtain {d.curtain_id}' if d.stairs is None else f'stairs {d.stairs["id"]}'
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
                                   + ('cross it' if d.stairs is None else 'check them'))
            self.publish_hold()
        if d.stairs is not None:
            return self.stairs_spot(d, xy, frontiers)
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

    def stairs_spot(self, d, xy, frontiers):
        """Released held stairs: go stairs_approach m in front of the first riser, facing up
        the stairs, then release them to stair_detector (its check takes over from there)."""
        st = d.stairs
        ux, uy = math.cos(st['heading']), math.sin(st['heading'])
        a = self.cfg['stairs_approach']
        spot = (st['edge'][0] - a * ux, st['edge'][1] - a * uy)
        if any(math.hypot(spot[0] - g[0], spot[1] - g[1]) < 0.4 for g in self.failed):
            st['done'] = True
            self.release_stairs(st['id'], 'the spot in front can\'t be reached; checking '
                                'from where the robot is')
            return frontiers
        if math.hypot(spot[0] - xy[0], spot[1] - xy[1]) < 0.35:
            st['done'] = True
            self.release_stairs(st['id'], f'{a:.1f} m in front of them')
            return None                              # stair_detector hands over next tick
        return [Frontier(goal=spot, yaw=st['heading'], path_length=0.0, size=1, centroid=spot)]

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
        one's edge makes it a way on again (drop it); a running goal into one is cancelled."""
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
        if self.state in ('EXITED', 'NO_EXIT', 'STOPPED'):
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
            return
        if self.state == 'TURNING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy or self.turn_creep is not None:
                return                              # spinning, or creep_tick creeping
            self.set_state('EXPLORING', 'turned; going on')   # replan from the new heading
            return
        if self.state == 'LOOKING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy:
                return
            if self.look_turns:
                self.send_spin(self.look_turns.pop(0), 'look around')
                return
            self.set_state('EXPLORING', 'looked around; going on')   # replan from here
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
        self.update_objects(xy)
        self.keep_out_of_dead_ends()
        grid = self.grid_now()
        if time.monotonic() - self.started > self.cfg['max_runtime']:
            return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')

        if self.maybe_face(grid, xy, yaw):
            return
        if self.maybe_look(grid, xy, yaw):
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

    def on_look_request(self, msg):
        """poster_detector: a QR room with no poster yet; look around there once."""
        self.looks.append((msg, time.monotonic()))
        self.get_logger().info(f'look-around requested at {msg.header.frame_id} '
                               f'({msg.point.x:.2f}, {msg.point.y:.2f})')

    def maybe_look(self, grid, xy, yaw):
        """
        EXPLORING: near a requested point, stop and sweep left and right (as far as
        look_angle or until the body would touch a wall, like the start scan) so the camera
        sees the whole room. Returns True if a look-around started.
        """
        now = time.monotonic()
        self.looks = [(m, t) for m, t in self.looks if now - t < self.cfg['look_timeout']]
        if not self.looks or not self.cfg['go'] or self.state != 'EXPLORING' or \
                self.spin_busy or self.turn_creep is not None or \
                not self.spin.server_is_ready():
            return False
        for i, (msg, _) in enumerate(self.looks):
            try:   # requests are in odom; the map <- odom correction moves with SLAM
                tf = self.tf.lookup_transform('map', msg.header.frame_id, Time())
            except TransformException:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            px = t.x + math.cos(th) * msg.point.x - math.sin(th) * msg.point.y
            py = t.y + math.sin(th) * msg.point.x + math.cos(th) * msg.point.y
            if math.hypot(px - xy[0], py - xy[1]) > self.cfg['look_radius']:
                continue
            del self.looks[i]
            a, m = abs(self.cfg['look_angle']), self.cfg['turn_margin']
            left = max_turn(grid, xy, yaw, self.box, +1, a, m, pivot=self.pivot)
            right = max_turn(grid, xy, yaw, self.box, -1, a, m, pivot=self.pivot)
            turns = [t for t in (left, -(left + right), right) if abs(t) > math.radians(5)]
            if not turns:
                self.get_logger().warn('look-around skipped: walls too close to turn')
                return False
            self.halt_for_look()
            self.look_turns = turns
            self.set_state('LOOKING', f'QR room at ({px:.2f}, {py:.2f}) with no survivor '
                           f'poster yet: {math.degrees(left):.0f} deg left, '
                           f'{math.degrees(left + right):.0f} right, {math.degrees(right):.0f} '
                           'left')
            return True
        return False

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
        self.blacklist.append(centre)               # never send a goal into it
        self.dead_end_cells.add(self.cells.cell(centre))   # nor anywhere else in it
        where = f'dead end at cell {rec.get("cell")} ({centre[0]:.2f}, {centre[1]:.2f})'
        if self.cells.cell(centre) in self.rooms_done:
            self.get_logger().info(f'{where}: a room already looked into; blacklisted')
            return
        if not self.cfg['go']:
            self.get_logger().info(f'{where}: blacklisted')
            return
        if self.in_open_space():
            self.get_logger().info(f'{where}: open space here ({self.exit.open_nearby:.2f} m '
                                   'of free floor), probably outside the maze; no look')
            return
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
            self.get_logger().info(f'{where}: blacklisted (junction: checked after the look)')
        elif self.state in ('JUNCTION', 'DEAD_END') and self.junction is not None:
            self.junction['dead_end'] = steps       # hold at this way, then scan its sides
            self.get_logger().info(f'{where}: blacklisted; holding at this way, then '
                                   'scanning its side walls')
        else:
            self.get_logger().info(f'{where}: blacklisted ({self.state})')

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
        self.halt_for_look()
        self.junction = self.new_look([
            {'name': f'junction way {n}', 'heading': h, 'short': k == 0,
             'hold': self.cfg['junction_hold'] if k < len(ways) - 1 else 0.0}
            for k, (n, h) in enumerate(ways)])
        self.junction['rec'] = rec                  # look_tick follows it up afterwards
        self.set_state('JUNCTION', f'cell {rec.get("cell")}: {len(ways)} ways on ('
                       + ', '.join(f'{n} {math.degrees(wrap(h - yaw)):+.0f} deg'
                                   for n, h in ways) + '); facing each, left to right')

    def at_dead_end(self):
        """True while the robot is looking at a dead end (user 2026-11-19: check for QR codes
        and survivors only there): DEAD_END (scene_trigger's /scene/trigger: hold, then the
        side walls); in a JUNCTION look, a step named 'dead end ...' or the hold facing one;
        in the junction follow-up, squaring up to / holding at a dead-end way (group
        'dead end ...'). FACING_QR and LOOKING too, so a code or survivor seen there is
        followed up."""
        if self.state in ('DEAD_END', 'FACING_QR', 'LOOKING'):
            return True
        j = self.junction
        if self.state != 'JUNCTION' or not j:
            return False
        if j.get('dead_end'):
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
            # a pose look held inside the pose already (the body swings back after it)
            j['hold_until'] = now + (0.0 if j.pop('posed', False)
                                     else j['steps'][j['i'] - 1]['hold'])
        if j['dead_end'] is not None:        # facing a dead end: its hold + side walls next
            j['steps'][j['i']:j['i']] = j['dead_end']
            j['dead_end'] = None
        if j['hold_until'] is not None and now < j['hold_until']:
            return
        if j['hold_until'] is not None and 0 < j['i'] <= len(j['steps']):
            held = j['steps'][j['i'] - 1]
            self.capture_view(held.get('scene'), held['hold'])   # posed: sent in pose_tick
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

    def capture_view(self, scene, hold):
        """End of a dead-end view's hold (still facing it): one /scene/capture for
        room_scene, which takes the camera frame stamped inside [hold_start, stamp]. Once per
        view (scene['sent']); the explorer never waits for scene understanding."""
        if not scene or scene.get('sent') or self.scene_capture_pub is None:
            return
        scene['sent'] = True
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
        """After facing each way at a junction: trace every way on the map (scene_cells.
        trace_way). Each that ends in a dead end (nearest first): drive to the cell before
        it, square up to the walls facing into it, hold junction_dead_end_hold s, then scan
        its left and right side walls (Pose-mode body yaw, or spins without it). Then drive
        one cell into the rightmost way that doesn't end in a dead end (rightmost seen from
        the way in), and explore on. Returns the list of actions for after_junction_tick."""
        cells, cell = rec.get('cells'), rec['cell']
        if cells is None:
            self.get_logger().warn('/scene/junction has no grid (old scene_trigger?): no '
                                   'dead-end check, exploring on')
            return []
        grid = self.grid_now()
        if grid is None:
            return []
        hw = rec['half_width']

        def openings(c, d):
            return opening(grid, cells, c, d, half_width=hw)

        came = (cell[0] - rec['from_cell'][0], cell[1] - rec['from_cell'][1])
        entry = cells.heading(came) if came in DIRECTIONS.values() else None
        traces = []
        for name, heading in rec['paths']:
            d = DIRECTIONS.get(name)
            if d is None:
                continue
            t = trace_way(openings, cell, d, int(self.cfg['junction_trace_cells']))
            traces.append((name, d, heading, t))
            self.get_logger().info(
                f'junction {list(cell)} way {name}: {t["kind"].replace("_", " ")} '
                f'{"at" if t["kind"] == "dead_end" else "after"} cell {list(t["cell"])} '
                f'({t["length"]} cells)')
        back = [x for x in traces
                if self.back_to_start(cells.centre((cell[0] + x[1][0], cell[1] + x[1][1])))
                or (x[3]['kind'] == 'dead_end' and self.back_to_start(cells.centre(x[3]['cell'])))]
        for name, *_ in back:
            self.get_logger().info(f'junction {list(cell)} way {name}: back to the start cell; '
                                   'not followed')
        traces = [x for x in traces if x not in back]
        for x in [x for x in traces if x[1] in (rec.get('skip_dirs') or [])]:
            self.get_logger().info(f'junction {list(cell)} way {x[0]}: a held curtain; '
                                   'crossed after the other ways')
            traces.remove(x)
        todo = []
        dead = sorted((x for x in traces if x[3]['kind'] == 'dead_end'),
                      key=lambda x: x[3]['length'])
        for name, d, heading, t in dead:
            before, into = t['before'], t['dir']
            self.blacklist.append(cells.centre(t['cell']))     # never a goal into it
            self.dead_end_cells.add(self.cells.cell(cells.centre(t['cell'])))   # nor in it
            group = f'dead end {list(t["cell"])}'
            face = cells.heading(into)
            hold = {'do': 'hold', 's': float(self.cfg['junction_dead_end_hold']),
                    'group': group, 'name': f'{group}: facing in', 'view': 'centre'}
            # then its side walls, left first (Pose mode if available, else spins),
            # like a DEAD_END from /scene/trigger (user 2026-11-20: no Pose looks here)
            look = {'do': 'look', 'group': group, 'name': f'{group}: side walls',
                    'steps': [{'name': f'{group} {k} wall', 'point': xy, 'facing': face,
                               'view': k, 'hold': self.cfg['dead_end_side_hold'],
                               'short': True}
                              for k, xy in side_walls(cells, t['cell'], into).items()]}
            dead_end_scene([hold] + look['steps'], t['cell'], cells.centre(t['cell']))
            todo += [
                {'do': 'goto', 'xy': cells.centre(before), 'yaw': face, 'group': group,
                 'name': f'way {name}: to cell {list(before)}, before dead end '
                         f'{list(t["cell"])}'},
                {'do': 'align', 'heading': face, 'group': group, 'tries': 0,
                 'name': f'{group}: squaring up facing in'},
                hold, look]
        rest = [x for x in traces if x[3]['kind'] != 'dead_end']
        if rest:
            ref = entry if entry is not None else rest[0][2]
            name, d, heading, t = min(rest, key=lambda x: wrap(x[2] - ref))   # most to the right
            nxt = (cell[0] + d[0], cell[1] + d[1])
            todo.append({'do': 'goto', 'xy': cells.centre(nxt), 'yaw': heading,
                         'group': 'rightmost', 'name': f'rightmost way {name}: to cell '
                                                       f'{list(nxt)}'})
        self.get_logger().info(
            f'junction {list(cell)}: '
            + (f'{len(dead)} way(s) end in a dead end, visiting first; ' if dead else
               'no dead end; ')
            + (f'then the rightmost way {name}' if rest else 'no other way: exploring on'))
        return todo

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
                and not self.in_object(f.goal)
                and self.cells.cell(f.goal) not in self.dead_end_cells]

    def in_reach(self, frontiers, xy):
        off = self.cfg['goal_max_offset']
        return [f for f in frontiers
                if abs(f.goal[0] - xy[0]) < off and abs(f.goal[1] - xy[1]) < off]

    def recover_or_finish(self, grid, xy, why):
        """Goals keep failing: retry while open frontiers remain, else NO_EXIT."""
        open_ = self.in_reach(self.find(grid, xy, self.blacklist), xy)
        if not open_:
            self.finish('NO_EXIT', f'{why}; no open frontier left')
            self.stop_robot()
            return
        if self.recoveries >= self.cfg['max_recoveries']:
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
        if self.state not in ('SCANNING', 'TURNING', 'LOOKING', 'FACING_QR',
                              'JUNCTION', 'DEAD_END'):   # stopped meanwhile
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
        """ExitDetector on the current map; finishes EXITED (and returns True) once out."""
        xy = pose[0]
        if not self.exit.update(self.grid_now(), xy, time.monotonic()):
            return False
        self.finish('EXITED', f'open space around ({xy[0]:.2f}, {xy[1]:.2f}), '
                    f'{self.exit.open_nearby:.2f} m of open floor next to it'
                    + (f' (ended {self.state})' if self.state != 'EXPLORING' else ''))
        return True

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
        if node.state not in ('EXITED', 'NO_EXIT', 'STOPPED') and rclpy.ok():
            node.finish('STOPPED', 'shutdown')
            end = time.monotonic() + 0.3     # let the cancel requests and status go out
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
