"""Maze explorer: decides where to go (plan.py), drives there cell by cell, and switches the
robot's modes. See README.md for the whole picture.

  in   /map                SLAM map (slam_toolbox)
       /maze/cloud         levelled cloud with heights above the floor (wall_scan)
       TF map -> base_level   where the robot is
       /odom               only to catch a diverged odometry
       /nav/start, /explore/stop   std_msgs/Empty
  out  /cmd_vel            velocity (never backwards), /simple_cmd  mode and gait requests
       /explore/status     String, one line: state, cell, job, action
       /explore/event      String: "curtain", "room|object centre|left|right", "stairs", "exit" ...
                           with the cell and direction (for perception / logging)
       /explore/cells      MarkerArray: visited cells and what is on each side
"""
import json
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from transfer_interfaces.msg import MotionSimpleCMD
from visualization_msgs.msg import Marker, MarkerArray

from maze_explore import robot
from maze_explore.curtain_node import centred, edge_along
from maze_explore.grid import Grid, fit_grid, nb, opp, wrap
from maze_explore.plan import next_job
from maze_explore.sense import ACROSS, ALONG, OCC, HeightMap, Occupancy, Sense
from stairs import stair_levels

COLOURS = {'wall': (0.3, 0.3, 0.3), 'free': (0.1, 0.8, 0.1), 'room': (0.1, 0.4, 1.0),
           'curtain': (1.0, 0.5, 0.8), 'stairs': (1.0, 0.6, 0.0), 'blocked': (0.8, 0.1, 0.1),
           'unknown': (1.0, 1.0, 0.2), 'exit': (1.0, 0.0, 0.0), 'object': (0.6, 0.0, 0.8)}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Explorer(Node):

    def __init__(self):
        super().__init__('maze_explorer')
        p = self.declare_parameter
        self.cell_size = p('cell_size', 1.02).value
        self.v_max = p('v_max', 0.20).value            # m/s (team limit 0.25)
        self.w_max = p('w_max', 0.5).value             # rad/s
        self.min_spin = p('min_spin', 0.2).value       # the Lite3 does not turn below ~0.1
        self.creep = p('creep', 0.10).value            # m forward at the start
        self.gait = p('gait', 'hstep').value           # hstep = H-Step (default), rug = RUG general gear
        # Gait by floor (user, 2026-11-26): `gait` is the gait for uneven terrain, `flat_gait`
        # the one for plain floor. Before a step from a cell that is not uneven: into a cell
        # that is not uneven either -> flat_gait; into an uneven cell -> gait. Standing in an
        # uneven cell nothing is switched. gait_switch false = `gait` for the whole run.
        # The flat gaits have never been run on this robot (robot_info.md 6.3).
        self.flat_gait = p('flat_gait', 'flat_slow').value   # flat_slow | flat_medium | flat_fast | rug
        self.gait_switch = p('gait_switch', True).value
        self.cur_gait = self.flat_gait if self.gait_switch else self.gait
        self.rough_cells = set()      # cells entered as uneven terrain
        # The last cell (user, 2026-11-26): the maze is maze_rows rows deep along the start
        # heading (the start cell is in row 1) and the end cell is the only cell of the row
        # after, open at its far edge. An open way from the last row into that row is the
        # exit and nothing else is. maze_rows 0 = the exit is told from the map alone.
        # There the robot stops with its nose end_gap before the open far edge, turns
        # end_turn AWAY from maze_side (user, 2026-11-26, later the same day) and lies down.
        self.maze_rows = p('maze_rows', 5).value
        self.end_gap = p('end_gap', 0.10).value            # m, nose to the open edge
        self.end_turn = math.radians(p('end_turn', 15.0).value)
        self.lie_down = p('lie_down', True).value
        self.lying = False            # lying down in a pause (no exit found)
        # The way into the end cell can read wrong (user, 2026-11-26): beyond it is open
        # floor with little for the lidar to hit, so the opening may stay 'unknown', or
        # something may read as tall in there. Row maze_rows + 1 holds nothing but the end
        # cell, so on the far side of a last-row cell everything that is not a mapped wall
        # is a POSSIBLE exit: unknown after facing it (exit_open), blocked, object, possible
        # stairs. exit_weak[edge] = share of that edge mapped as wall. A plain open way
        # beats a possible one, and of two possible ones the less walled is tried first.
        # A possible exit is walked into with the obstacle stop on; stopped = it is a wall,
        # and the next one is tried.
        self.exit_weak, self.exit_open = {}, set()
        # After a curtain, at the centre of the cell behind (user, 2026-11-26): this far to
        # the right, then this far back to the left. It clears the fabric off the lidar.
        self.curtain_right = math.radians(p('curtain_turn_right', 180.0).value)
        self.curtain_left = math.radians(p('curtain_turn_left', 50.0).value)
        self.curtain_turns = p('curtain_turns', False).value   # off: no turns after a curtain
        # Holding the pivot while turning on the spot (do_turn; config/speeds.yaml)
        self.turn_kp = p('turn_kp', 1.5).value             # 1/s on the heading error
        self.pivot_kp = p('pivot_kp', 1.5).value           # 1/s on the distance off the pivot
        self.pivot_vx = p('pivot_vx', 0.12).value          # m/s, most forward correction
        self.pivot_vy = p('pivot_vy', 0.10).value          # m/s, most sideways correction
        self.drive_on = p('drive', True).value         # false: /explore/cmd_vel_preview only
        self.send_modes = p('send_modes', True).value  # false: no /simple_cmd at all
        self.enter_rooms = p('enter_rooms', False).value
        # Plain navigation switches (launch: stairs / curtains / terrain, or plain:=true for
        # all three). Off: stairs are just a blocked cell (no depth check, no AI mode), a
        # curtain is a wall (not faced, not pushed), uneven cells are walked into head first.
        self.stairs_on = p('stairs', True).value
        self.curtains_on = p('curtains', True).value
        self.terrain_on = p('terrain', True).value
        # Curtain seen by the camera (curtain_node, YOLO; launch curtain_cv:=true, OFF by
        # default). A sighting only names a direction. The first closed edge that way becomes
        # a candidate after cv_hits frames; the robot then faces it from a cell centre and it
        # counts when the box lies across the middle of the image on cv_verify_hits frames.
        # A verified edge is a curtain (passable) even where the map and the height map call
        # it a wall.
        # With curtain_cv on, the lidar's curtain (tall returns along the edge, sense.py) is a
        # CANDIDATE only (user, 2026-11-25): no push until the camera has confirmed it. An
        # aluminium-frame wall reads as a curtain to the lidar (bag stairs10, 23.4 s: 0.84 m
        # high, zero curtain boxes on 81 frames). The camera gets two looks at a candidate:
        # while the robot greets it (a miss there is not final) and just before the push (a
        # miss there makes the edge a wall). No camera frames at all = not confirmed either.
        self.cv_on = p('curtain_cv', True).value and self.curtains_on   # on by default (user, 2026-11-25)
        self.cv_conf = p('cv_conf', 0.5).value
        self.cv_hits = p('cv_hits', 3).value                 # frames, within cv_window s
        self.cv_window = p('cv_window', 3.0).value
        self.cv_verify_time = p('cv_verify_time', 3.0).value   # s facing the edge
        self.cv_verify_hits = p('cv_verify_hits', 3).value
        self.cv_seen = {}             # (cell, d) -> times of the frames that pointed at it
        self.cv_pending, self.cv_curtains, self.cv_rejected = set(), set(), set()
        # Curtain edges are latched (user, 2026-11-25): once the lidar has read an edge as a
        # curtain it stays one, both sides of it, whatever the map shows later. Fabric that
        # sways, is parted or pushed aside changes the map; it must not turn the edge into an
        # open way (walked into head first) or a plain wall (never crossed). Only a failed
        # crossing (stuck) or the first grid fit (cells move) undoes it.
        self.curtain_latched = set()
        self.cv_frames = deque(maxlen=60)     # (time, box across the image centre?) per frame
        self.cv_checking = set()              # edges whose verification is queued or running
        self.cv_first = p('cv_first', True).value   # a lidar curtain needs a camera sighting first
        # Before a room / object look is shown to scene understanding (the VLM prompt goes
        # out after its three views), the way ahead is checked for stairs (depth detector)
        # and for a curtain (camera, with curtain_cv). If one is verified the look is
        # called off: no views, no prompt (user, 2026-11-25).
        self.look_check_on = p('look_check', True).value
        self.look_checked = set()             # (cell, d) whose check has been made
        self.look_depth = p('look_depth', True).value    # depth stair check before a look (no terrain verdict)
        # Uneven terrain is entered SIDEWAYS (user, 2026-11-24): turn on the spot at the cell
        # centre until the body is square to the way in, then step sideways into the cell.
        self.side_speed = p('side_speed', 0.10).value      # m/s sideways into uneven terrain
        self.side_offset = p('side_offset', 0.075).value   # m the base_link line is moved
        #    towards the nose, so the body (0.27 m ahead, 0.42 m behind) is centred in the way in
        self.terrain_min = p('terrain_min', 0.06).value    # m above the floor = uneven
        self.terrain_squares = p('terrain_squares', 15).value   # 5 cm squares of the cell
        # The lidar's lowest ray meets the floor ~1.3 m ahead, so from the cell before it the
        # next cell's floor is mostly too close to see; it is known only if it was seen from
        # further back. With fewer than terrain_seen_min of its 121 squares seen, the standing
        # depth-camera check runs before walking in, and reads the floor there too (user,
        # 2026-11-25). The camera's verdict, when it has one, beats the lidar's.
        self.terrain_depth = p('terrain_depth', True).value
        self.terrain_seen_min = p('terrain_seen_min', 25).value   # 40 until 2026-11-26 (user:
        #    fewer depth checks; the lidar's reading from further back is taken on less)
        self.rough_checked = {}       # (cell, d) -> True | False | None (camera saw too little)
        self.rough_known = set()      # cells the depth camera has confirmed as uneven
        self.terrain_confirm = p('terrain_confirm', 3).value   # uneven frames that confirm it
        self.look_angle = math.radians(p('look_angle', 30.0).value)
        # QR approach (user, 2026-11-25). From the cell before, a code on a side wall is
        # seen about 60 deg off square and cannot be decoded (10:54 run, 11-24: 34 x 68 px).
        # Scene understanding (the VLM) can SEE it, though. After the three views of a look
        # the robot waits for that answer; with a code in the left or right view it turns
        # qr_angle towards that side and steps into the looked-at cell with its body kept
        # at that angle (so it goes in almost sideways, the camera sweeping that wall from
        # close up), then steps back out. A code read at any moment ends the move.
        self.qr_approach = p('qr_approach', False).value
        self.qr_wait = p('qr_wait', 8.0).value             # s for the VLM answer after the views
        self.qr_angle = math.radians(p('qr_angle', 75.0).value)
        self.qr_speed = p('qr_speed', 0.08).value          # m/s while stepping in and out
        self.qr_hold = p('qr_hold', 1.5).value             # s standing at the far end
        # user, 2026-11-26: never deeper than qr_depth into the room (base_link past the
        # border it came in through). Code still not read at the far end: turn slowly
        # towards the room's back wall, until it is read or the robot faces that wall.
        self.qr_depth = p('qr_depth', 0.3).value         # m (user, 2026-11-26: 0.3, was 0.5)
        self.qr_sweep_rate = p('qr_sweep_rate', 0.25).value   # rad/s of that turn
        # A dead end with nothing taller than an ordinary wall in it is EMPTY (user,
        # 2026-11-26: every room with a poster, sign, prop or code has something taller
        # than the 0.53 m walls). It still gets the three views with the QR scanner on,
        # but scene understanding is not sent anything, and it counts as looked at.
        self.empty_skip = p('empty_skip', True).value
        self.empty_squares = p('empty_squares', 4).value   # 5 cm squares seen tall = not empty
        # user, 2026-11-26: a room the lidar already shows to be empty when the robot plans
        # (it sees all round) gets no look at all: no turns, no views, no QR scan.
        self.empty_early = p('empty_early', True).value
        # user, 2026-11-26: once all five missions have a report the scoring SERVER marked
        # as successful (task_reporter passes its answer on, missions_topic), rooms and
        # props are no longer looked at; what is left is paths, stairs, curtains and the
        # way out. The latest answer per mission counts. missions_stop:=false = off.
        self.missions_stop = p('missions_stop', True).value
        self.missions_want = {m for m in p('missions_all', 'M01,M02,M03,M04,M05').value.split(',') if m}
        self.missions_seen = set()
        # Maze on the right (R) or the left (L) of the start cell: asked for by the launch
        # (user, 2026-11-26). Kept for the end-cell behaviour; nothing here uses it yet.
        self.maze_side = str(p('maze_side', '').value).strip().upper()[:1]
        self.scene_results = {}       # looked-at cell -> the /scene/result for it
        self.qr_read_t = -1e9         # time of the last /qr/read
        self.qr_t0 = 0.0              # start of the look under way
        self.look_hold = p('look_hold', 1.5).value
        self.greet_hold = p('greet_hold', 2.0).value
        self.cmd_gap = p('cmd_gap', 1.5).value         # s between mode requests
        self.ai_speed = p('ai_speed', 0.5).value    # stairs copy: was 0.15 (Rey, 2026-11-25: 0.5)
        self.ai_wz = p('ai_wz', 0.0).value          # stairs copy: IMU heading hold, off by default
        # stairs copy (Rey, 2026-11-25): AI mode ends once the robot is in the cell after the
        # stairs: 1.5 cells from the centre of the cell before = the border into the cell after,
        # plus ai_body_clear so the hind legs are past it too.
        self.ai_walk_cells = p('ai_walk_cells', 1.5).value
        self.ai_body_clear = p('ai_body_clear', 0.35).value   # m, about half the body length
        self.ai_distance = p('ai_distance', 'time').value   # stairs copy: 'time' or 'leg'
        self.ai_wall_min = p('ai_wall_min', 1.0).value     # stairs copy: m before the wall stop
        self.ai_walking = False                     # stairs copy: AI walk under way
        # user, 2026-11-26: after the stairs, before the gait is changed back, the robot
        # is walked onto the centre of the cell after if it is more than ai_centre_tol off.
        self.ai_centre_on = p('ai_centre', True).value
        self.ai_centre_tol = p('ai_centre_tol', 0.10).value      # m off the centre = centred
        self.ai_centre_speed = p('ai_centre_speed', 0.20).value  # m/s for that short walk
        self.ai_settle = p('ai_settle', 3.0).value   # s standing after the AI walk, for the pose
        self.ai_last_walked = 0.0                   # stairs copy: m of the last AI walk
        self.ai_reanchor = p('ai_reanchor', True).value   # stairs copy: re-anchor the grid after
        self.reanchor_min = p('reanchor_min', 0.15).value  # m off before it re-anchors
        self.reanchor_fit = 0
        self.imu_yaw = self.leg_vx = None           # stairs copy: body IMU yaw, leg-odometry speed
        self.leg_time = 0.0
        self.curtain_speed = p('curtain_speed', 0.10).value   # m/s sideways through a curtain
        self.pose_timeout = p('pose_timeout', 1.0).value
        # A curtain the CAMERA has confirmed is pushed through with the obstacle stop off
        # (2026-11-25): the fabric itself is 'something beside the body', so with the stop
        # on the robot gave up 3 cm into the step (10:42 run, cell (2, 2) side 0, confirmed
        # on 9 of 9 frames) and the curtain became a wall. push_stop still rules a curtain
        # only the lidar has seen (curtain_cv off). No progress for 8 s still ends a push.
        self.push_sure = False
        # Stall stop (user, 2026-11-25): during a push the lidar is under the fabric, so the
        # guard is whether the robot MOVES. Fabric gives way, a wall or a prop does not.
        # Movement is read from the robot's own leg odometry (/leg_odom2 pose; Fast-LIO is
        # not to be trusted with the lidar covered), subscribed only during the push (it
        # is a 200 Hz stream across the bridge). Commanded for push_stall_grace +
        # push_stall_time s and moved less than push_stall_min in the last push_stall_time
        # s = something solid: the push is given up (the side becomes a wall). On the bags
        # the leg pose follows Fast-LIO at about 0.85 of its distance while walking and
        # moves 0-2 cm in 1.5 s standing. Not yet seen: what it reads while the robot is
        # pressed against something (feet slipping). No leg data = the 8 s rule alone.
        self.push_stall = p('push_stall', True).value
        self.push_stall_time = p('push_stall_time', 1.5).value
        self.push_stall_grace = p('push_stall_grace', 0.7).value
        self.push_stall_min = p('push_stall_min', 0.03).value
        self.leg_topic = p('leg_odom_topic', '/leg_odom2').value
        self.leg, self.leg_sub = deque(maxlen=200), None
        self.push_stop = p('push_stop', True).value    # obstacle stop also while pushing (a
        #                                                real curtain then stops the push too)
        # Body outline from base_link, m. The rear is the 0.37 m body plus 0.05 m for the bent
        # hind legs (user, 2026-11-24). The rear corner sweeps turn_radius when turning on the
        # spot: 0.48 m, against 0.485 m from a cell's centre to its walls, so turns are made
        # ON the cell centre and held there (do_turn).
        self.fp_front = p('footprint_front', 0.27).value
        self.fp_rear = p('footprint_rear', 0.42).value
        self.fp_half_width = p('footprint_half_width', 0.23).value
        self.turn_radius = math.hypot(self.fp_rear, self.fp_half_width)
        self.pivot_offset = p('pivot_offset', 0.0).value    # m behind base_link: the point the
        #    Lite3 turns about by itself. Measured on 17 recorded turns (bags of 11-25 and
        #    11-26, commanded motion taken out): 1 cm AHEAD of base_link, quartiles 3.5 cm
        #    ahead to 1 cm behind. So it is base_link itself.
        self.pivot_slack = p('pivot_slack', 0.03).value     # m the pivot may drift uncorrected
        self.turn_ff = p('turn_forward', 0.02).value     # m/s forward at full turn rate: the
        #                                                  Lite3 drifts ~9 cm back per 90 deg
        self.recentre = p('recentre', 0.20).value         # m off the cell centre from which the
        #                    robot walks back onto it before the next job (10:03 run, 11-24: a
        #                    look was made 0.5 m ahead of the centre, between two cells)
        self.recentre_tries = 0
        # 03:29 run, 2026-11-26: the first grid fit put the start cell's centre 0.3-0.4 m
        # BEHIND the robot, the recentre rule walked it at the back wall three times, and
        # the third time the obstacle stop saw nothing (below ~14 cm the wall is inside
        # the lidar's blind range) and the robot pushed into the wall. Three guards:
        # 1. The start centre along the start heading comes from where the robot was put
        #    down (the lidar does not see the wall behind it: 0 points there in 30 scans
        #    of bag explore_20261125_100238; side walls ~2000 each). While the robot is
        #    still in the start cell a fit may move that centre start_u_max at most along
        #    the heading; sideways and the angle still come from the walls.
        self.start_u_max = p('start_u_max', 0.12).value
        self.start_pose = None        # where the robot stood at /nav/start (map x, y)
        # 2. A walk back onto the centre that is blocked means the GRID is off, not the
        #    robot: no second try from that cell (no_recentre), and in the start cell the
        #    wide grid fit is made again.
        self.recentre_failed, self.no_recentre, self.refitted = False, set(), False
        # On uneven terrain turns are made only at the centre of the cell (user,
        # 2026-11-26: off-centre the legs come down on the ends of the ramps). Standing in
        # an uneven cell more than terrain_centre off its centre, the robot first SHIFTS
        # onto the centre without turning (forward and sideways steps, heading kept;
        # never backwards), and only then turns. One shift per stay in the cell.
        self.terrain_centre = p('terrain_centre', 0.10).value
        self.shift_speed = p('shift_speed', 0.08).value
        self.shifted_in = None
        # 3. The obstacle stop remembers: what stopped the robot is kept (map frame) for
        #    block_memory s and still blocks when it is anywhere between the body and
        #    20 cm ahead of it, also where the lidar can no longer see it.
        self.block_memory = p('block_memory', 60.0).value
        self.front_seen = []          # (time, x, y) of what blocked the way ahead
        self.pivot_snap = p('pivot_snap', 0.25).value     # m: turn about the cell centre when
        #                                                  the robot stands this near it
        # Stair check with the depth camera (stair_levels.py, from ~/stairs_detect). The depth
        # stream is only subscribed while the robot stands still for the check: one raw depth
        # subscriber slows every stream from the robot, lidar and its IMU included.
        self.stair_check_on = p('stair_check', True).value and self.stairs_on
        self.ai_auto = p('ai_auto', True).value     # stairs copy: AUTO after AI gait
        self.stair_check_time = p('stair_check_time', 4.5).value     # s of looking (stairs copy: was 3.0)
        self.stair_min_hits = p('stair_min_hits', 5).value   # stairs copy: or this many hits in the whole look
        self.stair_rails = p('stair_rails', False).value   # stairs copy: was True (handrails in the lidar too)
        self.stair_raise_min = p('stair_raise_min', 0.28).value   # m: lidar height in the next
        #                                    cell from which it is looked at before walking in
        self.stair_params = stair_levels.LevelParams(
            rise=p('stair_rise', 0.12).value, tread=p('stair_tread', 0.20).value,
            steps=p('stair_steps', 3).value,
            # Wider than the detector's defaults, for close range (2026-11-24, on the real
            # bags: 0.26 m shot 12 -> 18 of 21 frames, walk-in 61 -> 80 of 191, the look-alike
            # obstacle still 0 of 27). Close up the floor is out of view, so the top tread's
            # height is less exact, and less of each tread is seen.
            top_min=p('stair_top_min', 0.31).value, top_max=p('stair_top_max', 0.42).value,
            edge_max_foreign=p('stair_edge_foreign', 4).value)
        self.stair_quick_no = p('stair_quick_no', 8).value   # frames without one hit = not stairs
        self.depth_topic = p('depth_topic', '/camera/depth/image_raw').value
        self.depth_info_topic = p('depth_info_topic', '/camera/depth/camera_info').value
        self.stairs_checked = {}      # (cell, d) -> True (stairs) | False (not) | 'lidar' (no depth)
        self.stair_agree = p('stair_agree', True).value          # lidar must agree with the camera
        self.stair_lidar_top = p('stair_lidar_top', 0.38).value  # m it must show in the cell
        self.stair_lidar_clouds = p('stair_lidar_clouds', 10).value
        self.heights_n = 0            # clouds put into the height map since it was (re)started
        # Floor first, stairs after (user, 2026-11-26): see do_stair_check.
        self.stair_gate = p('stair_gate', True).value
        self.stair_gate_min = p('stair_gate_min', 0.20).value       # m: the second step is 0.24
        self.stair_gate_squares = p('stair_gate_squares', 6).value  # 5 cm squares that high
        self.stair_gate_frames = p('stair_gate_frames', 3).value    # usable frames to decide
        self.depth_high = None
        self.odom_rot = None          # base_link orientation in odom (roll / pitch for the check)
        self.body_height = None
        self.clouds = deque(maxlen=12)        # the last ~1.2 s of /maze/cloud, for the handrails
        self.odom_grace_until = 0.0
        # Less standing (user, 2026-11-26: "allow slightly more odometry jump, pausing for
        # too long"; 05:30 run: 30 s of late-feed pauses and 4 jump pauses in 129 s). A step
        # of /odom counts as a jump from odom_jump_min (was 0.15 m) above what the robot can
        # walk; after one the robot stands odom_settle (was 3 s); jumps within
        # odom_burst_gap of each other still count as one burst for the FAULT rule.
        self.odom_jump_min = p('odom_jump_min', 0.25).value
        self.odom_burst_gap = p('odom_burst_gap', 3.0).value
        self.odom_settle = p('odom_settle', 1.5).value   # s of pause after one odometry jump
        self.odom_jumps = []                  # times of the jumps of the burst under way
        self.odom_burst_from = (0.0, 0.0)     # /odom position just before that burst
        self.odom_burst_max = p('odom_burst_max', 6.0).value   # s a burst may last, standing
        self.odom_net_max = p('odom_net_max', 1.5).value       # m it may end from its start
        self.odom_hold_until = 0.0
        # Late-feed pause (user, 2026-11-26). Every /odom jump on record came right after
        # the sensor feed arrived late while the robot was MOVING (three bags: 12 late
        # episodes moving -> 2 jumps, after the two longest delays, 0.39 and 1.86 s; 24
        # standing -> none; no jump without one). The delay is read from /odom itself: its
        # stamp is the lidar scan's, so arrival minus stamp is how late the lidar / IMU
        # data reached Fast-LIO (normally 0.04 s). More than feed_late over the usual
        # delay (the lowest tenth of the last 30 s, so a clock offset between the robot
        # and this computer does not count) = stand still until it has been normal for
        # feed_settle s. Never longer than feed_hold_max in one go: then it carries on and
        # says so. Not during the depth check (it slows every stream on purpose and the
        # robot stands anyway) or the AI walk on the stairs.
        self.feed_pause = p('feed_pause', True).value
        self.feed_late = p('feed_late', 0.30).value     # was 0.15: 11 of 15 pauses in the 05:30
        #    run began under 0.30 s late, and in the bags delays up to 0.34 s gave no jump
        self.feed_settle = p('feed_settle', 0.5).value    # was 1.0
        self.feed_hold_max = p('feed_hold_max', 6.0).value
        # After a pause that gave up (feed still late after feed_hold_max) the robot goes
        # on, but SLOWLY for those 10 s: nothing faster than feed_slow m/s. 05:03 run,
        # 2026-11-26: it gave up, walked on at 0.30 m/s and /odom swung sideways 5 s later.
        self.feed_slow = p('feed_slow', 0.10).value
        self.feed_lags = deque(maxlen=300)
        # Map drift (user, 2026-11-26; 05:57 run: the share of mapped walls on cell borders
        # fell from 0.63 to 0.44). Scans taken while odometry is not to be trusted are
        # placed with a wrong pose or tilt and bend the map, so SLAM gets none of them
        # (/maze/blind, as under a curtain): during a burst of odometry jumps and
        # map_blind_after s after it, while the sensor feed is late, and for the whole AI
        # walk over the stairs. The height map takes no clouds then either.
        self.map_blind = p('map_blind', True).value
        self.map_blind_after = p('map_blind_after', 1.0).value
        self.map_blind_until = 0.0
        self.grid_fit_radius = p('grid_fit_radius', 2.0).value   # m, walls used for corrections
        # Re-anchor after a jump (user, 2026-11-26). The robot stands from the first jump
        # of a burst on, so where it believed itself just before is still where it is.
        # Once Fast-LIO has settled (no jump for odom_settle s, the pose still for 1 s,
        # the tilt it reports plausible) the map pose is compared with that: if it is
        # more than jump_fix_min off, the CELL GRID is moved and turned by the difference,
        # so the robot is in the same cell, at the same spot of it, as before, and
        # everything mapped from now on lines up with the cells. Then a few medium wall
        # fits near the robot refine it (as after the stairs). What was known of the
        # other cells before the jump is KEPT as it was (self.kept): the map of them no
        # longer lines up with the moved grid, so it is not read again. The height map
        # and the obstacle memory are in the old frame and start afresh.
        # Not done: Fast-LIO still drifting or tilted after jump_fix_wait s (it has
        # diverged: nothing to anchor to), or more than jump_fix_max / 30 deg off.
        # OFF by default since its first real use (05:57 run, 2026-11-26): 2 s after a
        # 0.32 m jump the pose read 0.38 m / 9.7 deg off, the grid was turned by that, and
        # the map did NOT turn with it (slam_toolbox kept matching scans to the walls it
        # had), so the cells ended up 10 deg askew to every mapped wall and the wall fit
        # could not bring them back (10 deg / 0.25 m limits). jump_fix:=true = on.
        self.jump_fix = p('jump_fix', False).value
        self.jump_fix_min = p('jump_fix_min', 0.10).value
        self.jump_fix_max = p('jump_fix_max', 1.0).value
        self.jump_fix_wait = p('jump_fix_wait', 6.0).value
        self.jump_from = None          # (time, pose before the burst), until dealt with
        self.poses = deque(maxlen=40)  # (time, pose) of the last 4 s of ticks
        self.kept = {}                 # (cell, d) -> kind, frozen at a re-anchor
        self.feed_bad_at, self.feed_hold_from, self.feed_free_until = -1e9, None, 0.0
        self.feed_worst = 0.0
        auto_start = p('auto_start', False).value

        self.state = 'WAIT'           # WAIT sensors, READY, RUN, DONE, STOPPED, FAULT
        self.why = ''
        self.grid = None
        self.aligned = False          # the grid has been lined up with the walls once
        self.heights = HeightMap()
        self.heights_turn_max = p('heights_turn_max', 0.2).value   # rad/s: faster = no heights
        self.heights_settle = p('heights_settle', 0.3).value       # s after the turning stops
        self.turned_at, self.last_yaw = -1e9, None
        self.occ = None
        self.pose = None              # x, y, yaw in map
        self.pose_time = None
        self.cloud = None             # latest /maze/cloud as (n, 3), base_level
        self.cloud_time = None
        self.last_odom = None
        self.blind_until = 0.0        # curtain over the lidar: its returns are not walls
        # True from the start of a curtain crossing until its last turn is done (user,
        # 2026-11-25): the fabric can lie over the lidar all that time, so /maze/blind is held
        # on (no /scan to SLAM, no height-map updates) and odometry jumps don't count to FAULT.
        self.curtain_blind = False
        self.queue = deque()
        self.act = None
        self.act_t0 = 0.0
        self.act_mem = {}
        self.moving = False
        self.job = ''
        self.kinds = {}
        self.start = (0, 0)
        self.left_start = False       # the robot has walked out of the start cell
        self.visited, self.closed = set(), set()
        self.walked, self.forced_wall = set(), set()
        self.links, self.looked, self.greeted = {}, set(), set()
        self.exit_cell = self.exit_d = None
        self.step = None              # (from cell, d, dest) of the drive under way
        self.auto_start = auto_start

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        vel_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             lifespan=Duration(seconds=0.5))
        self.vel_pub = self.create_publisher(
            Twist, '/cmd_vel' if self.drive_on else '/explore/cmd_vel_preview', vel_qos)
        self.cmd_pub = self.create_publisher(MotionSimpleCMD, '/simple_cmd', latched)
        self.status_pub = self.create_publisher(String, '/explore/status', 10)
        self.event_pub = self.create_publisher(String, '/explore/event', 10)
        self.blind_pub = self.create_publisher(Bool, '/maze/blind', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/explore/cells', 2)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PointCloud2, '/maze/cloud', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        # stairs copy: only when asked for (a 200 Hz IMU callback in Python costs CPU)
        if self.ai_wz > 0:
            self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        if self.ai_distance == 'leg':
            self.create_subscription(Odometry, '/leg_odom2', self.on_leg_speed, qos_profile_sensor_data)
        self.create_subscription(Float32, '/maze/body_height',
                                 lambda m: setattr(self, 'body_height', m.data), 5)
        if self.cv_on:
            self.create_subscription(String, '/curtain/seen', self.on_curtain_seen, 10)
        if self.qr_approach:
            self.create_subscription(String, '/scene/result', self.on_scene_result, 10)
            self.create_subscription(String, '/qr/read',
                                     lambda m: setattr(self, 'qr_read_t', self.now()), 10)
        if self.missions_stop:
            self.create_subscription(String, p('missions_topic', '/mission/result').value,
                                     self.on_report, 10)
        self.create_subscription(Empty, '/nav/start', lambda m: self.begin(), 1)
        self.create_subscription(Empty, '/explore/stop', lambda m: self.halt('STOPPED', 'stop asked'), 1)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_cells)

    # ---- inputs -------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scene_result(self, msg):
        try:
            r = json.loads(msg.data)
            self.scene_results[tuple(r['cell'])] = r
        except (ValueError, KeyError, TypeError):
            pass

    def on_report(self, msg):
        """The scoring server's answer to one report, from task_reporter:
        {task_id, ok, reply}. Only ok counts; a later answer that is not ok takes it back."""
        try:
            r = json.loads(msg.data)
            task, ok = str(r['task_id']), r['ok'] is True
        except (ValueError, KeyError, TypeError):
            return
        if task not in self.missions_want or ok == (task in self.missions_seen):
            return
        (self.missions_seen.add if ok else self.missions_seen.discard)(task)
        left = sorted(self.missions_want - self.missions_seen)
        self.event(f"missions: {task} {'marked successful by the server' if ok else 'NOT successful any more'}; "
                   + (f'still open: {", ".join(left)}' if left else
                      'all successful: no more room or object looks'))

    def missions_done(self):
        return self.missions_stop and bool(self.missions_want) \
            and self.missions_want <= self.missions_seen

    def room_empty(self, c, d):
        """The dead end behind side d of cell c is seen to hold nothing: nothing taller
        than an ordinary wall in it or on its walls, nothing stair-high inside, and the
        far wall's top measured (so it was really seen)."""
        squares, measured = self.sense().tall_inside(c, d)
        return measured and squares < self.empty_squares and not self.raised(c, d), squares

    def skip_looks(self):
        """Before a job is chosen: looks that need not be made count as made."""
        for (c, d), k in self.kinds.items():
            if k not in ('room', 'object') or (c, d) in self.looked:
                continue
            if self.missions_done():
                self.looked.add((c, d))
                self.event(f'look skipped, cell {c} side {d}: all missions marked successful')
            elif self.empty_skip and self.empty_early and k == 'room':
                empty, squares = self.room_empty(c, d)
                if empty:
                    self.looked.add((c, d))
                    self.event(f'empty room cell {nb(c, d)} (from cell {c} side {d}): nothing '
                               f'taller than a wall ({squares} squares); no look')

    def on_curtain_seen(self, msg):
        """One camera frame from curtain_node (it only works outside room checks)."""
        try:
            dets = json.loads(msg.data).get('detections') or []
        except (ValueError, AttributeError):
            return
        now = self.now()
        self.cv_frames.append((now, any(centred(d, self.cv_conf) for d in dets)))
        if self.state != 'RUN' or self.grid is None or self.pose is None or self.occ is None \
                or (self.act and self.act[0] in ('turn', 'cv_verify')):
            return                    # while turning the heading of the frame is not known
        sense = self.sense()
        for d in dets:
            if d.get('conf', 0.0) < self.cv_conf:
                continue
            edge = edge_along(self.grid, sense, self.pose, float(d.get('bearing', 0.0)))
            if edge is None or edge in self.cv_curtains or edge in self.cv_rejected \
                    or edge in self.cv_pending or edge in self.cv_checking \
                    or self.kinds.get(edge) == 'curtain' \
                    or edge in self.links:
                continue
            if sense.low_wall(*edge):
                # an ordinary wall, and the camera sees over it: the curtain hangs somewhere
                # behind. It is found when the robot gets to its open side.
                self.get_logger().info(
                    f'camera: curtain seen over the wall of cell {edge[0]} side {edge[1]}: '
                    'it is behind that wall, not on it; ignored', throttle_duration_sec=5.0)
                continue
            times = [t for t in self.cv_seen.get(edge, []) if now - t <= self.cv_window] + [now]
            self.cv_seen[edge] = times
            if len(times) >= self.cv_hits:
                self.cv_pending.add(edge)
                self.get_logger().info(f'camera: curtain seen towards cell {edge[0]} side '
                                       f'{edge[1]} on {len(times)} frames: to be verified')

    def on_map(self, msg):
        i = msg.info
        self.occ = Occupancy(np.asarray(msg.data, np.int8).reshape(i.height, i.width),
                             i.resolution, i.origin.position.x, i.origin.position.y)

    def on_cloud(self, msg):
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, 3)
        self.cloud, self.cloud_time = pts, self.now()
        self.clouds.append(pts)
        if self.pose is None or not len(pts) or self.now() < self.blind_until:
            return
        # Not while the robot turns: the pose used here is the last tick's, a little older
        # than the cloud, and at turning speed that swings the walls of the next cell into
        # its middle. On 2026-11-25 10:02 that made the terrain cell ahead of the start an
        # 'object' during the first turn (bag explore_20261125_100238, 17.9 s) and set off
        # a look and scene understanding there. Replayed: 12 squares as it was, 0 with this.
        if self.now() - self.turned_at < self.heights_settle:
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.heights_n = getattr(self, 'heights_n', 0) + 1
        self.heights.add(x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1],
                         pts[:, 2])

    def on_imu(self, msg):                       # stairs copy
        q = msg.orientation
        self.imu_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_leg_speed(self, msg):                 # stairs copy (was a second on_leg: the
        #                         push-stall on_leg below replaced it, so ai_distance:=leg got no speed)
        self.leg_vx, self.leg_time = msg.twist.twist.linear.x, self.now()

    def feed_check(self, stamp):
        """One /odom message with this stamp has just arrived: is the feed late?"""
        now = self.now()
        lag = now - stamp
        usual = float(np.percentile(self.feed_lags, 10)) if len(self.feed_lags) >= 20 else lag
        self.feed_lags.append(lag)
        if lag - usual > self.feed_late:
            self.feed_bad_at = now
            self.map_blind_until = max(getattr(self, 'map_blind_until', 0.0),
                                       now + getattr(self, 'map_blind_after', 1.0))
            self.feed_worst = max(self.feed_worst, lag - usual)

    def feed_hold(self):
        """True while the robot should stand still because the sensor feed is late."""
        now = self.now()
        late = self.feed_pause and now - self.feed_bad_at < self.feed_settle \
            and now >= self.feed_free_until
        if not late:
            if self.feed_hold_from is not None:
                self.get_logger().info(
                    f'sensor feed on time again: stood {now - self.feed_hold_from:.1f} s '
                    f'(worst {self.feed_worst:.2f} s late)')
                self.feed_hold_from, self.feed_worst = None, 0.0
            return False
        if self.feed_hold_from is None:
            self.feed_hold_from = now
            self.get_logger().warn(f'sensor feed {self.feed_worst:.2f} s late: standing still '
                                   'until it is on time (odometry jumps when it moves now)')
        elif now - self.feed_hold_from > self.feed_hold_max:
            self.get_logger().warn(
                f'sensor feed still late after {self.feed_hold_max:.0f} s (worst '
                f'{self.feed_worst:.2f} s): carrying on at {self.feed_slow:.2f} m/s at most, '
                'no pause for the next 10 s')
            self.feed_free_until, self.feed_hold_from, self.feed_worst = now + 10.0, None, 0.0
            return False
        return True

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.feed_check(t)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom_rot = stair_levels.quat_to_matrix(q.x, q.y, q.z, q.w)
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.last_yaw is not None and t > self.last_yaw[0] and \
                abs(wrap(yaw - self.last_yaw[1])) / (t - self.last_yaw[0]) > self.heights_turn_max:
            self.turned_at = self.now()
        self.last_yaw = (t, yaw)
        if self.last_odom is not None:
            t0, x0, y0 = self.last_odom
            jump = math.hypot(p.x - x0, p.y - y0)
            dt = max(t - t0, 0.0)
            # More than the robot can walk in that time (0.6 m/s) plus 0.15 m. After a gap in
            # the lidar stream Fast-LIO catches up in one step (run3, 2026-11-24: 0.48 m in
            # 0.40 s after a 1 s stall, pose fine afterwards), so one jump only pauses the
            # robot. A runaway keeps jumping or goes far: that latches.
            big = getattr(self, 'odom_jump_min', 0.25) + 0.6 * dt
            if jump > big and self.ai_walking:   # stairs copy: expected on the steps
                self.odom_jumps = []
            elif jump > big:
                now = self.now()
                self.jump_report(jump, dt)
                # Some jumping is allowed (user, 2026-11-26). Jumps that follow each other
                # within odom_settle s are ONE burst: the robot stands until odom_settle s
                # after the last of them and then carries on. In the 05:03 run /odom swung
                # sideways +-0.3 m for 1.7 s on a late feed (10 jumps) and was steady from
                # then on, but the old rule (3 jumps in 10 s) had ended the run after the
                # third. FAULT is for odometry that does not come back: a burst still
                # going after odom_burst_max s of standing, a pose that ends up more than
                # odom_net_max from where the burst began, or one jump over 2 m.
                burst_max = getattr(self, 'odom_burst_max', 6.0)
                net_max = getattr(self, 'odom_net_max', 1.5)
                if not self.odom_jumps or now - self.odom_jumps[-1] > getattr(self, 'odom_burst_gap', 3.0):
                    self.odom_jumps, self.odom_burst_from = [], (x0, y0)
                self.odom_jumps.append(now)
                lasted = now - self.odom_jumps[0]
                fx, fy = getattr(self, 'odom_burst_from', (x0, y0))
                net = math.hypot(p.x - fx, p.y - fy)
                if now < self.odom_grace_until:     # depth stream on: the lidar feed is slow
                    self.odom_jumps = []
                elif jump > 2.0 or lasted > burst_max or net > net_max:
                    why = (f'one jump of {jump:.2f} m' if jump > 2.0 else
                           f'still jumping after {lasted:.1f} s of standing' if lasted > burst_max
                           else f'{net:.2f} m from where the jumping began')
                    self.halt('FAULT', f'/odom jumped {jump:.2f} m in {dt:.2f} s ({why}, '
                                       f'{len(self.odom_jumps)} jumps): odometry has '
                                       'diverged. Stop, run explore_check, relaunch')
                elif self.state == 'RUN' and len(self.odom_jumps) == 1:
                    if getattr(self, 'jump_fix', False) and self.jump_from is None:
                        before = [q for t_, q in self.poses if now - t_ >= 0.3]
                        if before:
                            self.jump_from = (now, before[-1])
                    self.get_logger().warn(f'/odom jumped {jump:.2f} m in {dt:.2f} s: standing until '
                                           f'it has been steady for {self.odom_settle:.1f} s')
                self.odom_hold_until = now + self.odom_settle
                self.map_blind_until = max(getattr(self, 'map_blind_until', 0.0), now
                                           + self.odom_settle + getattr(self, 'map_blind_after', 1.0))
        self.last_odom = (t, p.x, p.y)

    def jump_report(self, jump, dt):
        """One log line per odometry jump with what could explain it (2026-11-26: two
        FAULTs with nothing on record but the jump itself): what the robot was doing and
        commanding, how many lidar points the last cloud had and how old it is, and how
        close the nearest thing is on each side (Fast-LIO drops returns within 0.30 m of
        the lidar, so a wall that close is gone from its view)."""
        try:
            p, now = self.cloud, self.now()
            near = {}
            if p is not None and len(p):
                h = p[(p[:, 2] > 0.15) & (p[:, 2] < 1.0)]
                for name, m, d in (('ahead', (h[:, 0] > 0) & (np.abs(h[:, 1]) < 0.25), h[:, 0]),
                                   ('left', (h[:, 1] > 0) & (np.abs(h[:, 0]) < 0.3), h[:, 1]),
                                   ('right', (h[:, 1] < 0) & (np.abs(h[:, 0]) < 0.3), -h[:, 1])):
                    near[name] = f'{float(np.percentile(d[m], 5)):.2f}' if m.sum() >= 5 else 'none'
            cmd = getattr(self, 'last_cmd', (0.0, 0.0, 0.0))
            self.get_logger().warn(
                f'odom jump {jump:.2f} m in {dt:.2f} s during act={self.act[0] if self.act else "-"} '
                f'gait={self.cur_gait} cmd vx {cmd[0]:+.2f} vy {cmd[1]:+.2f} wz {cmd[2]:+.2f}; '
                f'last cloud {0 if p is None else len(p)} points, '
                f'{now - (self.cloud_time or now):.2f} s old; pose {now - (self.pose_time or now):.2f} s old; '
                f'nearest (m from base_link) ' + ' '.join(f'{k} {v}' for k, v in near.items()))
        except Exception as e:                # a report must never stop the node
            self.get_logger().warn(f'odom jump report failed: {e}')

    def read_pose(self):
        if self.curtain_blind or (self.map_blind and self.state == 'RUN' and (
                self.ai_walking or self.now() < self.map_blind_until + 1.0)):
            return self.read_pose_blind()     # SLAM gets no scans: its map -> base_level stands still
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_level', Time())   # never blocks
        except TransformException:
            return False
        q, t = tf.transform.rotation, tf.transform.translation
        self.pose = (t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        self.pose_time = stamp
        return True

    def read_pose_blind(self):
        """The pose during a curtain crossing. /maze/blind stops the scans to SLAM, so SLAM
        stops renewing map -> odom and the newest map -> base_level stays at the moment the
        crossing began: the pose froze and, 1 s later, the robot stood still as 'pose old'
        for good, with the scans still off (simulator, 2026-11-26: frozen 0.18 m into the
        push for 7 minutes). So: the last map -> odom, held, with the newest
        odom -> base_level. Its age is the odometry's."""
        try:
            mo = self.tf_buffer.lookup_transform('map', 'odom', Time())
            ob = self.tf_buffer.lookup_transform('odom', 'base_level', Time())
        except TransformException:
            return False

        def flat(tf):
            q, t = tf.transform.rotation, tf.transform.translation
            return t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        (mx, my, myaw), (ox, oy, oyaw) = flat(mo), flat(ob)
        c, s = math.cos(myaw), math.sin(myaw)
        self.pose = (mx + c * ox - s * oy, my + s * ox + c * oy, wrap(myaw + oyaw))
        self.pose_time = ob.header.stamp.sec + ob.header.stamp.nanosec * 1e-9
        return True

    # ---- outputs ------------------------------------------------------------------------
    def send(self, vx=0.0, vy=0.0, wz=0.0):
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            if not self.moving:
                return                      # one zero, then silence (teleop works again)
            self.moving = False
        else:
            self.moving = True
        if self.feed_free_until and not self.ai_walking and self.now() < self.feed_free_until:
            top = max(abs(vx), abs(vy))           # the feed is late and the pause gave up
            if top > self.feed_slow > 0:
                vx, vy = vx * self.feed_slow / top, vy * self.feed_slow / top
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(max(0.0, vx)), float(vy), float(wz)
        self.last_cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.vel_pub.publish(m)

    def simple_cmd(self, code):
        self.get_logger().info(f'/simple_cmd 0x{code:08X}' + ('' if self.sending() else ' (not sent)'))
        if self.sending():
            self.cmd_pub.publish(MotionSimpleCMD(cmd_code=code, size=0, type=0))

    def sending(self):
        return self.drive_on and self.send_modes

    def event(self, text):
        self.get_logger().info(f'event: {text}')
        self.event_pub.publish(String(data=text))

    def halt(self, state, why):
        if self.state in ('FAULT',) or (self.state == state and self.why == why):
            return
        self.moving = True
        self.send()
        self.queue.clear()
        self.act = None
        self.curtain_blind = False
        self.leg_watch(False)
        self.ai_walking = False               # stairs copy
        self.state, self.why = state, why
        (self.get_logger().error if state == 'FAULT' else self.get_logger().info)(f'{state}: {why}')

    # ---- start --------------------------------------------------------------------------
    def begin(self):
        if self.state not in ('READY', 'STOPPED'):
            self.get_logger().warn(f'/nav/start ignored in state {self.state} ({self.why})')
            return
        if self.grid is None:
            x, y, yaw = self.pose
            self.grid = Grid(x, y, yaw, self.cell_size)
            self.start_pose = (x, y)
            self.visited = {self.start}
            for code in robot.start_sequence(self.cur_gait):
                self.queue.append(('cmd', code))
                self.queue.append(('hold', self.cmd_gap))
            fx, fy = x + self.creep * math.cos(yaw), y + self.creep * math.sin(yaw)
            self.queue.append(('drive', fx, fy, yaw, 'creep'))
            self.queue.append(('hold', 1.5))
            self.queue.append(('call', self.align_grid))
            self.queue.append(('resolve', self.start))
        else:
            # Resuming after a stop. Whoever stopped it may have taken over with the handheld
            # controller, or left AI mode with lite3_ai_off: both leave the robot in manual
            # mode, where it ignores /cmd_vel. So switch it back before anything else
            # (Move -> gait -> navigation mode, 1.5 s apart, standing still).
            if getattr(self, 'lying', False):     # paused lying down (no exit found): up first
                self.lying = False
                self.get_logger().info('resuming: standing up first (stand / sit toggle)')
                self.queue.extend([('cmd', robot.STAND_SIT), ('hold', 4.0)])
            self.get_logger().info(f'resuming: sending Move, {self.cur_gait} and navigation mode again')
            for code in robot.start_sequence(self.cur_gait):
                self.queue.append(('cmd', code))
                self.queue.append(('hold', self.cmd_gap))
        self.state, self.why = 'RUN', ''
        self.get_logger().info(f"exploring (maze_side: {self.maze_side or 'not given'})")

    def align_grid(self):
        """Line the grid up with the walls in the map. Returns True when this call was the
        first fit that worked.

        Until one fit has worked the limits are wide (30 deg, 0.40 m): that is what lets the
        robot start off-centre and at an angle, the start cell's centre and the maze
        directions come from the walls, not from where the robot was put down. The map right
        after the start creep is one or two scans and often too thin for it (run1: 37 wall
        points, the grid then stayed 6 deg and 0.27 m off), so the wide fit is tried again at
        every planning step until it works. After that only small corrections (3 deg, 0.08 m)."""
        if self.occ is None:
            return False
        wide = not self.aligned
        max_yaw, max_shift = (math.radians(30), 0.40) if wide else (math.radians(3), 0.08)
        # Corrections use the walls near the robot only (user, 2026-11-26; was 4 m): once the
        # map is a little bent, walls far away pull the fit off and lower its quality until
        # it is refused (05:57 run, twice), just when it is needed. The first, wide fit
        # still takes everything within 4 m.
        radius = 4.0 if wide else self.grid_fit_radius
        if self.reanchor_fit and not wide:   # stairs copy: just re-anchored after the stairs
            max_yaw, max_shift, radius = math.radians(10), 0.25, 1.6
            self.reanchor_fit -= 1
        px, py = self.occ.occupied_points()
        near = np.hypot(px - self.pose[0], py - self.pose[1]) < radius
        g, q = fit_grid(self.grid, px[near], py[near], max_yaw, max_shift)
        if g is None or q < 0.5:
            how = 'too few wall points or further off than the limits' if g is None \
                else f'quality {q:.2f}'
            self.get_logger().warn(
                f"grid {'NOT LINED UP YET' if wide else 'not corrected'}: no fit within "
                f'{math.degrees(max_yaw):.0f} deg / {max_shift:.2f} m ({how}, '
                f'{int(near.sum())} wall points); kept', throttle_duration_sec=5.0)
            return False
        g = self.anchor_start(g)
        turned, moved = math.degrees(wrap(g.yaw - self.grid.yaw)), \
            math.hypot(g.x0 - self.grid.x0, g.y0 - self.grid.y0)
        self.grid = g
        if wide:
            self.aligned = True
            # sides taken as walls and rooms looked at with the unfitted grid were judged in
            # the wrong places
            self.forced_wall.clear()
            self.looked.clear()
            self.curtain_latched.clear()      # same: they name cells of the unfitted grid
            self.exit_weak.clear()
            self.exit_open.clear()
            self.cv_curtains.clear()
            self.cv_rejected.clear()
            self.cv_pending.clear()
            self.cv_seen.clear()
            if moved > 0.10:
                self.no_recentre.clear()      # the cells have really moved: centres are new
            self.get_logger().info(f'grid lined up with the walls: turned {turned:+.1f} deg, '
                                   f'moved {moved:.2f} m (quality {q:.2f})')
            return True
        if radius < 4.0:
            self.reanchor_fit = 0
            self.event(f'grid fitted after re-anchoring: {turned:+.1f} deg, {moved:.2f} m '
                       f'(quality {q:.2f})')
        self.get_logger().info(f'grid corrected: {turned:+.1f} deg, {moved:.2f} m '
                               f'(quality {q:.2f})', throttle_duration_sec=10.0)
        return False

    def anchor_start(self, g):
        """A fitted grid, with its start centre held near where the robot was put down,
        along the start heading (see start_u_max). Only while the robot is in the start cell."""
        if self.start_pose is None or self.left_start or self.start_u_max <= 0:
            return g
        c, s = math.cos(g.yaw), math.sin(g.yaw)
        du = (g.x0 - self.start_pose[0]) * c + (g.y0 - self.start_pose[1]) * s
        if abs(du) <= self.start_u_max:
            return g
        self.get_logger().warn(
            f'grid fit puts the start cell centre {abs(du):.2f} m '
            f'{"ahead of" if du > 0 else "behind"} where the robot was put down (limit '
            f'{self.start_u_max:.2f} m): not believed along the heading; the centre stays at '
            'the placed spot there, sideways and angle from the walls', throttle_duration_sec=5.0)
        return Grid(g.x0 - du * c, g.y0 - du * s, g.yaw, g.size)

    # ---- what is around -----------------------------------------------------------------
    def kind(self, sense, cell, d):
        b = nb(cell, d)
        if (cell, d) in self.walked:
            return 'free'
        if (cell, d) in self.forced_wall or (cell == self.start and d != 0) or b in self.closed:
            return 'wall'
        if (cell, d) in self.cv_curtains and self.curtains_on:
            return 'curtain'          # verified with the camera: passable, whatever the map says
        if (cell, d) in self.curtain_latched and self.curtains_on:
            return 'curtain'          # read as a curtain before: stays one
        k = getattr(self, 'kept', {}).get((cell, d)) or sense.kind(cell, d, may_exit=cell != self.start)
        if k == 'curtain' and (not self.curtains_on or (cell, d) in self.cv_rejected):
            k = 'wall'                # off, or a lidar candidate the camera turned down
        if k == 'curtain' and self.cv_on and getattr(self, 'cv_first', True) \
                and not self.camera_pointed(cell, d):
            # False curtains (user, 2026-11-26; 04:42 and 05:30 runs: tall walls beside the
            # start cell and beside cell (1, 0) latched as curtains). The lidar cannot
            # tell a tall wall from a curtain, so with the camera on it proposes nothing
            # by itself: a tall edge is a wall until the camera has pointed at it at
            # least once while the robot was moving about (curtain_node's sightings).
            k = 'wall'
        if k == 'curtain':
            self.curtain_latched.update({(cell, d), (b, opp(d))})
            self.get_logger().info(f'curtain latched: cell {cell} side {d} (and cell {b} side {opp(d)})')
        if k == 'stairs' and not self.stairs_on:
            k = 'blocked'
        # The depth camera has the last word on stairs. Until it has looked, something that
        # blocks the next cell is kept as possible stairs (looked at before anything is done).
        checked = self.stairs_checked.get((cell, d))
        if checked is True:
            return 'stairs'
        if checked is False and k in ('stairs', 'blocked'):
            # Looked at and not stairs: then it is something standing in that cell, a prop.
            # It gets a look (scene understanding) like any object cell and is not entered.
            # 11:41 run, 2026-11-25: the prop room (0, 1) read as possible stairs, was put
            # off to the end with the stairs, and would have ended as 'blocked', unseen.
            k = 'object'
        if checked is None and k == 'blocked' and self.stair_check_on:
            k = 'stairs'
        # stairs copy: our stairs often map as 'object'; check those too if stair-high
        if checked is None and k == 'object' and self.stair_check_on and self.stairs_on \
                and self.raised(cell, d):
            k = 'stairs'
        # stairs copy: never walk normally into a stair-high cell. If the depth check did not
        # confirm stairs (or had no frames), a 'free' side that is stair-high counts as blocked
        # (terrain is 0.25 m at most, stair_raise_min is 0.28 m). 2026-11-25: climbed in H-Step.
        if checked in (False, 'lidar') and k == 'free' and self.stairs_on and self.raised(cell, d):
            k = 'blocked'
        if k == 'room' and any((b, e) in self.cv_curtains or (b, e) in self.curtain_latched
                               for e in range(4)):
            k = 'free'                # not a dead end: one of its "walls" is a curtain
        if self.maze_rows > 0:
            # The end cell is the one cell of the row after the last (row index maze_rows),
            # open at its far edge. The way into it is the exit: taken at the very end, never
            # walked as an ordinary visit. Nothing is entered beyond or beside the end cell,
            # and what the map calls an exit anywhere else is not one.
            if cell[0] >= self.maze_rows:
                k = 'exit' if d == 0 else 'wall'
            elif cell[0] == self.maze_rows - 1 and d == 0:
                if k in ('free', 'room', 'exit'):
                    k = 'exit'
                    self.exit_weak.pop((cell, d), None)
                    # seen open once: if the map loses it later (unknown), it stays a
                    # possible exit instead of being withdrawn (simulator, 2026-11-26:
                    # "exit withdrawn: cell (4, 2) side 0 is unknown")
                    self.exit_open.add((cell, d))
                elif k in ('blocked', 'object', 'stairs') or \
                        (k == 'unknown' and (cell, d) in self.exit_open):
                    share = self.wall_share(cell, d)
                    if share < 0.4:           # no wall mapped on the edge itself
                        if (cell, d) not in self.exit_weak:
                            self.event(f'exit: possible way out, cell {cell} side {d} (reads '
                                       f'{k}, {share * 100:.0f} % of the edge mapped as wall)')
                        self.exit_weak[(cell, d)] = share
                        k = 'exit'
            elif k == 'exit':
                k = 'wall'
        if self.exit_cell is not None and (cell, d) == (self.exit_cell, self.exit_d) and k != 'exit':
            self.exit_cell = self.exit_d = None       # seen better since: not the way out
            self.event(f'exit withdrawn: cell {cell} side {d} is {k}')
        now = (self.exit_cell, self.exit_d)
        if k == 'exit' and (cell, d) != now and (
                self.exit_cell is None or (now in self.exit_weak and
                self.exit_weak.get((cell, d), -1.0) < self.exit_weak[now] - 0.05)):
            self.exit_cell, self.exit_d = cell, d      # the first, or a better one
            self.event(f'exit seen from cell {cell} side {d}'
                       + (' (possible only)' if (cell, d) in self.exit_weak else ''))
        return k

    def camera_pointed(self, cell, d):
        """The curtain camera has pointed at this edge (from either side) at some time."""
        both = ((cell, d), (nb(cell, d), opp(d)))
        seen = getattr(self, 'cv_seen', {})
        return any(seen.get(e) or e in self.cv_pending or e in self.cv_curtains
                   or e in getattr(self, 'cv_checking', ()) for e in both)

    def wall_share(self, cell, d):
        """Share of the edge on side d of `cell` that the map shows as wall (0-1)."""
        x, y = self.grid.box(cell, d, ALONG, ACROSS)
        return float((self.occ.at(x, y) >= OCC).any(axis=1).mean())

    def sense(self):
        return Sense(self.grid, self.occ, self.heights,
                     dict(terrain_min=self.terrain_min, terrain_squares=self.terrain_squares))

    def update_kinds(self):
        sense = self.sense()
        self.kinds = {(c, d): self.kind(sense, c, d)
                      for c in self.visited if c not in self.closed for d in range(4)}

    def cur_cell(self):
        return self.grid.cell_of(self.pose[0], self.pose[1])

    def arrive(self, cell):
        self.visited.add(cell)
        if cell != self.start and not self.left_start:
            self.left_start = True
            # The start cell can be any of the three cells of the first row (user,
            # 2026-11-26), so it is not always a dead end. It is closed (never entered
            # again) only when the way just taken is its one way out; with a second way
            # out it stays open, or the cells behind that way could never be reached.
            ways = [d for d in range(4) if self.kinds.get((self.start, d)) not in (None, 'wall')]
            if len(ways) <= 1:
                self.closed.add(self.start)
            else:
                self.event(f'start cell {self.start} stays open: {len(ways)} ways out '
                           f'(sides {", ".join(map(str, ways))})')
        self.step = None
        self.shifted_in = None
        self.queue.appendleft(('resolve', cell))

    # ---- planning -----------------------------------------------------------------------
    def plan(self):
        cur = self.cur_cell()
        if cur not in self.visited or cur in self.closed:
            cur = min(self.visited - self.closed, key=lambda c: math.dist(self.grid.centre(c), self.pose[:2]))
        if self.align_grid():                 # first fit: the cells have moved, look again
            cur = self.cur_cell()
            if cur not in self.visited or cur in self.closed:
                cur = min(self.visited - self.closed,
                          key=lambda c: math.dist(self.grid.centre(c), self.pose[:2]))
            self.queue.append(('resolve', cur))
            return
        # Every job starts from a cell centre. Turns end on heading alone and nothing drives
        # backwards, so a robot that has ended up off the centre is walked back onto it first.
        cx, cy = self.grid.centre(cur)
        # measured at the robot's pivot (body centre), which a turn does not move: base_link
        # itself swings pivot_offset round it, up to twice that after a half turn
        bx, by = self.body_centre()
        off = math.hypot(cx - bx, cy - by)
        if self.recentre_failed:
            self.recentre_failed = False
            self.no_recentre.add(cur)
            again = self.aligned and not self.left_start and not self.refitted
            self.get_logger().warn(
                f'the way back onto the centre of cell {cur} is blocked ({off:.2f} m off): the '
                'grid is off, not the robot. Not tried again from this cell'
                + ('; fitting the grid to the walls afresh' if again else ''))
            if again:
                self.aligned, self.refitted = False, True      # once
                return                        # the next plan() makes the wide fit first
        uneven_here = cur in self.rough_cells or cur in getattr(self, 'rough_known', ())
        if uneven_here and self.terrain_on and off > self.terrain_centre and self.shifted_in != cur:
            self.shifted_in = cur
            self.get_logger().info(f'uneven cell {cur}: {off:.2f} m off its centre, shifting onto '
                                   'it without turning before anything else')
            self.queue.append(('shift', cx, cy))
            return
        if off > self.recentre and self.recentre_tries < 2 and cur not in self.no_recentre:
            self.recentre_tries += 1
            back = math.atan2(cy - self.pose[1], cx - self.pose[0])
            self.get_logger().info(f'{off:.2f} m off the centre of cell {cur}: walking back onto it')
            self.queue.extend([('turn', back, 5.0), ('drive', cx, cy, back, 'recentre')])
            return
        self.recentre_tries = 0
        self.update_kinds()
        # A side of this cell that reads 'unknown' NOW (it was known when the robot
        # arrived; the map has changed since) is settled before a job is chosen: the
        # planner passes over unknown sides. Simulator, 2026-11-26: just through the
        # curtain, grid corrected, the only way on read 'unknown' and the run ended
        # 'nothing left to explore' in the cell behind the curtain.
        if any(self.kinds.get((cur, d)) == 'unknown' and (cur, d) not in getattr(self, 'exit_open', ())
               for d in range(4)) and cur not in self.closed:
            self.get_logger().info(f'cell {cur}: a side reads unknown again: settling it first')
            self.queue.append(('resolve', cur))
            return
        # A curtain the camera has seen is verified from a cell centre, looking straight at
        # its edge: from the cell before it (the camera is then 1.3 m away, inside the range
        # the model was trained for), or from its own cell when the robot is already there.
        self.cv_checking.clear()              # plan() runs with an empty queue: none is under way
        for a, d in sorted(self.cv_pending):
            if a == cur or (nb(cur, d) == a and self.kinds.get((cur, d)) in ('free', 'room')):
                self.cv_pending.discard((a, d))
                self.cv_checking.add((a, d))
                h = self.grid.heading(d)
                self.queue.extend([('turn', h, 2.0), ('hold', 0.5), ('cv_verify', a, d)])
                return
        # Possible stairs beside this cell are checked NOW, with the depth camera, not at
        # the end with the crossings: what is not stairs is a prop to look at (kind()).
        if self.stair_check_on:
            for d in range(4):
                if self.kinds.get((cur, d)) == 'stairs' and (cur, d) not in self.stairs_checked:
                    h = self.grid.heading(d)
                    self.queue.extend([('drive', cx, cy, h, 'square'), ('turn', h, 2.0),
                                       ('hold', 0.5), ('stair_check', cur, d)])
                    return
        facing = self.grid.nearest_dir(self.pose[2])
        # how closed the cell behind each open way already looks (dead ends go first)
        sense, beyond = self.sense(), {}
        for (c, d), k in self.kinds.items():
            b = nb(c, d)
            if k == 'free' and b not in self.visited:
                beyond[(c, d)] = sum(sense.edge(b, e)[0] == 'wall' for e in range(4) if e != opp(d))
        self.skip_looks()
        j = next_job(self.kinds, self.visited, self.closed, self.links, cur, facing,
                     self.looked, self.greeted, self.exit_cell, self.enter_rooms, beyond)
        job = f"{j['job']} cell {j['cell']} side {j['d']} via {len(j['path'])} cells"
        if job != self.job:
            here = ' '.join(self.kinds.get((cur, k), '-') for k in range(4))
            self.get_logger().info(f'job: {job}   [in {cur}: ahead/left/back/right = {here}; '
                                   f'{off:.2f} m off its centre]')
            self.job = job
        q = self.queue
        if j['path']:
            d, how, dest = j['path'][0]
            self.go(cur, d, how)
            return
        c, d, h = j['cell'], j['d'], None if self.grid is None else self.grid.heading(j['d'])
        if j['job'] == 'greet':
            q.extend([('turn', h, 2.0), ('event', f'curtain cell {c} side {d}')])
            if self.cv_on and (c, d) not in self.cv_curtains:
                # facing it squarely anyway: the camera's first look at the candidate
                q.extend([('hold', 0.5), ('cv_verify', c, d, False)])
            else:
                q.append(('hold', self.greet_hold))
            q.append(('call', lambda: self.greeted.add((c, d))))
        elif j['job'] == 'visit':
            self.go(c, d, 'walk')
        elif j['job'] == 'look' and self.look_check(c, d, h):
            pass                              # the check is queued; the look is planned again after it
        elif j['job'] == 'look':
            if self.qr_approach:
                q.append(('call', lambda: (self.scene_results.pop(nb(c, d), None),
                                           setattr(self, 'qr_t0', self.now()))))
            what = 'object' if self.kinds.get((c, d)) == 'object' else 'room'
            for name, off in (('centre', 0.0), ('left', self.look_angle)):
                q.extend([('turn', wrap(h + off), 3.0), ('event', f'{what} {name} cell {c} side {d}'),
                          ('hold', self.look_hold)])
            # the third view: scene understanding sends its prompt when it gets this one
            q.extend([('turn', wrap(h - self.look_angle), 3.0), ('look_last', c, d, what)])
        elif j['job'] in ('curtain', 'stairs'):
            self.go(c, d, j['job'])
        elif j['job'] == 'exit':
            # The end stop (user, 2026-11-26). `c` has the way out on side exit_d. With
            # the row rule c is the cell BEFORE the end cell: walk into the end cell first.
            # Then on to where the nose (fp_front ahead of base_link) is end_gap short of
            # the open far edge, still inside the cell; turn end_turn AWAY from maze_side
            # (maze on the left = turn right; user, 2026-11-26: was towards it)
            # (about the spot, not the cell centre); lie down. That ends the run.
            d = self.exit_d
            h = self.grid.heading(d)
            end = c if self.maze_rows <= 0 or c[0] >= self.maze_rows else nb(c, d)
            ex, ey = self.grid.centre(end)
            reach = self.cell_size / 2 - self.end_gap - self.fp_front
            tx, ty = ex + reach * math.cos(h), ey + reach * math.sin(h)
            side = {'L': -1.0, 'R': 1.0}.get(str(getattr(self, 'maze_side', '')).upper()[:1], 0.0)
            if not side:
                self.get_logger().warn('maze_side not given (L / R): no turn at the end')
            # A possible exit only: a step like any other, so that being stopped on the
            # way in makes this side a wall (stuck()) and the next candidate is tried. A
            # plain open way keeps the old rule: not reached = STOPPED, for a human.
            weak = (c, d) in self.exit_weak
            q.extend([('turn', h, 3.0), ('drive', tx, ty, h, 'square'),
                      ('call', lambda: setattr(self, 'step', (c, d, end) if weak and end != c else None))])
            if end != c:
                q.extend([('event', f'exit: into the end cell {end} from cell {c} side {d}'
                                    + (' (possible exit: trying it)' if weak else '')),
                          ('drive', ex, ey, h, 'normal'),
                          ('call', lambda: (setattr(self, 'step', None), self.visited.add(end)))])
            def reached():
                off = math.hypot(tx - self.pose[0], ty - self.pose[1])
                if off > 0.20:        # something was in the way: do not lie down out here
                    self.halt('STOPPED', f'end stop not reached ({off:.2f} m short): finish by '
                                         'hand, or clear the way and send /nav/start')
            q.extend([('drive', tx, ty, h, 'creep'), ('drive', tx, ty, h, 'square'),
                      ('call', reached)])
            if side:
                q.append(('turn', wrap(h + side * self.end_turn), 2.0, 'here'))
            q.extend([('hold', 1.0),
                      ('event', f'exit: stopped in the end cell {end}, nose {self.end_gap:.2f} m '
                                f'before its open edge' + (f', turned {math.degrees(self.end_turn):.0f} '
                                f'deg {"left" if side > 0 else "right"}' if side else ''))])
            if self.lie_down:
                q.extend([('cmd', robot.STAND_SIT), ('event', 'exit: lying down'), ('hold', 2.0)])
            q.append(('call', lambda: self.halt('DONE', 'in the end cell')))
        else:
            # Everything reachable is explored and no exit is known (user, 2026-11-26):
            # the navigation pauses and the robot lies down where it stands. STOPPED, not
            # DONE: /nav/start stands it up again and it plans afresh.
            self.event(f'no exit found: everything reachable is explored; pausing in cell {cur}'
                       + (' and lying down' if self.lie_down else ''))
            q.append(('hold', 1.0))
            if self.lie_down:
                q.extend([('cmd', robot.STAND_SIT),
                          ('call', lambda: setattr(self, 'lying', True)), ('hold', 2.0)])
            q.append(('call', lambda: self.halt(
                'STOPPED', 'everything reachable explored, no exit found: paused'
                           + (', lying down' if self.lie_down else '')
                           + '. /nav/start to stand up and go on')))

    def look_check(self, c, d, h):
        """Queue the stairs and curtain checks that come before a room / object look.
        Returns False when there is nothing (left) to check."""
        if not self.look_check_on or (c, d) in self.look_checked:
            return False
        # user, 2026-11-26: before a look the stair check is made (depth camera), but NOT
        # the terrain check: the cell is only looked into, never walked into. For a few
        # hours that day there was no depth check here at all. look_depth:=false = none.
        stairs = self.look_depth and self.stair_check_on and (c, d) not in self.stairs_checked
        edge = None
        if self.cv_on:                        # the first closed edge straight ahead
            cx, cy = self.grid.centre(c)
            edge = edge_along(self.grid, self.sense(), (cx, cy, h), 0.0)
            if edge in self.cv_curtains or edge in self.curtain_latched or edge in self.cv_rejected \
                    or edge in self.links:
                edge = None
            elif edge is not None and self.sense().low_wall(*edge):
                edge = None               # an ordinary wall: nothing for the camera to check
        self.look_checked.add((c, d))
        if not stairs and edge is None:
            return False
        q = self.queue
        q.extend([('turn', h, 2.0), ('hold', 0.5)])
        if stairs:
            q.append(('stair_check', c, d, True, False))      # stairs only, no terrain verdict
        if edge is not None:
            self.cv_pending.discard(edge)
            self.cv_checking.add(edge)
            q.append(('cv_verify', *edge))

        def verdict():
            found = [w for w, yes in (('stairs', self.stairs_checked.get((c, d)) is True),
                                      ('curtain', edge is not None and edge in self.cv_curtains)) if yes]
            if found:
                self.looked.add((c, d))       # no views: scene understanding gets nothing to send
                self.event(f'look called off, cell {c} side {d}: {" and ".join(found)} verified')
        q.append(('call', verdict))
        return True

    def do_look_last(self, c, d, what):
        """The robot stands at the third (right) view of a look. What the cell has turned
        out to be decides how the look ends (user, 2026-11-26):
          no longer a room / object (a curtain, stairs or a way on has shown up since):
              called off, nothing sent; the planner goes by what it is now
          a dead end with nothing taller than an ordinary wall in it: empty. The view is
              held (the QR scanner is on) but scene understanding gets no third view, so
              it sends nothing. Looked at.
          anything else: the third view goes out, then the wait for a QR code to approach."""
        self.send()
        self.update_kinds()
        k = self.kinds.get((c, d))
        if k not in ('room', 'object'):
            self.event(f'look called off, cell {c} side {d}: it is {k} now')
            return True
        q, done = self.queue, ('call', lambda: self.looked.add((c, d)))
        empty, squares = self.room_empty(c, d) if k == 'room' else (False, 0)
        if k == 'room' and not empty:         # why this room goes to scene understanding
            self.get_logger().info(
                f'room behind cell {c} side {d} not empty: {squares} tall squares (empty under '
                f'{self.empty_squares}), far wall measured: {self.sense().tall_inside(c, d)[1]}, '
                f'stair-high inside: {self.raised(c, d)}')
        if self.empty_skip and empty:
            q.extendleft(reversed([
                ('event', f'empty room cell {nb(c, d)} (from cell {c} side {d}): nothing taller '
                          f'than a wall ({squares} squares); no scene understanding'),
                ('hold', self.look_hold), done]))
            return True
        tail = [('event', f'{what} right cell {c} side {d}'), ('hold', self.look_hold)]
        if self.qr_approach and self.qr_read_t < 0:       # the one code is not read yet
            tail.append(('qr_wait', c, d))
        q.extendleft(reversed(tail + [done]))
        return True

    def go(self, a, d, how):
        """Queue one step from the centre of cell a through side d."""
        q, h, b = self.queue, self.grid.heading(d), nb(a, d)
        ax, ay = self.grid.centre(a)
        stairs = self.stair_check_on and (a, d) not in self.stairs_checked and (
            how == 'stairs' or (how == 'walk' and b not in self.visited and self.raised(a, d)))
        # the lidar has hardly seen the floor in there: let the depth camera read it
        terrain = (self.terrain_on and self.terrain_depth and how == 'walk'
                   and b not in self.visited and (a, d) not in self.rough_checked
                   and b not in self.rough_known and b not in self.rough_cells
                   and self.sense().floor_seen(a, d) < self.terrain_seen_min
                   and not self.sense().rough(a, d))      # already uneven: nothing to add
        if stairs or terrain:
            # look first; the next plan then knows whether this is stairs / uneven
            q.extend([('drive', ax, ay, h, 'square'), ('turn', h, 2.0), ('hold', 0.5),
                      ('stair_check', a, d, stairs)])
            return
        rough = how == 'walk' and (self.is_rough(a, d) or b in self.rough_cells)
        if how in ('walk', 'curtain'):
            self.gait_for_step(a, b, rough)
        if rough:
            self.rough_cells.add(b)   # now, not on arrival: a step that ends short of the
            #                           centre still counts as arrived (stuck()), on terrain
        if how == 'walk' and self.terrain_on and rough:
            # Uneven cell ahead: turn on the spot until the body is square to the way in
            # (whichever quarter turn is nearer), then step sideways to the next centre.
            body = min((wrap(h + math.pi / 2), wrap(h - math.pi / 2)),
                       key=lambda t: abs(wrap(t - self.pose[2])))
            bx, by = self.grid.centre(b)
            q.extend([('turn', body, 3.0),
                      ('event', f'terrain: sideways into cell {b} from cell {a} side {d}'),
                      ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('strafe', bx, by, h, body),
                      ('call', lambda: (self.walked.update({(a, d), (b, opp(d))}),
                                        self.rough_cells.add(b), self.arrive(b)))])
        elif how == 'walk':
            bx, by = self.grid.centre(b)
            q.extend([('turn', h, 4.0), ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('drive', bx, by, h, 'normal'),
                      ('call', lambda: (self.walked.update({(a, d), (b, opp(d))}),
                                        self.rough_cells.add(b) if rough else None,
                                        self.arrive(b)))])
        elif how == 'curtain' and self.cv_on and (a, d) not in self.cv_curtains \
                and (a, d) not in self.links:
            # A lidar candidate: the camera has the last word before any push. Confirmed ->
            # the next plan pushes; not confirmed -> the edge is a wall (kind()).
            q.extend([('drive', ax, ay, h, 'square'), ('turn', h, 2.0), ('hold', 0.5),
                      ('event', f'curtain: camera check before the push, cell {a} side {d}'),
                      ('cv_verify', a, d, True)])
        elif how == 'curtain':
            bx, by = self.grid.centre(b)

            def crossed():
                self.links[(a, d)], self.links[(b, opp(d))] = ('curtain', b), ('curtain', a)
                self.arrive(b)
            def blind(on):
                self.curtain_blind = on
                self.push_sure = bool(on and self.cv_on and (a, d) in self.cv_curtains)
                if not on:
                    self.leg_watch(False)
                if self.push_sure:
                    self.event(f'curtain: confirmed by the camera, obstacle stop off for '
                               f'this push, cell {a} side {d}')
            # Sideways (user, 2026-11-25): square up on the centre line facing the curtain,
            # make the nearer quarter turn on the spot, step sideways through it all the way
            # to the centre of the cell behind. There, on the spot about that centre:
            # curtain_right (180 deg) to the RIGHT, in steps of a quarter turn at most so
            # do_turn cannot take the short way round to the left, then curtain_left
            # (50 deg, user 2026-11-26; was 90) to the LEFT. The lidar counts as covered by
            # the fabric from the first step until the last of these turns is done.
            body = min((wrap(h + math.pi / 2), wrap(h - math.pi / 2)),
                       key=lambda t: abs(wrap(t - self.pose[2])))
            n = max(1, math.ceil(self.curtain_right / (math.pi / 2) - 1e-6))
            rights = [('turn', wrap(body - self.curtain_right * (i + 1) / n), 3.0) for i in range(n)]
            ends = wrap(body - self.curtain_right + self.curtain_left)
            say = (f'turned {math.degrees(self.curtain_right):.0f} right, '
                   f'{math.degrees(self.curtain_left):.0f} left')
            # No turns after the curtain (user, 2026-11-26, later the same day): the robot
            # stands 1 s at the centre of the cell behind and goes on. curtain_turns:=true
            # brings the 180 right / 50 left back.
            turns = rights + [('turn', ends, 3.0)]
            q.extend([('drive', ax, ay, h, 'square'), ('turn', body, 2.0),
                      ('event', f'curtain: going through sideways, cell {a} side {d}'),
                      ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('call', lambda: blind(True)),
                      ('strafe', bx, by, h, body, 'push'),
                      *(turns if self.curtain_turns else [('hold', 1.0)]),
                      ('call', lambda: blind(False)),
                      ('event', f'curtain: through, cell {b}; '
                                + (say if self.curtain_turns else 'no turns')),
                      ('call', crossed)])
        elif how == 'stairs':
            c = nb(b, d)
            cx, cy = self.grid.centre(c)

            def crossed():
                self.links[(a, d)], self.links[(c, opp(d))] = ('stairs', c), ('stairs', a)
                self.closed.add(b)
                self.arrive(c)
            q.extend([('drive', ax, ay, h, 'square'), ('turn', h, 2.0),
                      ('event', f'stairs: AI mode, cell {a} side {d}')])
            for code in robot.ai_enter_sequence():
                q.extend([('cmd', code), ('hold', self.cmd_gap)])
            if self.ai_auto:                  # stairs copy: follow /cmd_vel in the AI gait
                q.extend([('cmd', robot.AUTO), ('hold', self.cmd_gap)])
            q.extend([('hold', 1.0), ('call', lambda: setattr(self, 'step', (a, d, c))),
                      ('drive', cx, cy, h, 'ai'),
                      # Before the gait is changed back (user, 2026-11-26): is the robot in
                      # the middle of the cell after the stairs? The AI walk ends just past
                      # that cell's border, by distance. Stand until the pose has settled,
                      # line the grid up with where the walk put the robot (reanchor, which
                      # used to come after the gait change), and if the robot is off the
                      # centre walk onto it, still in the AI gait (do_ai_centre).
                      ('hold', self.ai_settle),
                      ('call', lambda: self.reanchor(ax, ay, h)),    # stairs copy
                      ('ai_centre', c, h), ('hold', 1.0)])
            for code in robot.ai_leave_sequence(self.gait):
                q.extend([('cmd', code), ('hold', self.cmd_gap)])
            q.extend([('call', lambda: setattr(self, 'cur_gait', self.gait)),
                      ('event', f'stairs: crossed, {self.gait} again'),
                      ('call', crossed)])

    def do_ai_centre(self, c, h):
        """After the stairs, still in the AI gait: walk onto the centre of cell c if the
        robot is more than ai_centre_tol off it. Forward only, steering towards the centre
        (the AI gait is driven with forward speed and a little yaw, nothing sideways or
        backwards). Not attempted, with the reason in the event, when the pose is old, the
        centre is not ahead of the robot (within 70 deg), or it is further than a cell
        away (then the pose is not to be believed). Ends at the centre, at a wall just
        ahead, or on its time limit. Whatever is left is the recentre rule's job after
        the gait change."""
        mem, now = self.act_mem, self.now()
        cx, cy = self.grid.centre(c)
        x, y, yaw = self.pose
        dx, dy = cx - x, cy - y
        off = math.hypot(dx, dy)
        bearing = wrap(math.atan2(dy, dx) - yaw)

        def end(text):
            self.send()
            self.ai_walking = False
            if 't0' in mem:                   # it walked: the pose may step once more
                self.odom_jumps, self.odom_hold_until = [], 0.0
                self.odom_grace_until = now + 3.0
            self.event(f'stairs: {text}, cell {c} ({off:.2f} m off its centre)')
            return True
        if 't0' not in mem:
            if not self.ai_centre_on or off <= self.ai_centre_tol:
                return end('in the middle of the cell after' if off <= self.ai_centre_tol
                           else 'centre check off')
            if now - (self.pose_time or 0.0) > self.pose_timeout:
                return end('not walked onto the centre: pose too old')
            if off > self.cell_size or abs(bearing) > math.radians(70):
                return end('not walked onto the centre: it is '
                           + ('further than a cell away' if off > self.cell_size else
                              f'{math.degrees(bearing):+.0f} deg off straight ahead'))
            mem.update(t0=now, tmax=off / max(self.ai_centre_speed, 0.05) * 2.5 + 3.0, wall=0)
            self.ai_walking = True            # as in the AI walk: odometry steps are expected
            self.get_logger().info(f'after the stairs: {off:.2f} m off the centre of cell {c}: '
                                   'walking onto it in the AI gait before the gait change')
        ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
        if off <= 0.04 or ahead <= 0.02:
            return end('walked onto the centre of the cell after')
        mem['wall'] = mem['wall'] + 1 if self.cloud is not None and self.front_blocked() else 0
        if mem['wall'] >= 3:
            return end('walk onto the centre ended by a wall ahead')
        if now - mem['t0'] > mem['tmax'] or now - (self.pose_time or 0.0) > self.pose_timeout:
            return end('walk onto the centre ended (time limit or old pose)')
        self.send(vx=min(self.ai_centre_speed, max(0.10, 1.5 * ahead)),
                  wz=clamp(1.2 * bearing, -0.4, 0.4))
        return False

    def gait_for_step(self, a, b, rough):
        """Queue a gait change before the step from cell a into cell b, if the floor asks
        for one (see flat_gait). Move -> gait -> navigation mode, standing, cmd_gap apart."""
        if not self.gait_switch or a in self.rough_cells:
            return                    # standing on uneven terrain: the gait stays
        want = self.gait if rough else self.flat_gait
        if want == self.cur_gait:
            return
        self.queue.append(('event', f'gait: {want} for the step into cell {b} '
                                    f'({"uneven" if rough else "plain"} floor), was {self.cur_gait}'))
        for code in robot.start_sequence(want):
            self.queue.extend([('cmd', code), ('hold', self.cmd_gap)])
        self.queue.append(('call', lambda: setattr(self, 'cur_gait', want)))
        self.cur_gait = want          # go() runs again before these are done: queue it once

    def reanchor(self, ax, ay, h):
        """stairs copy: after the AI walk the lidar pose is off (Fast-LIO drifts or diverges
        on the steps), so everything mapped from here on is shifted against the cells. The
        robot really is where the walk put it: ai_last_walked straight on along h from the
        centre of the cell before (ax, ay). Move and turn the grid by the difference between
        that and the believed pose, so the cells line up with the walls again as they are now
        mapped; then one medium wall fit near the robot (reanchor_fit) refines it."""
        if not self.ai_reanchor or self.pose is None or self.grid is None:
            return
        w = self.ai_last_walked
        ex, ey = ax + w * math.cos(h), ay + w * math.sin(h)
        px, py, psi = self.pose
        dth, shift = wrap(psi - h), math.hypot(px - ex, py - ey)
        if shift < self.reanchor_min and abs(dth) < math.radians(4):
            self.get_logger().info(f'after the stairs: pose {shift:.2f} m / '
                                   f'{math.degrees(dth):+.1f} deg from expected, grid kept')
            return
        if abs(dth) > math.radians(30):      # the walk did not stay straight: don't guess
            self.get_logger().warn(f'after the stairs: heading {math.degrees(dth):+.0f} deg off, '
                                   'too far to re-anchor; grid kept')
            return
        c, s = math.cos(dth), math.sin(dth)
        gx, gy = self.grid.x0 - ex, self.grid.y0 - ey
        self.grid = Grid(px + c * gx - s * gy, py + s * gx + c * gy,
                         wrap(self.grid.yaw + dth), self.grid.size)
        self.reanchor_fit = 6                 # planning steps that may use the medium fit
        self.event(f'stairs: grid re-anchored, pose was {shift:.2f} m / '
                   f'{math.degrees(dth):+.1f} deg off the expected spot')

    def do_cv_verify(self, cell, d, final=True):
        """Standing still, facing side d of `cell` squarely: does the camera keep seeing a
        curtain straight ahead? Yes = that edge is a curtain, also where it reads as a wall.
        No and `final` = not a curtain: a lidar candidate there becomes a wall. No and not
        final (the greet) = it stays a candidate, looked at again before the push."""
        self.send()
        if self.elapsed() < self.cv_verify_time:
            return False
        frames = [hit for t, hit in self.cv_frames if t >= self.act_t0]
        b, hits = nb(cell, d), sum(frames)
        both = {(cell, d), (b, opp(d))}
        if hits >= self.cv_verify_hits and self.sense().low_wall(cell, d):
            # the camera looks over a 0.53 m wall at a curtain further back
            self.cv_rejected.update(both)
            self.cv_pending.difference_update(both)
            self.event(f'curtain check cell {cell} side {d}: the camera sees a curtain '
                       f'({hits} of {len(frames)} frames) but an ordinary wall stands here: '
                       'the curtain is behind it; this edge stays a wall')
            return True
        if hits >= self.cv_verify_hits:
            self.cv_curtains.update(both)
            self.forced_wall.difference_update(both)
            self.event(f'curtain: seen by the camera, cell {cell} side {d} '
                       f'({hits} of {len(frames)} frames): passable')
            return True
        why = f'{hits} of {len(frames)} frames'
        if not frames:
            why = 'no camera frames: is curtain_node running? see /curtain/status'
            self.get_logger().warn(f'curtain check cell {cell} side {d}: {why}')
        if not final:
            self.event(f'curtain check cell {cell} side {d}: not confirmed by the camera yet '
                       f'({why}); looked at again before the push')
            return True
        lidar = (cell, d) in self.curtain_latched
        self.cv_rejected.update(both)
        self.cv_pending.difference_update(both)
        self.curtain_latched.difference_update(both)
        self.event(f'curtain check cell {cell} side {d}: not confirmed by the camera '
                   f'({why})' + ('; the lidar curtain is a wall now' if lidar else ''))
        return True

    def raised(self, cell, d):
        """The lidar shows something stair-high (not just terrain) in the cell behind side d."""
        _, front, top = self.sense().interior(cell, d)
        return max([v for v in (front, top) if not math.isnan(v)], default=0.0) >= self.stair_raise_min

    def do_stair_check(self, cell, d, stairs=True, terrain=True):
        """Standing still, facing side d: watch the depth camera for stair_check_time s.
        It looks for stairs (unless stairs=False) and reads the floor of the next cell for
        uneven terrain (terrain_from_depth)."""
        self.send()
        mem, now = self.act_mem, self.now()
        self.odom_grace_until = now + 4.0
        if 'subs' not in mem:
            mem.update(frame=None, k=None, frames=0, hits=0, why='no depth frame arrived',
                       confirm=stair_levels.Confirm(), rails='', mount=None, rough=[])
            mem['subs'] = [
                self.create_subscription(Image, self.depth_topic,
                                         lambda m: mem.update(frame=m), qos_profile_sensor_data),
                self.create_subscription(CameraInfo, self.depth_info_topic,
                                         lambda m: mem.update(k=(m.k[0], m.k[4], m.k[2], m.k[5])),
                                         qos_profile_sensor_data)]
            self.get_logger().info(f'stair check: cell {cell} side {d}, depth camera on')
            return False
        if mem['mount'] is None:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', 'camera_link', Time())
                t, q = tf.transform.translation, tf.transform.rotation
                mem['mount'] = (stair_levels.quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z]))
            except TransformException:       # measured mount: 0.252 m ahead, 20 deg down
                mem['mount'] = (stair_levels.quat_to_matrix(0.0, 0.17364817766693033, 0.0,
                                                            0.984807753012208),
                                np.array([0.25228, -0.0005, 0.0741]))
        found = None
        look = stairs and mem.get('gate') is not False     # run the stair detector on this frame
        msg, mem['frame'] = mem['frame'], None
        if msg is not None and mem['k'] is not None and self.odom_rot is not None:
            depth = self.depth_image(msg)
            if depth is not None:
                level = stair_levels.roll_pitch_only(self.odom_rot)
                r, t = stair_levels.level_camera(level, *mem['mount'])
                # Floor first, stairs after (user, 2026-11-26). The first frames are read
                # for the floor only. The stair detector (up to stair_check_time s of raw
                # depth) runs only if that reading shows something stair_gate_min or
                # higher in the cell, or gives no usable reading at all (close up, stairs
                # hide the floor: 04:42 run, stairs confirmed with '0 usable frames'). Three
                # usable frames with nothing that high = not stairs, and depth closes with
                # the floor verdict. 05:06 run: 33 frames (5 s) to say 'not stairs' of a
                # flat cell.
                gated = stairs and self.stair_gate and self.terrain_on and self.terrain_depth
                if gated and mem.get('gate') is None:
                    look = False
                if self.terrain_on and self.terrain_depth and (
                        not mem.get('uneven') or (gated and mem.get('gate') is None)):
                    reading = self.terrain_from_depth(depth, mem['k'], r, t, cell, d)
                    if not mem.get('uneven'):
                        mem['rough'].append(reading)
                    if gated and mem.get('gate') is None:
                        g = mem.setdefault('gate_frames', [])
                        g.append(self.depth_high if reading[0] is not None else None)
                        usable = [n for n in g if n is not None]
                        if len(usable) >= self.stair_gate_frames:
                            mem['gate'] = any(n >= self.stair_gate_squares for n in usable)
                            mem['gate_high'] = max(usable)
                        elif len(g) >= 2 * self.stair_gate_frames:
                            mem['gate'], mem['gate_high'] = True, None     # no reading: look
                    votes = [v for v, _ in mem['rough'] if v is not None]
                    # Uneven confirmed (user, 2026-11-26): terrain_confirm frames say so
                    # and they are the majority. The floor is not read again, in this
                    # check or any later one for that cell; and with no stairs to look
                    # for, the check (and the depth stream) ends here.
                    # terrain=False (the check before a room / object look: the cell is not
                    # walked into): no terrain verdict. The floor frames are still read,
                    # but only as the quick gate for the stair detector.
                    if terrain and sum(votes) >= self.terrain_confirm and sum(votes) * 2 > len(votes):
                        mem['uneven'] = True
                        self.rough_known.add(nb(cell, d))
                # The detector looks for the floor within 12 cm of the standing height it is
                # given. Try the measured one, then the two usual ones (0.35 nominal, 0.30).
                h = self.body_height
                seen = None
                for height in (([h] if h and 0.25 <= h <= 0.40 else []) + [0.35, 0.30]) if look else []:
                    self.stair_params.body_height = height
                    seen = stair_levels.detect(depth, mem['k'], r, t, self.stair_params)
                    if seen is not None:
                        break
                mem['why'] = ('stairs seen' if seen else stair_levels.detect.last_why) if look \
                    else 'stairs not looked for'
                if seen is not None and self.stair_rails:
                    pts = np.concatenate(list(self.clouds)).astype(float) if self.clouds else None
                    if pts is not None:      # /maze/cloud z = height above the floor
                        pts[:, 2] += seen.floor_z
                    ok, mem['rails'] = stair_levels.rails_check(pts, seen)
                    if not ok:
                        mem['why'], seen = f'treads look like stairs but {mem["rails"]}', None
                mem['frames'] += 1
                mem['looked'] = mem.get('looked', 0) + bool(look)
                mem['hits'] += seen is not None
                if seen is not None:
                    mem['last_seen'] = seen          # stairs copy
                found = mem['confirm'].add(now, seen)
        waited = self.elapsed()
        quick_no = mem['hits'] == 0 and (mem.get('looked', 0) >= self.stair_quick_no or (
            mem.get('gate') is not True and mem['frames'] >= self.stair_quick_no))   # close depth early
        quick_no = quick_no and sum(r is not None for r, _ in mem['rough']) >= 3 \
            if self.terrain_on and self.terrain_depth else quick_no
        if mem.get('uneven') and (not stairs or mem.get('gate') is False):
            quick_no = True
        if found is None and not quick_no and \
                waited < self.stair_check_time + (2.0 if mem['frames'] == 0 else 0.0):
            return False
        for sub in mem['subs']:
            self.destroy_subscription(sub)
        if found is None and mem['hits'] >= self.stair_min_hits and mem.get('last_seen') is not None:
            # stairs copy: hits spread over the look (6 of 28 frames, run8, 2026-11-25) never
            # made 3 of 5 in 1.2 s. The block obstacle gives 0 hits, so this many in one look
            # still means stairs.
            found = mem['last_seen']
            mem['rails'] = f'{mem["hits"]} of {mem["frames"]} frames over the look'
        lidar_no = getattr(self, 'stairs_not_in_lidar', lambda c_, d_: '')(cell, d) \
            if found is not None else None
        if found is not None and lidar_no:
            verdict = False
            text = (f'NOT stairs: the depth camera reports {found.summary()}, but {lidar_no}')
        elif found is not None:
            verdict = True
            text = f'STAIRS: {found.summary()}; {mem["rails"]}'
        elif mem['frames'] == 0:
            verdict = 'lidar'                # no camera: the lidar's reading stands
            text = f'no depth frames in {waited:.1f} s; keeping what the lidar says'
        elif mem.get('gate') is False:
            verdict = False
            text = (f'not stairs: floor read first, nothing {self.stair_gate_min:.2f} m or higher '
                    f'in the cell (at most {mem["gate_high"]} squares); stair detector not run')
        else:
            verdict = False
            text = f'not stairs ({mem["hits"]} of {mem.get("looked", 0)} frames; last: {mem["why"]})'
        if stairs:
            self.stairs_checked[(cell, d)] = verdict
            self.event(f'stair check cell {cell} side {d}: {text}')
        if terrain and self.terrain_on and self.terrain_depth:
            votes = [r for r, _ in mem['rough'] if r is not None]
            seen = max((n for _, n in mem['rough']), default=0)
            rough = None if len(votes) < 3 else sum(votes) * 2 > len(votes)
            self.rough_checked[(cell, d)] = rough
            self.event(f'terrain check cell {nb(cell, d)} (from cell {cell} side {d}): ' + (
                f'too little floor seen ({len(votes)} usable frames, {seen} squares)' if rough is None
                else f'{"UNEVEN" if rough else "flat"} ({sum(votes)} of {len(votes)} frames uneven, '
                     f'up to {seen} squares seen)'))
        self.queue.appendleft(('hold', 1.5))        # the robot's streams recover
        return True

    def stairs_not_in_lidar(self, cell, d):
        """Why the lidar says there are no stairs behind side d, or '' when it agrees or
        cannot tell. False stairs (user, 2026-11-26; 04:42 run: 'stairs 0.70 m, levels
        0.12 / 0.25 / 0.38' ahead of the start, in a cell the robot later walked through):
        the depth camera alone is not believed. Stairs are 0.45 m high and a cell wide, and
        the lidar looks straight at them from the cell before, so its height map must show
        something stair_lidar_top or higher in that cell (terrain stays under 0.30 m).
        Not applied until the height map has had stair_lidar_clouds clouds (it starts
        afresh after a re-anchor)."""
        if not self.stair_agree or self.heights_n < self.stair_lidar_clouds:
            return ''
        _, front, top = self.sense().interior(cell, d)
        seen = [v for v in (front, top) if not math.isnan(v)]
        high = max(seen, default=0.0)
        if high >= self.stair_lidar_top:
            return ''
        return (f'the lidar shows nothing {self.stair_lidar_top:.2f} m high in that cell '
                + (f'(highest {high:.2f} m)' if seen else '(nothing at all)')
                + ': not believed')

    def terrain_from_depth(self, depth, k, r, t, cell, d):
        """One depth frame -> (rough, squares seen) for the cell behind side d, or
        (None, n). Points in the level frame (base_link without roll / pitch), the floor
        found as the lowest dense layer near the standing height, then onto the map."""
        g = stair_levels.grid_points(depth, k, r, t, self.stair_params)
        pts = g.reshape(-1, 3)
        pts = pts[np.isfinite(pts[:, 2])]
        self.depth_high = None                # squares stair-high in this frame; None = no reading
        if len(pts) < 100 or self.pose is None:
            return None, 0
        h = self.body_height if self.body_height and 0.25 <= self.body_height <= 0.40 else 0.30
        self.stair_params.body_height = h
        floor, measured = stair_levels.find_floor(pts, self.stair_params)
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]
        sense, above = self.sense(), pts[:, 2] - floor
        if measured:                          # for the stair gate (do_stair_check)
            self.depth_high = int((sense.square_heights(cell, d, mx, my, above)
                                   >= self.stair_gate_min).sum())
        return sense.rough_from_points(cell, d, mx, my, above)

    def is_rough(self, a, d):
        """Uneven terrain behind side d of cell a: the depth camera's verdict if it gave
        one, else the lidar's height map."""
        if nb(a, d) in self.rough_known:      # confirmed once, from whichever side
            return True
        checked = self.rough_checked.get((a, d))
        return checked if checked is not None else self.sense().rough(a, d)

    @staticmethod
    def depth_image(msg):
        if msg.encoding in ('16UC1', 'mono16'):
            kind = np.uint16
        elif msg.encoding == '32FC1':
            kind = np.float32
        else:
            return None
        a = np.frombuffer(bytes(msg.data), kind)
        a = a.reshape(msg.height, msg.step // np.dtype(kind).itemsize)[:, :msg.width]
        return a.byteswap() if msg.is_bigendian else a

    # ---- actions ------------------------------------------------------------------------
    def jump_settled(self):
        """A burst of odometry jumps is over (odom_hold_until has passed). Returns True
        when it has been dealt with; False = stand a little longer, Fast-LIO still moves."""
        now = self.now()
        t0, before = self.jump_from
        recent = [q for t_, q in self.poses if now - t_ <= 1.0]
        still = len(recent) >= 5 and all(
            math.hypot(q[0] - self.pose[0], q[1] - self.pose[1]) < 0.05
            and abs(wrap(q[2] - self.pose[2])) < math.radians(3) for q in recent)
        tilt = 0.0
        if self.odom_rot is not None:
            tilt = math.degrees(math.acos(clamp(float(self.odom_rot[2][2]), -1.0, 1.0)))
        if not still or tilt > 25.0:
            if now - self.odom_hold_until < self.jump_fix_wait:
                self.why = 'odometry jumped: waiting for it to settle before re-anchoring'
                return False
            self.get_logger().warn(
                f'after the jump: odometry has not settled in {self.jump_fix_wait:.0f} s '
                f'(still {"moving" if not still else "tilted %.0f deg" % tilt}): no re-anchor')
            self.jump_from = None
            return True
        self.jump_from = None
        ex, ey, eh = before
        px, py, psi = self.pose
        dth, shift = wrap(psi - eh), math.hypot(px - ex, py - ey)
        if shift < self.jump_fix_min and abs(dth) < math.radians(4):
            self.get_logger().info(f'after the jump: pose {shift:.2f} m / {math.degrees(dth):+.1f} '
                                   'deg from where it was: grid kept')
            return True
        if shift > self.jump_fix_max or abs(dth) > math.radians(30):
            self.get_logger().warn(f'after the jump: pose {shift:.2f} m / {math.degrees(dth):+.0f} '
                                   'deg from where it was: too far to re-anchor, grid kept')
            return True
        here = self.grid.cell_of(ex, ey)
        self.update_kept(here)
        c, s = math.cos(dth), math.sin(dth)
        gx, gy = self.grid.x0 - ex, self.grid.y0 - ey
        self.grid = Grid(px + c * gx - s * gy, py + s * gx + c * gy,
                         wrap(self.grid.yaw + dth), self.grid.size)
        self.reanchor_fit = 6                 # planning steps that may use the medium fit
        self.heights, self.heights_n = HeightMap(), 0
        self.front_seen = []
        self.event(f'odometry: grid re-anchored after a jump, pose was {shift:.2f} m / '
                   f'{math.degrees(dth):+.1f} deg off; still in cell {here}; '
                   f'{len(self.kept)} sides of other cells kept as known')
        return True

    def update_kept(self, here):
        """Freeze what is known of every visited cell but `here` (see jump_fix)."""
        for (cell, d), k in self.kinds.items():
            if cell != here and k != 'unknown':
                self.kept.setdefault((cell, d), k)

    def tick(self):
        have = self.read_pose()
        if have:
            self.poses.append((self.now(), self.pose))
        if self.curtain_blind:
            self.blind_pub.publish(Bool(data=True))
            self.blind_until = self.odom_grace_until = self.now() + 1.5
        elif self.map_blind and (self.ai_walking or self.now() < self.map_blind_until):
            self.blind_pub.publish(Bool(data=True))      # odometry not to be trusted now
            self.blind_until = self.now() + 1.0          # no height-map updates either
        if self.state == 'WAIT':
            miss = [n for n, ok in (('/map', self.occ is not None), ('map -> base_level', have),
                                    ('/maze/cloud', self.cloud is not None)) if not ok]
            self.why = 'waiting for ' + ', '.join(miss)
            if not miss:
                self.state, self.why = 'READY', 'send /nav/start'
                self.get_logger().info('ready: ros2 topic pub --once /nav/start std_msgs/msg/Empty')
                if self.auto_start:
                    self.begin()
        elif self.state == 'RUN':
            age = max(self.now() - self.pose_time, self.now() - (self.cloud_time or 0.0))
            if self.jump_from is not None and self.now() >= self.odom_hold_until \
                    and not self.ai_walking and not self.jump_settled():
                self.send()
                self.act_t0 += 0.1
                if 't' in self.act_mem:
                    self.act_mem['t'] += 0.1
            elif self.now() < self.odom_hold_until and not self.ai_walking:   # stairs copy
                self.send()
                self.why = 'odometry jumped: paused until it has held steady'
                self.act_t0 += 0.1
                if 't' in self.act_mem:
                    self.act_mem['t'] += 0.1
            elif not self.ai_walking and not (self.act and self.act[0] == 'stair_check') \
                    and self.feed_hold():
                self.send()
                self.why = 'sensor feed late: standing still until it is on time'
                self.act_t0 += 0.1                     # the action's clock stands still too
                if 't' in self.act_mem:
                    self.act_mem['t'] += 0.1
            elif age > self.pose_timeout and not self.ai_walking and \
                    not (self.act and self.act[0] == 'stair_check'):
                self.send()
                self.why = f'pose / lidar {age:.1f} s old: standing still until it is fresh'
                self.act_t0 += 0.1                     # the action's clock stands still too
            else:
                self.why = ''
                for _ in range(20):                    # instant actions do not cost a tick
                    if self.state != 'RUN':            # an action ended the run: no new plan
                        break
                    if self.act is None:
                        if not self.queue:
                            self.plan()
                        if not self.queue or self.state != 'RUN':
                            break
                        self.act, self.act_t0, self.act_mem = self.queue.popleft(), self.now(), {}
                    done = getattr(self, 'do_' + self.act[0])(*self.act[1:])
                    if not done:
                        break
                    self.act = None
        cell = self.cur_cell() if self.grid and self.pose else None
        self.status_pub.publish(String(
            data=f"{self.state} cell={cell} job=[{self.job}] act={self.act[0] if self.act else '-'} {self.why}"))

    def elapsed(self):
        return self.now() - self.act_t0

    def do_hold(self, seconds):
        self.send()
        return self.elapsed() >= seconds

    def do_cmd(self, code):
        self.send()
        self.simple_cmd(code)
        return True

    def do_event(self, text):
        self.event(text)
        return True

    def do_call(self, fn):
        fn()
        return True

    def body_centre(self):
        """The robot's natural pivot in the map: pivot_offset behind base_link."""
        x, y, yaw = self.pose
        return x - self.pivot_offset * math.cos(yaw), y - self.pivot_offset * math.sin(yaw)

    def do_turn(self, target, tol_deg, pivot=None):
        """Turn on the spot to heading `target`, about the robot's NATURAL pivot (user,
        2026-11-26), not about the cell centre. That pivot is base_link (pivot_offset 0,
        measured on recorded turns), taken where it is when the turn begins. A plain turn
        keeps it nearly still by itself, so nothing is commanded but the rotation (and
        the small turn_forward against the Lite3's backward creep) unless the pivot
        drifts more than pivot_slack; then it is steered back (forward only, never a
        backward command; sideways both ways).
        Steering base_link onto the cell centre instead made the robot walk while it
        turned (03:45 run: 0.13 m/s forward with the turn hunting left and right).
        `pivot` is ignored (it used to choose the cell centre or the spot).
        Gains and limits: turn_kp, pivot_offset, pivot_slack, pivot_kp, pivot_vx, pivot_vy,
        turn_forward, w_max (config/speeds.yaml). The log line at the end says how far
        base_link and the pivot moved."""
        x, y, yaw = self.pose
        mem = self.act_mem
        px, py = self.body_centre()
        if 'pivot' not in mem:
            mem.update(pivot=(px, py), turn=wrap(target - yaw), start=(x, y, yaw))
        dx, dy = mem['pivot'][0] - px, mem['pivot'][1] - py
        ahead = dx * math.cos(yaw) + dy * math.sin(yaw)       # pivot ahead of where it is now (+)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        err = wrap(target - yaw)
        if abs(err) < math.radians(tol_deg):
            mem['ok'] = mem.get('ok', 0) + 1
            self.send()
            if mem['ok'] >= 3 and abs(mem['turn']) > math.radians(20):
                x0, y0, yaw0 = mem['start']
                fwd = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
                self.get_logger().info(
                    f'turn {math.degrees(mem["turn"]):+.0f} deg done in {self.elapsed():.1f} s: '
                    f'base_link moved {math.hypot(x - x0, y - y0) * 100:.0f} cm '
                    f'({fwd * 100:+.0f} cm along the heading it started with), '
                    f'pivot moved {math.hypot(dx, dy) * 100:.0f} cm')
            return mem['ok'] >= 3
        mem['ok'] = 0
        if self.elapsed() > 25.0:
            self.get_logger().warn(f'turn timed out {math.degrees(err):.0f} deg short')
            self.send()
            return True
        wz = clamp(self.turn_kp * err, -self.w_max, self.w_max)
        wz = math.copysign(max(abs(wz), self.min_spin), wz)
        vx = vy = 0.0
        if math.hypot(dx, dy) > self.pivot_slack:             # drifted: steer the pivot back
            vx = clamp(self.pivot_kp * ahead, 0.0, self.pivot_vx)
            vy = clamp(self.pivot_kp * left, -self.pivot_vy, self.pivot_vy)
        if ahead > -0.03:
            vx += self.turn_ff * abs(wz) / self.w_max
        if self.front_blocked():
            vx = 0.0
        self.send(vx=vx, vy=vy, wz=wz)
        return False

    def front_blocked(self):
        """Something from knee height up, 6-20 cm ahead of the body, in the body's width;
        or something that was seen there within the last block_memory s and is now
        anywhere from the body to 20 cm ahead of it. The second part is for the lidar's
        blind range: Fast-LIO drops returns within 0.30 m of the lidar, which leaves a
        0.53 m wall straight ahead visible only while the nose is 14-20 cm from it."""
        p = self.cloud
        m = (p[:, 0] > self.fp_front + 0.06) & (p[:, 0] < self.fp_front + 0.20) & \
            (np.abs(p[:, 1]) < self.fp_half_width - 0.03) & (p[:, 2] > 0.30) & (p[:, 2] < 1.0)
        seen = int(m.sum()) >= 6
        if self.pose is None or self.ai_walking or self.block_memory <= 0:
            return seen               # on the stairs the map pose is not to be trusted
        x, y, yaw = self.pose
        c, s, now = math.cos(yaw), math.sin(yaw), self.now()
        self.front_seen = [f for f in self.front_seen if now - f[0] < self.block_memory]
        if seen:
            bx, by = float(p[m, 0].mean()), float(p[m, 1].mean())
            ox, oy = x + c * bx - s * by, y + s * bx + c * by
            self.front_seen = [f for f in self.front_seen
                               if math.hypot(f[1] - ox, f[2] - oy) > 0.10][-20:] + [(now, ox, oy)]
            return True
        for _, ox, oy in self.front_seen:
            ahead, left = c * (ox - x) + s * (oy - y), -s * (ox - x) + c * (oy - y)
            if 0.0 < ahead < self.fp_front + 0.20 and abs(left) < self.fp_half_width:
                return True
        return False

    def do_drive(self, tx, ty, heading, mode):
        if mode == 'ai':
            return self.do_ai_walk(tx, ty)
        x, y, yaw = self.pose
        dx, dy = tx - x, ty - y
        c, s = math.cos(heading), math.sin(heading)
        along, lat, herr = dx * c + dy * s, -dx * s + dy * c, wrap(heading - yaw)
        mem = self.act_mem
        if mode == 'push':                   # curtain over the lidar: wall_scan sends no scans
            self.blind_pub.publish(Bool(data=True))
            self.blind_until = self.now() + 1.5
        if mode == 'square':                 # onto the cell's centre line, facing `heading`
            if abs(lat) < 0.04 and abs(herr) < math.radians(3) or self.elapsed() > 15.0:
                self.send()
                return True
            self.send(vy=clamp(0.8 * lat, -0.10, 0.10) if abs(herr) < math.radians(15) else 0.0,
                      wz=clamp(1.5 * herr, -self.w_max, self.w_max))
            return False
        if along <= 0.03:
            self.send()
            return True
        mem.setdefault('best', along)
        if along < mem['best'] - 0.05:
            mem['best'], mem['t'] = along, self.now()
        if self.now() - mem.setdefault('t', self.now()) > (12.0 if mode == 'ai' else 8.0):
            self.send()
            self.get_logger().warn(f'{mode} drive: no progress, {along:.2f} m short')
            return self.stuck(mode, along)
        if abs(herr) > math.radians(25):
            self.send(wz=math.copysign(max(self.min_spin, min(self.w_max, 1.5 * abs(herr))), herr))
            return False
        checked = mode in ('normal', 'creep', 'recentre') or (mode == 'push' and self.push_stop
                                                  and not self.push_sure)
        if checked and along > 0.10 and self.front_blocked():
            self.send()
            mem['blocked'] = mem.get('blocked', 0) + 1
            if mem['blocked'] >= 8:
                self.get_logger().warn(f'way blocked {along:.2f} m before the cell centre')
                return self.stuck(mode, along)
            return False
        mem['blocked'] = 0
        top = {'ai': self.ai_speed, 'push': self.curtain_speed, 'creep': 0.10, 'recentre': 0.10}.get(mode, self.v_max)
        self.send(vx=clamp(1.0 * along, 0.08, top), vy=clamp(0.8 * lat, -0.10, 0.10),
                  wz=clamp(1.5 * herr, -0.4, 0.4))
        return False

    def do_ai_walk(self, tx, ty):
        """stairs copy: walk straight over the stairs in the AI gait, then return True so the
        explorer's AI leave sequence (AI_OFF, MANUAL, MOVE, gait, AUTO) runs and exploring goes on.
        The lidar pose is not used (Fast-LIO diverges on the steps, run7). Distance is counted
        from the commanded speed and time (ai_distance:=leg counts leg odometry instead: in run9
        it under-counted and the robot walked into the wall of the cell after). The walk also
        ends early once the lidar sees a wall just ahead of the nose after ai_wall_min metres."""
        now, mem = self.now(), self.act_mem
        if 'dist' not in mem:
            self.ai_walking = True
            mem.update(dist=self.ai_walk_cells * self.cell_size + self.ai_body_clear, t0=now,
                       last=now, walked=0.0, leg=0.0, wall=0, yaw0=self.imu_yaw)
            mem['tmax'] = mem['dist'] / max(self.ai_speed, 0.05) * 2.5 + 5.0
            self.get_logger().info(f'AI walk: {mem["dist"]:.2f} m at {self.ai_speed:.2f} m/s '
                                   f'(by {self.ai_distance}, wall stop after {self.ai_wall_min:.1f} m)')
        dt, mem['last'] = now - mem['last'], now
        fresh = self.leg_vx is not None and now - self.leg_time < 0.5
        if self.ai_distance == 'leg' and fresh:
            mem['leg'] += max(0.0, self.leg_vx) * dt
            mem['walked'] += max(0.0, self.leg_vx) * dt
        else:
            mem['walked'] += self.ai_speed * dt
        why = None
        if mem['walked'] >= mem['dist']:
            why = 'distance reached'
        elif mem['walked'] >= self.ai_wall_min and self.cloud is not None and self.front_blocked():
            mem['wall'] += 1
            if mem['wall'] >= 3:
                why = 'wall ahead'
        else:
            mem['wall'] = 0
        if why is None and now - mem['t0'] > mem['tmax']:
            why = 'time limit'
        if why is not None:
            self.send()
            self.ai_walking = False
            self.ai_last_walked = mem['walked']
            self.odom_jumps, self.odom_hold_until = [], 0.0
            self.odom_grace_until = now + 3.0
            self.get_logger().info(f'AI walk done ({why}): {mem["walked"]:.2f} of {mem["dist"]:.2f} m '
                                   f'in {now - mem["t0"]:.1f} s')
            return True
        herr = 0.0
        if self.ai_wz > 0 and mem['yaw0'] is not None and self.imu_yaw is not None:
            herr = wrap(mem['yaw0'] - self.imu_yaw)
        self.send(vx=self.ai_speed, wz=clamp(0.8 * herr, -self.ai_wz, self.ai_wz))
        return False

    def do_shift(self, tx, ty):
        """Step onto (tx, ty) WITHOUT turning: forward and sideways only, the heading the
        robot has is kept. For getting onto the centre of an uneven cell before a turn.
        What lies behind the robot is left (nothing drives backwards). Ends at the spot,
        when only a backward move is left, when something is in the way, or after 8 s."""
        x, y, yaw = self.pose
        mem = self.act_mem
        mem.setdefault('yaw', yaw)
        dx, dy = tx - x, ty - y
        ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        ahead = max(ahead, 0.0)                               # never backwards
        todo = math.hypot(ahead, left)
        if todo <= 0.04 or self.elapsed() > 8.0:
            self.send()
            if math.hypot(dx, dy) > 0.08:
                self.get_logger().info(f'shift ended {math.hypot(dx, dy):.2f} m from the centre '
                                       '(the rest is behind the robot, or time ran out)')
            return True
        if self.travel_blocked(yaw + math.atan2(left, ahead)):
            mem['blocked'] = mem.get('blocked', 0) + 1
            self.send()
            return mem['blocked'] >= 5
        mem['blocked'] = 0
        v = clamp(1.0 * todo, 0.05, self.shift_speed)
        self.send(vx=v * ahead / todo, vy=v * left / todo,
                  wz=clamp(1.5 * wrap(mem['yaw'] - yaw), -0.3, 0.3))
        return False

    def side_blocked(self, sign):
        """Something from knee height up, 6-20 cm beside the body on the side it steps
        towards (sign +1 = left), along the body's length."""
        p = self.cloud
        y = sign * p[:, 1]
        m = (y > self.fp_half_width + 0.06) & (y < self.fp_half_width + 0.20) & \
            (p[:, 0] > -self.fp_rear + 0.03) & (p[:, 0] < self.fp_front - 0.03) & \
            (p[:, 2] > 0.30) & (p[:, 2] < 1.0)
        return int(m.sum()) >= 6

    def do_strafe(self, tx, ty, travel, body, mode='side'):
        """Step sideways to (tx, ty): the robot moves along `travel` while its body keeps
        the heading `body` (travel +- 90 deg). Used to enter uneven cells. The base_link line
        is side_offset towards the nose, which centres the body's length in the way in."""
        x, y, yaw = self.pose
        c, s = math.cos(travel), math.sin(travel)
        tx, ty = tx + self.side_offset * math.cos(body), ty + self.side_offset * math.sin(body)
        dx, dy = tx - x, ty - y
        along, lat, herr = dx * c + dy * s, -dx * s + dy * c, wrap(body - yaw)
        mem = self.act_mem
        if mode == 'push':                   # curtain over the lidar: wall_scan sends no scans
            self.blind_pub.publish(Bool(data=True))
            self.blind_until = self.now() + 1.5
        if along <= 0.03:
            self.send()
            return True
        mem.setdefault('best', along)
        if along < mem['best'] - 0.05:
            mem['best'], mem['t'] = along, self.now()
        if self.now() - mem.setdefault('t', self.now()) > 8.0:
            self.send()
            self.get_logger().warn(f'sideways step: no progress, {along:.2f} m short')
            return self.stuck(mode, along)
        if mode == 'push' and self.push_stall:
            self.leg_watch(True)
            if abs(herr) > math.radians(25):
                mem.pop('pushing', None)             # not stepping sideways just now
            elif self.push_stalled(mem):
                self.send()
                self.leg_watch(False)
                return self.stuck(mode, along)
        if abs(herr) > math.radians(25):         # knocked round: turn back first, on the spot
            self.send(wz=math.copysign(max(self.min_spin, min(self.w_max, 1.5 * abs(herr))), herr))
            return False
        side = 1.0 if math.sin(wrap(travel - body)) > 0 else -1.0     # +1: stepping to the left
        if (mode != 'push' or (self.push_stop and not self.push_sure)) and along > 0.10 \
                and self.side_blocked(side):
            self.send()
            mem['blocked'] = mem.get('blocked', 0) + 1
            if mem['blocked'] >= 8:
                self.get_logger().warn(f'way blocked {along:.2f} m before the cell centre (sideways)')
                return self.stuck(mode, along)
            return False
        mem['blocked'] = 0
        # wanted velocity in the map frame, then into the body frame as it stands now
        top = self.curtain_speed if mode == 'push' else self.side_speed
        v, w = clamp(1.0 * along, 0.06, top), clamp(0.8 * lat, -0.10, 0.10)
        wx, wy = v * c - w * s, v * s + w * c
        self.send(vx=wx * math.cos(yaw) + wy * math.sin(yaw),          # send() never lets
                  vy=-wx * math.sin(yaw) + wy * math.cos(yaw),         # vx go backwards
                  wz=clamp(1.5 * herr, -0.4, 0.4))
        return False

    # ---- QR approach ---------------------------------------------------------------------
    def qr_got(self):
        """A QR code has been read since this look began."""
        return self.qr_read_t >= self.qr_t0

    def do_qr_wait(self, c, d):
        """After the three views: stand until scene understanding has answered for this
        look. A QR code seen in the left or right view queues the approach."""
        self.send()
        if self.qr_got():
            return True
        r = self.scene_results.get(nb(c, d))
        if r is None:
            if self.elapsed() < self.qr_wait:
                return False
            self.get_logger().info(f'look cell {c} side {d}: no scene answer in '
                                   f'{self.qr_wait:.0f} s, moving on')
            return True
        view = r.get('qr_view') if r.get('qr') else None
        if view not in ('left', 'right'):
            if r.get('qr'):
                self.event(f'qr: code seen {view or "somewhere"}, cell {c} side {d}: no approach '
                           '(only for the left / right view)')
            return True
        sign = 1.0 if view == 'left' else -1.0
        h = self.grid.heading(d)
        body = wrap(h + sign * self.qr_angle)
        # as far as qr_depth past the border, counted for base_link (do_qr_step moves the
        # line side_offset along the body, a little of which points into the room)
        ax, ay = self.grid.centre(c)
        far = self.grid.size / 2 + self.qr_depth - self.side_offset * math.cos(self.qr_angle)
        bx, by = ax + far * math.cos(h), ay + far * math.sin(h)
        self.step = None                      # not a step between cells: nothing becomes a wall
        self.queue.extendleft(reversed([
            ('event', f'qr: code seen in the {view} view, cell {c} side {d}: turning '
                      f'{math.degrees(self.qr_angle):.0f} deg and stepping in'),
            ('qr_turn', body),
            ('qr_step', bx, by, h, body, True),
            ('qr_hold',),
            ('qr_sweep', h),
            ('qr_back', c, d, sign)]))
        return True

    def do_qr_turn(self, body):
        if self.qr_got():
            self.send()
            return True
        return self.do_turn(body, 3.0)

    def do_qr_hold(self):
        self.send()
        return self.qr_got() or self.elapsed() >= self.qr_hold

    def do_qr_sweep(self, back):
        """Far end reached and no code read: turn slowly towards the room's back wall
        (heading `back`), about the spot, until the code is read or the robot faces it."""
        if self.qr_got():
            self.send()
            return True
        fast, self.w_max = self.w_max, min(self.w_max, max(self.qr_sweep_rate, self.min_spin))
        try:
            return self.do_turn(back, 3.0, 'here')   # not about the room centre: that is deeper in
        finally:
            self.w_max = fast

    def do_qr_back(self, c, d, sign):
        """Out again, to the centre of the cell the look was made from: body square to the
        way back (15 deg more), then purely sideways, so nothing is driven backwards."""
        ax, ay = self.grid.centre(c)
        self.event(f'qr: {"code read" if self.qr_got() else "not read"}, cell {c} side {d}')
        if math.hypot(ax - self.pose[0], ay - self.pose[1]) > 0.08:
            h = self.grid.heading(d)
            body = wrap(h + sign * math.pi / 2)
            self.queue.extendleft(reversed([('turn', body, 3.0, 'here'),
                                            ('qr_step', ax, ay, wrap(h + math.pi), body, False)]))
        return True

    def travel_blocked(self, direction):
        """Something from knee height up within 6-20 cm of the body, the way it is moving
        (`direction`: map heading of the motion). Works for any body angle: a point counts
        if moving the body 6-20 cm that way would put it inside the body's outline."""
        p = self.cloud
        p = p[(p[:, 2] > 0.30) & (p[:, 2] < 1.0)]
        if len(p) == 0:
            return False
        a = wrap(direction - self.pose[2])
        ux, uy = math.cos(a), math.sin(a)
        hit = np.zeros(len(p), bool)
        for s in (0.06, 0.10, 0.15, 0.20):
            x, y = p[:, 0] - s * ux, p[:, 1] - s * uy
            hit |= (x > -self.fp_rear + 0.03) & (x < self.fp_front - 0.03) & \
                   (np.abs(y) < self.fp_half_width - 0.03)
        inside = (p[:, 0] > -self.fp_rear) & (p[:, 0] < self.fp_front) & \
                 (np.abs(p[:, 1]) < self.fp_half_width)          # the robot itself
        return int((hit & ~inside).sum()) >= 6

    def do_qr_step(self, tx, ty, travel, body, reading):
        """Move to (tx, ty) along `travel` with the body held at `body`. The base_link line
        is side_offset towards the nose (as in do_strafe), which centres the body in the way
        through. Ends at the target, when something is in the way, after 6 s without
        progress, or (reading) as soon as a QR code is read. Never makes a wall of it."""
        if reading and self.qr_got():
            self.send()
            return True
        x, y, yaw = self.pose
        c, s = math.cos(travel), math.sin(travel)
        if reading:
            tx, ty = tx + self.side_offset * math.cos(body), ty + self.side_offset * math.sin(body)
        dx, dy = tx - x, ty - y
        along, lat, herr = dx * c + dy * s, -dx * s + dy * c, wrap(body - yaw)
        mem = self.act_mem
        if along <= 0.03:
            self.send()
            return True
        mem.setdefault('best', along)
        if along < mem['best'] - 0.05:
            mem['best'], mem['t'] = along, self.now()
        if self.now() - mem.setdefault('t', self.now()) > 6.0:
            self.send()
            self.get_logger().info(f'qr step: no progress, {along:.2f} m short: ending it')
            return True
        if abs(herr) > math.radians(20):         # knocked round: turn back first, on the spot
            self.send(wz=math.copysign(max(self.min_spin, min(self.w_max, 1.5 * abs(herr))), herr))
            return False
        if self.travel_blocked(travel):
            self.send()
            mem['blocked'] = mem.get('blocked', 0) + 1
            if mem['blocked'] >= 5:
                self.get_logger().info(f'qr step: something in the way {along:.2f} m short: '
                                       'ending it')
                return True
            return False
        mem['blocked'] = 0
        v, w = clamp(1.0 * along, 0.05, self.qr_speed), clamp(0.8 * lat, -0.08, 0.08)
        wx, wy = v * c - w * s, v * s + w * c
        self.send(vx=wx * math.cos(yaw) + wy * math.sin(yaw),          # send() never lets
                  vy=-wx * math.sin(yaw) + wy * math.cos(yaw),         # vx go backwards
                  wz=clamp(1.5 * herr, -0.4, 0.4))
        return False

    def leg_watch(self, on):
        """Listen to the leg odometry only while pushing through a curtain."""
        if on and self.leg_sub is None:
            self.leg.clear()
            self.leg_sub = self.create_subscription(Odometry, self.leg_topic, self.on_leg,
                                                    qos_profile_sensor_data)
        elif not on and self.leg_sub is not None:
            self.destroy_subscription(self.leg_sub)
            self.leg_sub = None

    def on_leg(self, msg):
        now, p = self.now(), msg.pose.pose.position
        if self.leg and now - self.leg[-1][0] < 0.05:
            return                               # 20 a second is plenty
        if self.leg and math.hypot(p.x - self.leg[-1][1], p.y - self.leg[-1][2]) > 0.5:
            self.leg.clear()                     # its pose jumps now and then (bags): start over
        self.leg.append((now, p.x, p.y))

    def push_stalled(self, mem):
        """Pushing for long enough, and the legs say the robot has hardly moved."""
        now = self.now()
        since = mem.setdefault('pushing', now)
        if now - since < self.push_stall_grace + self.push_stall_time:
            return False
        if not self.leg or now - self.leg[-1][0] > 0.5:
            if not mem.get('no_leg'):
                mem['no_leg'] = True
                self.get_logger().warn(f'push: no {self.leg_topic}: no stall stop, the 8 s rule only')
            return False
        old = [s for s in self.leg if s[0] <= now - self.push_stall_time]
        if not old or old[-1][0] < since:
            return False                         # not a full window of pushing on record yet
        moved = math.hypot(self.leg[-1][1] - old[-1][1], self.leg[-1][2] - old[-1][2])
        if moved >= self.push_stall_min:
            return False
        self.get_logger().warn(f'push stalled: {moved * 100:.0f} cm in {self.push_stall_time:.1f} s '
                               'by the legs while stepping: something solid, giving it up')
        self.event(f'curtain: push stalled ({moved * 100:.0f} cm in {self.push_stall_time:.1f} s), given up')
        return True

    def stuck(self, mode, along):
        """A drive that cannot go on. Returns True (the action is over)."""
        if mode == 'ai':
            self.halt('STOPPED', 'no progress on the stairs in AI mode: take over by hand '
                                 '(lite3_ai_off), then /nav/start')
            return True
        if mode == 'recentre':
            self.recentre_failed = True       # plan() takes it from here
            return True
        if mode == 'creep' or self.step is None:
            return True
        a, d, b = self.step
        self.queue.clear()
        self.curtain_blind = False      # the crossing is over, whichever way
        self.push_sure = False
        self.leg_watch(False)
        cur = self.cur_cell()
        self.step = None
        if cur == b and along < 0.35:           # far enough in: count the cell as reached
            self.walked.update({(a, d), (b, opp(d))})
            self.arrive(b)
            return True
        self.forced_wall.update({(a, d), (b, opp(d))})
        self.walked.difference_update({(a, d), (b, opp(d))})    # blocked now: not "walked"
        self.cv_curtains.difference_update({(a, d), (b, opp(d))})
        self.curtain_latched.difference_update({(a, d), (b, opp(d))})
        self.cv_rejected.add((a, d))
        self.links.pop((a, d), None)
        ax, ay = self.grid.centre(a)
        back = math.atan2(ay - self.pose[1], ax - self.pose[0])
        if math.hypot(ax - self.pose[0], ay - self.pose[1]) > 0.10:
            self.queue.extend([('turn', back, 5.0), ('drive', ax, ay, back, 'creep')])
        return True

    def do_resolve(self, cell):
        """Standing in a new cell: wait until all four sides are known; face the ones that
        stay unknown (the lidar sees low walls best ahead); still unknown = wall."""
        self.send()
        if self.elapsed() < 1.2:               # one map update
            return False
        self.update_kinds()
        unknown = [d for d in range(4) if self.kinds.get((cell, d)) == 'unknown']
        if not unknown or cell in self.closed:
            return True
        if self.elapsed() < 2.5:
            return False
        for d in unknown:
            def settle(d=d):
                self.update_kinds()
                if self.kinds.get((cell, d)) == 'unknown':
                    if self.maze_rows > 0 and cell[0] == self.maze_rows - 1 and d == 0:
                        self.exit_open.add((cell, d))     # may be the way out: see exit_weak
                        return
                    self.get_logger().warn(f'cell {cell} side {d} still unknown: taken as a wall')
                    self.forced_wall.add((cell, d))
            self.queue.extendleft(reversed([('turn', self.grid.heading(d), 4.0), ('hold', 2.0),
                                            ('call', settle)]))
        return True

    # ---- display ------------------------------------------------------------------------
    def publish_cells(self):
        if self.grid is None:
            return
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        stamp = self.get_clock().now().to_msg()

        def marker(i, kind, rgb, scale):
            m = Marker(ns='explore', id=i, type=kind, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 1.0
            m.pose.orientation.w = 1.0
            return m
        cells = marker(1, Marker.CUBE_LIST, (0.2, 0.7, 0.9), 0.15)
        cells.points = [Point(x=self.grid.centre(c)[0], y=self.grid.centre(c)[1]) for c in self.visited]
        out.markers.append(cells)
        for i, (name, rgb) in enumerate(COLOURS.items()):
            m = marker(10 + i, Marker.LINE_LIST, rgb, 0.05)
            for (c, d), k in self.kinds.items():
                if k == name:
                    x, y = self.grid.box(c, d, [-0.4, 0.4], [0.0])
                    m.points += [Point(x=float(x[0, 0]), y=float(y[0, 0])), Point(x=float(x[1, 0]), y=float(y[1, 0]))]
            if m.points:
                out.markers.append(m)
        self.marker_pub.publish(out)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.moving = True
        try:
            node.send()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':      # stairs copy: run as a script, not an entry point
    main()
