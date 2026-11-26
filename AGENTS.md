# challenge_ws: guide for agents and teammates

Read this before changing anything here. It records the current strategy for the maze run,
how the code is laid out, how to build and test it, and what the old stack taught us about
the robot and the arena.

Last updated 2026-11-26. Competition day is **27 November 2026**. Keep this file current
when the strategy or the interfaces change.

- Robot: DEEP Robotics **Lite3** quadruped, Livox **Mid-360** lidar (tilted 15°), Orbbec
  RGB-D camera. ROS 2 **Humble**, Ubuntu 22.04, Python packages (`ament_python`).
- Rules: `SUTD_Legged_Robot_Challenge_Booklet_2026_v6.md`. Arena facts: `maze_info.md` and
  `maze_examples/`. Robot interface (topics, QoS, `/simple_cmd` codes): `robot_info.md`.

## 1. Where things stand

On 2026-11-23 the exploration stack was restarted from scratch on branch `exploration-v2`.

| | |
|---|---|
| New stack | `src/maze_explore` (v2), on branch `exploration-v2`. |
| Old stack | `maze_nav` (slam_toolbox + Nav2 + frontier explorer). Removed here in `044e17c`; still on branch `navigation` (`4303e33`) with its docs (`agent.md`, `skills.md`, `src/bt.md`, `src/to_do.md`). Use it for reference only. |
| Kept | `src/team_perception`, `src/scene_understanding`, `datasets/curtain`, `runs/ROS_bags` tools, booklet, `robot_info.md`, `maze_info.md`, `maze_examples/`. |

State of the new stack, as found when this file was written:

- Per `src/maze_explore/README.md`: one dry run on the robot (no driving) and one simulated
  run. **It has not driven the real robot.** Treat curtain and stairs handling as untested.
- The only robot bag with this stack, `runs/ROS_bags/demo` (hand-driven, 2026-11-23), has
  good camera frames but a useless map and pose: the lidar IMU arrived at 8 Hz instead of
  200 and Fast-LIO ran away (section 7, "The bridge").
- Perception is **not connected** to the new explorer (section 5).

## 2. What scores (short form of the booklet)

| Task | Points |
|---|---|
| Each uneven-terrain cell crossed autonomously (4 cells) | 5 each |
| Stairs crossed autonomously | 10 |
| Movable obstacle (curtain) passed autonomously | 15 |
| M01 object (always `hand_tools`), M02 hazard sign (type varies), M03 survivor with QR, M05 survivor without QR (label / scene type 5, position 5) | 10 each |
| M04 QR message decoded | 5 |
| Stop autonomously inside the final cell | 10 |
| Drone landing | up to 50 |
| Autonomy bonus: reach the final cell with at most 1 / 2 / 3 manual interventions, and at least one valid mission report | +20 / +10 / +5 |
| Drone kept mounted for the whole ground run | +10 |
| Major collision | −10 |

Rules that shape the design:

- No pre-built map. The layout changes on the morning of the competition.
- **AI gait is allowed only for the stairs.** Enter it at most one cell before them and
  leave it at most one cell after. No points for any other cell crossed in AI mode.
- A manual intervention gives 0 points for that obstacle. A mission reached with manual
  help is capped at 8 points.
- Each restart is a new attempt with mission scores and reports reset; the best ground
  attempt counts. The drone launch may be started at any time by carrying the robot to the
  launch cell, but ends the ground mission (booklet update 2026-11-24).
- "Inside the final cell" means all four legs inside it, nothing touching the frame.
- Reported positions use the arena frame: origin at the centre of the start cell, +X along
  the start heading, +Y to the robot's left, cell centres 1.02 m apart. A position counts if
  it is within 0.5 m of the true cell centre on both axes. The latest valid report per
  `task_id` is the one scored.
- Reports go by HTTP POST to `http://192.168.10.141:8001/task_report` (booklet section E).

## 3. Current strategy: `maze_explore`

The idea: treat the maze as a **grid of 1.02 m cells** and move **cell centre to cell
centre**. There is no Nav2, no costmap and no path planner. The robot classifies the four
sides of each cell it stands in, picks the nearest unexplored thing, turns on the spot and
walks straight to the next centre.

### 3.1 Pipeline

```
Fast-LIO + odom_relay   /odom, TF odom -> base_link
        |  /cloud_registered_body
wall_scan               /scan (points 0.32-0.48 m above the floor), /maze/cloud (heights
        |               above the floor), TF odom -> base_level (no roll, no pitch)
slam_toolbox            /map, TF map -> odom
        |
maze_explorer           /cmd_vel, /simple_cmd, /explore/status, /explore/event, /explore/cells
```

| File (`src/maze_explore/maze_explore/`) | Job | ROS? |
|---|---|---|
| `wall_scan.py` | Lidar cloud → levelled, floor-referenced scan and cloud | yes |
| `explorer.py` | State machine, action queue, driving, mode switches | yes |
| `grid.py` | Cell grid, `fit_grid` (lines the grid up with the mapped walls) | no |
| `sense.py` | What is on each side of a cell, from `/map` and a height map | no |
| `plan.py` | Which job next (`next_job`) | no |
| `robot.py` | `/simple_cmd` codes and mode sequences | no |

Keep `grid.py`, `sense.py` and `plan.py` free of ROS so they can be unit-tested.

### 3.2 Why the scan is floor-referenced

Walls are 0.53 m tall and terrain reaches about 0.25 m. `wall_scan` keeps only points
0.32–0.48 m above the **floor**, in a gravity-levelled frame, so terrain does not look like
a wall and the body's pitch on terrain does not tilt the slice. The floor height is measured
once at the start with the robot standing still, then follows odometry z. While the robot
pushes through a curtain the explorer publishes `/maze/blind` = True and `wall_scan` stops
sending `/scan`, because the fabric lies over the lidar and SLAM would match against it.

### 3.3 The grid

- Confirmed by the user (2026-11-26): the start cell is always (0, 0); the robot always
  starts facing the start cell's only way out, along the LONG edge of the grid (the 5 rows);
  cells in front have positive x, cells to the right negative y.
- Origin: centre of the start cell. Direction 0 = the start heading, then 1 = left,
  2 = back, 3 = right. This is the same frame the judges use, so a cell `(i, j)` reports as
  `pos = [i * 1.02, j * 1.02]`.
- After the first 0.10 m creep the grid is fitted to the mapped walls (up to 30° and
  0.40 m). That first map is often too thin (`run1`: 37 wall points, the grid stayed 6° and
  0.27 m off all run), so the wide fit is retried at every planning step until it works
  once. From then on it is nudged before every planning step (up to 3° and 0.08 m).
- The start cell can be any of the three cells of the first row (user, 2026-11-26), but
  **its left, right and back sides are taken as walls, always** (user, later the same day):
  nothing is sensed, faced or latched there, so the start costs no turns and a tall wall
  beside it cannot read as a curtain (05:30 run: side 3 latched as a lidar curtain, side 1
  was faced for 7 s and still unknown). The only way out of it is ahead, and it is closed
  once the robot has left it. If a start side is really open, the cells behind it are
  reached another way or not at all.

### 3.4 What each side of a cell can be (`sense.py`)

| Kind | Meaning | How it is told |
|---|---|---|
| `wall` | wall-high obstacle on the edge | ≥ 40 % of the edge occupied in `/map` |
| `curtain` | as a wall, but it carries on above wall height | tall returns (≥ 0.65 m) along ≥ 50 % of the edge |
| `unknown` | edge not seen well enough | resolved by facing it; still unknown = wall |
| `exit` | open, no walls for 2 m beyond, and the cell is the last cell: a stub that sticks out of the maze, walled on both sides of the way out and not walled behind (user, 2026-11-25; open floor alone read as an exit in the 09:38 run) | never from the start cell. An exit that later stops reading as one is withdrawn |
| `object` | open, something taller than terrain stands inside the cell behind (toy set, prop) | height map: ≥ 8 squares of the interior (0.25 m clear of the borders) at ≥ 0.38 m, each seen 3 times. Checked before `room` |
| `room` | open, the cell behind is closed on its other three sides (dead end) | |
| `stairs` | open, the cell behind is blocked by something that rises like steps | height map: top 0.33–0.60 m, front 0.08–0.42 m; then confirmed by the depth-camera stair check (section 3.6) |
| `blocked` | open, the cell behind is blocked by something else | |
| `free` | open, the cell behind can be walked into | |

### 3.5 Job order (`plan.py`)

Order set by the user on 2026-11-25: explore everything reachable first, dead ends first;
curtains and stairs last. Cells are plain, uneven terrain, dead ends / rooms, curtain,
stairs, start and the last cell.

1. **visit / look**: the nearest unexplored thing: an open way into an unvisited cell (plain
   or uneven terrain), or a room or object cell not looked into yet (an object cell is never
   entered). Nearest = fewest cells to walk, then looks before visits (every room / prop
   beside a cell is looked at before the robot walks on; 11:41 run, two rooms were passed
   and the robot had to come back), then the way that looks most like a dead end (most walls
   already seen around the cell behind it), then least turning.
   Possible stairs beside the current cell are checked with the depth camera at once, before any other job (a check, not a crossing): confirmed = `stairs` (crossed last), not stairs = a prop = `object` (looked at, never entered). Before, such a cell ended as `blocked` and was never shown to scene understanding.
2. **greet**: only when nothing in 1 is left, and only for the curtain that 3 is about to
   cross, from the cell beside it: face it squarely, publish an event, first camera look
   (3 s). Until 2026-11-26 every curtain beside the cell was greeted before stairs too:
   in the 04:42 run the robot, with confirmed stairs ahead in the start cell, turned to two
   lidar curtains at its sides first and the odometry failed before the stairs' turn.
3. **stairs / curtain**: only when nothing in 1 is left: the nearest one not crossed yet; at
   the same distance stairs come before a curtain.
4. **exit**: nothing left at all: go to the cell that has the way out, do the end stop (section 3.6, `DONE`).
5. **no exit**: no exit known either (user, 2026-11-26): the navigation pauses and the robot
   lies down where it stands (`STOPPED`, not `DONE`; `lie_down:=false` = pause standing).
   `/nav/start` stands it up again and it plans afresh.

Junction rules (user, 2026-11-26). At one junction the order is room, path, stairs, curtain.
A job beside the current cell beats one that needs walking back through explored cells;
among those, the fewest cells re-crossed. A room / object cell that has been looked at is
explored (no second look from another side). An open edge into a cell the robot has
travelled through is never checked. A look is called off when, at its third view, the cell
no longer reads as a room / object (a curtain, stairs or a way on has shown up).
Between several ways on at one junction (same distance): a way already seen to be a dead end
first; then the one furthest from the far row, where the end cell is (lowest row along the
start heading; user, 2026-11-26), so the robot works towards the exit; then most walls
already seen; then least turning.

So curtains and stairs are crossed **last**, after everything reachable on this side has
been explored, and the robot ends the run standing in the final cell.

### 3.6 How each thing is handled (`explorer.py`)

- **Start**: stand still, centred in the start cell, facing into the maze. The explorer
  waits for `/map`, `map -> base_level` and `/maze/cloud` (`WAIT` → `READY`), then for
  `/nav/start`. It sends Move → gait → navigation mode, creeps 0.10 m, fits the grid.
- **Turning (user, 2026-11-26)**: on the spot, about the robot's own pivot where it stands
  when the turn begins, **not about the cell centre**. Measured on 17 recorded turns, the
  Lite3 turns about base_link itself (`pivot_offset` 0). Only the rotation is commanded,
  plus 0.02 m/s forward at full turn rate (`turn_forward`) against its backward creep; the
  pivot is steered back (forward and sideways only) once it has drifted more than 3 cm
  (`pivot_slack`). Until 2026-11-26 the turn steered base_link onto the cell centre when
  within 0.25 m of it: with a pose or grid that was off, the robot walked while it turned
  (03:45 run: 0.13 m/s forward, the turn hunting left and right). Getting back onto a cell
  centre is the recentre rule's job only (more than 0.20 m off, before a job). Each turn
  over 20° logs how far base_link and the pivot moved. Body outline: 0.27 m ahead of
  base_link, **0.42 m behind** (0.37 m body + 0.05 m for the bent hind legs), 0.23 m to
  each side; the rear corner sweeps 0.48 m, against 0.485 m from a cell centre to its
  walls, so a turn made off-centre has less room than that.
- **Walking**: turn to the heading, drive straight to the next centre. Top speed 0.20 m/s
  (team limit 0.25). Sideways and heading corrections on the way. **Never backwards**
  (`linear.x` is clamped at 0: nothing looks behind).
- **Rooms**: not entered by default (`enter_rooms:=false`). From the cell before, the robot
  looks centre, 30° left and 30° right (`look_angle`; 25° until 2026-11-26, user), holds 1.5 s each and publishes
  `room centre|left|right cell (i, j) side d` on `/explore/event`.
- **Empty rooms (user, 2026-11-26; unit-tested only)**: at the third view of a look
  (`do_look_last`) a dead end with nothing taller than an ordinary wall in it
  (`Sense.tall_inside`: under 4 squares at 0.65 m or more in the cell and on its walls, far
  wall's top measured) is empty: the view is held with the QR scanner on, the event is
  `empty room cell …` instead of `room right …`, so scene understanding sends nothing, and
  the cell counts as looked at. Rests on the user's word that every room with content has
  something taller than the walls. `object` cells always go to the VLM. `empty_skip:=false`
  (node parameter) turns it off.
- **Looks that are not made (user, 2026-11-26; unit-tested only)**: before every job is
  chosen (`skip_looks`), (1) a `room` the lidar already shows to be empty (same test, plus
  nothing stair-high inside) counts as looked at without any turn, view or QR scan
  (`empty_early:=false` = decide at the third view as above); (2) once the scoring server has
  marked a report for each of M01-M05 as successful, every remaining room / object look is
  dropped (`missions_stop:=false` = off). task_reporter passes each answer on as
  `/mission/result` (`{task_id, ok, reply}`); ok = `status: accepted` and, where the reply
  carries an `evaluation` (practice mode), every field true. The competition reply has no
  evaluation, so there accepted is all that can be checked. The latest answer per mission
  counts. Not ok for M01 / M02: that sighting lets go of the mission and the next one seen
  elsewhere is reported in its place (`Missions.refused`). With `report:=false`, or no
  answer from the server, (2) never applies.
- **QR approach, 2026-11-26 changes (user)**: no deeper than 0.3 m into the room
  (`qr_depth`, base_link past the border; 0.5 at first, 0.3 since the same day). Turns in
  there are made about the spot the robot stands on, not the room centre. Not read at the far end: after the hold the
  robot turns slowly (`qr_sweep_rate`, 0.25 rad/s) towards the room's back wall until the
  code is read or it faces that wall, then leaves sideways as before.
- **QR approach (user, 2026-11-25; `qr_approach:=false` = off; never run on the robot)**:
  from the cell before, a code on a side wall is seen about 60° off square and cannot be
  decoded (10:54 run, 11-24: 34 x 68 px). The VLM can see it, though: the look prompt also
  asks `qr` / `qr_view` (left, ahead, right), passed on in `/scene/result`. After the three
  views the robot stands up to 8 s (`qr_wait`) for that answer. With a code in the left or
  right view it turns 75° (`qr_angle`) towards that side at the cell centre, then moves
  into the looked-at cell with its body kept at that angle (almost sideways, camera
  sweeping that wall from close up) at 0.08 m/s, up to that cell's centre or until
  something is 6-20 cm in its way, holds 1.5 s, turns 15° more and steps back out purely
  sideways. A code read (`/qr/read`) at any moment ends the move and the robot goes back.
  Nothing becomes a wall because of it. The scanner stays on throughout (the job is still
  the look). A code in the `ahead` view gets no approach. Skipped once the code is read.
  Simulated only (`test/test_qr_approach.py`; rendered views decode from 0.65 m in).
- **Curtain**: square up on the cell's centre line, make a quarter turn on the spot, then
  step **sideways** through it (user, 2026-11-25) at 0.10 m/s with `/maze/blind` on, all the
  way to the centre of the cell behind, where it stands 1 s and goes on. **No turns after a
  curtain** (user, 2026-11-26; earlier that day it turned 180° right then 50° left to clear the
  fabric off the lidar: `curtain_turns:=true` brings that back). The crossing is remembered as a link so later routes can use it.
  **Latched (2026-11-25):** once an edge reads `curtain` it stays a curtain on both sides, whatever the map shows later (swaying or parted fabric must not turn it into an open way or a plain wall). A failed crossing, or the first grid fit (the cells move), undoes it.
  **Obstacle stop during the push:** off for a curtain the camera has confirmed (2026-11-25; with the stop on, the fabric itself stopped the step 3 cm in and the curtain became a wall: 10:42 run, cell (2, 2) side 0). For a curtain only the lidar has seen (`curtain_cv:=false`) `push_stop` still rules (default true = the fabric stops the robot, user 2026-11-23; `push_stop:=false` goes through). No progress for 8 s still ends any push.
  **Stall stop (user, 2026-11-25; unit-tested only):** the lidar is under the fabric during a push, so the guard is whether the robot moves. The explorer listens to the leg odometry (`/leg_odom2` pose, only during the push: it is a 200 Hz stream) and gives the push up (side = wall) when, after 2.2 s of stepping, the legs report under 3 cm in the last 1.5 s (`push_stall`, `push_stall_time`, `push_stall_grace`, `push_stall_min`). On the bags the leg pose follows Fast-LIO at about 0.85 of its distance while walking and moves 0-2 cm in 1.5 s standing; what it reads while the robot presses on something (feet slipping) has not been recorded. No leg data = the 8 s rule alone.
- **Curtain seen by the camera (2026-11-25, ON by default since the same day, user; `curtain_cv:=false` = off)**:
  `curtain_node` runs the v3 curtain YOLO model of branch `curtain_cv` (OpenVINO INT8
  320x256, `src/maze_explore/models/`, about 10 ms a frame, no ultralytics) on the compressed
  colour stream, 3 frames a second, only while the explorer runs and is NOT on a room /
  object look (the camera then belongs to scene understanding and the QR scanner). It has
  no depth: a box gives a direction. The explorer takes the first closed edge in that
  direction (`edge_along`); after 3 frames in 3 s it is a candidate. From the centre of the
  cell before it (or its own cell) the robot faces the edge for 3 s; a box across the middle
  of the image on 3 frames makes that edge a `curtain`, **also where the map and the height
  map call it a wall** (and the cell before it is then no longer a dead-end `room`). A push
  that fails takes the claim back. The model is not accepted yet (val: 20 of 23 curtains,
  false boxes on 2 of 51 frames without one).
  **Not over a wall (user, 2026-11-25).** The camera sees a curtain OVER a 0.53 m wall and
  has no depth, so such a sighting used to be pinned on that wall (09:42 run: cell (0, 0)
  side 1). An edge that is an ordinary wall (`Sense.low_wall`: wall in the map, its top
  measured along at least half of it, under 20 % of it curtain-high) collects no sightings,
  is not queued for a check, and camera hits there do not make it a curtain. An edge whose
  height is not known yet can still be claimed. Same code in the stairs copy
  (`src/stairs_crossing/patches/low_wall.patch`).
  **The camera confirms before every push (user, 2026-11-25).** With `curtain_cv` on, a
  lidar `curtain` (tall returns) is only a candidate: an aluminium-frame wall reads the same
  (bag `stairs10`, 23.4 s: 0.84 m high, no curtain box on 81 frames). The camera looks at it
  twice: during the greet (3 s instead of the 2 s hold; a miss there is not final) and,
  squared up in the cell before, just before the push. Not confirmed there, or no camera
  frames at all (`curtain_node` down), = the edge is a wall for good. `curtain_cv:=false`
  = the lidar decides alone, as before. Do not merge `curtain_cv`: it deletes the
  navigation code; bring files over one by one. Never run on the robot so far.
- **Stair check (2026-11-24)**: before walking into an unvisited cell where the lidar shows
  something 0.28 m or higher, and before any stairs crossing, the robot squares up, stands
  still and runs the teammates' depth-camera detector (package `stairs`: `src/stairs/stairs/stair_levels.py`, copied from
  `~/stairs_detect`: treads at 12 / 24 / 36 cm, plus the triangle handrails in the lidar)
  for up to 3 s. Its verdict overrides the lidar's: `stairs` only when it confirms. Depth is
  subscribed only during the check, because one raw depth subscriber slows every stream
  from the robot; odometry jumps in that window do not count towards `FAULT`. A cell that
  is `blocked` stays a stairs candidate until it has been looked at. `stair_check:=false`
  turns it off (lidar only).
  **Around the depth stream (user, 2026-11-26; unit-tested only):** `curtain_node` gives
  up its colour-camera subscription while the explorer's action is `stair_check`. After
  depth is closed the robot stands 1.5 s as before (a wait for the feed to be on time was
  added and taken out again the same day, user). The floor
  depth check is skipped when the lidar has seen 25 of the cell's 121 squares
  (`terrain_seen_min`, 40 before).
  **Floor first, stairs after (user, 2026-11-26; unit-tested only):** the first depth frames
  are read for the floor only. The stair detector runs only if that reading shows 6 or more
  squares at 0.20 m or higher in the cell (`stair_gate_min`, `stair_gate_squares`), or
  gives no usable reading in 6 frames (close up, stairs hide the floor). Three usable
  frames with nothing that high = not stairs, depth closes with the floor verdict (about
  1 s instead of up to 4.5 s). `stair_gate:=false` (node parameter) = both at once, as before.
- **Stairs**: square up in the cell before, enter AI mode (manual → AI on → AI gait), walk
  two cells (over the stair cell to the centre of the cell after) at 0.15 m/s, leave AI mode
  (AI off → manual → Move → gait → navigation mode). The stair cell is closed afterwards.
  This matches the one-cell-before, one-cell-after rule.
  **Centred before the gait change (user, 2026-11-26; unit-tested only):** the AI walk
  ends by distance just past the border of the cell after, about 0.16 m short of its
  centre. The robot now stands 3 s (`ai_settle`), the grid is re-anchored (this used to
  come after the gait change), and if the robot is more than 0.10 m (`ai_centre_tol`) off
  the centre it walks onto it, still in the AI gait: forward only at 0.20 m/s
  (`ai_centre_speed`), steering towards the centre (`do_ai_centre`). Not attempted when the
  pose is old, the centre is not ahead (within 70 deg) or is more than a cell away; a wall
  ahead or a time limit ends it. Only then AI off -> manual -> Move -> gait -> navigation.
  `ai_centre:=false` (node parameter) = off.
- **Terrain (user, 2026-11-24; simulator only so far)**: a cell with uneven terrain is
  entered **sideways**. Uneven = 15 or more 5 cm squares of the cell's interior at 0.06 m or
  more above the floor, below object height (`Sense.rough`). In the cell before, the robot
  turns on the spot until its body is square to the way in, then steps sideways to the next
  centre at 0.10 m/s (`do_strafe`); the obstacle stop looks to that side. `terrain:=false`
  walks in head first. The H-Step gait (`gait:=hstep`, default again since 2026-11-24, user; `rug` = RUG
  general gear) is used for the whole run.
  **Depth camera for the floor (user, 2026-11-25):** the lidar's lowest ray meets the floor
  about 1.3 m ahead, so from the cell before, the next cell's floor is mostly unseen. When
  fewer than 25 (40 until 2026-11-26) of its 121 squares have been seen (and the lidar does not already say
  uneven), the standing depth check runs before walking in (`stair_check` action, stairs
  only looked for when that check was due anyway). It bins the depth points of the next
  cell into the same 5 cm squares, takes the median height per square, and calls it uneven
  on the same 0.06 m / 15 squares rule, by majority over the frames. Its verdict beats the
  lidar's; with too little seen the lidar's stands. Costs about 2-3 s standing and a short
  raw-depth subscription per new cell. `terrain_depth:=false` (node parameter) turns it off.
  **Uneven confirmed = no more floor checks (user, 2026-11-26; unit-tested only):** after 3
  frames that say uneven (`terrain_confirm`, and they are the majority) the floor is not
  read again. With no stairs to look for, the check ends there and depth closes (about
  0.4 s instead of 8 frames). The cell is remembered (`rough_known`): no floor check for it
  from any other side either. A flat verdict still takes the 8 frames.
- **Exit / end cell (user, 2026-11-26; unit-tested and simulated only)**: the maze is 3 x 5 and the
  end cell is the only cell of the 6th row along the start heading (`maze_rows:=5`), open at
  its far edge towards a wide open area. An open way from row 5 into row 6 reads `exit`: it
  is not walked as a visit, only as the last job. Nothing beside or beyond the end cell is
  entered, and an `exit` the map shows anywhere else is a wall. `maze_rows:=0` = the old
  rule (exit from the map alone, section 3.4). End stop: walk into the end cell, creep on
  until the nose is 0.10 m (`end_gap`) before the open far edge (base_link 0.14 m ahead of
  the cell centre), turn 15° (`end_turn`) AWAY from `maze_side` (maze on the left = turn right; towards it until
  later on 2026-11-26), lie down (stand / sit toggle
  `0x21010202`, sent once; `lie_down:=false` = off), `DONE`. More than 0.20 m short of that
  spot = `STOPPED`, no lying down. The red line on the floor (`maze_info.md`) is not used.
  **Possible exits (user, 2026-11-26; unit-tested only):** the way into the end cell can
  read wrong (open floor beyond, little for the lidar to hit). On the far side of a last-row
  cell everything that is not a mapped wall (under 40 % of the edge occupied) is a possible
  exit: `unknown` after the robot has faced it (not made a wall there), `blocked`, `object`,
  possible `stairs` (no stair check, no look). A plain open way beats a possible one; of two
  possible ones the less walled is tried first. A possible exit is walked into with the
  obstacle stop on: stopped = that side is a wall and the next one is tried.
- **`maze_side`**: `explore.launch.py` asks "R or L" in the terminal at every launch
  (`maze_side:=R|L` answers it). Used for the end turn only.
- **Gait by floor (user, 2026-11-26; never run on the robot)**: before a step from a cell
  that is not uneven, into a cell that is not uneven either: `flat_gait` (default
  `flat_slow`, `0x21010300`; the flat gaits are untested on this robot); into an uneven
  cell: `gait` (H-Step). Standing in an uneven cell nothing changes. Each change is Move →
  gait → navigation mode, about 4.5 s standing. The run starts in `flat_gait`.
  `gait_switch:=false` = `gait` for the whole run, as before.
- **Speeds**: `src/maze_explore/config/speeds.yaml` sets normal walk (`v_max`), uneven
  terrain (`side_speed`), stairs (`ai_speed`), curtains (`curtain_speed`), QR sideways check
  (`qr_speed`), turning on the spot (`w_max`) and the gains that hold the turn on its pivot.
  Read at launch, no rebuild. `v_max` and `ai_speed` are no longer launch arguments. After
  each turn over 20° the explorer logs how far off the pivot it ended.

- **Turns on uneven terrain only at the cell centre (user, 2026-11-26; unit-tested only)**:
  off-centre the legs come down on the ends of the ramps. Standing in an uneven cell more
  than 0.10 m (`terrain_centre`) off its centre, the robot first shifts onto the centre
  WITHOUT turning (`do_shift`: forward and sideways steps at 0.08 m/s, heading kept), once
  per stay in the cell, and only then turns. Nothing drives backwards, so the part of the
  offset that lies behind the robot stays.
- **Start-cell guards (2026-11-26, after the 03:29 run; unit-tested only)**: in that run the
  first grid fit put the start centre 0.3-0.4 m behind the robot, the recentre rule walked
  it at the back wall three times and the third time it pushed into the wall. (1) While the
  robot is in the start cell a grid fit may move the start centre at most 0.12 m
  (`start_u_max`) along the start heading from where the robot was put down; beyond that
  the centre stays at the placed spot along the heading (sideways and angle still from the
  walls). So **place the robot centred fore and aft**. The back wall cannot be the
  reference: the lidar has no returns behind the robot there (bag
  `explore_20261125_100238`: 0 points behind, about 2000 on each side wall in 30 scans).
  (2) A walk back onto a cell centre that is blocked is not tried again from that cell,
  and in the start cell the wide grid fit is made once more. (3) The obstacle stop
  remembers what it saw for 60 s (`block_memory`): Fast-LIO drops returns within 0.30 m of
  the lidar, so a 0.53 m wall straight ahead is visible only while the nose is 14-20 cm
  from it. Not covered: a drive that STARTS with the nose already closer than 14 cm to a
  wall the stop has never seen.

- **Changes of the evening of 2026-11-26 (user; all unit-tested only, none run on the robot)**:
  - **Before a room / object look: stair check yes, terrain check no.** The depth stair
    check runs (the camera curtain check too), but no terrain verdict is made or recorded
    for that cell: it is only looked into, never walked into. The floor frames are still
    read as the quick gate for the stair detector. For a few hours there was no depth
    check before looks at all. `look_depth:=false` (node parameter) = none.
  - **False curtains:** with `curtain_cv` on, the lidar alone proposes no curtain. A tall
    edge (tall walls read exactly like curtains: 04:42 and 05:30 runs, beside the start
    cell and beside cell (1, 0)) is a wall until the curtain camera has pointed at it at
    least once (`camera_pointed`); then the usual checks follow. Cost: a real curtain the
    camera never fires on while the robot moves about is never looked at squarely.
    `cv_first:=false` = as before. With `curtain_cv:=false` the lidar decides alone.
  - **False stairs:** a depth-camera STAIRS verdict counts only if the lidar's height map
    shows something 0.38 m or higher in that cell (`stair_lidar_top`; stairs are 0.45 m,
    terrain under 0.30 m). 04:42 run: "stairs 0.70 m, levels 0.12 / 0.25 / 0.38" ahead of
    the start, in a cell later walked through; what the camera saw there is not known (no
    depth on record). `stair_agree:=false` = the camera alone, as before.
  - **Re-anchor after an odometry jump: OFF by default (`jump_fix:=true` = on).** First real
    use, 05:57 run: 2 s after a 0.32 m jump the pose read 0.38 m / 9.7° off, the grid was
    turned by that, the map did not turn with it (slam_toolbox kept matching scans to the
    walls it had), and the cells ended up 10° askew to every mapped wall; the wall fit
    could not bring them back. What it does when on: the robot stands from the first jump on, so where
    it believed itself just before is where it still is. Once Fast-LIO has settled (no jump
    for `odom_settle`, pose still for 1 s, reported tilt under 25°) and the map pose is more
    than 0.10 m / 4° off that, the cell grid is moved and turned by the difference, a few
    medium wall fits near the robot refine it, and what was known of the OTHER cells is
    kept as it was (`self.kept`; the map of them no longer lines up). The height map and
    the obstacle memory start afresh. Not done when Fast-LIO has not settled after 6 s
    (it has diverged) or the pose is more than 1.0 m / 30° off. Known weak point: walls
    mapped before the jump stay in `/map` beside the new ones near the robot.
    `jump_fix:=false` = off.

- **Against map drift (user, 2026-11-26; unit-tested only)**: (1) SLAM gets no scans while
  odometry is not to be trusted (`/maze/blind`, as under a curtain): during a burst of
  odometry jumps and 1 s after it, while the sensor feed is late, and for the whole AI walk
  over the stairs; the explorer's pose then comes from the held `map -> odom` and the
  newest odometry, and the height map takes no clouds. `map_blind:=false` = off. (2) The
  grid corrections before each planning step fit the walls within 2 m of the robot
  (`grid_fit_radius`; was 4 m), so a slightly bent map no longer lowers the fit until it is
  refused. The first, wide fit still uses 4 m.

Safety behaviour:

| Condition | Reaction |
|---|---|
| `/odom` moves more than 0.25 m (`odom_jump_min`; 0.15 until the 05:30 run) + 0.6 m/s × the time since the last message | stand until it has been steady for 1.5 s (`odom_settle`; was 3 s: user, "pausing for too long"), then carry on. Jumps that follow each other within 3 s (`odom_burst_gap`) are one burst and do not add up (user, 2026-11-26: allow some jumping; 05:03 run: /odom swung ±0.3 m sideways for 1.7 s on a late feed, 10 jumps, then held, but 3 jumps in 10 s was a `FAULT` then). `FAULT` (stop for good, relaunch) only when it does not come back: a burst still going after 6 s of standing (`odom_burst_max`), a pose more than 1.5 m from where the burst began (`odom_net_max`), or one jump over 2 m. Unit-tested on the 05:03 trace only |
| Sensor feed late: `/odom` arrives more than 0.30 s (`feed_late`; 0.15 until the 05:30 run, where 11 of 15 pauses began under 0.30 s) later than usual after its stamp (2026-11-26) | stand still until it has been on time for 0.5 s (`feed_settle`); at most 6 s in one go, then carry on for 10 s at 0.10 m/s at most (`feed_slow`; 05:03 run: it went on at 0.30 m/s and odometry swung 5 s later). Every odometry jump on record followed a late feed while the robot was moving (bags of 11-25 and 11-26: 12 late episodes moving, 2 jumps; 24 standing, none). `feed_pause:=false` (node parameter) = off |
| Pose or lidar cloud older than 1 s | stand still until fresh (the action clock waits too) |
| Something knee-high or taller 6–20 cm ahead while walking, and while pushing (`push_stop`) | stop; after 8 ticks give up the step |
| No progress for 8 s (12 s in AI mode) | give up the step |
| A step given up | the side becomes a wall, robot creeps back to the cell centre. In AI mode: `STOPPED`, take over by hand (`lite3_ai_off`), then `/nav/start` |
| `/explore/stop` | `STOPPED`; `/nav/start` resumes, and first sends Move → gait → navigation mode again (about 4.5 s standing), so a controller takeover or `lite3_ai_off` in between needs no `lite3_auto` (2026-11-25) |

### 3.7 Topics

| Topic | Type | Notes |
|---|---|---|
| `/nav/start`, `/explore/stop` | `std_msgs/Empty` | in |
| `/explore/status` | `String` | `STATE cell=… job=[…] act=… why`, 10 Hz. States: `WAIT`, `READY`, `RUN`, `DONE`, `STOPPED`, `FAULT` |
| `/explore/event` | `String` | `curtain …`, `room centre\|left\|right …`, `stairs: …`, `terrain: …`, `exit …`, each with cell and side. Meant for perception and logging |
| `/explore/cells` | `MarkerArray` | visited cells and the kind of each side, frame `map` (RViz) |
| `/maze/blind` | `Bool` | True while pushing through a curtain |
| `/curtain/seen`, `/curtain/status` | `String` | `curtain_node` (on by default; `curtain_cv:=false` = off): JSON per camera frame with the curtain boxes and their direction; `UNAVAILABLE: why` / `IDLE` / `RUNNING` |
| `/cmd_vel` | `Twist` | RELIABLE + TRANSIENT_LOCAL, 0.5 s lifespan. One zero, then silence, so teleop works again |
| `/simple_cmd` | `MotionSimpleCMD` | mode and gait requests, 1.5 s apart |

## 4. Build, run, test

```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash     # transfer_interfaces, livox_ros_driver2
source ~/ros2_ws/install/local_setup.bash      # lite3_launcher (odom_relay), fast_lio
cd ~/challenge_ws
colcon build --symlink-install                 # ALWAYS --symlink-install
source install/setup.bash
```

`~/.bashrc` on the robot computer already sources these and sets `ROS_DOMAIN_ID=4`. A plain
`colcon build` (without `--symlink-install`) replaces the links with copies and breaks every
other session's edits. `~/test_scene_understanding/install` uses `~/challenge_ws/install`
as an underlay, so do not delete `install/`.

Robot:

```bash
ros2 launch maze_explore explore.launch.py                 # full stack
ros2 launch maze_explore explore.launch.py explore:=false  # mapping only (hand-driven)
ros2 launch maze_explore explore.launch.py drive:=false    # decides, sends nothing
ros2 topic pub --once /nav/start std_msgs/msg/Empty    # go
ros2 topic pub --once /explore/stop  std_msgs/msg/Empty    # stop
```

Navigation only: `plain:=true` = no scene understanding trigger (`junction_trigger`), no
stairs (check and crossing; stairs are a blocked cell), curtains are walls, uneven cells are
walked into head first. Single switches: `scene_understanding`, `stairs`, `curtains`,
`terrain`, `qr_approach` (needs `scene_understanding` and `qr`). `plain` also turns `curtain_cv` (on by default) off. `task_reporter` keeps running (`report:=false` stops it).

`maze_side` (user, 2026-11-26): the launch asks "maze on the right (R) or on the left (L)?"
in the terminal before anything starts; `maze_side:=R|L` answers it. Not asked with
`explore:=false`. Without a terminal and without the argument the launch refuses to start.
It reaches the explorer as parameter `maze_side` (`self.maze_side`), for the end-cell
behaviour.

Other launch arguments: `auto_start`, `gait` (`hstep` / `rug`, for uneven terrain), `flat_gait`,
`gait_switch`, `maze_rows`, `enter_rooms`,
`stand_height`, `cloud_topic`, `fastlio`. Defaults are in `launch/explore.launch.py`.

Simulator (no robot needed, cannot reach the robot):

```bash
src/maze_explore/sim/run_sim.sh [maze=challenge_a.json] [timeout_s=900] [log_dir] [domain=89]
```

It runs `sim/fake_maze_lite3.py` and the launch on a spare ROS domain, localhost only, and
exits 0 when the explorer reports `DONE`. Mazes are in `maze_examples/sim/`
(`challenge_a*.json`, `challenge_b*.json`, `maze_curtains.json`, `maze_stairs.json`,
`curtain_maze.json`, `extru_maze.json`, …). Logs: `launch.log`, `sim.log`, `status.log`,
`events.log` in the log directory. The docstring in `fake_maze_lite3.py` still names
`maze_nav` paths. The simulator has no colour camera, so with `curtain_cv` on (the default)
no curtain is ever confirmed and every curtain becomes a wall: pass `curtain_cv:=false` to
test curtain crossings there.

**Sessions start simulators through the queue (2026-11-26):**
`src/maze_explore/sim/queue_sim.sh <tag> [maze] [timeout_s] [launch args...]`. It holds a
lock (`logs/sim/lock`), so runs follow each other with no gap and never overlap, uses
domain 97, writes its logs to `logs/sim/<date-time>-<tag>/` and one line per run to
`logs/sim/index.tsv`. A run of the same maze, arguments and code (HEAD + uncommitted `src`
changes) is not repeated: the script prints the earlier result (`FORCE=1` runs anyway).
Tell the other session what you are about to run, so one run covers both sets of edits.

Unit tests: `nice -n 19 python3 -m pytest src/<package>/test -q`. `team_perception` and
`scene_understanding` have tests; `maze_explore` has `test/test_logic.py`.

## 5. Perception and reporting

| Package | Nodes | State |
|---|---|---|
| `team_perception` | `depth_relay`, `qr_scanner`, `human_detector`, `poster_detector` | Guides: `launch_guide.md`, `perception_guide.md`, `data_required.md`. `challenge_ws` is the master copy; do not re-sync from Robot-Dog-Grp4 |
| `scene_understanding` | `junction_trigger`, `room_scene` | **Connected to `maze_explore` (2026-11-24)**, started by `explore.launch.py`. `junction_trigger` takes the three views of each room / object look (compressed colour, subscribed only during the look), sends them as one 2 x 2 image with `LOOK_PROMPT` (scene type, hand tools, hazard, survivor; no QR) to the lab VLM (`192.168.10.141:8000`, images at most 1024 x 768, 2 s cooldown) and publishes `/scene/result`. `scene_understanding:=false` turns it off. `room_scene` is the old path, not started |
| `task_reporter` (own package, user 2026-11-24) | `task_reporter` | Takes `/scene/result` (scene understanding) and `/qr/read` (`qr_scanner`), maps them to the five missions (`missions.py`), checks the format (`missions.check`) and POSTs each report as `group04_grp4` to `http://192.168.10.141:8001/task_report_A` or `..._B`: **the course follows `maze_side`, L = course A, R = course B** (user, 2026-11-26; `missions.course_url`; a `report_url` ending in `_A` / `_B` overrides it; with neither, nothing is posted and the node logs an error; practice mode, live since 2026-11-24; the reply carries the per-field evaluation). Every report carries `pos`, M04 too (user). `pos` is always the centre of the cell the robot is LOOKING AT (user, 2026-11-25), taken from the explorer's look events and from its status while the job is a look, so both coordinates are whole multiples of 1.02; never where the robot stands. One answer per mission: M01 only for the object the mission asks for (`m01_object`, default `hand_tools`; a setting because the missions may still change, user 2026-11-24), M02 for a hazard sign (`m02_hazard`, default any); the first sighting holds it. Everything else seen is recorded on `/mission/sightings`, not submitted. Imports nothing from the other packages. `report:=false` turns it off |
| QR during looks | `qr_scanner` (in `team_perception`) | `explore.launch.py` starts one (`qr:=false` = off) with `on_demand`: its camera is subscribed, and every frame fully scanned, only while `/explore/status` shows a look job, the same moment scene understanding takes its views. Do not run `robot.launch.py` as well (a second scanner). It ends with `maze_explore` (user, 2026-11-24): the launch interrupts it when the `maze_explorer` process exits, and it stops itself when the launch dies in any way (`EXIT_WITH_PARENT`, plus a parent check every 2 s). 2026-11-24 10:54 frame: the code on the survivor poster was about 35 px wide from the cell before and the decoder did not even locate it |
| `datasets/curtain` | one-class curtain YOLO set | See its `README.md` |

**The gap to close.** These packages were written against the old explorer and still
listen to its topics, which nothing publishes now:

| They expect | From the old stack | New stack offers |
|---|---|---|
| `/maze/perceive`, `/maze/status` (`run_gate.py`) | when to run image work | `/explore/event`, `/explore/status` |
| `/maze/grid` (`poster_detector`) | cell grid for survivor positions | nothing yet |
| `/scene/capture` (`room_scene`) | room views to analyse | `room centre\|left\|right` events (used by `junction_trigger` and `task_reporter`) |
| `/qr/unread`, `/perception/look_request` | asks the explorer to face something | not subscribed |

Scene understanding and reporting are bridged (table above). Still open: `poster_detector`
and the `/qr/unread` / look-request path.

Decisions from the user that still hold:

- All perception runs on the robot. No offboard computer.
- A survivor is a poster. Report the **cell centre**, not the poster's position. Track
  posters in `odom` and place them with the latest `map <- odom`; never track in `map`
  (it jumps on SLAM corrections).
- QR codes appear only in rooms with survivors, on the left or right wall. QR positions are
  not reported. **Booklet update 2026-11-24:** there are exactly two survivors and one QR
  code. The survivor WITH the QR code is M03 and its decoded text is M04 (reported exactly
  as decoded, e.g. "Please help me."); the survivor WITHOUT one is M05. The QR text is no
  longer a hint to another survivor's room.
  While no code has been read: a room where the VLM saw a survivor AND a QR code is M03 at
  once; failing that, the first survivor found is M03 and the second M05 (user,
  2026-11-25). A code read later puts M03 in its room and both go out again.
- Detect QR codes and survivors only at dead ends (rooms); plain QR reading may run always.
- The exit is a plain opening, never a curtain or stairs.

## 6. Working rules

**Several agent sessions share this workspace and this computer.**

- Another session may be editing the same package. Message it before you change a file it
  is working on. Never `pkill` by name; kill only your own PIDs.
- ROS domains: **4** = the real robot. **89** = the user's simulators. **117 / 121** = other
  sessions. Check that a domain is empty before using it, and say which one you use.
- The robot's own stack (Fast-LIO and the rest) runs on this computer. Run simulators and
  offline jobs with `nice -n 19` and `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`. Queue
  simulator runs instead of running several at once.
- Our nodes never talk to the hardware. `lite3_domain_bridge_4` republishes sensors into
  domain 4 and forwards `/cmd_vel` and `/simple_cmd`. Do not start the manufacturer
  `transfer` launch here.
- Git: the user pushes. Commit only when asked; hand over the push command and PR text.
  `robot_challenge_workspace` is the main repo; PRs go to `main`.
- Never commit bags (`*.db3`), `runs/`, `maps/`, `logs/`, `build/`, `install/`, `log/`.
- Logs go to `~/challenge_ws/logs` (`ros/` through the `~/.ros/log` link). A daily cron job
  deletes anything older than 7 days there, except `cpu/` and `junctions/`. Since
  2026-11-25 every `explore.launch.py` run has one folder, `logs/ros/<date>-<time>-…/`, with
  `launch.log` and each node's log named after the node (`maze_explorer.log`, …). Logs of
  nodes started with `ros2 run` stay at the top of `logs/ros/` as `<node>_<date-time>_<pid>.log`.
- Bags: `runs/ROS_bags/ROSBAG_GUIDE.md`, aliases in `runs/ROS_bags/aliases.sh`. `rec_test` and
  `rec_lean` are one recorder since 2026-11-26 (user): local topics only, no camera and no
  `/livox/imu` (add `-- /camera/color/image_raw/compressed` or `-- /livox/imu` for one
  run), about 55 MB a minute. With `/livox/imu` in the preset (one run, 04:42, bag `run1`)
  the feed was late 26 % of the run and the IMU arrived at 109 Hz: practice runs only. Hand-driven
  data run: `explore_check`, `explore.launch.py explore:=false`, then `rec_manual <name>`
  (guide 2.2). The recorder prints sensor rates after recording (no check before), compresses with zstd, and
  refuses depth.
- Python on the robot: apt `cv2` 4.5.4 only (no pip OpenCV), numpy pinned to 1.24.4,
  OpenVINO 2024.6 for YOLO.

## 7. Lessons from the old stack

These are facts about the robot and the arena. They cost real runs to learn; keep them in
mind when changing `maze_explore`.

Arena and body:

- Cells are 1.02 m. Walls are 0.53 m tall; the lab's own `/scan` passes over them. Some
  walls have 10–25 cm gaps. The aluminium-frame ("extru") walls have see-through side
  windows 10 cm wide.
- The Lite3 stands about **0.29 m** high (base_link above the floor), not 0.35. The
  simulator uses 0.35.
- Curtains hang from about 0.9 m, with a gap of about 15 cm at the bottom and a 1–3 cm
  split. Stairs: pyramid, 3 levels, 15 cm rise, 20 cm tread, reflective metal, railings.
- Terrain tiles are raised about 10 cm; rubble up to 15 cm.

Sensors and odometry:

- The bridge sends one full copy of a stream per subscribing process and saturates past
  about 4 copies. Use **one subscriber per heavy stream** and republish locally.
- **Nothing keeps raw depth open (2026-11-24).** `depth_relay` is on demand by default
  (`depth_on_demand:=true`): it subscribes only while the explorer holds a room look
  (`/explore/event` starting with `room` / `object`) or on `/scene/capture`, and closes 6 s
  after the last one. The stair check opens depth itself for at most 3 s (1.6 s when no
  frame looks like stairs). `~/stairs_detect/stair_watch.py` still keeps depth and
  `/livox/points` open: do not run it during a run.
- **One subscriber to raw depth (`/camera/depth/image_raw`), even BEST_EFFORT, slows every
  bridge stream to about 10 Hz** (measured 2026-11-23: `/imu/data` 195 → 11 Hz). The same
  goes for `ros2 topic hz` / `echo` on a big topic or RViz showing depth during a run.
  There is no compressed depth or compressed lidar topic; bag compression saves disk only.
- Fast-LIO needs `/livox/imu` near 200 Hz. At 8-20 Hz its pose runs away by kilometres
  within seconds. Run `explore_check` before every run.
- Fast-LIO with `blind: 0.5` lost walls closer than 0.5 m and its odometry jumped. Keep
  `blind: 0.3` (`config/fastlio_mid360.yaml`).
- Near lidar returns from the robot's own body show up as obstacles. Crop them.
- **No height-map updates while the robot turns (2026-11-25).** The explorer places each
  cloud with its last tick's pose, a little older than the cloud; at turning speed that
  swings the walls of the next cell into its middle. 10:02 run: the terrain cell ahead of
  the start (real heights 0.2-0.3 m) read as an `object` during the first turn and set off
  a look and scene understanding in the first cell. Clouds are skipped while /odom turns
  faster than 0.2 rad/s and for 0.3 s after (`heights_turn_max`, `heights_settle`).
  Replayed on bag `explore_20261125_100238`: 12 object squares before, 0 after.
- **The map pose stops while `/maze/blind` is on (2026-11-26).** No `/scan`, so slam_toolbox
  stops renewing `map -> odom` and the newest `map -> base_level` stays at the moment the
  crossing began. The explorer's pose froze and its stale-pose rule then held the robot for
  good, 0.18 m into the push (simulator, `challenge_a`, `push_stop:=false`). During a
  crossing the pose is now the held `map -> odom` with the newest `odom -> base_level`
  (`read_pose_blind`). Crossing completed in the simulator after that; not run on the robot.
- Fast-LIO z drifts slowly; do not trust odometry height over a long run without
  re-measuring the floor.
- **Never do a blocking TF lookup in a callback.** A 0.1 s blocking lookup once left a node
  5 s behind. Use `lookup_transform(..., Time())` and handle the exception.
- The camera has a second, unpitched TF parent in the robot's `tf_static` that is 20° wrong.
  Use `camera_link`.
- Fast DDS over shared memory stalled on this machine: use `config/fastdds_udp.xml`
  (UDP only, 4 MB buffers) in the simulator.

Motion:

- The Lite3 does not turn below about 0.1 rad/s. Command at least 0.15–0.2 rad/s
  (`min_spin`), or a slow final turn never finishes.
- AI state cannot be entered from navigation mode: go to manual first (`robot.py`).
- Pose-mode yaw: about 16° is the full range; the robot buzzed at the limit in a tight cell.
  Side looks are done by turning the body 25° instead.
- Lying down to look under a curtain was refused when odometry height had drifted.
- `/cmd_vel` and `/simple_cmd` need RELIABLE + TRANSIENT_LOCAL QoS to reach the bridge.

Measuring:

- This computer is a container on a shared host. The `_total_` CPU figure includes other
  tenants: add up the per-process rows instead.
- A simulator started during a robot run starved odometry once. Do not start one mid-run.

## 8. Open items

- [ ] Run `maze_explore` in the simulator on every maze in `maze_examples/sim/` and record
      the results.
- [ ] First robot run: mapping only (`explore:=false`), then `drive:=false`, then a full run.
- [ ] Unit tests for `grid.py`, `sense.py`, `plan.py`.
- [ ] Connect perception and scene understanding to `/explore/event` and the cell grid
      (section 5).
- [ ] Wire the VLM into `room_scene`. `task_reporter` follows the 2026-11-24 report format but
      has never posted to the real server: test it against `192.168.10.141:8001` with our
      `group_id`.
- [x] Decide whether the curtain YOLO model confirms a curtain before the robot pushes:
      yes, with `curtain_cv` on (user, 2026-11-25; section 3.6).
- [ ] `/livox/imu` reaches this computer at 10-20 Hz instead of 200 (2026-11-23 afternoon,
      nothing subscribed). Robot or bridge side; no mapping or autonomous run until
      `explore_check` says GO.
- [ ] Depth cannot be recorded with the mapping stack running. A `compressedDepth`
      publisher on the robot side of the bridge would fix it (ask the lab).
