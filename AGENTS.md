# challenge_ws: guide for agents and teammates

Read this before changing anything here. It records the current strategy for the maze run,
how the code is laid out, how to build and test it, and what the old stack taught us about
the robot and the arena.

Last updated 2026-11-23. Competition day is **27 November 2026**. Keep this file current
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

- Origin: centre of the start cell. Direction 0 = the start heading, then 1 = left,
  2 = back, 3 = right. This is the same frame the judges use, so a cell `(i, j)` reports as
  `pos = [i * 1.02, j * 1.02]`.
- After the first 0.10 m creep the grid is fitted to the mapped walls (up to 30° and
  0.40 m). That first map is often too thin (`run1`: 37 wall points, the grid stayed 6° and
  0.27 m off all run), so the wide fit is retried at every planning step until it works
  once. From then on it is nudged before every planning step (up to 3° and 0.08 m).
- The start cell's back side counts as a wall, and the start cell is closed once the robot
  has left it.

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
2. **greet**: only when nothing in 1 is left: a curtain beside the current cell, not faced
   yet: face it squarely, publish an event, first camera look (3 s). Until 2026-11-25 this
   came first, as soon as the robot stood beside a curtain.
3. **curtain / stairs**: only when nothing in 1 is left: the nearest one not crossed yet.
4. **exit**: nothing left at all: go to the cell that has the way out, face out, stop (`DONE`).
5. **done**: no exit known either: stop.

So curtains and stairs are crossed **last**, after everything reachable on this side has
been explored, and the robot ends the run standing in the final cell.

### 3.6 How each thing is handled (`explorer.py`)

- **Start**: stand still, centred in the start cell, facing into the maze. The explorer
  waits for `/map`, `map -> base_level` and `/maze/cloud` (`WAIT` → `READY`), then for
  `/nav/start`. It sends Move → gait → navigation mode, creeps 0.10 m, fits the grid.
- **Turning**: on the spot, about the cell centre, and held there: left alone the Lite3
  walks backwards while turning (run1: 9 cm per 90°, 0.3 m in a half turn). The turn steers
  back onto its pivot (forward and sideways only) and adds 0.03 m/s forward at full turn
  rate. Body outline: 0.27 m ahead of base_link, **0.42 m behind** (0.37 m body + 0.05 m
  for the bent hind legs), 0.23 m to each side; the rear corner sweeps 0.48 m, against
  0.485 m from a cell centre to its walls.
- **Walking**: turn to the heading, drive straight to the next centre. Top speed 0.20 m/s
  (team limit 0.25). Sideways and heading corrections on the way. **Never backwards**
  (`linear.x` is clamped at 0: nothing looks behind).
- **Rooms**: not entered by default (`enter_rooms:=false`). From the cell before, the robot
  looks centre, 25° left and 25° right, holds 1.5 s each and publishes
  `room centre|left|right cell (i, j) side d` on `/explore/event`.
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
  way to the centre of the cell behind, and there turn the quarter back on the spot. The crossing is remembered as a link so later routes can use it.
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
- **Stairs**: square up in the cell before, enter AI mode (manual → AI on → AI gait), walk
  two cells (over the stair cell to the centre of the cell after) at 0.15 m/s, leave AI mode
  (AI off → manual → Move → gait → navigation mode). The stair cell is closed afterwards.
  This matches the one-cell-before, one-cell-after rule.
- **Terrain (user, 2026-11-24; simulator only so far)**: a cell with uneven terrain is
  entered **sideways**. Uneven = 15 or more 5 cm squares of the cell's interior at 0.06 m or
  more above the floor, below object height (`Sense.rough`). In the cell before, the robot
  turns on the spot until its body is square to the way in, then steps sideways to the next
  centre at 0.10 m/s (`do_strafe`); the obstacle stop looks to that side. `terrain:=false`
  walks in head first. The H-Step gait (`gait:=hstep`, default again since 2026-11-24, user; `rug` = RUG
  general gear) is used for the whole run.
  **Depth camera for the floor (user, 2026-11-25):** the lidar's lowest ray meets the floor
  about 1.3 m ahead, so from the cell before, the next cell's floor is mostly unseen. When
  fewer than 40 of its 121 squares have been seen (and the lidar does not already say
  uneven), the standing depth check runs before walking in (`stair_check` action, stairs
  only looked for when that check was due anyway). It bins the depth points of the next
  cell into the same 5 cm squares, takes the median height per square, and calls it uneven
  on the same 0.06 m / 15 squares rule, by majority over the frames. Its verdict beats the
  lidar's; with too little seen the lidar's stands. Costs about 2-3 s standing and a short
  raw-depth subscription per new cell. `terrain_depth:=false` (node parameter) turns it off.
- **Exit**: found from the map (the far side of the last cell, section 3.4). The robot stops in the MIDDLE of that cell facing out (user, 2026-11-25): squares up on its centre line, creeps until the body (not base_link) is centred, base_link 0.075 m ahead of the cell centre, so both ends are 0.165 m inside. The red line on the floor
  (`maze_info.md`) is not used.

Safety behaviour:

| Condition | Reaction |
|---|---|
| `/odom` moves more than 0.15 m + 0.6 m/s × the time since the last message | pause 3 s, then carry on (after a stall in the lidar feed Fast-LIO catches up in one step: run3). Three such jumps in 10 s, or one over 2 m: `FAULT`, stop for good, relaunch needed |
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

Other launch arguments: `auto_start`, `gait` (`hstep` / `rug`), `enter_rooms`, `v_max`,
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

Unit tests: `nice -n 19 python3 -m pytest src/<package>/test -q`. `team_perception` and
`scene_understanding` have tests; `maze_explore` has `test/test_logic.py`.

## 5. Perception and reporting

| Package | Nodes | State |
|---|---|---|
| `team_perception` | `depth_relay`, `qr_scanner`, `human_detector`, `poster_detector` | Guides: `launch_guide.md`, `perception_guide.md`, `data_required.md`. `challenge_ws` is the master copy; do not re-sync from Robot-Dog-Grp4 |
| `scene_understanding` | `junction_trigger`, `room_scene` | **Connected to `maze_explore` (2026-11-24)**, started by `explore.launch.py`. `junction_trigger` takes the three views of each room / object look (compressed colour, subscribed only during the look), sends them as one 2 x 2 image with `LOOK_PROMPT` (scene type, hand tools, hazard, survivor; no QR) to the lab VLM (`192.168.10.141:8000`, images at most 1024 x 768, 2 s cooldown) and publishes `/scene/result`. `scene_understanding:=false` turns it off. `room_scene` is the old path, not started |
| `task_reporter` (own package, user 2026-11-24) | `task_reporter` | Takes `/scene/result` (scene understanding) and `/qr/read` (`qr_scanner`), maps them to the five missions (`missions.py`), checks the format (`missions.check`) and POSTs each report as `group04_grp4` to `http://192.168.10.141:8001/task_report_B` (course B since 2026-11-25, user; practice mode, live since 2026-11-24; the reply carries the per-field evaluation). Every report carries `pos`, M04 too (user). `pos` is always the centre of the cell the robot is LOOKING AT (user, 2026-11-25), taken from the explorer's look events and from its status while the job is a look, so both coordinates are whole multiples of 1.02; never where the robot stands. One answer per mission: M01 only for the object the mission asks for (`m01_object`, default `hand_tools`; a setting because the missions may still change, user 2026-11-24), M02 for a hazard sign (`m02_hazard`, default any); the first sighting holds it. Everything else seen is recorded on `/mission/sightings`, not submitted. Imports nothing from the other packages. `report:=false` turns it off |
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
- Bags: `runs/ROS_bags/ROSBAG_GUIDE.md`, aliases in `runs/ROS_bags/aliases.sh`. Hand-driven
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
