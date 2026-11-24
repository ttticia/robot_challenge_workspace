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
| `exit` | open, and no walls for 2 m beyond | never from the start cell |
| `object` | open, something taller than terrain stands inside the cell behind (toy set, prop) | height map: ≥ 8 squares of the interior (0.25 m clear of the borders) at ≥ 0.38 m, each seen 3 times. Checked before `room` |
| `room` | open, the cell behind is closed on its other three sides (dead end) | |
| `stairs` | open, the cell behind is blocked by something that rises like steps | height map: top 0.33–0.60 m, front 0.08–0.42 m |
| `blocked` | open, the cell behind is blocked by something else | |
| `free` | open, the cell behind can be walked into | |

### 3.5 Job order (`plan.py`)

1. **greet**: a curtain beside the current cell, not faced yet: face it squarely, publish an
   event, hold 2 s.
2. **visit / look**: the nearest unexplored thing: an open way into an unvisited cell, or a
   room or object cell not looked into yet (an object cell is never entered). Nearest = fewest cells to walk, then least turning.
3. **curtain / stairs**: only when nothing in 2 is left: the nearest one not crossed yet.
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
- **Curtain**: square up on the cell's centre line, then push through at 0.20 m/s with
  `/maze/blind` on. The crossing is remembered as a link so later routes can use it.
  **Off for now (user, 2026-11-23):** the obstacle stop stays on during the push
  (`push_stop`, default true), so the fabric stops the robot and the side becomes a wall.
- **Stairs**: square up in the cell before, enter AI mode (manual → AI on → AI gait), walk
  two cells (over the stair cell to the centre of the cell after) at 0.15 m/s, leave AI mode
  (AI off → manual → Move → gait → navigation mode). The stair cell is closed afterwards.
  This matches the one-cell-before, one-cell-after rule.
- **Terrain**: no special handling. The RUG general gait (`gait:=rug`, default since 2026-11-24; `hstep` = H-Step) is used for the
  whole run.
- **Exit**: found from the map (open edge, nothing beyond). The red line on the floor
  (`maze_info.md`) is not used.

Safety behaviour:

| Condition | Reaction |
|---|---|
| `/odom` moves more than 0.15 m + 0.6 m/s × the time since the last message | pause 3 s, then carry on (after a stall in the lidar feed Fast-LIO catches up in one step: run3). Three such jumps in 10 s, or one over 2 m: `FAULT`, stop for good, relaunch needed |
| Pose or lidar cloud older than 1 s | stand still until fresh (the action clock waits too) |
| Something knee-high or taller 6–20 cm ahead while walking, and while pushing (`push_stop`) | stop; after 8 ticks give up the step |
| No progress for 8 s (12 s in AI mode) | give up the step |
| A step given up | the side becomes a wall, robot creeps back to the cell centre. In AI mode: `STOPPED`, take over by hand (`lite3_ai_off`), then `/nav/start` |
| `/explore/stop` | `STOPPED`; `/nav/start` resumes |

### 3.7 Topics

| Topic | Type | Notes |
|---|---|---|
| `/nav/start`, `/explore/stop` | `std_msgs/Empty` | in |
| `/explore/status` | `String` | `STATE cell=… job=[…] act=… why`, 10 Hz. States: `WAIT`, `READY`, `RUN`, `DONE`, `STOPPED`, `FAULT` |
| `/explore/event` | `String` | `curtain …`, `room centre\|left\|right …`, `stairs: …`, `exit …`, each with cell and side. Meant for perception and logging |
| `/explore/cells` | `MarkerArray` | visited cells and the kind of each side, frame `map` (RViz) |
| `/maze/blind` | `Bool` | True while pushing through a curtain |
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
`maze_nav` paths.

Unit tests: `nice -n 19 python3 -m pytest src/<package>/test -q`. `team_perception` and
`scene_understanding` have tests; `maze_explore` has `test/test_logic.py`.

## 5. Perception and reporting

| Package | Nodes | State |
|---|---|---|
| `team_perception` | `depth_relay`, `qr_scanner`, `human_detector`, `poster_detector` | Guides: `launch_guide.md`, `perception_guide.md`, `data_required.md`. `challenge_ws` is the master copy; do not re-sync from Robot-Dog-Grp4 |
| `scene_understanding` | `room_scene`, `task_reporter` | The VLM is **not wired** (`vlm` = `none`): rooms are saved and skipped by `task_reporter`. `task_reporter` (2026-11-24) maps results to the five fixed missions (`missions.py`). For now it only publishes the QR report (M04) on `/mission/task_completed` (`missions:=all`, `post:=true` + `group_id` turn the rest on) |
| `datasets/curtain` | one-class curtain YOLO set | See its `README.md` |

**The gap to close.** These packages were written against the old explorer and still
listen to its topics, which nothing publishes now:

| They expect | From the old stack | New stack offers |
|---|---|---|
| `/maze/perceive`, `/maze/status` (`run_gate.py`) | when to run image work | `/explore/event`, `/explore/status` |
| `/maze/grid` (`poster_detector`) | cell grid for survivor positions | nothing yet |
| `/scene/capture` (`room_scene`, `task_reporter`) | room views to analyse | `room centre\|left\|right` events |
| `/qr/unread`, `/perception/look_request` | asks the explorer to face something | not subscribed |

Until this is bridged, a `maze_explore` run scores navigation points only.

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
  deletes anything older than 7 days there, except `cpu/` and `junctions/`.
- Bags: `runs/ROS_bags/ROSBAG_GUIDE.md`, aliases in `runs/ROS_bags/aliases.sh`. Hand-driven
  data run: `explore_check`, `explore.launch.py explore:=false`, then `rec_manual <name>`
  (guide 2.6). The recorder checks sensor rates before and after, compresses with zstd, and
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
- **One subscriber to raw depth (`/camera/depth/image_raw`), even BEST_EFFORT, slows every
  bridge stream to about 10 Hz** (measured 2026-11-23: `/imu/data` 195 → 11 Hz). The same
  goes for `ros2 topic hz` / `echo` on a big topic or RViz showing depth during a run.
  There is no compressed depth or compressed lidar topic; bag compression saves disk only.
- Fast-LIO needs `/livox/imu` near 200 Hz. At 8-20 Hz its pose runs away by kilometres
  within seconds. Run `explore_check` before every run.
- Fast-LIO with `blind: 0.5` lost walls closer than 0.5 m and its odometry jumped. Keep
  `blind: 0.3` (`config/fastlio_mid360.yaml`).
- Near lidar returns from the robot's own body show up as obstacles. Crop them.
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
- [ ] Decide whether the curtain YOLO model confirms a curtain before the robot pushes
      (today the height map alone decides).
- [ ] `/livox/imu` reaches this computer at 10-20 Hz instead of 200 (2026-11-23 afternoon,
      nothing subscribed). Robot or bridge side; no mapping or autonomous run until
      `explore_check` says GO.
- [ ] Depth cannot be recorded with the mapping stack running. A `compressedDepth`
      publisher on the robot side of the bridge would fix it (ask the lab).
