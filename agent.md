agents allowed to use bash

# challenge_ws: developer guide

Read this before you change anything in this workspace. It covers the robot setup, the
packages, the ROS topics, and how motion commands reach the Lite3. It is written for
teammates and coding agents working from a fresh clone.

**Rules for coding agents (several sessions share this workspace):**
- If another agent is editing a file you need, check in with that session first.
- Step-by-step recipes for recurring jobs (simulator runs, tests, robot runs, bags, syncing
  repos) are in `skills.md`. Follow them, and add or fix a recipe there when you work one out.
- Queue simulator runs across sessions instead of running several at once: they share the
  CPU, cost tokens to watch, and each needs its own ROS domain (section 8).
- The current task list is `src/to_do.md`; section 12 below records known issues with context.

ROS 2 **Humble**, Ubuntu 22.04, Python packages (`ament_python`). Robot: DEEP Robotics
**Lite3** quadruped with a Livox **Mid-360** lidar and an Orbbec RGB-D camera.

---

## 1. Hardware and network

| Machine | Address | Role |
|---|---|---|
| Robot computer (this workspace) | `192.168.10.74` (team/lab network, `eth1`) | Runs our ROS nodes. Reach it by SSH or VS Code Remote. FileBrowser: `http://192.168.10.74:8080` |
| | `192.168.1.74` (robot internal network, `eth0`) | Talks to the motion host and sensors |
| Motion host | `192.168.1.120`, UDP `43893` | Locomotion controller (manufacturer firmware) |

- **ROS_DOMAIN_ID=4** is our group's domain (set in `~/.bashrc`). Every terminal and
  laptop that should see the robot needs it.
- **Our nodes never talk to the hardware directly.** A shared, already-running node,
  `lite3_domain_bridge_4`, republishes all sensor topics into domain 4 and forwards our
  `/cmd_vel` and `/simple_cmd` to the robot. Don't start the manufacturer `transfer`
  launch (`ros2 launch transfer transfer_launch.py`) on this computer while that bridge is
  running, unless the team agrees. It would send a second stream of commands to the motion host.
- The robot computer has no screen. GUI tools (RViz, rqt) run on a laptop on the same
  network with `ROS_DOMAIN_ID=4`.

## 2. Getting started from a fresh clone

```bash
# 1. ROS + system packages (once per machine)
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan ros-humble-twist-mux \
    ros-humble-cv-bridge ros-humble-tf2-geometry-msgs ros-humble-image-transport-plugins \
    python3-colcon-common-extensions python3-pyzbar python3-pip
bash src/team_perception/install_deps.sh --no-apt   # zxing-cpp from pip, checks imports

# 2. Underlays this workspace depends on (see section 4), in this order:
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash     # transfer_interfaces, livox_ros_driver2
source ~/ros2_ws/install/local_setup.bash      # lite3_launcher, fast_lio

# 3. Build and source this workspace
cd ~/challenge_ws
colcon build --symlink-install --packages-select maze_nav team_perception
source install/setup.bash
```

On the robot computer, `~/.bashrc` already sources all of these and sets
`ROS_DOMAIN_ID=4`. It also defines the `lite3_auto` and `lite3_manual` aliases (section 7).

All of perception runs on the robot computer. `human_detector` (YOLO person detection) was
pulled into `team_perception` from the old `team_perception_external` package; that package
no longer exists in either repo. `human_detector` and `poster_detector` are built but not
launched (section 12, CPU budget).

## 3. Packages in this repository

```
challenge_ws/
├── agent.md                  ← this file
├── skills.md                 step-by-step recipes for recurring jobs
├── robot_info.md             Lite3 interface reference (topics, QoS, /simple_cmd codes, UDP)
├── src/
│   ├── maze_nav/             maze traversal: wall scan + SLAM + Nav2 + frontier explorer,
│   │                         plus stairs, curtains, scene triggers (README.md, nav_launch.md)
│   ├── team_perception/      perception: QR scanner, depth relay (YOLO person + poster
│   │                         detectors are built but not launched)
│   └── to_do.md              current task list
├── ROS_bags/                 rosbag recordings + ROSBAG_GUIDE.md (bags git-ignored, never commit)
├── hardware_testing/         logs and results from earlier hardware sessions
├── maps/, recordings/, runs/ created by runs (git-ignored)
├── build/ install/ log/      colcon output          (should not be in git; see section 11)
```

The Robot-Dog-Grp4 copy (`challenge_files/`) also has `src/bt_diagram.md` (mermaid flowchart
of the intended mission behaviour) and `SUTD_Legged_Robot_Challenge_Booklet_2026_v5.md` (the
challenge rules). Neither is in the team repo (section 11).

**Maze grid:** the explorer assumes a `maze_cols` x `maze_rows` grid (default 5 x 3) of
`cell_size` m cells (1.02), with the start cell (0, 0) in a corner, centred on the robot's
start pose, and +i along its start heading. Every 5 s the grid is refitted to the mapped walls
(at most 8° / 0.35 m off the start-pose grid; `scene_cells.fit_grid`) and published on
`/maze/grid`, which `scene_trigger` follows. **The robot never leaves the grid:** frontier
goals outside it are dropped. Once every cell inside is explored, the exit is an outer side
open on the final map (and on >= `exit_confirm` (3) replans during the run). It is not
decided earlier because the start cell's sides read open until its walls are mapped. The robot
drives to the cell inside the exit (the drone's launch cell): Nav2 stops within 0.2 m, so it
creeps to the centre, then turns to face out, and stands still: `TO_EXIT` → `AT_EXIT`.
`/maze/launch_cell` (latched JSON: `launch_cell`, `exit_cell`, `centre`, `heading`) says
`found` when the exit is chosen and `at` when parked. The drone is launched by hand.
The exit is always a plain opening in the outer wall, never a curtain or stairs (user
2026-11-22), so curtain and stair crossings stay inside the maze.
`maze_cols:=0` = size unknown: the old behaviour (walk out, `EXITED` on open floor).

`maze_nav` nodes (all started by `maze_nav.launch.py`):

| Node | Job | Switch |
|---|---|---|
| `lidar_relay` | takes `/livox/points` across the bridge once → `/livox/points_local` + cropped `/livox/points_crop` | `lidar_relay`, `lidar_crop` |
| `wall_scan` | `/wall_scan`: wall band 0.30–0.45 m above the measured floor | always |
| `slam_toolbox` + Nav2 | `/map`, planning (Smac 2D), control (RPP), BT `config/maze_bt.xml` | always |
| `maze_explorer` (`explorer.py`) | frontier goals, junction and dead-end looks, exit detection, `/maze/status` | always |
| `cmd_vel_bridge` | QoS relay, speed clamp, sensor watchdog, stand / gait / Pose / AI-mode commands | always (`drive:=false` = preview only) |
| `stair_detector` | stairs from depth + lidar height map → line-up and AI-mode crossing | `stairs` |
| `curtain_detector` | curtains (wall with a low gap) → clear from the map and walk through; lie-down inspection | `curtains`, `curtain_inspect` |
| `scene_trigger` | `/scene/trigger` at dead ends, `/scene/junction` for junction looks | `scene_trigger` |
| `room_scene` + `task_reporter` | dead-end camera views → VLM queue; `/mission/task_completed` | `scene_understanding` (off) |
| `cpu_monitor` | per-process CPU log + summary (not a ROS node) | `cpu_log` |
| `depth_relay` (team_perception) | raw depth once → `/camera/depth_relay/*` at 10 Hz | with stairs or perception |

`odom_guard.py` (Fast-LIO odom with slips bridged by leg odometry, tested in
`test_odom_guard.py`) is **not wired in yet**: it isn't an entry point in `setup.py` and the
launch still uses `lite3_launcher`'s `odom_relay`.

### `maze_nav`: get through the maze

```bash
ros2 launch maze_nav maze_nav.launch.py            # robot standing on the start cell
ros2 launch maze_nav maze_nav.launch.py go:=false  # goals on /maze/goal only, robot doesn't move
# Optional parts, each on by default; switch off any combination:
ros2 launch maze_nav maze_nav.launch.py stairs:=false           # no stair_detector
ros2 launch maze_nav maze_nav.launch.py curtains:=false         # no curtain_detector
ros2 launch maze_nav maze_nav.launch.py perception:=false       # no QR scanner
ros2 launch maze_nav maze_nav.launch.py scene_trigger:=false    # no dead-end / junction triggers
ros2 launch maze_nav maze_nav.launch.py stairs:=false curtains:=false perception:=false \
    scene_trigger:=false                                        # navigation only
# Off by default, switch on:
ros2 launch maze_nav maze_nav.launch.py scene_understanding:=true  # room_scene + task_reporter (VLM not wired yet)
ros2 launch maze_nav maze_nav.launch.py curtain_lab:=true          # lab test curtain (gap 17-25 cm)
ros2 launch maze_nav maze_nav.launch.py start_scan:=true           # turn 30° L / 60° R / 30° L after the creep
```

The docstring at the top of `launch/maze_nav.launch.py` lists every argument with its
default; read it before adding one. Operator guide: `src/maze_nav/nav_launch.md`.

A switched-off part starts no nodes (`depth_relay` runs only with stairs or perception on) and
is disconnected from the explorer (`stairs_topic` / `look_topic` / `qr_unread_topic` are blanked), so a stray
publisher can't stop the robot. Checked on 2026-11-18 by launching each combination. If one of
these parts dies mid-run, navigation carries on (the launch's death banner says so). **A new
optional feature must follow the same rule:** a launch switch, an explorer topic parameter that
the switch blanks, and a `DEATH_HINTS` entry.

The robot holds still (`/maze/status` = `PAUSED`) until
`ros2 topic pub --once /maze/start std_msgs/msg/Empty`. Start `rec_maze` before sending it.
Then it drives `creep_distance` (default 0.10 m) straight ahead (`CREEPING`), optionally scans
(`SCANNING`, only with `start_scan:=true`), and sends the first Nav2 goal (`EXPLORING`).

Explores with `slam_toolbox` + Nav2 until there is open floor next to the robot, then stops.
On the way:
- **Junctions** (`JUNCTION`): it looks down each way and explores the left-most first.
  Possible stairs or a curtain seen at a junction are deferred until the junction's other
  ways are fully explored (`curtain_branch.py`).
- **Dead ends** (`DEAD_END`): it holds facing in, then looks at each side wall in Pose mode
  (tilt compensation over 30°). Perception runs only during these looks (`/maze/perceive`).
  A checked dead end is never entered. Object cells (e.g. a toy kitchen) are looked into from
  an open neighbour and kept out of Nav2 by a KeepoutFilter.
- **Stairs** (`STAIRS` → `AT_STAIRS`): anything stair-like is a candidate, checked by facing it
  and counting risers (`stair_check.py`). Once lined up, `cmd_vel_bridge` switches to AI mode
  and walks `ai_walk_distance` (1.2 m) forward, then stops.
- **Curtains** (`CURTAIN` → `AT_CURTAIN`): a wall with a gap at the bottom under
  `curtain_max_gap` (0.15 m). An unconfirmed one is inspected by lying down in front of it
  (`curtain_inspect.py`); a 12–20 cm gap confirms it, and the robot walks through.

The full list of explorer states is in `maze_nav/explorer.py`.
`/maze/status` ends as `AT_EXIT` (parked in the launch cell), `NO_EXIT`, `STOPPED`, or
`EXITED` (only if the robot got outside the grid), and the map is saved to `maps/`. It uses its own
`/wall_scan`, which keeps points 0.30–0.45 m above the floor (`band_low` / `band_high`, measured in `odom`). Maze walls are only
0.45 m tall, and the lab `/scan` starts at ~0.45 m. A simulator of the example maze is in
`src/maze_nav/test/fake_maze_lite3.py` (other mazes: `test/*.json`, section 9). Details: `src/maze_nav/README.md`.

### `team_perception`: perception (all on the robot)

```bash
ros2 launch team_perception robot.launch.py                 # qr_scanner only (human/poster detection removed 2026-11-19: bridge overload)
ros2 launch team_perception robot.launch.py --show-args     # every topic name is an argument
```

| Executable | Subscribes | Publishes |
|---|---|---|
| `depth_relay` | `/camera/depth/image_raw` + `/camera/depth/camera_info`, ONCE | `/camera/depth_relay/image_raw` + `camera_info`: the same frames at ≤ 10 Hz for every local depth user. The raw stream comes from `lite3_domain_bridge_4` (another VM), which sends a full 18 MB/s copy per subscribing process |
| `qr_scanner` | `/camera/color/image_raw/compressed`, `/camera/color/camera_info`, `/camera/depth_relay/image_raw` + `camera_info` (unaligned depth, mapped in software) | `/qr/detections` (String JSON: hint `text` + internal `room_odom`; QR locations are not reported), `/qr/read` (text, every read), `/qr/unread` (PointStamped: code seen but unreadable, for `maze_explorer`'s `FACING_QR`), `/found` (`QR found`) |
| `human_detector` | `/camera/color/image_raw/compressed` | `/humans/detections` (String JSON person boxes, YOLO at ≤ 5 Hz, 2 torch threads, ~0.5 core) |
| `poster_detector` | `/humans/detections`, `/qr/detections`, relay depth + camera_info, `/map`, TF | `/posters/detections` (String JSON: survivor `cell` + `cell_centre`), `/found` (`Human found`), `/perception/look_request` (PointStamped, to `maze_explorer`) |

Only `depth_relay` and `qr_scanner` are launched. In a `maze_nav` run they run niced on all
but the first 4 cores, and image work runs only while `/maze/perceive` is True (dead-end
looks, `FACING_QR`, `LOOKING`; `run_gate.py`). Everything runs on the robot computer (there
is no external-computer mode any more; see section 12 for the CPU budget). Inputs, requirements and what works with less data:
`src/team_perception/data_required.md`. Snapshots and CSV logs go to `~/qr_logs/` and
`~/poster_logs/`. Recording guide: `ROS_bags/ROSBAG_GUIDE.md`.

## 4. Dependencies that are NOT in this repository

| Package | Where it lives on the robot | Source |
|---|---|---|
| `lite3_launcher` | `~/ros2_ws/src/lite3_launcher` | **Not in any git repo yet.** Copy it from the robot if you need it on another machine |
| `fast_lio` | `~/ros2_ws/src/FAST_LIO` | github.com/hku-mars/FAST_LIO (branch `ROS2`) |
| `livox_ros_driver2` | `~/lite3_ws/src/livox_ros_driver2` | Livox SDK2 driver |
| `transfer_interfaces` (`MotionSimpleCMD`, `MotionComplexCMD`) | `~/lite3_ws/src/transfer_interfaces` | DEEP Robotics Lite3_ROS |
| `transfer` (manufacturer motion bridge, for reference) | `~/hardware_support/Lite3_ROS/src/transfer` | github.com/DeepRoboticsLab/Lite3_ROS (branch `ros2-foxy`) |
| Sensor drivers + `lite3_domain_bridge_4` | Already running (driver container) | Shared lab setup; don't modify |

## 5. Topics (domain 4, as seen live on the robot)

### Sensors (from `lite3_domain_bridge_4`)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | Uncompressed, large. Prefer `/compressed` |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 Hz | Use this one for perception and bags |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (16-bit, mm) | 30 Hz | Not aligned to color |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | | Very large; rebuild from depth if needed |
| `/image_raw`, `/camera_info` | `Image`, `CameraInfo` | | Second camera, uncompressed only |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 10 Hz | Fast-LIO input |
| `/livox/points` | `sensor_msgs/PointCloud2` | 10 Hz | Standard cloud for RViz and `/scan` |
| `/livox/imu` | `sensor_msgs/Imu` | ~190 Hz | Lidar IMU (Fast-LIO input) |
| `/imu/data` | `sensor_msgs/Imu` | ~190 Hz | Body IMU from the motion host |
| `/leg_odom` | `geometry_msgs/PoseWithCovarianceStamped` | ~190 Hz | Leg odometry, frame `odom` |
| `/leg_odom2` | `nav_msgs/Odometry` | ~190 Hz | Same pose plus body velocity |
| `/joint_states` | `sensor_msgs/JointState` | | |
| `/handle_state` | `geometry_msgs/Twist` | | Handheld controller sticks |
| `/commands/joy_raw` | `sensor_msgs/Joy` | | App/joystick passthrough |
| `/sensor_status/*_isalive` | `std_msgs/Bool` | | Driver health |
| `/tf_static` | | | `base_link → livox_frame` and camera frames |

### Localization (from `lite3_bringup` in `lite3_launcher`)

| Topic | Type | Publisher |
|---|---|---|
| `/odom`, TF `odom → base_link` | `nav_msgs/Odometry` | Fast-LIO + `odom_relay` (`lite3_bringup`) |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` (0.1–0.5 m above `base_link`, 0.4–20 m) |

### Perception

`/qr/detections`, `/posters/detections` (JSON in `std_msgs/String`), `/found` (plain
`QR found` / `Human found`), `/humans/detections` (person boxes from `human_detector`).
To `maze_explorer`: `/qr/unread` and `/qr/read` (turn to face an unreadable code,
`FACING_QR`), `/perception/look_request` (look around a QR room, `LOOKING`). The perception
nodes publish no debug images. Full list: `src/team_perception/launch_guide.md` §4.

### Navigation (`maze_nav`)

| Topic | From | Notes |
|---|---|---|
| `/livox/points_local`, `/livox/points_crop` | `lidar_relay` | Local copies of `/livox/points`; crop drops no-return and out-of-maze points |
| `/camera/depth_relay/image_raw`, `camera_info` | `depth_relay` | Raw depth at ≤ 10 Hz for every local depth user |
| `/wall_scan` | `wall_scan` | SLAM and costmap input |
| `/maze/status` | `maze_explorer` | Latched `STATE` or `STATE: why` |
| `/maze/start` | operator | `std_msgs/Empty`, leaves `PAUSED` (also releases `AT_STAIRS` / `AT_CURTAIN` holds) |
| `/maze/goal` | `maze_explorer` | Current frontier goal |
| `/maze/perceive` | `maze_explorer` | Latched Bool: perception may run |
| `/stairs/status`, `/stairs/detection`, `/stairs/pose` | `stair_detector` | |
| `/scene/trigger`, `/junction_detected`, `/scene/junction` | `scene_trigger` | |
| `/scene/capture` | `maze_explorer` | JSON per dead-end view (for `room_scene`) |
| `/scene/result`, `/mission/task_completed` | `room_scene`, `task_reporter` | Only with `scene_understanding:=true` |

### TF tree

```
odom ──(Fast-LIO/odom_relay)──> base_link ──(static)──> livox_frame
                                          └─(static)──> camera frames
```

`base_link → livox_frame`: translation (0.187, 0, 0.125) m, pitched about 15° down. `/tf` is
**empty unless Fast-LIO is running** (`lite3_bringup`). Leg odometry is published as topics only, not as TF.
Camera frames: `base_link → camera_link` (pitched 20° down) `→ camera_depth_frame →`
`camera_depth_optical_frame` / `camera_color_frame → camera_color_optical_frame`. Since
2026-11-18 `/tf_static` also has a wrong direct `base_link → camera_depth_optical_frame`
(section 12): don't trust a plain TF lookup of the depth camera.

## 6. Motion control: velocity

**Publish `geometry_msgs/Twist` on `/cmd_vel`.** `lite3_domain_bridge_4` forwards it to the
motion host.

| Field | Meaning | Robot command (UDP, inside the bridge) |
|---|---|---|
| `linear.x` | forward m/s (+ = forward) | complex cmd `320` |
| `linear.y` | sideways m/s (passed through unchanged; ROS standard is + = left, **not yet verified on the robot**) | complex cmd `325` |
| `angular.z` | yaw rad/s (+ = counter-clockwise, ROS standard) | complex cmd `321`, **sign flipped by the bridge** |

The UDP packet is `MotionComplexCMD {int32 cmd_code, int32 size=8, int32 type=1, float64 data}`,
sent to `192.168.1.120:43893`. This is from the manufacturer `transfer/src/Jetson2Motion.cpp`;
the group bridge does the same job. Always use ROS-standard signs; never negate `angular.z`
yourself.

**QoS rule (important):** publish `/cmd_vel` as **RELIABLE + TRANSIENT_LOCAL, depth 1**,
ideally with a short `lifespan` (~0.5 s). The group bridge has been seen asking for
TRANSIENT_LOCAL, and a default (VOLATILE) publisher never connects to a TRANSIENT_LOCAL
subscriber. TRANSIENT_LOCAL publishers work with every subscriber type, including the
manufacturer bridge's BEST_EFFORT one. Check what the bridge currently wants with
`ros2 topic info -v /cmd_vel`.
Nav2 and `twist_mux` publish VOLATILE, so put a QoS-converting relay in front of them.

```bash
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'     # manual test, robot must be in auto mode
```

**Safety rules for anything that publishes `/cmd_vel`:**
- **Only one publisher at a time.** Teleop, Nav2 and your node all writing `/cmd_vel` fight
  each other. Check with `ros2 topic info -v /cmd_vel` before driving.
- Clamp speeds. The values used so far are ≤ 0.25 m/s and ≤ 0.5 rad/s, and 0.1 m/s for first trials.
- On timeout, send one zero Twist and then stop publishing (don't hold the topic at zero, or teleop can't take over).
- Send a zero Twist on shutdown.
- Someone with the handheld controller stays next to the robot during any autonomous trial.

## 7. Motion control: modes and gaits (`/simple_cmd`)

`transfer_interfaces/msg/MotionSimpleCMD {int32 cmd_code, int32 size, int32 type}`; send
`size: 0, type: 0`. Same QoS rule as `/cmd_vel` (reliable + transient local, depth 1).

| Code | Meaning | How we use it |
|---|---|---|
| `0x21010C03` | **Auto mode**: the robot follows `/cmd_vel` | Alias `lite3_auto`. A human operator sends it. Our code should not |
| `0x21010C02` | **Manual mode**: the handheld controller drives, `/cmd_vel` is ignored | Alias `lite3_manual`. Safe fallback; send it on shutdown if you entered auto |
| `0x21010D06` | Move mode (needed before gait requests) | `maze_nav`'s `cmd_vel_bridge` sends it once at launch, before the gait |
| `0x21010401` | RUG gait, General gear (gait state 2) | **Default gait for `maze_nav`** (again since 2026-11-21): `cmd_vel_bridge` sends it once at launch (`gait:=rug`; `gait:=none` = don't) |
| `0x21010407` | H-Step (high-stepping RUG gait, gait state 13) | Optional: `gait:=hstep`. Was the default 2026-11-20 only. For rough ground; made the robot unsteady once (2026-11-12). See [DEEP Robotics docs](https://developer.deeprobotics.cn/docs/lite3/interfaces/control/state/) |
| `0x21020C0E` | Possibly a software STOP | Check it against the manufacturer docs before relying on it |
| `0x21010528` + `0x2101052A` / `0x2101052B` | Enter AI state + AI basic gait / exit AI state | Aliases `lite3_ai` / `lite3_ai_off` (the latter also sends manual mode). **In AI state the robot may ignore `/cmd_vel`** (manufacturer list; untested). Exception, the user's call 2026-11-19: once lined up at the stairs, `cmd_vel_bridge` switches to AI mode and drives `ai_walk_distance` (1.2 m) forward at 0.1 m/s on `/odom`, then stops; it gives up after 8 s without progress. `robot_info.md` §6.3 |

Usual sequence: robot standing (handheld controller) → gait (maze_nav: sent at launch, log
`gait sent`) → `lite3_auto` → drive with
`/cmd_vel` → `lite3_manual` when done. From AI mode, `lite3_ai_off` first (`lite3_auto` and
`lite3_manual` don't leave AI state). There is no gait or mode feedback topic through the
bridge, so watch the robot or the handheld controller to confirm a mode change.

## 8. DDS and launching

- Fast DDS with **UDP-only transport** (`src/maze_nav/config/fastdds_udp.xml`). Nav2's
  lifecycle startup stalled on this robot with the default shared-memory transport. The
  `maze_nav` launch sets it for its own nodes (`udp_only:=true`). For manual runs:
  `export FASTRTPS_DEFAULT_PROFILES_FILE=~/challenge_ws/src/maze_nav/config/fastdds_udp.xml`.
- **Before you launch, check that nothing else owns SLAM, Nav2 or `/cmd_vel`:** run
  `ros2 node list`. Two `slam_toolbox` or Nav2 stacks on domain 4 corrupt `/map` and TF and
  can both drive the robot. Stop the old one first.
- Use another domain for any test that must not reach the robot. `89` is the simulator's
  (`src/maze_nav/test/run_sim.sh`), so pick a different one for one-off test rigs
  (e.g. `export ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1`) or you will mix into a running sim.
  Other Claude sessions may be using one too: check it's empty (`ros2 node list`) first, and
  say which you use (section 12, Testing and workflow).
- `maze_nav.launch.py` puts real runs (`bringup:=true`, the default) on domain 4 with
  `ROS_LOCALHOST_ONLY=0` even if the terminal is on another domain (`domain_id:=auto`). The
  simulator (`bringup:=false`) keeps the terminal's domain. `domain_id:=N` forces one.

## 9. Testing

```bash
cd ~/challenge_ws && source install/setup.bash
(cd src/team_perception && python3 -m pytest -q test)   # QR utils, depth alignment, poster tests, tf_pin
(cd src/maze_nav && python3 -m pytest -q test)          # wall scan, frontiers, exit, stairs, curtains, scene cells, looks
src/maze_nav/test/run_sim.sh [timeout_s=600] [log_dir=/tmp/maze_sim] [maze=example_maze.json] [domain=89]
```

`team_perception` has no ament lint tests (flake8/pep257/copyright).

`run_sim.sh` uses localhost only (domain 89 unless given), starts `fake_maze_lite3.py` with
the maze and the launch with `bringup:=false perception:=false wait_for_start:=false
floor_z:=-0.35` and waits for the end state. A maze whose `grid` names a `launch_cell` passes
only if the robot parked there (`AT_EXIT`) and the sim never logged `OUT OF THE MAZE`; other
mazes pass on `AT_EXIT` or `EXITED`. The maze file's `grid` key sets `maze_cols` /
`maze_rows` / `cell_size`; without one, `maze_cols:=0` (the old walk-out exit).

Simulator mazes (`src/maze_nav/test/`; regenerate from their scripts, never edit the JSON):

| Maze | Grid | What it tests |
|---|---|---|
| `challenge_exit_{far,left,right,near}.json` | 5 x 3, 1.02 m | The challenge shape (start in a corner facing the 5-cell side), exit through each outer side; `near` is next to the start. `make_challenge_mazes.py` |
| `challenge_offset.json` | 5 x 3, 1.02 m | `exit_far` with a hand-placed start (5 cm, 5°) |
| `example_maze.json`, `object_maze.json` | none (old exit) | The lab example maze (stairs, ramps, mats); with a blocking object. Its exit is the stairs, not a challenge exit |
| `maze_curtains.json`, `maze_stairs.json` | 2 x 3, 1.0 m | The presentation mazes. `make_presentation_mazes.py` |
| `curtain_maze.json`, `extru_maze.json` | none | A corridor with a curtain; built from extru walls |

The simulator publishes lidar, odometry and a depth camera (`/camera/depth/image_raw`, 640 x 480
at 5 Hz, the robot's camera mount and intrinsics; `depth_rate:=0` turns it off), so stairs and
curtains can be seen by depth too. It has no colour camera (no QR codes), no body pitch on
slopes, and no `/simple_cmd` (Pose, lie-down and AI-mode commands do nothing).

Offline tools in `src/maze_nav/tools/`: `curtain_bag_replay.py` (run a bag through the curtain
detector), `curtain_yolo_frames.py` (bag → draft YOLO curtain dataset), `step_to_boxes.py`
(example maze STEP → `example_maze.json`).

## 10. Recording data

Use the `rec_*` aliases from `ROS_bags/aliases.sh` (loaded by `~/.bashrc` on the robot). Each
runs a preset in `ROS_bags/record.sh`. Full guide: `ROS_bags/ROSBAG_GUIDE.md`.

```bash
rec_full <run_name>      # teleop, everything incl. depth (Ctrl+C to stop)
rec_nav                  # autonomous run, no depth; name defaults to nav_<date>_<time>
rec_maze                 # full maze_nav run (nav + perception + explorer topics), no depth
rec_list                 # all presets: perception, detections, navdebug, lidar, video
rec_last                 # ros2 bag info for the newest bag
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/<run_name>   # bag -> mp4, see ROSBAG_GUIDE §4
```

`rec_full` uses about 1.6 GB per minute; the robot disk had about 11 GB free (check `df -h`). `/tf`
is only recorded when Fast-LIO is running. Bags are git-ignored. Copy them off the robot.

**Not while Nav2 drives:** recording `/camera/depth/image_raw` delayed `/livox/points` by
~1.2 s and collision_monitor held the robot at zero (2026-11-13). Use `rec_nav` for autonomous
runs; `rec_full` and `rec_perception` refuse to start while Nav2 is running.

## 11. Git workflow

- Team remote: `https://github.com/ttticia/robot_challenge_workspace.git` (`main`).
  Work on a feature branch and open a pull request; don't push to `main`. Current
  navigation work is on the `navigation` branch.
- The robot's repo has `origin` set to the team remote. Pushes are done by hand
  (no GitHub credentials are stored on the robot computer).
- `ttticia/Robot-Dog-Grp4` is for working away from the robot. Its `challenge_files/` is a
  mirror of the team repo's `navigation` branch, with the same layout, plus
  `src/bt_diagram.md` and the challenge booklet. Change code in the team repo, then refresh
  the mirror (recipe in `skills.md`); don't hand-edit the mirrored files, or the next refresh
  overwrites them.
- Never commit `build/`, `install/`, `log/`, bags in `ROS_bags/`, `maps/`, or other
  files over 100 MB.

## 12. Known issues and TODO

### Open items (2026-11-16; newer items and plans are in `src/to_do.md`)

Summary of `src/to_do.md` (2026-11-21): stair detection CPU and missed stairs; curtain false
hits on wall cavities; turning / costmap tuning; Fast-LIO map delay and walls not sensed;
Pose-mode looks untested on the robot; VLM backend for `room_scene` not chosen, and nothing
detects survivors since `poster_detector` was dropped; 15 cm obstacles and a terrain-adaptive
gait (low priority).

Navigation (`maze_nav`, section 3; test with `run_sim.sh`, section 9):
- **The simulator run ends `NO_EXIT` after the robot is out.** Twice on 2026-11-16, on separate
  domains, the robot got pinned with its side on the end of the stair handrail just past the
  stairs (maze (-3.6, -0.23)). Each run logged 96 "Starting point in lethal space", 12 failed
  goals, then `NO_EXIT`, although the simulator logged `OUT OF THE MAZE`. Earlier runs that
  day exited about 1 m further on. Suspect Nav2's recovery spins (`config/maze_bt.xml`)
  grinding against the rail. Fix before a robot run: the real maze has the same rails.
- **Turns on the spot don't fit a 0.9 m corridor.** The rear corners sweep a 0.437 m circle;
  a corridor leaves 0.425 m to each wall. `maze_explorer` creeps ahead to room before such a
  turn (`pre_turn`, `pre_turn_max_creep`), but where the frontier goal leaves no room ahead
  (the ramp corner: 0.32 m) the turn is left to Nav2 and the robot scrapes. Idea: put
  frontier goals at the spot with the most room (the cell centre), so the robot always stops
  where it can turn.
- **Costmap inflation was raised on 2026-11-16** (`nav2_params.yaml`: local 0.5 / 6.0,
  global 0.7 / 3.5) after one simulator run. Confirm it over several runs on an idle
  machine (CPU budget below).

Stairs (`stair_detector`, section 3):
- **The full stair pause has not been run in the simulator yet.** `fake_maze_lite3.py` got a
  depth camera on 2026-11-22, so `run_sim.sh ... maze_stairs.json` can now detect and line up
  on stairs. The AI-mode crossing can't be simulated (no `/simple_cmd`).
- **Tune the line-up wall checks after robot trials.** They only know walls on the SLAM map.
  The cell before the stairs leaves the turning circle about 1 cm, so an off-centre or
  turned-away arrival holds as `AT_STAIRS: NOT LINED UP` for a hand line-up. Settings:
  `Approach` `room_max` (0.3 m) and `square_loose_tol` (8°), `stair_detector`
  `turn_margin` / `min_margin`.

Scene understanding (`scene_trigger` in `maze_nav`, consumer in `~/test_scene_understanding`):
- **The consumer drops most triggers.** New cells come every 4–7 s, but `junction_trigger`
  skips any trigger while a VLM request runs (timeout 75 s). It also reads only the
  `/junction_detected` Bool, so results can't be tied to a cell. Fix: queue triggers, read
  `/scene/trigger` and copy its `seq` and `cell` into the result.
- **Check the VLM group id.** A 11:05 test got `HTTP 422 … invalid group` for `group04`;
  `vlm_group` now defaults to `group04_grp4`.
- **`openings` in `/scene/trigger` is a hint only.** A beam aimed at wall height on a far
  wall can pass just over a nearby 0.45 m wall and clear it on the map (section 3 on the
  wall band); the start cell's sides read as open. Ideas: cap the laser range at ~3 m in
  `slam_toolbox.yaml` and `wall_scan`, or keep a wall layer that never clears.
- **The cell grid is set by where the robot was when Fast-LIO started** (section 5,
  Localization). Place it square on the start cell and launch it standing (`auto_stand`
  moves it). Ideas: take the grid from the pose at `/maze/start`, fit it to the mapped
  walls, and ignore SLAM jumps of more than a cell.

Testing and workflow (sections 8, 9):
- **Several Claude sessions work in this workspace at once.** Each needs its own ROS
  domain for simulator runs and test setups, and must kill only its own PIDs:
  `pkill -f stair_detector` from one session killed another's simulator run, and
  `run_sim.sh` aborts the whole run when any node dies. Check a domain is empty
  (`ros2 node list`) before using it.
- **The `rec_*` presets miss the newer topics** (section 10): `/stairs/*`, `/scene/trigger`,
  `/junction_detected`, `/qr/read`, `/qr/unread` and `/perception/look_request` aren't in any
  group in `ROS_bags/record.sh`, and group G still lists the dead
  `/camera/color/image_offboard/compressed`. Add them after `--` until `record.sh` is updated
  (`ROS_bags/ROSBAG_GUIDE.md` §1).
- **This machine's `install/` is a copy, not symlinks** (built without
  `--symlink-install`, section 2). Rebuild `maze_nav` after changing its config, launch
  files or code, or runs use the old version.

### CPU budget (read before adding a node or raising a rate)

The robot computer is an Intel i5-13420H, **10 threads, no usable GPU**: SLAM, Nav2 and
perception all share the CPU. Navigation (Fast-LIO, `slam_toolbox`, Nav2's controller)
misses its deadlines when the CPU is short, and YOLO is the largest single load.

- **Not yet measured on a real run with everything on.** `maze_nav` starts `cpu_monitor`
  (`cpu_log:=true`, default): CPU per process every 2 s in
  `~/challenge_ws/logs/cpu/run_<time>.csv`, a mean/p95/max summary on Ctrl+C
  (`run_<time>_summary.txt`), and a WARNING when the computer stays above 85 % of all cores.
  100 % = one core, 1000 % = all 10. Read the summary after every real run.
- **Measured so far** (2026-11-16): the simulator with the nav stack only (no Fast-LIO, no
  perception) averaged 130–170 %, peaks 290–340 %; `wall_scan` ~11 %, `maze_explorer` ~8 %,
  `cmd_vel_bridge` ~6 %, each Nav2 server 2–5 %. YOLO (`yolo11n`, 640 px, 2 torch threads)
  costs ~100 ms of CPU per frame: ~50 % of a core at the default 5 Hz.
- **Big consumers to watch on the robot:** `fastlio_mapping`, `human_detector` (YOLO), and
  raw depth (640x480 16-bit at 30 Hz, ~18 MB/s) deserialised in Python by BOTH
  `qr_scanner` and `poster_detector`.
- **human_detector / poster_detector are no longer launched** (2026-11-19). With them on, the
  robot bridge fell behind (IMU 23 Hz instead of 200, lidar 1.1 s late, no map). Running them
  again means one colour subscriber from the bridge (a relay like lidar_relay / depth_relay).
  If run by hand: `-p rate_hz:=3.0`, and never raise threads above 2 (ultralytics would take
  ~8 threads of busy-waiting OpenMP).
- **Traps that already cost CPU or starved nodes here:**
  - OpenBLAS busy-wait threads (`wall_scan` hit ~190 %): `OPENBLAS_NUM_THREADS=1` is set in
    `maze_nav/__init__.py`. Set it in any new numpy-heavy process.
  - ultralytics resets torch to `min(8, cpus - 1)` threads on its first predict: `human_utils.py`
    re-caps it after warm-up. Keep that if you touch YOLO.
  - `msg.data = bytes(...)` for big messages in rclpy is slow (65 ms per cloud): use
    `array('B', ...)`.
  - A blocking TF lookup (`timeout=`) inside an executor callback stops that node from
    taking `/tf`, and the buffer falls seconds behind (poster_detector locked 3.5-5 s
    behind): look up without a timeout in callbacks (`qr_utils.transform_first`), wait only
    in worker threads.
  - Don't record raw depth (`/camera/depth/image_raw`) during autonomous runs: it stalled Nav2.

- **The robot's `/tf_static` gives the depth camera two parents** (seen 2026-11-18, robot side,
  via `lite3_domain_bridge_4`): the correct `camera_depth_frame -> camera_depth_optical_frame`
  (under `base_link -> camera_link`, pitched 20° down) and a wrong, unpitched
  `base_link -> camera_depth_optical_frame` at (0.25, 0, 0.05). TF keeps one, and live it's the
  wrong one: camera points land 20° off and colour/depth alignment is off by 20°. Workarounds:
  `qr_scanner`, `poster_detector` and `human_detector` keep the correct parent in their TF
  buffers (`team_perception/tf_pin.py`, parameter `tf_pin`; it warns once when it sees the
  duplicate). `stair_detector` builds the camera pose from `camera_link`
  (`camera_link_frame`). Any new node that uses camera TF needs one of these. The real fix is
  removing the extra transform on the robot side.
- The robot's depth image is not aligned to colour. `qr_scanner` and `poster_detector` map
  colour pixels into it in software (`DepthAlign` in `qr_utils.py`), a temporary fix. If
  the camera driver's depth registration is turned on, set `depth_topic` to the aligned
  topic and `depth_camera_info_topic:=''`.
- `lite3_launcher/config/pointcloud_to_scan.yaml`: `min_height` is still marked TODO. If the
  floor shows up as obstacles in `/scan` or the map, tune it.
- `lite3_launcher` isn't version-controlled (section 4).
