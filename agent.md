# challenge_ws: developer guide
If you notice an agent is editing another file, check in with the other session.

Add skills and edit skills.md as needed.

Read this before you change anything in this workspace. It covers the robot setup, the
packages, the ROS topics, and how motion commands reach the Lite3. It is written for
teammates and coding agents working from a fresh clone.

Queue simulation tasks across sessions to save tokens

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

All of perception runs on the robot computer, YOLO person detection (`human_detector`)
included. `human_detector` was pulled into `team_perception` from the
`team_perception_external` package (Robot-Dog-Grp4 repo, `challenge_files/external/`).

## 3. Packages in this repository

```
challenge_ws/
├── agent.md                  ← this file
├── src/
│   ├── maze_nav/             maze traversal: wall scan + SLAM + Nav2 + frontier explorer
│   └── team_perception/      perception: QR scanner, YOLO person detector, poster detector
├── maps/                     maps saved by maze_nav (created on first run, git-ignored)
├── ROS_bags/                 rosbag recordings + ROSBAG_GUIDE.md (bags git-ignored, never commit)
├── hardware_testing/, recordings/, runs/   logs from earlier hardware sessions
├── build/ install/ log/      colcon output          (should not be in git; see section 11)
```

### `maze_nav`: get through the maze

```bash
ros2 launch maze_nav maze_nav.launch.py            # robot standing on the start cell
ros2 launch maze_nav maze_nav.launch.py go:=false  # goals on /maze/goal only, robot doesn't move
# Optional parts, each on by default; switch off any combination:
ros2 launch maze_nav maze_nav.launch.py stairs:=false           # no stair_detector
ros2 launch maze_nav maze_nav.launch.py perception:=false       # no QR / YOLO / posters
ros2 launch maze_nav maze_nav.launch.py scene_trigger:=false    # no scene understanding
ros2 launch maze_nav maze_nav.launch.py stairs:=false perception:=false scene_trigger:=false
#                                                               # navigation only
```

A switched-off part starts no nodes (`depth_relay` runs only with stairs or perception on) and
is disconnected from the explorer (`stairs_topic` / `look_topic` / `qr_unread_topic` are blanked), so a stray
publisher can't stop the robot. Checked on 2026-11-18 by launching each combination. If one of
these parts dies mid-run, navigation carries on (the launch's death banner says so). **A new
optional feature must follow the same rule:** a launch switch, an explorer topic parameter that
the switch blanks, and a `DEATH_HINTS` entry.

The robot holds still (`/maze/status` = `PAUSED`) until
`ros2 topic pub --once /maze/start std_msgs/msg/Empty`. Start `rec_maze` before sending it.
Then it drives 10 cm straight ahead (`CREEPING`) and turns 45° left, 90° right, 45° left (`SCANNING`) before the first Nav2 goal.

Explores with `slam_toolbox` + Nav2 until there is open floor next to the robot, then stops.
`/maze/status` ends as `EXITED`, `NO_EXIT` or `STOPPED`, and the map is saved to `maps/`. It uses its own
`/wall_scan`, which keeps points 0.30–0.43 m above the floor (measured in `odom`). Maze walls are only
0.45 m tall, and the lab `/scan` starts at ~0.45 m. A simulator of the example maze is in
`src/maze_nav/test/fake_maze_lite3.py`. Details: `src/maze_nav/README.md`.

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

Everything runs on the robot computer (there is no external-computer mode any more; see
section 12 for the CPU budget). Inputs, requirements and what works with less data:
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

From `maze_nav`'s optional parts: `/stairs/status` (+ `/stairs/detection`, `/stairs/pose`)
from `stair_detector`, and `/scene/trigger` + `/junction_detected` from `scene_trigger`.

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
| `0x21010401` | RUG gait, General gear (gait state 2) | `maze_nav`'s default until 2026-11-20 (`gait:=rug`) |
| `0x21010407` | H-Step (high-stepping RUG gait, gait state 13) | **Default gait for `maze_nav`** since 2026-11-20 (user): `cmd_vel_bridge` sends it once at launch (`gait:=hstep`; `gait:=none` = don't). For rough ground; made the robot unsteady once (2026-11-12). See [DEEP Robotics docs](https://developer.deeprobotics.cn/docs/lite3/interfaces/control/state/) |
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
(cd src/maze_nav && python3 -m pytest -q test)          # wall scan, frontiers, exit, stairs, scene cells
```

`team_perception` has no ament lint tests (flake8/pep257/copyright).

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
  Work on a feature branch and open a pull request; don't push to `main`.
- The robot's repo has `origin` set to the team remote. Pushes are done by hand
  (no GitHub credentials are stored on the robot computer).
- `ttticia/Robot-Dog-Grp4` is for building away from the robot. `src/team_perception` is
  copied from its `challenge_files/onboard/team_perception`; fix perception there first,
  then copy it over.
- Never commit `build/`, `install/`, `log/`, bags in `ROS_bags/`, `maps/`, or other
  files over 100 MB.

## 12. Known issues and TODO

### Open items (2026-11-16)

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
- **The full stair pause is untested in `run_sim.sh`.** `fake_maze_lite3.py` publishes no
  depth image, so stairs are never detected there. The line-up and its wall checks have only
  been tested with a rendered-depth rig on domain 117 (section 8). Test on the robot, or add
  depth to the simulator.
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
