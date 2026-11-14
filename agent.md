# challenge_ws: developer guide

Read this before you change anything in this workspace. It covers the robot setup, the
packages, the ROS topics, and how motion commands reach the Lite3. It is written for
teammates and coding agents working from a fresh clone.

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

This workspace holds only the **onboard** half of perception. YOLO person detection
(`human_detector`) runs on a separate external computer from the `team_perception_external`
package (Robot-Dog-Grp4 repo, `challenge_files/external/`), which is not in this workspace.

## 3. Packages in this repository

```
challenge_ws/
├── agent.md                  ← this file
├── src/
│   ├── maze_nav/             maze traversal: wall scan + SLAM + Nav2 + frontier explorer
│   └── team_perception/      onboard perception: QR scanner, poster detector, image throttle
├── maps/                     maps saved by maze_nav (created on first run, git-ignored)
├── ROS_bags/                 rosbag recordings + ROSBAG_GUIDE.md (bags git-ignored, never commit)
├── hardware_testing/, recordings/, runs/   logs from earlier hardware sessions
├── build/ install/ log/      colcon output          (should not be in git; see section 11)
```

### `maze_nav`: get through the maze

```bash
ros2 launch maze_nav maze_nav.launch.py            # robot standing on the start cell
ros2 launch maze_nav maze_nav.launch.py go:=false  # goals on /maze/goal only, robot doesn't move
```

Explores with `slam_toolbox` + Nav2 until there is open floor next to the robot, then stops.
`/maze/status` ends as `EXITED`, `NO_EXIT` or `STOPPED`, and the map is saved to `maps/`. It uses its own
`/wall_scan`, which keeps points 0.30–0.43 m above the floor (measured in `odom`). Maze walls are only
0.45 m tall, and the lab `/scan` starts at ~0.45 m. A simulator of the example maze is in
`src/maze_nav/test/fake_maze_lite3.py`. Details: `src/maze_nav/README.md`.

### `team_perception`: perception (onboard build)

```bash
ros2 launch team_perception robot.launch.py                 # all three nodes
ros2 launch team_perception robot.launch.py posters:=false  # QR scanner only
ros2 launch team_perception robot.launch.py --show-args     # every topic name is an argument
```

| Executable | Subscribes | Publishes |
|---|---|---|
| `qr_scanner` | `/camera/color/image_raw/compressed`, `/camera/color/camera_info`, `/camera/depth/image_raw` + `/camera/depth/camera_info` (unaligned depth, mapped in software) | `/qr/detections` (String JSON, once per new code), `/found` (`QR found`) |
| `image_throttle` | `/camera/color/image_raw/compressed` | `/camera/color/image_offboard/compressed` (10 Hz, for the external computer) |
| `poster_detector` | `/humans/detections` (from the external computer), depth + camera_info, `/map`, TF | `/posters/detections` (String JSON), `/found` (`Human found`) |

Person boxes come from `human_detector` on the external computer; without it there are no
poster detections, but QR scanning still works (setup: `external_computer_checklist.md`).
Only the throttled JPEG stream and `/humans/detections` cross the network. Inputs, requirements and what works with less data:
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
`QR found` / `Human found`), `/camera/color/image_offboard/compressed` (to the external
computer), `/humans/detections` (from the external computer). The onboard nodes publish no
debug images.

### TF tree

```
odom ──(Fast-LIO/odom_relay)──> base_link ──(static)──> livox_frame
                                          └─(static)──> camera frames
```

`base_link → livox_frame`: translation (0.187, 0, 0.125) m, pitched about 15° down. `/tf` is
**empty unless Fast-LIO is running** (`lite3_bringup`). Leg odometry is published as topics only, not as TF.

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
| `0x21010407` | H-Step (high-stepping RUG gait, gait state 13) | For rough ground. See [DEEP Robotics docs](https://developer.deeprobotics.cn/docs/lite3/interfaces/control/state/) |
| `0x21020C0E` | Possibly a software STOP | Check it against the manufacturer docs before relying on it |

Usual sequence: robot standing (handheld controller) → `lite3_auto` → drive with
`/cmd_vel` → `lite3_manual` when done. There is no gait or mode feedback topic through the
bridge, so watch the robot or the handheld controller to confirm a mode change.

## 8. DDS and launching

- Fast DDS with **UDP-only transport** (`src/maze_nav/config/fastdds_udp.xml`). Nav2's
  lifecycle startup stalled on this robot with the default shared-memory transport. The
  `maze_nav` launch sets it for its own nodes (`udp_only:=true`). For manual runs:
  `export FASTRTPS_DEFAULT_PROFILES_FILE=~/challenge_ws/src/maze_nav/config/fastdds_udp.xml`.
- **Before you launch, check that nothing else owns SLAM, Nav2 or `/cmd_vel`:** run
  `ros2 node list`. Two `slam_toolbox` or Nav2 stacks on domain 4 corrupt `/map` and TF and
  can both drive the robot. Stop the old one first.
- Use another domain (`export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1`) for any test that
  must not reach the robot.

## 9. Testing

```bash
cd ~/challenge_ws && source install/setup.bash
(cd src/team_perception && python3 -m pytest -q test)   # QR utils, depth alignment, poster tests
(cd src/maze_nav && python3 -m pytest -q test)          # wall scan, frontiers, exit, example maze
```

`team_perception` has no ament lint tests (flake8/pep257/copyright).

## 10. Recording data

Use the `rec_*` aliases from `ROS_bags/aliases.sh` (loaded by `~/.bashrc` on the robot). Each
runs a preset in `ROS_bags/record.sh`. Full guide: `ROS_bags/ROSBAG_GUIDE.md`.

```bash
rec_full <run_name>      # teleop, everything incl. depth (Ctrl+C to stop)
rec_nav                  # autonomous run, no depth; name defaults to nav_<date>_<time>
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
- The robot's local repo currently has **no remote** and a separate one-commit history.
  Before pushing from the robot, add the remote, run `git fetch`, and rebase or cherry-pick
  onto `origin/main`.
- Never commit `build/`, `install/`, `log/`, bags in `ROS_bags/`, `maps/`, or other
  files over 100 MB.

## 12. Known issues and TODO

- The robot's depth image is not aligned to colour. `qr_scanner` and `poster_detector` map
  colour pixels into it in software (`DepthAlign` in `qr_utils.py`), a temporary fix. If
  the camera driver's depth registration is turned on, set `depth_topic` to the aligned
  topic and `depth_camera_info_topic:=''`.
- `lite3_launcher/config/pointcloud_to_scan.yaml`: `min_height` is still marked TODO. If the
  floor shows up as obstacles in `/scan` or the map, tune it.
- `lite3_launcher` isn't version-controlled (section 4).
