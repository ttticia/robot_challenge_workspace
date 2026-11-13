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
pip3 install --user -r src/team_perception/requirements.txt   # zxing-cpp, onnxruntime

# 2. Underlays this workspace depends on (see section 4), in this order:
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash     # transfer_interfaces, livox_ros_driver2
source ~/ros2_ws/install/local_setup.bash      # lite3_launcher, fast_lio

# 3. Build and source this workspace
cd ~/challenge_ws
colcon build --symlink-install --packages-select navigation team_perception
source install/setup.bash
```

On the robot computer, `~/.bashrc` already sources all of these and sets
`ROS_DOMAIN_ID=4`. It also defines the `lite3_auto` and `lite3_manual` aliases (section 7).

People detection needs a model that is not in git:
`python3 src/team_perception/scripts/download_people_model.py` (writes
`models/yolov5n.onnx`).

## 3. Packages in this repository

```
challenge_ws/
├── agent.md                  ← this file
├── src/
│   ├── navigation/           SLAM + Nav2 mapping, automatic map saving      (active)
│   ├── nav_straight/         walk straight, stop before the first obstacle  (active; see its README)
│   ├── team_perception/      QR scanner, people detector, webcam bridge     (active)
│   ├── archive/              old mission stack + its design notes          (reference only)
│   │   └── lite3_mission/    frontier exploration, velocity gate, maze session
│   └── ROS_bags/             COLCON_IGNORE; old test clips (to be removed)
├── ROS_bags/                 rosbag recordings      (git-ignored *.db3, never commit)
├── maps/                     saved maps from navigation (created on first run)
├── models/                   ONNX models            (download, don't commit)
├── hardware_testing/, recordings/, runs/   logs from earlier hardware sessions
├── build/ install/ log/      colcon output          (should not be in git; see section 11)
```

### `navigation`: build and save a map

```bash
ros2 launch navigation mapping.launch.py               # SLAM + Nav2 + map saving
ros2 launch navigation mapping.launch.py nav2:=false   # SLAM only, drive by teleop
ros2 launch navigation mapping.launch.py drive:=false  # Nav2 plans, nothing reaches /cmd_vel
```

| Node | Does |
|---|---|
| `lite3_bringup` include (from `lite3_launcher`) | Fast-LIO → `/odom` + `odom→base_link`; `/livox/points` → `/scan`. Skip with `bringup:=false` |
| `slam_toolbox` (online async) | `/scan` + odometry → `/map`, `map→odom` |
| Nav2: `planner_server`, `controller_server` (Regulated Pure Pursuit), `behavior_server`, `bt_navigator`, `velocity_smoother`, `lifecycle_manager_navigation` | Goals on `/navigate_to_pose` (RViz *2D Goal Pose*) → `/nav_cmd_vel`. Plans through unknown space |
| `cmd_vel_bridge` | `/nav_cmd_vel` → `/cmd_vel` with the QoS the robot bridge needs; clamps to 0.25 m/s, 0.5 rad/s; one stop when Nav2 goes quiet |
| `map_autosaver` | Saves `/map` every 30 s and on Ctrl+C to `~/challenge_ws/maps/map_<date>_<time>.{pgm,yaml}`; `maps/latest.yaml` points to the newest |

Config: `src/navigation/config/nav2_params.yaml` (speeds, footprint, inflation),
`slam_toolbox.yaml`, `fastdds_udp.xml`. More detail: `src/navigation/README.md`.

### `team_perception`: perception

| Executable | Subscribes | Publishes |
|---|---|---|
| `qr_scanner` | `image_topic` (default `/camera/color/image_raw`, or `/compressed` with `image_transport:=compressed`), `/camera/color/camera_info`, `depth_topic` (default `/camera/aligned_depth_to_color/image_raw`, **which the robot doesn't publish**; see section 12) | `/qr/detections` (String JSON, once per new code), `/qr/debug_image[/compressed]` |
| `yolo_node` (people) | `/camera/color/image_raw/compressed` | `/people/detections` (String JSON), `/mission/observations`, `/people/debug_image/compressed` |
| `webcam_bridge` | UDP port 5005 (laptop webcam sender) | `/camera/color/image_raw[/compressed]` for testing without the robot |

Scripts in `src/team_perception/scripts/`: `bag_to_video.py` (rosbag → mp4),
`qr_bag_video.py`, `eval_qr_video.py`, `webcam_qr_test.py`, `webcam_sender_windows.py`.
Full guide: `src/team_perception/QR_test_guide.md`. Recording guide:
`src/team_perception/ROSBAG_GUIDE.md`.

### `archive/lite3_mission` (reference only)

The earlier autonomy stack: frontier exploration, a velocity gate, a maze session
coordinator, and QR/scene adapters. Its notes are `src/archive/*.md`. It is kept so you
can reuse its ideas and tests (`test/nav2_fixture.py` and `test/test_ros_contract.py` are
good examples). **colcon still builds it** because it has a `package.xml`. Add an empty
`src/archive/COLCON_IGNORE` file if you want it out of the build. Don't run it alongside
`navigation`: both start SLAM and Nav2.

## 4. Dependencies that are NOT in this repository

| Package | Where it lives on the robot | Source |
|---|---|---|
| `lite3_launcher` | `~/ros2_ws/src/lite3_launcher` | **Not in any git repo yet.** Copy it from the robot if you need it on another machine |
| `fast_lio` | `~/ros2_ws/src/FAST_LIO` | github.com/hku-mars/FAST_LIO (branch `ROS2`) |
| `livox_ros_driver2` | `~/lite3_ws/src/livox_ros_driver2` | Livox SDK2 driver |
| `transfer_interfaces` (`MotionSimpleCMD`, `MotionComplexCMD`) | `~/lite3_ws/src/transfer_interfaces` | DEEP Robotics Lite3_ROS |
| `transfer` (manufacturer motion bridge, for reference) | `~/hardware_support/Lite3_ROS/src/transfer` | github.com/DeepRoboticsLab/Lite3_ROS (branch `ros2-foxy`) |
| Sensor drivers + `lite3_domain_bridge_4` | Already running (driver container) | Shared lab setup; don't modify |

To develop off-robot without these, use `bringup:=false` and the simulator in section 9.

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

### Localization and navigation (from our launch files)

| Topic | Type | Publisher |
|---|---|---|
| `/odom`, TF `odom → base_link` | `nav_msgs/Odometry` | Fast-LIO + `odom_relay` (`lite3_bringup`) |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` (0.1–0.5 m above `base_link`, 0.4–20 m) |
| `/map`, TF `map → odom` | `nav_msgs/OccupancyGrid` (reliable, transient local) | `slam_toolbox` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Send goals here (`frame_id: map`) |
| `/cmd_vel_nav` → `/nav_cmd_vel` | `Twist` | Nav2 controller → velocity smoother/behaviors |
| `/cmd_vel` | `Twist` | `cmd_vel_bridge` (only while Nav2 is driving), or teleop |

### Perception

`/qr/detections`, `/qr/debug_image[/compressed]`, `/people/detections`,
`/people/debug_image/compressed`, `/mission/observations` (all detections are JSON in
`std_msgs/String`).

### TF tree

```
map ──(slam_toolbox)──> odom ──(Fast-LIO/odom_relay)──> base_link ──(static)──> livox_frame
                                                                    └─(static)──> camera frames
```

`base_link → livox_frame`: translation (0.187, 0, 0.125) m, pitched about 15° down. `/tf` is
**empty unless Fast-LIO is running** (`lite3_bringup`, or the `navigation` launch with
`bringup:=true`). Leg odometry is published as topics only, not as TF.

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
`ros2 topic info -v /cmd_vel`. For an example, see `src/navigation/navigation/cmd_vel_bridge.py`.
Nav2 and `twist_mux` publish VOLATILE, so put a bridge like that in front of them.

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
| `0x21020C0E` | Treated as a software STOP by `archive/lite3_mission` | Check it against the manufacturer docs before relying on it |

Usual sequence: robot standing (handheld controller) → `lite3_auto` → drive with
`/cmd_vel` → `lite3_manual` when done. There is no gait or mode feedback topic through the
bridge, so watch the robot or the handheld controller to confirm a mode change.

## 8. DDS and launching

- Fast DDS with **UDP-only transport** (`src/navigation/config/fastdds_udp.xml`). Nav2's
  lifecycle startup stalled on this robot with the default shared-memory transport. The
  `navigation` launch sets it for its own nodes (`udp_only:=true`). For manual runs:
  `export FASTRTPS_DEFAULT_PROFILES_FILE=~/challenge_ws/src/navigation/config/fastdds_udp.xml`.
- **Before you launch, check that nothing else owns SLAM, Nav2 or `/cmd_vel`:** run
  `ros2 node list`. Two `slam_toolbox` or Nav2 stacks on domain 4 corrupt `/map` and TF and
  can both drive the robot. Stop the old one first.
- Use another domain (`export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1`) for any test that
  must not reach the robot.

## 9. Testing

```bash
cd ~/challenge_ws && source install/setup.bash
python3 -m pytest -q src/navigation/test          # map file writer
(cd src/team_perception && python3 -m pytest -q test)   # QR utils, people detector
bash src/archive/lite3_mission/test/run_team_tests.sh    # old stack incl. ROS contract tests
```

**Full stack without the robot** (simulated room, real SLAM + Nav2 + map saving):

```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
python3 src/navigation/test/fake_lite3.py &
ros2 launch navigation mapping.launch.py bringup:=false map_dir:=/tmp/sim_maps save_interval:=5
# another terminal (same exports):
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 1.2}, orientation: {w: 1.0}}}}"
```

The goal should finish `SUCCEEDED`, and `/tmp/sim_maps/` should contain the room with a box.

Known lint state: `team_perception`'s `test_flake8` and `test_pep257` fail on the
compact code style in `yolo_node.py` and `people.py`. These are style issues only.

## 10. Recording data

```bash
mkdir -p ~/challenge_ws/ROS_bags
ros2 bag record -o ~/challenge_ws/ROS_bags/<run_name> \
  /camera/color/image_raw/compressed /camera/color/camera_info \
  /camera/depth/image_raw /camera/depth/camera_info \
  /livox/lidar /livox/points /livox/imu /imu/data /leg_odom2 /tf /tf_static
ros2 bag info ~/challenge_ws/ROS_bags/<run_name>
python3 src/team_perception/scripts/bag_to_video.py ~/challenge_ws/ROS_bags/<run_name>
```

This uses about 1.6 GB per minute; the robot disk had about 11 GB free (check `df -h`). `/tf`
is only recorded when Fast-LIO is running. Bags are git-ignored. Copy them off the robot.

**Not while Nav2 drives:** recording `/camera/depth/image_raw` delayed `/livox/points` by
~1.2 s and collision_monitor held the robot at zero (2026-11-13). Leave raw depth out during
autonomous runs (see `src/team_perception/ROSBAG_GUIDE.md`).

## 11. Git workflow

- Team remote: `https://github.com/ttticia/robot_challenge_workspace.git` (`main`).
  Work on a feature branch and open a pull request; don't push to `main`.
- The robot's local repo currently has **no remote** and a separate one-commit history.
  Before pushing from the robot, add the remote, run `git fetch`, and rebase or cherry-pick
  onto `origin/main`.
- Never commit `build/`, `install/`, `log/`, `ROS_bags/`, `maps/`, `models/*.onnx`, or other
  files over 100 MB. `install/` and `log/` are tracked by mistake right now
  (`git rm -r --cached build install log` and add them to `.gitignore`).

## 12. Known issues and TODO

- `qr_scanner`'s default `depth_topic` (`/camera/aligned_depth_to_color/image_raw`) doesn't
  exist on the robot, so QR map positions are missing unless depth alignment is turned on
  in the camera driver or the parameter is changed.
- `lite3_launcher/config/pointcloud_to_scan.yaml`: `min_height` is still marked TODO. If the
  floor shows up as obstacles in `/scan` or the map, tune it.
- slam_toolbox runs with `use_scan_matching: false` (trusts Fast-LIO odometry). Turn it on if
  walls look doubled.
- `lite3_launcher` isn't version-controlled (section 4).
- `src/ROS_bags/` inside `src` is leftover test data. Recordings now go to `~/challenge_ws/ROS_bags/`.
- `nav_straight`: the robot must be **standing before launch** (the floor height is fixed at
  launch). Launched lying down, it stops at once with `Travelled 0.00 m`. Obstacle band is
  0.10–1.05 m above the floor so table tops count. Not yet tested on the robot: low box,
  full 5 m run.
