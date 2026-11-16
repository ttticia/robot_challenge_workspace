# Launching team_perception

How to start perception on the Lite3: QR codes (`qr_scanner`) and person posters
(`poster_detector`, with YOLO on the external computer). Inputs and fallbacks are in
[data_required.md](data_required.md). Setting up the external computer for the first time is
covered in [external_computer_checklist.md](../../external_computer_checklist.md).

Perception runs in **two places**:

| Machine | Launch | Nodes |
|---|---|---|
| Robot computer (`192.168.10.74`) | `ros2 launch team_perception robot.launch.py` | `qr_scanner`, `image_throttle`, `poster_detector` |
| External computer (same `192.168.10.x` network) | `ros2 launch team_perception_external external.launch.py` | `human_detector` (YOLO) |

QR codes need only the robot side. Posters need both: with no external computer, nothing is
ever reported as `Human found`.

## 1. Before you launch

1. **Robot computer: built and sourced** (first time, or after changing the package):
   ```bash
   cd ~/challenge_ws
   bash src/team_perception/install_deps.sh --no-apt   # once per machine: zxing-cpp, numpy<2
   colcon build --symlink-install --packages-select team_perception
   source install/setup.bash
   ```
2. **`ROS_DOMAIN_ID=4` on every machine** (robot, external computer, laptop). Leave
   `ROS_LOCALHOST_ONLY` unset. The robot's `~/.bashrc` already sets this.
3. **Camera topics are there.** On the robot:
   ```bash
   ros2 topic hz /camera/color/image_raw/compressed     # about 30 Hz
   ros2 topic list | grep -E "camera/(color|depth)/camera_info|depth/image_raw"
   ```
   If the names differ, pass the real ones as launch arguments (section 2).
4. **Navigation is up if you want positions in the map.** Positions come from TF `map → …
   → camera_color_optical_frame` and the poster map test uses `/map`. Both are published by
   `maze_nav` (see [nav_launch.md](../maze_nav/nav_launch.md)). Perception can start before or
   after it; until TF to `map` exists, positions are reported in the camera frame (log:
   `No tf to map`).
5. **Write access** to `~/qr_logs/` and `~/poster_logs/` on the robot (snapshots and CSVs).
6. **Don't record raw depth** (`/camera/depth/image_raw`) during an autonomous run: it stalled
   Nav2 on 2026-11-13. Record perception results with the `rec_*` presets in
   `ROS_bags/ROSBAG_GUIDE.md` (group G).

## 2. Launch

### Robot computer

**`maze_nav` starts this for you** (`perception:=true`, the default; `posters:=false` = QR
codes only). Launch it by hand only when you're not running maze_nav, or with
`perception:=false`, for example to change the camera topics. Don't run both: every code and
poster would be reported twice.

```bash
cd ~/challenge_ws && source install/setup.bash
ros2 launch team_perception robot.launch.py
```

Expected startup lines:

```
[qr_scanner]: QR scanner on /camera/color/image_raw/compressed using ..., logging to ~/qr_logs/qr_<date>_<time>.csv
[image_throttle]: Forwarding /camera/color/image_raw/compressed -> /camera/color/image_offboard/compressed at <= 10 Hz
[poster_detector]: Poster detector: /humans/detections + /camera/depth/image_raw, map_mode=veto (/map)
[poster_detector]: Static map received: <w>x<h> @ 0.050 m      # once maze_nav's /map is up
```

Common variants:

```bash
ros2 launch team_perception robot.launch.py posters:=false          # QR codes only, no external computer
ros2 launch team_perception robot.launch.py offboard_rate_hz:=5.0   # slow Wi-Fi to the external computer
ros2 launch team_perception robot.launch.py --show-args             # list every argument
```

### External computer

```bash
source ~/ext_ws/install/setup.bash
export ROS_DOMAIN_ID=4
ros2 launch team_perception_external external.launch.py              # CPU
ros2 launch team_perception_external external.launch.py device:=0    # NVIDIA GPU
```

Its `rate_hz` (default 10) must not be higher than the robot's `offboard_rate_hz`.

### Arguments (`robot.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `qr` | `true` | Run `qr_scanner` |
| `posters` | `true` | Run `image_throttle` + `poster_detector` (needs the external computer) |
| `image_topic` | `/camera/color/image_raw` | Colour image, raw name (without `/compressed`) |
| `image_transport` | `compressed` | `compressed` = subscribe to `<image_topic>/compressed`; `raw` = the uncompressed topic |
| `camera_info_topic` | `/camera/color/camera_info` | Colour intrinsics |
| `depth_topic` | `/camera/depth/image_raw` | Depth image (aligned to colour or not) |
| `depth_camera_info_topic` | `/camera/depth/camera_info` | Depth intrinsics, used to map colour pixels into the unaligned Lite3 depth. `''` = `depth_topic` is already aligned |
| `use_depth` | `true` | Place QR codes in 3-D using depth |
| `map_topic` | `/map` | Static map for the poster map test (transient local) |
| `map_frame` | `map` | Frame positions are reported in |
| `map_mode` | `veto` | Poster map test. `veto`: depth test decides, the map can overrule it only by saying "free space". `require`: poster only if the map also confirms a wall. `off`: depth test only |
| `offboard_topic` | `/camera/color/image_offboard/compressed` | Throttled JPEG stream sent to the external computer |
| `offboard_rate_hz` | `10.0` | Frames per second sent to the external computer (about 6 Mbit/s at 10) |

If the driver ever publishes aligned depth:

```bash
ros2 launch team_perception robot.launch.py \
    depth_topic:=/camera/aligned_depth_to_color/image_raw depth_camera_info_topic:=''
```

## 3. Check it works

| # | Where | Command | Expect |
|---|---|---|---|
| 1 | robot | `ros2 node list \| grep -E "qr_scanner\|poster_detector\|image_throttle"` | all three (only `qr_scanner` with `posters:=false`) |
| 2 | external | `ros2 topic hz /camera/color/image_offboard/compressed` | about `offboard_rate_hz` |
| 3 | robot | `ros2 topic hz /humans/detections` (someone in front of the camera) | a steady rate |
| 4 | robot | `ros2 topic echo /found` | `QR found` when a code is shown; `Human found` a few seconds after a person poster is shown |

## 4. Outputs

| Topic / file | Type | Content |
|---|---|---|
| `/found` | `std_msgs/String` | `QR found` once per new code text, `Human found` once per new poster position |
| `/qr/detections` | `std_msgs/String` (JSON) | One message per new code: text, position, frame |
| `/posters/detections` | `std_msgs/String` (JSON) | One message per newly confirmed poster: track id, position, frame |
| `/humans/detections` | `std_msgs/String` (JSON) | From the external computer: `stamp`, `image_size`, `people` boxes |
| `~/qr_logs/` | CSV + JPEG snapshot | One row and snapshot per new code (created on the first find) |
| `~/poster_logs/` | CSV | One row per confirmed poster (created on the first find) |

The same QR code is reported again after 30 s, the same poster after 60 s. A poster is confirmed
once it wins 5 of the last 8 frames, within 4 m of the camera.

Watch a run from the laptop (`ROS_DOMAIN_ID=4`):

```bash
ros2 topic echo /found
ros2 topic echo /qr/detections
ros2 topic echo /posters/detections
```

### Stopping

Ctrl+C in each launch terminal. The CSV logs are already on disk (written per detection).

## 5. When something goes wrong

| What you see | Likely cause | Do |
|---|---|---|
| No startup lines, `package 'team_perception' not found` | Workspace not built or not sourced | Section 1, step 1 |
| `ModuleNotFoundError: zxingcpp` / `pyzbar` | Dependencies missing | `bash src/team_perception/install_deps.sh` |
| cv_bridge error mentioning numpy | numpy 2 installed | `pip3 install --user "numpy<2"` |
| QR scanner runs but never finds codes | Wrong `image_topic` / `image_transport` | `ros2 topic hz` on the topic it logged at startup |
| `No tf to map (...); reporting camera frame` | `maze_nav` (SLAM) not running yet | Start navigation; it clears on its own |
| `depth not used: ...` | Depth or depth `camera_info` missing, wrong topic | Check `depth_topic` / `depth_camera_info_topic` with `ros2 topic hz` |
| Check 2 shows nothing on the external computer | `posters:=false`, or different `ROS_DOMAIN_ID` / subnet, or multicast blocked | See the checklist's "If something fails" |
| `no depth image within max_dt_s of the detection` | Round trip to the external computer over 2 s | Lower `offboard_rate_hz`, use the GPU, or move closer to the access point |
| People detected but never `Human found` | No TF to `map`, poster more than 4 m away, or not enough votes yet | Start localisation, get closer, hold the view for a few seconds |
| Real people reported as posters | Map test off, or the map has a wall where they stand | Keep `map_mode:=veto` (or `require`); the static map must be built **without people in it** |
| `ros2 topic echo` fails with `!rclpy.ok()` | Stale ROS daemon | `ros2 daemon stop; ros2 daemon start` |

## 6. Without the robot

Unit tests (no ROS graph needed):

```bash
cd ~/challenge_ws/src/team_perception && python3 -m pytest -q test
```

To replay a recording, use a domain that is not the robot's so nothing reaches it:

```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
ros2 bag play <bag> &
ros2 launch team_perception robot.launch.py posters:=false   # posters also need the external side on this domain
```
