# Launching team_perception

How to start perception on the Lite3: QR codes (`qr_scanner`) and person posters
(`human_detector` YOLO boxes, judged by `poster_detector`). Inputs and fallbacks are in
[data_required.md](data_required.md).

Everything runs on the **robot computer** (`192.168.10.74`), on its CPU:

| Launch | Nodes |
|---|---|
| `ros2 launch team_perception robot.launch.py` | `qr_scanner`, `human_detector` (YOLO), `poster_detector` |

YOLO is capped (`yolo_threads:=2`, `yolo_rate_hz:=5.0`) so it leaves the CPU to navigation.

## 1. Before you launch

1. **Robot computer: built and sourced** (first time, or after changing the package):
   ```bash
   cd ~/challenge_ws
   bash src/team_perception/install_deps.sh --no-apt   # once: zxing-cpp, CPU torch, ultralytics, YOLO weights
   colcon build --symlink-install --packages-select team_perception
   source install/setup.bash
   ```
   Built without `--symlink-install` (as `install/` currently is on the robot computer),
   rebuild after every change, or the old code runs.
2. **`ROS_DOMAIN_ID=4` on every machine** (robot, laptop). Leave
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
   after it. Posters are tracked in Fast-LIO's `odom` (there from launch); the survivor is
   reported as the **centre of a maze cell**, placed with `map` once SLAM is up (before
   that, `odom`, which matches `map` at the start). When SLAM comes up, or a correction
   moves a survivor into another cell, it is reported again (CSV `ring` column `SLAM is up`
   / `SLAM correction: was cell ...`), with no second `Human found`.
   **The robot's camera TF has a known fault** (since 2026-11-18): `/tf_static` gives
   `camera_depth_optical_frame` a second, unpitched parent (`base_link`), and TF picks it, so
   camera points land 20° off and colour/depth alignment is wrong. `qr_scanner`,
   `poster_detector` and `human_detector` work around it: they keep the correct parent
   (`camera_depth_frame`) in their own TF buffers (`team_perception/tf_pin.py`, node
   parameter `tf_pin`, `['']` = off) and log one warning when they see the duplicate.
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
[human_detector]: Loading YOLO model yolo11n.pt on cpu (2 threads) ...
[human_detector]: Human detector on /camera/color/image_raw/compressed, <= 5 Hz, publishing /humans/detections
[depth_relay]: /camera/depth/image_raw -> /camera/depth_relay/image_raw at <= 10.0 Hz; ...
[poster_detector]: Poster detector: /humans/detections + /camera/depth_relay/image_raw, map_mode=veto (/map)
[poster_detector]: Static map received: <w>x<h> @ 0.050 m      # once maze_nav's /map is up
[WARN] [qr_scanner]: /tf_static also has base_link -> camera_depth_optical_frame; keeping camera_depth_frame -> camera_depth_optical_frame (tf_pin). ...
```

The `tf_pin` warning (once per node) means the camera TF fault above is still there and is
being worked around. It goes away when the robot side is fixed.

Common variants:

```bash
ros2 launch team_perception robot.launch.py posters:=false          # QR codes only, no YOLO
ros2 launch team_perception robot.launch.py yolo_rate_hz:=3.0       # less CPU if navigation struggles
ros2 launch team_perception robot.launch.py --show-args             # list every argument
```

### Arguments (`robot.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `qr` | `true` | Run `qr_scanner` |
| `posters` | `true` | Run `human_detector` (YOLO) + `poster_detector` |
| `image_topic` | `/camera/color/image_raw` | Colour image, raw name (without `/compressed`) |
| `image_transport` | `compressed` | `compressed` = subscribe to `<image_topic>/compressed`; `raw` = the uncompressed topic |
| `camera_info_topic` | `/camera/color/camera_info` | Colour intrinsics |
| `depth_topic` | `/camera/depth/image_raw` | Depth image (aligned to colour or not). With `depth_relay:=true` this is the relay's INPUT; the nodes read `/camera/depth_relay/*` |
| `depth_relay` | `true` | One `depth_relay` subscribes the raw depth once and passes ≤ `relay_rate_hz` frames on, unchanged, to `qr_scanner` and `poster_detector`. Every process subscribing raw depth costs the robot's bridge VM a full 18 MB/s copy. `maze_nav.launch.py` runs its own relay (shared with `stair_detector`) and passes `false` |
| `relay_rate_hz` | `10.0` | Frames a second the relay passes on |
| `depth_camera_info_topic` | `/camera/depth/camera_info` | Depth intrinsics, used to map colour pixels into the unaligned Lite3 depth. `''` = `depth_topic` is already aligned |
| `use_depth` | `true` | Place QR codes in 3-D using depth |
| `map_topic` | `/map` | Static map for the poster map test (transient local) |
| `map_frame` | `map` | Frame positions are reported in |
| `map_mode` | `veto` | Poster map test. `veto`: depth test decides, the map can overrule it only by saying "free space". `require`: poster only if the map also confirms a wall. `off`: depth test only |
| `yolo_rate_hz` | `5.0` | Max YOLO inferences per second |
| `yolo_threads` | `2` | torch CPU threads for YOLO (`0` = torch default, about 8) |
| `yolo_imgsz` | `640` | YOLO input size (smaller = faster, misses far people) |
| `yolo_conf` | `0.4` | Minimum person confidence |
| `yolo_model` | `yolo11n.pt` | Weights: bare name = `~/.cache/team_perception/`, or a path |

If the driver ever publishes aligned depth:

```bash
ros2 launch team_perception robot.launch.py \
    depth_topic:=/camera/aligned_depth_to_color/image_raw depth_camera_info_topic:=''
```

## 3. Check it works

| # | Where | Command | Expect |
|---|---|---|---|
| 1 | robot | `ros2 node list \| grep -E "qr_scanner\|human_detector\|poster_detector"` | all three (only `qr_scanner` with `posters:=false`) |
| 2 | robot | `ros2 topic hz /humans/detections` (someone in front of the camera) | up to `yolo_rate_hz` |
| 3 | robot | `ros2 topic echo /found` | `QR found` when a code is shown; `Human found` a few seconds after a person poster is shown |

## 4. Outputs

| Topic / file | Type | Content |
|---|---|---|
| `/found` | `std_msgs/String` | `QR found` once per new code text, `Human found` once per survivor (a new poster position, or a QR room left with no poster confirmed) |
| `/qr/detections` | `std_msgs/String` (JSON) | One message per new code (and every 30 s while seen): `text` (a hint to other survivors' locations, also logged as `QR hint: ...`), `room_odom` (internal: the code's room point, used by `poster_detector`; QR locations are not reported) |
| `/qr/read` | `std_msgs/String` | The code's text EVERY time a code is read (at most once a second per code); `maze_explorer` uses it to end `FACING_QR` |
| `/qr/unread` | `geometry_msgs/PointStamped` (`odom`) | A code located but not readable (needs zxing-cpp), seen in 3 of the last 5 frames, at most once a second: from depth, else 1.5 m along the camera ray. `maze_explorer` turns to face it (`FACING_QR`) until `/qr/read`. While one is in view, `qr_scanner` runs its thorough decode (contrast boost + 2x upscale) on every frame |
| `/perception/look_request` | `geometry_msgs/PointStamped` (`odom`) | From `poster_detector`: a QR room with no poster confirmed 3 s after the code was read; `maze_explorer` turns around there once |
| `/posters/detections` | `std_msgs/String` (JSON) | One message per confirmed survivor: `id`, `cell` `[i, j]`, `cell_centre` `[x, y]` (the reported location) and its `frame`, plus the poster's `odom` position, votes and reasons. Again when SLAM re-places it |
| `/humans/detections` | `std_msgs/String` (JSON) | From `human_detector`: `stamp`, `image_size`, `people` boxes |
| `~/qr_logs/` | CSV + JPEG snapshot | One row (time, text, snapshot) and snapshot per new code (created on the first find) |
| `~/poster_logs/` | CSV | One row per survivor report: cell, centre (the location to report), frame, poster position in `odom` (created on the first find) |

**QR codes and survivors.** Codes only appear in rooms with survivors (1-2 per room; not
every survivor room has a code). In a code's room (room points within `room_radius_m`,
default 1 m: rooms needn't be one cell) a poster is confirmed with `vote_need_qr` (3)
votes instead of 5. If no poster is confirmed there 3 s after the code is read,
`poster_detector` asks the explorer to look around (`/perception/look_request`). If the
robot then leaves the room (more than `leave_radius_m`, 1.5 m, after being inside), or
90 s pass, the survivor is reported from the code alone (CSV `poster_id` = `qr`). A poster
confirmed there later is that same survivor (no second `Human found`); a further poster
is a second survivor.

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
| cv_bridge error mentioning numpy, or SciPy "NumPy version >=1.17.3 and <1.25.0 is required" | Wrong pip numpy | `pip3 install --user "numpy==1.24.4"` |
| QR scanner runs but never finds codes | Wrong `image_topic` / `image_transport` | `ros2 topic hz` on the topic it logged at startup |
| `No tf to odom (...); person not tracked` | Fast-LIO / `maze_nav` not running | Start navigation; no poster is tracked without a fixed frame |
| Survivor in the wrong cell (all of them shifted) | Cell grid doesn't match the maze | Set `cell_size`, `origin_x`, `origin_y`, `grid_yaw_deg` (same values as `scene_trigger`) |
| `depth not used: ...` | Depth or depth `camera_info` missing, wrong topic | Check `depth_topic` / `depth_camera_info_topic` with `ros2 topic hz`, and `/camera/depth_relay/image_raw` (≈ 10 Hz) |
| `[depth_relay] no depth images for 5 s` | Camera driver or bridge down, or wrong `depth_topic` | `ros2 topic hz /camera/depth/image_raw` (expect ~30 Hz) |
| `could not start YOLO (...)` | torch / ultralytics or the weights missing | `bash src/team_perception/install_deps.sh --no-apt` |
| Check 2 shows nothing with a person in view | `posters:=false`, or `human_detector` died | Look for its error in the launch output |
| `no depth image within max_dt_s of the detection (closest ..., waited ...)` | Depth for that frame never arrived within `depth_wait_s` (0.5 s): depth stopped, or YOLO is more than 2 s behind (CPU overloaded). Detections whose depth is merely late (`depth_relay` is sometimes ~0.3 s late) are held and paired, not dropped | Check `ros2 topic hz /camera/depth_relay/image_raw`; lower `yolo_rate_hz` if CPU is short |
| People detected but never `Human found` | No TF to `map`, poster more than 4 m away, or not enough votes yet | Start localisation, get closer, hold the view for a few seconds |
| Real people reported as posters | Map test off, or the map has a wall where they stand | Keep `map_mode:=veto` (or `require`); the static map must be built **without people in it** |
| `ros2 topic echo` fails with `!rclpy.ok()` | Stale ROS daemon | `ros2 daemon stop; ros2 daemon start` |
| QR or survivor positions all shifted / too near or far, and no `tf_pin` warning at startup | The camera TF fault without the workaround (`tf_pin:=['']`, or a node built before 2026-11-18) | Rebuild `team_perception`; check the startup warning appears |

## 6. Without the robot

Unit tests (no ROS graph needed):

```bash
cd ~/challenge_ws/src/team_perception && python3 -m pytest -q test
```

To replay a recording, use a domain that is not the robot's so nothing reaches it. `89` is
for `maze_nav`'s simulator and other Claude sessions use their own, so pick a free one
(`ros2 node list` shows nothing):

```bash
export ROS_DOMAIN_ID=<free domain, e.g. 141> ROS_LOCALHOST_ONLY=1
ros2 bag play <bag> &
ros2 launch team_perception robot.launch.py
```
