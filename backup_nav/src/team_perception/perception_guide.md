# Perception guide

How `team_perception` works, how to run it, and how to watch and debug it on the robot.
This guide is the overview and the command reference. The full launch-argument table and the
troubleshooting table are in [launch_guide.md](launch_guide.md), and the robot inputs in
[data_required.md](data_required.md).

> **2026-11-19: human / survivor poster detection is off.** `robot.launch.py` (and so
> `maze_nav`) now starts `qr_scanner` only. With `human_detector` and `poster_detector`
> running, the extra camera copies from the robot bridge slowed it down (IMU 23 Hz instead of 200,
> lidar 1.1 s late, SLAM never built a map). The `posters` argument is gone. The node code
> is still here (`ros2 run team_perception human_detector` / `poster_detector`), so the
> poster sections below describe how they work if they are run by hand.

Everything runs on the **robot computer** (`192.168.10.74`), on its CPU, in
**`ROS_DOMAIN_ID=4`**. Use the same domain on a laptop to watch.

## 1. What it does

The robot has to find two things in the maze:

- **QR codes.** Every new code is reported once on `/found` as `QR found`. Its text is the
  conscious survivor's message (mission M04): it goes out on `/qr/read` exactly as decoded and
  `task_reporter` sends it. There is one QR code per arena, beside the M03 survivor.
- **Survivors**: pictures of people (posters) on walls. Each one is reported once on `/found`
  as `Human found`, and its location is the **centre of the maze cell** it is in. Real people
  standing in the maze must *not* be reported.

## 2. Pipeline

```
 Orbbec camera (via lite3_domain_bridge_4)
   │ /camera/color/image_raw/compressed  (JPEG, 30 Hz)
   │ /camera/depth/image_raw             (16UC1 mm, 30 Hz, NOT aligned to colour)
   │
   ├──► depth_relay ──► /camera/depth_relay/image_raw + camera_info   (same frames, ≤10 Hz)
   │                         │
   ├──► qr_scanner ◄─────────┤──► /found "QR found", /qr/detections, /qr/read, /qr/unread
   │                         │
   └──► human_detector       │
        (YOLO 'person')      │
          │ /humans/detections (boxes + camera stamp)
          ▼                  │
        poster_detector ◄────┘  + TF (odom, map) + /map
          └──► /found "Human found", /posters/detections, /perception/look_request

 maze_explorer (maze_nav) uses /qr/unread, /qr/read and /perception/look_request.
```

| Node | What it does |
|---|---|
| `depth_relay` | Subscribes to the raw depth **once** and passes at most 10 frames a second on, unchanged. Each process that subscribes to raw depth costs the bridge VM a full 18 MB/s copy. |
| `qr_scanner` | Decodes QR codes (zxing-cpp, falling back to pyzbar/OpenCV) on a background thread. For a new code it publishes `QR found`, saves a snapshot and a CSV row, and uses depth to work out the code's **room point** in `odom` (`room_odom`). A code that is found but can't be read is published on `/qr/unread`, and the explorer turns to face it. |
| `human_detector` | YOLO (`yolo11n`, 640 px, 2 torch threads, at most 5 Hz) on the newest JPEG frame. Publishes person boxes with the camera stamp. It does no positioning. |
| `poster_detector` | Decides for each person box whether it is a **poster or a real person**, tracks posters in `odom`, and reports each confirmed one as a survivor in a maze cell. |

### How a poster is told apart from a person

1. **Edge-ring test (depth).** Around a poster, the wall continues just outside the box at
   the same depth. Behind a real person, the depth jumps backwards.
2. **Static-map test.** A poster lies on a wall that is in `/map`. A real person stands in
   free space. With `map_mode:=veto` (the default) the depth test decides, and the map can
   only overrule it by saying "free space".
3. **Votes.** A track is confirmed once it wins **5 of the last 8 frames** within 4 m of the
   camera. In a room where a QR code was read it needs only **3**.

### QR codes and survivors

Codes only appear in rooms that have survivors. When a code is read, `poster_detector`
remembers its room. If no poster is confirmed there within 3 s, it sends a
`/perception/look_request` and the explorer turns around once. If the robot then leaves the
room (more than 1.5 m away) or 90 s pass, the survivor is reported from the code alone (CSV
`poster_id` = `qr`).

### Frames and locations

- Posters and QR rooms are **tracked in `odom`** (Fast-LIO). It is there from launch and
  never jumps.
- The reported location is the **cell centre in `map`**. The grid is set by `cell_size`,
  `origin_x`, `origin_y` and `grid_yaw_deg`, with the same values as `maze_nav`'s
  `scene_trigger`. Before SLAM is up, the cell is placed in `odom`. When SLAM comes up, or a
  correction moves a survivor into another cell, it is reported again, with no second
  `Human found`.
- **Camera TF fault (since 2026-11-18):** `/tf_static` gives `camera_depth_optical_frame` a
  second, wrong parent with no pitch. The nodes keep the correct parent in their own buffers
  (`tf_pin.py`) and log one `tf_pin` warning each at startup. That warning is expected.

## 3. Running it

**Normally `maze_nav` starts perception for you**, so don't also launch it by hand, or every
code and poster is reported twice:

```bash
ros2 launch maze_nav maze_nav.launch.py                    # perception:=true by default
ros2 launch maze_nav maze_nav.launch.py perception:=false  # navigation only
```

Run perception on its own (without navigation, or with `perception:=false`):

```bash
cd ~/challenge_ws && source install/setup.bash
ros2 launch team_perception robot.launch.py
ros2 launch team_perception robot.launch.py yolo_rate_hz:=3.0     # less CPU
ros2 launch team_perception robot.launch.py --show-args           # all arguments
```

Run a single node, for example to test it:

```bash
ros2 run team_perception qr_scanner --ros-args -p image_transport:=compressed
ros2 run team_perception human_detector --ros-args -p rate_hz:=2.0
ros2 run team_perception depth_relay --ros-args -p rate_hz:=15.0
```

Rebuild after you add a node or change the launch file. With `--symlink-install`, Python edits
are picked up without a rebuild:

```bash
cd ~/challenge_ws && colcon build --symlink-install --packages-select team_perception
```

## 4. Watching the important topics

Set up every terminal first:

```bash
export ROS_DOMAIN_ID=4
source ~/challenge_ws/install/setup.bash
```

### Results

```bash
ros2 topic echo /found                                  # "QR found" / "Human found"
ros2 topic echo --field data /posters/detections        # survivor JSON: id, cell, cell_centre, frame, votes, ring, map
ros2 topic echo --field data /qr/detections             # QR JSON: text, room_odom, snapshot, stamp
ros2 topic echo /qr/read                                # code text every time it is read (≤1 Hz per code)
ros2 topic echo /qr/unread                              # PointStamped (odom): code found but unreadable
ros2 topic echo /perception/look_request                # PointStamped (odom): "turn around here"
```

`--field data` prints only the JSON string, without the `data:` wrapper and `---` lines.

### YOLO

```bash
ros2 topic hz /humans/detections                        # ≤ yolo_rate_hz, only while a person is in view
ros2 topic echo --field data /humans/detections         # stamp, image_size, count, people[box, conf]
```

### Inputs

```bash
ros2 topic hz /camera/color/image_raw/compressed        # ~30 Hz
ros2 topic hz /camera/depth/image_raw                   # ~30 Hz (raw, from the bridge)
ros2 topic hz /camera/depth_relay/image_raw             # ~10 Hz (what the nodes read)
ros2 topic bw /camera/depth_relay/image_raw             # bandwidth
ros2 topic echo --once --no-arr /camera/color/camera_info   # intrinsics, without the big arrays
ros2 topic echo --once --field header /camera/depth_relay/image_raw   # stamp + frame only
ros2 topic info -v /map                                 # must be TRANSIENT_LOCAL
```

Never `ros2 topic echo` an image without `--field header` or `--no-arr`. It floods the terminal
and pulls a full copy across the bridge.

### TF

```bash
ros2 run tf2_ros tf2_echo odom camera_color_optical_frame       # tracking frame -> camera
ros2 run tf2_ros tf2_echo map odom                              # SLAM up? (map -> odom exists)
ros2 run tf2_ros tf2_echo camera_depth_optical_frame camera_color_optical_frame   # colour/depth offset (~14 mm)
ros2 run tf2_tools view_frames                                  # writes frames_<date>.pdf of the whole tree
ros2 topic echo --once /tf_static | grep -B2 -A2 camera_depth_optical_frame   # shows the duplicate parent
```

`tf2_echo` uses plain TF, so it shows the **wrong** depth-camera parent while the fault is
there. The nodes don't use it.

## 5. Debug commands

### Is everything running?

```bash
ros2 node list | grep -E "qr_scanner|human_detector|poster_detector|depth_relay"
ros2 node info /poster_detector             # its subscriptions and publishers
ros2 topic info -v /humans/detections       # who publishes and who subscribes, and their QoS
```

If there are two of any node, perception was started twice (by hand and by `maze_nav`).

### Parameters

```bash
ros2 param list /poster_detector
ros2 param get /poster_detector map_mode
ros2 param get /poster_detector cell_size        # also origin_x, origin_y, grid_yaw_deg
ros2 param dump /poster_detector                 # all values, as YAML
ros2 param get /qr_scanner image_transport
ros2 param get /human_detector rate_hz
```

Parameters are read at startup. To change one, relaunch with the launch argument or
`--ros-args -p name:=value`.

### Logs and saved results

```bash
ls -t ~/challenge_ws/logs/qr | head                        # newest QR CSV + JPEG snapshots
ls -t ~/challenge_ws/logs/posters | head                    # newest survivor CSV
column -s, -t < "$(ls -t ~/challenge_ws/logs/posters/*.csv | head -1)"   # read the latest survivor CSV as a table
ls -t ~/challenge_ws/logs/ros | head          # ROS launch logs (~/.ros/log points here)
grep -h -E "SURVIVOR|Human found|QR message|QR room" "$(ls -td ~/challenge_ws/logs/ros/*/ | head -1)"launch.log   # newest launch
```

Useful log lines:

| Line | Meaning |
|---|---|
| `QR message (M04): "..."` | New code read; this is its text, reported as mission M04 exactly as decoded |
| `QR code in view but unreadable at odom (...)` | Code found but not decoded; `/qr/unread` published |
| `QR room "..." ...: no poster yet, asking to look around` | Look request sent to the explorer |
| `SURVIVOR #n in cell (i, j), centre (x, y) map (...)` | Survivor reported, with its location |
| `Poster #n is the survivor already ...` | Poster matched an existing survivor (for example a QR-only one), so no new report |
| `No tf to odom (...); person not tracked` | Fast-LIO / `maze_nav` not running |
| `depth not used: ...` | Depth or its `camera_info` is missing |
| `no depth image within max_dt_s ...` | Depth late or stopped, or YOLO behind (CPU) |
| `/tf_static also has base_link -> camera_depth_optical_frame ...` | Expected: the camera TF workaround is active |

### CPU

```bash
top -o %CPU                                   # human_detector (YOLO) should be the biggest perception user
ls -t ~/challenge_ws/logs/cpu | head          # per-run CPU summaries from maze_nav's cpu_monitor
```

If navigation struggles, lower `yolo_rate_hz` (3.0) or `yolo_imgsz` (480).

### Stale ROS CLI

If `ros2 topic echo` fails with `!rclpy.ok()`, or `ros2 node list` shows old nodes:

```bash
ros2 daemon stop; ros2 daemon start
```

## 6. Quick checks before a run

| # | Command | Expect |
|---|---|---|
| 1 | `ros2 topic hz /camera/color/image_raw/compressed` | ~30 Hz |
| 2 | `ros2 topic hz /camera/depth_relay/image_raw` | ~10 Hz |
| 3 | `ros2 node list` | `qr_scanner`, `human_detector`, `poster_detector`, `depth_relay` (one of each) |
| 4 | Stand in front of the camera, `ros2 topic hz /humans/detections` | up to 5 Hz |
| 5 | Show a QR code, `ros2 topic echo /found` | `QR found` |
| 6 | Show a person poster on a wall for a few seconds | `Human found`, then a `SURVIVOR` log line |

## 7. Offline

Unit tests (no ROS graph needed):

```bash
cd ~/challenge_ws/src/team_perception && python3 -m pytest -q test
```

Replay a bag **in a free domain, never 4**, so nothing reaches the robot. Check that the
domain is empty with `ros2 node list` first; 89 is for `maze_nav`'s simulator:

```bash
export ROS_DOMAIN_ID=141 ROS_LOCALHOST_ONLY=1
ros2 bag play ~/challenge_ws/runs/ROS_bags/<bag> &
ros2 launch team_perception robot.launch.py
```

Record perception during a run with the `rec_*` presets (for example `rec_maze`) in
`~/challenge_ws/runs/ROS_bags/ROSBAG_GUIDE.md`. **Don't record raw depth during an autonomous run**:
it stalled Nav2.
