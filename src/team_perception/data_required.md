# Data required from the robot

A checklist of everything `team_perception` reads from the robot to make its detections.
All of it, YOLO person detection (`human_detector`) included, runs on the robot computer.
Defaults match the DEEP Robotics Lite3 (Orbbec RGB-D camera). Any topic name can be
changed with a launch argument (`ros2 launch team_perception robot.launch.py --show-args`).
For the outputs and how to launch, see [launch_guide.md](launch_guide.md).

## Checklist

| # | Data | Topic / source | Type | Lite3 value | Used by | Required? |
|---|---|---|---|---|---|---|
| 1 | Colour image | `/camera/color/image_raw/compressed` (or `/camera/color/image_raw` with `image_transport:=raw`) | `sensor_msgs/CompressedImage` (JPEG) or `sensor_msgs/Image` | 640×480, 30 Hz, frame `camera_color_optical_frame` | `qr_scanner`, `human_detector` | **Yes**: nothing is detected without it |
| 2 | Colour camera intrinsics | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | fx = fy ≈ 462, cx ≈ 320, cy ≈ 241 | `qr_scanner`, `poster_detector` | For positions and the poster test |
| 3 | Depth image | `/camera/depth/image_raw` | `sensor_msgs/Image`, `16UC1` (mm, 0 = no data) or `32FC1` (m) | 640×480, 30 Hz, frame `camera_depth_optical_frame`, **not aligned to colour** | `depth_relay` (once; it passes ≤ 10 Hz on to `qr_scanner`, `poster_detector`, `stair_detector`) | For positions. **Posters are never confirmed without it** |
| 4 | Depth camera intrinsics | `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | fx = fy ≈ 373, cx ≈ 319, cy ≈ 242 | `qr_scanner`, `poster_detector` | Yes while depth is unaligned. Not needed if the driver aligns depth (`depth_camera_info_topic:=''`) |
| 5 | Colour ↔ depth sensor offset | TF `camera_depth_optical_frame` ↔ `camera_color_optical_frame`, on `/tf_static` | TF | colour sensor 14 mm to the right of depth | `qr_scanner`, `poster_detector` | Recommended: without it, colour→depth mapping is up to 5 px off at 1 m |
| 6 | Camera pose in the map | TF chain `map → odom → base_link → camera_link → … → camera_*_optical_frame` | TF | `map → odom` from `slam_toolbox`, `odom → base_link` from `odom_relay` (Fast-LIO), the rest static | `qr_scanner`, `poster_detector` | For positions in `map`. Posters are tracked in `odom`; their cell is placed with `map` (with `odom` until SLAM is up, and the poster map test is skipped). With no `odom`, QR codes are in the camera frame and posters are not tracked |
| 7 | Static occupancy map | `/map` | `nav_msgs/OccupancyGrid`, **transient local** | from `slam_toolbox` | `poster_detector` | Recommended. Must be built **without people in it**. With the default `map_mode:=veto`, the depth test decides alone if it's missing |

Nothing has to cross the network: `human_detector` reads item 1 on the robot.

**Known fault in items 5 and 6 (since 2026-11-18):** the robot's `/tf_static` also publishes
`base_link → camera_depth_optical_frame` directly, at (0.25, 0, 0.05) with no pitch, beside the
correct `camera_depth_frame → camera_depth_optical_frame` (under `camera_link`, pitched 20° down).
TF keeps one parent and takes the wrong one, which puts depth points 20° off and the colour ↔
depth offset 20° wrong. The nodes keep the correct parent themselves (`tf_pin`, see
[launch_guide.md](launch_guide.md) section 1); the fix belongs on the robot side.

## Requirements on that data

- **Timestamps.** The depth image must have a stamp within `max_dt_s` (0.1 s) of the colour
  image it is paired with; otherwise `poster_detector` skips the frame. The pairing uses the
  camera stamp that `human_detector` copies into `/humans/detections`, so a detection may
  arrive up to 2 s (the depth buffer) after its frame without being lost. On the Lite3 all
  `/camera/*` topics and `/tf_static` use the wall clock, so they match.
  Don't mix in body-state topics (`/imu/data`, `/leg_odom`, …), which use a steady clock.
- **QoS.** Camera topics can be RELIABLE or BEST_EFFORT (the nodes subscribe with sensor-data
  QoS). `/map` must be TRANSIENT_LOCAL so a late-starting node still receives it.
- **Range.** Depth beyond 4 m (`max_range_m`) is treated as too noisy: a person there counts
  as unknown, never as a poster.
- **Depth alignment.** If the driver ever publishes depth aligned to colour, point
  `depth_topic` at it and set `depth_camera_info_topic:=''`. Aligned depth has a
  `camera_info` that matches `/camera/color/camera_info`.

## What works with less data

| Available | QR codes | People (`/humans/detections`) | Posters (`Human found`) |
|---|---|---|---|
| 1 only | Read, no position | Detected | **No** |
| 1 – 4 | Read, position in camera frame | Detected | **No**: posters are never tracked in the camera frame (it moves with the robot) |
| 1 – 5 + `odom → … → camera` | Position in `odom` | Detected | Yes, cell placed in `odom` (re-reported in `map` when SLAM is up), depth test only |
| 1 – 6 | Position in `map` | Detected | Yes, cell placed in `map`, depth test only |
| 1 – 7 (all) | Position in `map` | Detected | Yes, depth test + static-map test |

People get no position in `/humans/detections` (`human_detector` runs with `use_depth:=false`):
poster positions come from `poster_detector`, which does the depth test anyway.

## Not from the robot, but needed

- YOLO weights `yolo11n.pt` in `~/.cache/team_perception/` on the robot (`install_deps.sh`
  downloads them; without internet, copy the file there).
- On the robot: write access to `~/qr_logs/` and `~/poster_logs/` for the snapshots and CSV logs.
