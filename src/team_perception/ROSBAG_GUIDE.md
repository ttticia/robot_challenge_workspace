# Recording Sensor Data on the Lite3 (rosbag)

How to record the robot's compressed camera, depth and lidar while teleoperating, check the recording, and turn the camera stream into a viewable `.mp4`.

**Machine:** robot computer `192.168.10.74`, ROS 2 Humble.

---

## 1. Before you start

Open a terminal on the robot computer (VS Code terminal or `ssh`) and load ROS:

```bash
source /opt/ros/humble/setup.bash
```

Do this in **every new terminal**.

Check that the sensors are publishing:

```bash
ros2 topic list | grep -i -e image -e camera_info -e depth -e livox -e scan
ros2 topic hz /camera/color/image_raw/compressed     # Ctrl+C after a few lines
ros2 topic hz /livox/points
```

If a topic is missing, its driver isn't running, so start the robot first.

### Topics to record

| Topic | Type | Use |
|---|---|---|
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | Color camera, compressed (small files) |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Color intrinsics, needed for 3-D positions |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth image, uncompressed (large) |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth intrinsics |
| `/livox/points` | `sensor_msgs/PointCloud2` | Livox lidar point cloud |
| `/tf`, `/tf_static` | TF | Frames between camera, lidar and base. Needed to use depth/lidar together |

**Depth: check for aligned depth.** `/camera/depth/image_raw` is **not** aligned to the color image (see [INTEGRATION.md](INTEGRATION.md)). If the driver publishes an aligned topic (for example `/camera/aligned_depth_to_color/image_raw`), record that one too. Find it with `ros2 topic list | grep depth`.

**Optional extras:** `/odom` and `/imu/data` (robot motion), `/scan` (2-D laser scan, if published).

Avoid `/camera/color/image_raw` (uncompressed color). It fills the disk very quickly.

**Lidar vs depth.** `/livox/points` is 3-D (x, y, z in metres, so distance is built in), but
sparse: about 20,000 points per scan over 360°, only a small part inside the camera view, and
not pixel-aligned with the colour image. To get depth for image pixels, project the points
with `/camera/color/camera_info` and `/tf_static`; you get scattered dots, not a dense image.
For dense per-pixel depth you need `/camera/depth/image_raw`. Don't record
`/camera/depth/points` instead: it's even larger.

> **Recording while the robot drives itself (Nav2, `nav_straight`):** leave out
> `/camera/depth/image_raw`. On 2026-11-13, recording it during a `nav_straight` run made
> `/livox/points` arrive about 1.2 s late (normal ≈ 0.14 s); collision_monitor ignored the
> stale lidar and the robot never moved. Use the lighter command in section 2, or record
> depth only during teleop runs.

---

## 2. Record while teleoping

You need **two terminals**.

**Terminal 1: teleop.** Use your usual setup, for example:

```bash
ros2 launch transfer transfer_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keyboard teleop only responds while its terminal is focused.

**Terminal 2: start recording.**

All recordings go in `~/challenge_ws/ROS_bags/`. Create it once if it's missing:

```bash
mkdir -p ~/challenge_ws/ROS_bags
```

Record camera, depth and lidar:

```bash
ros2 bag record -o ~/challenge_ws/ROS_bags/<name> \
  /camera/color/image_raw/compressed \
  /camera/color/camera_info \
  /camera/depth/image_raw \
  /camera/depth/camera_info \
  /livox/points \
  /tf /tf_static
```

Add the aligned depth topic or the optional extras to the end of the list if you want them.

**Lighter version, while Nav2 / `nav_straight` is driving** (no raw depth):

```bash
ros2 bag record -o ~/challenge_ws/ROS_bags/<name> \
  /camera/color/image_raw/compressed /camera/color/camera_info \
  /livox/points /tf /tf_static
```

Untested whether this keeps the lidar lag under 0.5 s. Check during the recording with
`ros2 topic delay /livox/points` (should stay around 0.1–0.3 s).

- Start recording **before** you start driving.
- Press **Ctrl+C** in Terminal 2 to stop.
- Use a **new name** each time (`run1`, `run2`, …). The command fails if the folder already exists.
- Depth and lidar make bags large (roughly 1 GB per minute or more). Keep runs short and check free space with `df -h ~` first.

The recording is saved as a folder (`~/challenge_ws/ROS_bags/run1/`) containing a `.db3` file and `metadata.yaml`.

**Camera only** (small bag, for video):

```bash
ros2 bag record -o ~/challenge_ws/ROS_bags/<name> /camera/color/image_raw/compressed
```

---

## 3. Check the recording

```bash
ros2 bag info ~/challenge_ws/ROS_bags/<name>
```

Every recorded topic should have a **count above 0**. As a rough guide, the camera and depth counts should be about 30 × duration in seconds, and `/livox/points` about 10 × duration. A topic with `Count: 0` had the wrong name or wasn't publishing.

Forgot the folder name? List recordings, newest first:

```bash
ls -dt ~/challenge_ws/ROS_bags/*/metadata.yaml | xargs -n1 dirname
```

To replay a bag (for example, to test a perception node against it):

```bash
ros2 bag play ~/challenge_ws/ROS_bags/<name>
```

---

## 4. Convert the camera to video

```bash
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/<name>
```

This creates `~/challenge_ws/ROS_bags/<name>.mp4` (H.264, real-time speed) from `/camera/color/image_raw/compressed`. Depth and lidar are not included in the video.

Optional: choose the output name and/or a different camera topic:

```bash
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/run1 ~/challenge_ws/ROS_bags/front_cam.mp4 /image_raw
```

---

## 5. Watch the video / get the bag

The robot computer has no screen, so use your own computer:

- **VS Code:** click the `.mp4` in the Explorer sidebar to play it. Right-click a file or bag folder and choose **Download…** to save a copy.
- **Browser:** open **http://192.168.10.74:8080** (FileBrowser), log in, and click the file to play or download it.
- **Copy a whole bag** to your computer: `scp -r <user>@192.168.10.74:~/challenge_ws/ROS_bags/run1 .`

---

## Quick reference

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/challenge_ws/ROS_bags
ros2 bag record -o ~/challenge_ws/ROS_bags/run1 \
  /camera/color/image_raw/compressed /camera/color/camera_info \
  /camera/depth/image_raw /camera/depth/camera_info \
  /livox/points /tf /tf_static                         # Ctrl+C to stop
ros2 bag info ~/challenge_ws/ROS_bags/run1             # every topic Count > 0
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/run1 # -> ~/challenge_ws/ROS_bags/run1.mp4
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `invalid choice: 'mcap'` | Remove `-s mcap` from the command, or install it with `sudo apt install ros-humble-rosbag2-storage-mcap`. |
| A topic has `Count: 0` in `ros2 bag info` | Wrong topic name or driver not running. Re-check with `ros2 topic list` and `ros2 topic hz <topic>`. |
| `/livox/points` missing | The Livox driver isn't running. Start the robot's lidar driver, then check `ros2 topic hz /livox/points`. |
| Depth doesn't line up with the color image | `/camera/depth/image_raw` isn't aligned. Record the aligned depth topic, or turn on `depth_registration` in the camera driver. See [INTEGRATION.md](INTEGRATION.md). |
| Dropped messages / recorder warnings | Disk or CPU can't keep up. Record fewer topics, or try `--max-cache-size 1000000000`. |
| Robot won't move under Nav2 while recording; collision_monitor `Ignoring the source` | Recording raw depth delays `/livox/points` (~1.2 s seen). Stop the recording or drop `/camera/depth/image_raw`. |
| `could not connect to display` / Qt platform plugin error | GUI tools (e.g. `rqt_image_view`, `rviz2`) don't work over SSH/VS Code. Convert to `.mp4` (step 4), or copy the bag to a computer with a screen. |
| `Package 'image_view' not found` | Not needed. Use `bag_to_video.py`. |
| `Topic ... not in bag` (bag_to_video.py) | The script lists the topics that are in the bag. Pass one of them as the 3rd argument. |
| `ros2: command not found` | Run `source /opt/ros/humble/setup.bash`. |
| Disk filling up | Check with `df -h`. Don't record uncompressed color. Delete or copy off old bag folders. |
