# Recording Sensor Data on the Lite3 (rosbag)

How to record the robot's camera, depth, lidar and navigation topics with the `rec_*` aliases, check the recording, and turn the camera stream into a viewable `.mp4`.

**Machine:** robot computer `192.168.10.74`, ROS 2 Humble.

---

## 1. Before you start

Open a terminal on the robot computer (VS Code terminal or `ssh`). On the robot,
`~/.bashrc` already loads ROS and the recording aliases. Check that they're there:

```bash
rec_list            # prints the presets; "command not found" -> see below
```

If it isn't found (a new machine, or an old terminal), load them:

```bash
source /opt/ros/humble/setup.bash
source ~/challenge_ws/ROS_bags/aliases.sh     # add both lines to ~/.bashrc to make them permanent
```

Check that the sensors are publishing:

```bash
ros2 topic list | grep -i -e image -e camera_info -e depth -e livox -e scan
ros2 topic hz /camera/color/image_raw/compressed     # Ctrl+C after a few lines
ros2 topic hz /livox/points
```

If a topic is missing, its driver isn't running, so start the robot first.

### Topics to record

Topics come in groups. Each alias in section 2 records a fixed set of groups, always
including A. A topic that isn't being published just records `Count: 0`, so extra topics do
no harm, apart from depth (see the warning below). The groups are defined once, at the top
of [record.sh](record.sh); change topics there.

| Group | Topics | Rate | Needed for |
|---|---|---|---|
| **A. Always** | `/tf`, `/tf_static`, `/rosout` | small | Frames between map, odom, base, lidar and camera. Warnings from every node |
| **B. Colour camera** | `/camera/color/image_raw/compressed`, `/camera/color/camera_info` | 30 Hz | QR/poster replay, `.mp4` video, 3-D positions |
| **C. Depth** (large) | `/camera/depth/image_raw`, `/camera/depth/camera_info` | 30 Hz | Dense per-pixel depth, poster flatness test. **Teleop runs only** |
| **D. Lidar and IMU** | `/livox/points`, `/livox/lidar`, `/livox/imu`, `/imu/data`, `/leg_odom2` | 10 Hz / ~190 Hz | Re-running Fast-LIO (needs `/livox/lidar` + `/livox/imu`), rebuilding `/scan`, comparing with leg odometry |
| **E. Localization and map** | `/odom`, `/scan`, `/map` | 10 Hz, `/map` occasional | Positions in `map`, the static-map poster test, Nav2 replay. `/map` needs the QoS override below |
| **F. Navigation and commands** | `/plan`, `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/nav_cmd_vel`, `/cmd_vel`, `/simple_cmd`, `/handle_state` | ≤ 20 Hz | Why the robot moved or didn't: planner → controller → smoother → collision_monitor → bridge |
| **G. Perception results** | `/found`, `/qr/detections`, `/posters/detections`, `/humans/detections`, `/camera/color/image_offboard/compressed` | small | What the robot decided live. Shows a link failure apart from a logic failure |

Groups E–G only have messages while the matching launch is running: `/tf` to `map`, `/odom`,
`/scan` and `/map` need Fast-LIO and `slam_toolbox` (from `lite3_launcher`); group F needs
Nav2; group G needs the perception launch. `/tf` is **empty without
Fast-LIO**. Start those before recording (see the checks below).

**Don't record:** `/camera/color/image_raw` (uncompressed colour) or `/camera/depth/points`
(point cloud from depth). Both fill the disk very quickly and can be rebuilt from groups B/C.
The onboard perception nodes publish no debug images; QR and poster snapshots are saved to
`~/qr_logs/` and `~/poster_logs/` instead.

#### `/map` and `/tf_static`: QoS override

`/map` and `/tf_static` are published **once** (transient local), often before recording
starts. Without an override the recorder misses them, and a replay has no map and no
camera/lidar mounting frames. The `rec_*` aliases create
`~/challenge_ws/ROS_bags/qos_overrides.yaml` the first time and pass it to the recorder for you
(except `rec_video`, which doesn't need it). Its contents, for reference:

```yaml
/map:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
/tf_static:
  history: keep_last
  depth: 100          # one message per static publisher: keep them all
  reliability: reliable
  durability: transient_local
```

Pass it to `ros2 bag play` yourself (section 3). On replay, start the nodes **first**, then
play the bag.

#### Check before recording

```bash
ros2 topic info /map --verbose | grep -E "Publisher count|Durability"  # >= 1, TRANSIENT_LOCAL
ros2 run tf2_ros tf2_echo map camera_color_optical_frame               # prints a transform each second
ros2 topic hz /livox/points                                            # ~10 Hz
```

If `tf2_echo` says the `map` frame doesn't exist, SLAM isn't running and the bag will have no
`map` positions. The map should be built **without people in it**, otherwise the poster
map test treats them as walls.

**Depth: check for aligned depth.** `/camera/depth/image_raw` is **not** aligned to the color image (see [data_required.md](../src/team_perception/data_required.md)). If the driver publishes an aligned topic (for example `/camera/aligned_depth_to_color/image_raw`), record that one too by adding it after `--` (for example
`rec_full -- /camera/aligned_depth_to_color/image_raw`). Find it with `ros2 topic list | grep depth`.

**Lidar vs depth.** `/livox/points` is 3-D (x, y, z in metres, so distance is built in), but
sparse: about 20,000 points per scan over 360°, only a small part inside the camera view, and
not pixel-aligned with the colour image. To get depth for image pixels, project the points
with `/camera/color/camera_info` and `/tf_static`; you get scattered dots, not a dense image.
For dense per-pixel depth you need `/camera/depth/image_raw`. Don't record
`/camera/depth/points` instead: it's even larger.

> **Recording while the robot drives itself (Nav2):** leave out
> `/camera/depth/image_raw`. On 2026-11-13, recording it during an autonomous run made
> `/livox/points` arrive about 1.2 s late (normal ≈ 0.14 s); collision_monitor ignored the
> stale lidar and the robot never moved. Use `rec_nav` (section 2.2), or record depth only
> during teleop runs. `rec_full` and `rec_perception` refuse to start while Nav2 is running.

---

## 2. Record a run

You need **two terminals**.

**Terminal 1: drive the robot.** Teleop, for example:

```bash
ros2 launch transfer transfer_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keyboard teleop only responds while its terminal is focused. For an autonomous run, start
your Nav2 or `maze_nav` launch here instead.

**Terminal 2: record.** Pick the alias for the run, start it **before** the robot moves, and
press **Ctrl+C** to stop:

| Run | Alias | Groups | Depth? | Section |
|---|---|---|---|---|
| Teleop, record everything | `rec_full` | A–G | yes | [2.1](#21-full-run-teleop-rec_full) |
| Nav2 / `maze_nav` driving | `rec_nav` | A, B, D–G | **no** | [2.2](#22-autonomous-run-rec_nav) |
| Debug posters / QR | `rec_perception` | A, B, C, E, G | yes | [2.3](#23-perception-debug-rec_perception-rec_detections) |
| Check detections only (small) | `rec_detections` | A, `/map`, G | no | [2.3](#23-perception-debug-rec_perception-rec_detections) |
| Debug navigation (why it stopped or turned) | `rec_navdebug` | A, lidar, E, F | no | [2.4](#24-navigation-debug-rec_navdebug) |
| Re-run Fast-LIO / SLAM offline | `rec_lidar` | A, D | no | [2.5](#25-raw-sensors-for-fast-lio--slam-rec_lidar) |
| Video only | `rec_video` | B (compressed colour) | no | [2.6](#26-camera-only-rec_video) |

Every alias works the same way:

```bash
rec_nav                     # bag named nav_<date>_<time>, e.g. ROS_bags/nav_20261114_153000
rec_full teleop_run2        # choose the name yourself
rec_nav maze_try3 -- /camera/aligned_depth_to_color/image_raw   # name + extra topics after --
rec_nav -n                  # print the full ros2 bag record command, don't record
rec_list                    # every preset and its exact topics
rec_last                    # ros2 bag info for the newest bag
rec_rename maze_ok          # rename the newest bag to maze_ok
rec_rename nav_20261114_153000 maze_try3   # rename any bag
```

**Names:** use letters, digits, `.`, `_` and `-` only (no spaces or `/`), for example
`maze_try3` or `poster-test.2`. Leave the name out to get `<preset>_<date>_<time>`, which
never clashes.

Before recording, the alias:
- creates `ROS_bags/` and `qos_overrides.yaml` if they're missing
- refuses a name that already exists
- warns if less than 5 GB is free
- refuses depth while Nav2 is running (override with `ALLOW_DEPTH_WITH_NAV2=1`, not recommended)

After Ctrl+C it prints the duration, size and any topic with `Count: 0`, then asks for a
name:

```
Name for this bag (Enter keeps 'nav_20261114_153000'): maze_try3_exited
Renamed: ~/challenge_ws/ROS_bags/nav_20261114_153000 -> ~/challenge_ws/ROS_bags/maze_try3_exited
```

Press **Enter** to keep the name. This is handy for naming a bag after you know how the run
went. A bad or already-used name is refused, and you are asked again. To skip the question
(for example in scripts), set `REC_ASK_NAME=0`, e.g. `REC_ASK_NAME=0 rec_nav`.

Without the aliases, run the script directly: `~/challenge_ws/ROS_bags/record.sh <preset> [name]`
(rename: `record.sh --rename [old] new`).
To record a set of topics no preset covers, print the nearest one with `-n`, then copy and
edit the command.

#### 2.1 Full run (teleop): `rec_full`

Everything, for replaying navigation and perception together. Teleop only, because of depth.

#### 2.2 Autonomous run: `rec_nav`

The same as `rec_full` without depth, so the lidar isn't delayed and collision_monitor
doesn't stop the robot.

Untested whether this keeps the lidar lag under 0.5 s. Check during the recording with
`ros2 topic delay /livox/points` (should stay around 0.1–0.3 s). If it doesn't, use
`rec_navdebug` instead, which leaves out the camera and the raw lidar/IMU.

#### 2.3 Perception debug: `rec_perception`, `rec_detections`

`rec_perception` (teleop) records what's needed to replay `qr_scanner` and `poster_detector`
and get the same results: colour, depth, both `camera_info`s, TF to `map`, the map, and what
the robot decided live.

`rec_detections` only checks *whether* detection worked. It is a much smaller bag, with
no camera images or depth, but it still has `/tf`, `/map` and the detection topics.

#### 2.4 Navigation debug: `rec_navdebug`

Why the robot moved, stopped or turned: lidar, localization, the plan and every stage of the
velocity chain. Small enough to leave running during autonomous runs.

#### 2.5 Raw sensors for Fast-LIO / SLAM: `rec_lidar`

The inputs to re-run Fast-LIO and `slam_toolbox` offline. On replay, leave `/tf` out of
`--topics` so it doesn't clash with the `odom → base_link` that Fast-LIO publishes again.

#### 2.6 Camera only: `rec_video`

Small bag for `bag_to_video.py` (section 4).

#### While recording

- Start recording **before** you start driving, and stop with **Ctrl+C** in Terminal 2.
- Depth and lidar make bags large (roughly 1 GB per minute or more). Keep runs short.
- The recording is saved as a folder (for example `~/challenge_ws/ROS_bags/nav_20261114_153000/`)
  containing a `.db3` file and `metadata.yaml`.

---

## 3. Check the recording

The alias prints a short summary when you stop it. For the full listing:

```bash
rec_last                                       # newest bag
ros2 bag info ~/challenge_ws/ROS_bags/<name>   # any bag
```

Every recorded topic should have a **count above 0**. As a rough guide, the camera and depth counts should be about 30 × duration in seconds, and `/livox/points` about 10 × duration. A topic with `Count: 0` had the wrong name or wasn't publishing.

Forgot the folder name? List recordings, newest first:

```bash
ls -dt ~/challenge_ws/ROS_bags/*/metadata.yaml | xargs -n1 dirname
```

### Rename a recording

```bash
rec_rename maze_try3                       # newest bag -> maze_try3
rec_rename nav_20261114_153000 maze_try3   # a specific bag
```

Use `rec_rename` rather than `mv`. It renames the folder, the `.db3` files inside,
`metadata.yaml`, and the matching `.mp4` if you already made one, so the names stay
consistent and `ros2 bag info` / `ros2 bag play` keep working. It refuses a name that's
already used and a bag that's still being recorded.

To replay a bag (for example, to test a perception node against it):

```bash
ros2 bag play ~/challenge_ws/ROS_bags/<name> \
  --qos-profile-overrides-path ~/challenge_ws/ROS_bags/qos_overrides.yaml
```

Start the nodes you are testing **before** `bag play`, so they receive the single `/map` and
`/tf_static` messages. Add `--topics <list>` to replay only some topics. Replay on a test
domain (`export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1`), never on domain 4, or the replay
reaches the robot.

---

## 4. Convert the camera to video

`bag_to_video.py` is not in this workspace. It lives in the team repo
([Robot-Dog-Grp4](https://github.com/ttticia/Robot-Dog-Grp4), branch `video-conversion`,
`challenge_files/onboard/team_perception/scripts/`). Download it once:

```bash
curl -fsSL -o ~/bag_to_video.py \
    https://raw.githubusercontent.com/ttticia/Robot-Dog-Grp4/video-conversion/challenge_files/onboard/team_perception/scripts/bag_to_video.py
```

(After the branch is merged, replace `video-conversion` with `main` in the URL.) Then:

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
rec_list                     # presets and topics
rec_full run1                # teleop, everything (Ctrl+C to stop)
rec_nav                      # Nav2 / maze_nav driving, no depth
rec_last                     # check: every topic Count > 0
rec_rename run1_good         # rename the newest bag (or: rec_rename <old> <new>)
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/run1   # -> run1.mp4
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| A topic has `Count: 0` in `ros2 bag info` | Wrong topic name or driver not running. Re-check with `ros2 topic list` and `ros2 topic hz <topic>`. |
| `/map` or `/tf_static` has `Count: 0` | The recorder started after their one message. The `rec_*` aliases pass the QoS override for you; if you ran `ros2 bag record` by hand, add `--qos-profile-overrides-path` (section 1). If the map is still missing, save it while SLAM runs: `ros2 run nav2_map_server map_saver_cli -f ~/challenge_ws/ROS_bags/<name>_map` |
| `/tf` has `Count: 0` | Fast-LIO wasn't running. Start `lite3_bringup` (from `lite3_launcher`) before recording |
| `/livox/points` missing | The Livox driver isn't running. Start the robot's lidar driver, then check `ros2 topic hz /livox/points`. |
| Depth doesn't line up with the color image | `/camera/depth/image_raw` isn't aligned. Record the aligned depth topic, or turn on `depth_registration` in the camera driver. See [data_required.md](../src/team_perception/data_required.md). |
| Dropped messages / recorder warnings | Disk or CPU can't keep up. Record fewer topics, or try `--max-cache-size 1000000000`. |
| Robot won't move under Nav2 while recording; collision_monitor `Ignoring the source` | Recording raw depth delays `/livox/points` (~1.2 s seen). Stop the recording and use `rec_nav` or `rec_navdebug`. |
| `could not connect to display` / Qt platform plugin error | GUI tools (e.g. `rqt_image_view`, `rviz2`) don't work over SSH/VS Code. Convert to `.mp4` (step 4), or copy the bag to a computer with a screen. |
| `Package 'image_view' not found` | Not needed. Use `bag_to_video.py`. |
| `Topic ... not in bag` (bag_to_video.py) | The script lists the topics that are in the bag. Pass one of them as the 3rd argument. |
| `ros2: command not found` | Run `source /opt/ros/humble/setup.bash`. |
| `rec_nav: command not found` | Run `source ~/challenge_ws/ROS_bags/aliases.sh` (section 1). |
| `... already exists. Pick another name.` | Give a new name, or leave the name out to get a timestamped one. |
| `Bad name '...'` | Names may only use letters, digits, `.`, `_` and `-`. Replace spaces with `_`. |
| Renamed a bag folder with `mv` and the files inside still have the old name | Harmless (`metadata.yaml` refers to the files by their own names), but to tidy up rename it back and use `rec_rename` |
| `rec_rename: command not found` | Open a new terminal or `source ~/challenge_ws/ROS_bags/aliases.sh` (alias added 2026-11-14). |
| `Nav2 is running and preset ... records raw depth` | Use `rec_nav`, or stop Nav2 first. |
| Disk filling up | Check with `df -h`. Don't record uncompressed color. Delete or copy off old bag folders. |
