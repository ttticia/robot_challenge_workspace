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
source ~/challenge_ws/runs/ROS_bags/aliases.sh     # add both lines to ~/.bashrc to make them permanent
```

Check that the sensors are publishing:

```bash
ros2 topic list | grep -i -e image -e camera_info -e depth -e livox -e scan
ros2 topic hz /camera/color/image_raw/compressed     # Ctrl+C after a few lines
ros2 topic hz /livox/points                          # before maze_nav only; stop it (Ctrl+C)
#                                                      before launching: once lidar_relay runs,
#                                                      check /livox/points_local instead
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
| **D. Lidar and IMU** | `/livox/points_local` (lidar_relay's copy of `/livox/points`), `/livox/lidar`, `/livox/imu`, `/imu/data`, `/leg_odom2` | 10 Hz / ~190 Hz | Re-running Fast-LIO (needs `/livox/lidar` + `/livox/imu`), rebuilding `/scan`, comparing with leg odometry |
| **E. Localization and map** | `/odom`, `/scan`, `/map` | 10 Hz, `/map` occasional | Positions in `map`, the static-map poster test, Nav2 replay. `/map` needs the QoS override below |
| **F. Navigation and commands** | `/plan`, `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/nav_cmd_vel`, `/cmd_vel`, `/simple_cmd`, `/handle_state` | ≤ 20 Hz | Why the robot moved or didn't: planner → controller → smoother → collision_monitor → bridge |
| **G. Perception results** | `/found`, `/qr/detections`, `/posters/detections`, `/humans/detections`, `/camera/color/image_offboard/compressed` | small | What the robot decided live. `/camera/color/image_offboard/compressed` is left over from the removed external-computer mode: nothing publishes it any more, so it records nothing |
| **H. Maze explorer** | `/maze/status`, `/maze/goal`, `/wall_scan` | small | What `maze_nav`'s explorer was doing (state, current frontier goal, wall slice fed to SLAM) |

**Not recorded by any preset yet** (added to `maze_nav` / `team_perception` since the presets
were written): `/stairs/status`, `/stairs/detection`, `/stairs/pose` (stair line-up),
`/scene/trigger`, `/junction_detected` (scene trigger), `/qr/read`, `/qr/unread`,
`/perception/look_request` (explorer `FACING_QR` / `LOOKING`). All are small. Add them after
`--` for now, e.g. `rec_maze run1 -- /stairs/status /scene/trigger /qr/unread /qr/read
/perception/look_request`, or add them to groups G/H in [record.sh](record.sh).

Groups E–H only have messages while the matching launch is running: `/tf` to `map`, `/odom`,
`/scan` and `/map` need Fast-LIO and `slam_toolbox` (from `lite3_launcher` or `maze_nav`); group F needs
Nav2; group G needs the perception launch; group H needs `maze_nav`. `/tf` is **empty without
Fast-LIO**. Start those before recording (see the checks below).

**Don't record:** `/camera/color/image_raw` (uncompressed colour) or `/camera/depth/points`
(point cloud from depth). Both fill the disk very quickly and can be rebuilt from groups B/C.
The onboard perception nodes publish no debug images; QR and poster snapshots are saved to
`~/qr_logs/` and `~/poster_logs/` instead.

#### `/map` and `/tf_static`: QoS override

`/map` and `/tf_static` are published **once** (transient local), often before recording
starts. Without an override the recorder misses them, and a replay has no map and no
camera/lidar mounting frames. The `rec_*` aliases create
`~/challenge_ws/runs/ROS_bags/qos_overrides.yaml` the first time and pass it to the recorder for you
Its contents, for reference:

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
ros2 topic hz /livox/points_local                                      # ~10 Hz (lidar_relay)
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
> stale lidar and the robot never moved. Use `rec_maze` (section 2.2), or record depth only
> during teleop runs. `rec_full` and `rec_curtain` refuse to start while Nav2 is running.

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
| Full `maze_nav` run: navigation + perception | `rec_maze` | A, B, D–H | **no** | [2.2](#22-autonomous-run-rec_maze) |
| Debug navigation (why it stopped or turned) | `rec_navdebug` | A, lidar, E, F | no | [2.3](#23-navigation-debug-rec_navdebug) |
| Re-run Fast-LIO / SLAM offline | `rec_lidar` | A, D | no | [2.4](#24-raw-sensors-for-fast-lio--slam-rec_lidar) |
| Curtain YOLO training data (teleop) | `rec_curtain` | A, B, C, lidar, `/odom` | yes | [2.5](#25-curtain-yolo-training-data-rec_curtain) |

Every alias works the same way:

```bash
rec_maze                    # bag named maze_<date>_<time>, e.g. runs/ROS_bags/maze_20261114_153000
rec_full teleop_run2        # choose the name yourself
rec_maze maze_try3 -- /stairs/status /scene/result   # name + extra topics after --
rec_maze -n                 # print the full ros2 bag record command, don't record
rec_list                    # every preset and its exact topics
rec_last                    # ros2 bag info for the newest bag
rec_rename maze_ok          # rename the newest bag to maze_ok
rec_rename maze_20261114_153000 maze_try3   # rename any bag
```

**Names:** use letters, digits, `.`, `_` and `-` only (no spaces or `/`), for example
`maze_try3` or `poster-test.2`. Leave the name out to get `<preset>_<date>_<time>`, which
never clashes.

Before recording, the alias:
- creates `runs/ROS_bags/` and `qos_overrides.yaml` if they're missing
- refuses a name that already exists
- warns if less than 5 GB is free
- refuses depth while Nav2 is running (override with `ALLOW_DEPTH_WITH_NAV2=1`, not recommended)

After Ctrl+C it prints the duration, size and any topic with `Count: 0`, then asks for a
name:

```
Name for this bag (Enter keeps 'maze_20261114_153000'): maze_try3_exited
Renamed: ~/challenge_ws/runs/ROS_bags/maze_20261114_153000 -> ~/challenge_ws/runs/ROS_bags/maze_try3_exited
```

Press **Enter** to keep the name. This is handy for naming a bag after you know how the run
went. A bad or already-used name is refused, and you are asked again. To skip the question
(for example in scripts), set `REC_ASK_NAME=0`, e.g. `REC_ASK_NAME=0 rec_maze`.

Without the aliases, run the script directly: `~/challenge_ws/runs/ROS_bags/record.sh <preset> [name]`
(rename: `record.sh --rename [old] new`).
To record a set of topics no preset covers, print the nearest one with `-n`, then copy and
edit the command.

#### 2.1 Full run (teleop): `rec_full`

Everything, for replaying navigation and perception together. Teleop only, because of depth.

#### 2.2 Autonomous run: `rec_maze`

The same as `rec_full` without depth, plus the `maze_nav` explorer topics (group H), so the
lidar isn't delayed and collision_monitor doesn't stop the robot. Navigation and perception are
recorded together in one bag: lidar, localization, map, plan and velocity chain, plus the
colour camera and every detection. Use it for any autonomous run (Nav2 alone too).
**Lidar topic (all presets).** Every preset records `/livox/points_local` (maze_nav
`lidar_relay`'s copy, same messages), never the raw `/livox/points`: a second subscriber on the
raw topic pulls another full copy across the bridge and makes the lidar and Fast-LIO odom late
(2026-11-20 05:29: worst 0.67 s). The recorder subscribes once the topic appears, so starting
the recording before maze_nav is fine. Teleop bags **without maze_nav** have no
`/livox/points_local`: record those with `RAW_LIDAR=1 rec_...` (raw topic). Replaying for tools
that expect the raw name: `ros2 bag play <bag> --remap /livox/points_local:=/livox/points`.

Untested whether this keeps the lidar lag under 0.5 s. Check during the recording with
`ros2 topic delay /livox/points_local` (should stay around 0.1–0.3 s; not the raw topic: that
is one more bridge copy while it runs). If it doesn't, use
`rec_navdebug` instead, which leaves out the camera and the raw lidar/IMU.

##### Full maze run, step by step

A complete challenge run: `maze_nav` drives, perception reports codes and posters, and one bag
records it all. Use three terminals on the robot computer.

1. **Terminal 1: start navigation.** Do the clash checks in
   [nav_launch.md](../src/maze_nav/nav_launch.md) section 1 first. Perception, YOLO included,
   runs on the robot ([launch_guide.md](../src/team_perception/launch_guide.md)).
   ```bash
   cd ~/challenge_ws && source install/setup.bash
   ros2 launch maze_nav maze_nav.launch.py        # also starts perception (perception:=true)
   ```
   Wait until `/maze/status` reads `PAUSED`: `ros2 topic echo /maze/status`. The operator
   sends `lite3_auto` once the log says `gait sent`.

2. **Terminal 2: start recording.** Start it after `PAUSED`, so that SLAM, TF to `map` and
   `/map` are up, and **before** `/maze/start`:
   ```bash
   rec_maze maze_run1
   ```
   Don't use `rec_full` or `rec_curtain` here (raw depth stalls Nav2; both refuse to start
   anyway).

3. **Terminal 3: check the lidar delay, then start the run.**
   ```bash
   ros2 topic delay /livox/points_local  # ~0.1-0.3 s; Ctrl+C after a few lines
   ros2 topic pub --once /maze/start std_msgs/msg/Empty
   ```
   If the delay stays above 0.5 s, stop the recording and use
   `rec_navdebug maze_run1 -- /maze/status /maze/goal /found /qr/detections /posters/detections`
   instead.

4. **During the run** (Terminal 3, or a laptop with `ROS_DOMAIN_ID=4`):
   ```bash
   ros2 topic echo /maze/status          # CREEPING -> SCANNING -> EXPLORING -> EXITED / NO_EXIT / STOPPED
   ros2 topic echo /found                # QR found / Human found
   ```

5. **Stop.** Once the status reads `EXITED`, `NO_EXIT` or `STOPPED` (or you stop the robot):
   Ctrl+C in **Terminal 2** first. The recorder prints the summary; check that no topic you
   need shows `Count: 0`. Name the bag after the outcome, for example `maze_run1_exited`.
   Then press Ctrl+C in **Terminal 1**, which saves the map.

6. **Keep the run's other files with the bag.** The map, QR snapshots and logs aren't in the
   bag. Right after the run, copy everything written since the recording started into
   `<bag>_extras/`:
   ```bash
   BAG=~/challenge_ws/runs/ROS_bags/maze_run1_exited
   START=$(awk '/nanoseconds_since_epoch/{print int($2/1e9); exit}' $BAG/metadata.yaml)
   mkdir -p ${BAG}_extras
   (cd ~ && find challenge_ws/maps qr_logs poster_logs challenge_ws/logs/cpu \
        -type f -newermt "@$START" -exec cp --parents {} ${BAG}_extras/ \; 2>/dev/null)
   ls -R ${BAG}_extras
   ```
   | What | Where |
   |---|---|
   | Final map | `~/challenge_ws/maps/maze_<date>_<time>.{pgm,yaml}` |
   | QR snapshots (annotated JPEG) + CSV | `~/qr_logs/` |
   | Poster CSV | `~/poster_logs/` |
   | CPU per process + summary (`cpu_log`) | `~/challenge_ws/logs/cpu/run_<date>_<time>{.csv,_summary.txt}` |
   | Full node logs | `~/.ros/log/<date>-<time>-…/` (the folder `ros2 launch` printed at startup) |

   `rec_rename` doesn't rename `_extras/`, so rename it by hand if you rename the bag.

#### 2.3 Navigation debug: `rec_navdebug`

Why the robot moved, stopped or turned: lidar, localization, the plan and every stage of the
velocity chain. Small enough to leave running during autonomous runs.

#### 2.4 Raw sensors for Fast-LIO / SLAM: `rec_lidar`

The inputs to re-run Fast-LIO and `slam_toolbox` offline. On replay, leave `/tf` out of
`--topics` so it doesn't clash with the `odom → base_link` that Fast-LIO publishes again.

#### 2.5 Curtain YOLO training data: `rec_curtain`

Bags for training the curtain YOLO model (`maze_nav/curtain_yolo.py`; plan in
`src/to_do.md`, "YOLO curtain detector"). It records what `maze_nav/tools/curtain_yolo_frames.py`
reads: compressed colour + `camera_info`, raw depth + `camera_info`, `/tf` + `/tf_static` (camera
pose), and the lidar for its `--lidar` draft labels. Also `/odom` (Fast-LIO, only with maze_nav),
`/leg_odom2` (the robot's own odometry), `/imu/data` (body pitch / roll while walking) and
`/handle_state` + `/simple_cmd` (controller sticks and mode), to tell moving / blurred frames
from still ones. Teleop only (manual mode with the controller is fine): it records raw depth,
so it refuses to start while Nav2 runs. The lidar is recorded as raw `/livox/points` by itself
when maze_nav's `lidar_relay` isn't running (no `RAW_LIDAR=1` needed). It warns before
recording if the camera or lidar isn't publishing, and after Ctrl+C if the bag is missing one
of them. About 1.5 GB/min, so keep each bag to one approach of 1-2 minutes.

```bash
rec_curtain curtain_spec_square_lit_01        # name: curtain, angle, lighting, number
rec_curtain neg_extru_cavity_01               # negatives too (walls, cavities, doorways)
nice -n 19 python3 src/maze_nav/tools/curtain_yolo_frames.py ~/challenge_ws/runs/ROS_bags/<bag> \
    ~/challenge_ws/datasets/curtain --every 10 --lidar --cloud-topic /livox/points
    # --max-gap 0.25 for a curtain hanging ~20 cm up; /livox/points_local for a bag
    # recorded with maze_nav running
```

What to record: the competition curtains (both halves, closed with the 1-3 cm slit, parted,
swaying; also in front of the raised platform). Detection has to work from **1.5 m down to
0.3 m**, so approach slowly from about 2 m to 0.3 m and spend most of the bag in that range:
square on and 30-45° off, from both sides, lit and in shadow. Record negatives too: plain walls,
extrusion walls with cavities or windows, doorways, slats, open floor, people. Spread the bags
over several sessions and days.

#### While recording

- Start recording **before** you start driving, and stop with **Ctrl+C** in Terminal 2.
- Depth and lidar make bags large (roughly 1 GB per minute or more). Keep runs short.
- The recording is saved as a folder (for example `~/challenge_ws/runs/ROS_bags/maze_20261114_153000/`)
  containing a `.db3` file and `metadata.yaml`.

---

## 3. Check the recording

The alias prints a short summary when you stop it. For the full listing:

```bash
rec_last                                       # newest bag
ros2 bag info ~/challenge_ws/runs/ROS_bags/<name>   # any bag
```

Every recorded topic should have a **count above 0**. As a rough guide, the camera and depth counts should be about 30 × duration in seconds, and `/livox/points_local` (or `/livox/points` with `RAW_LIDAR=1`) about 10 × duration. A topic with `Count: 0` had the wrong name or wasn't publishing.

Forgot the folder name? List recordings, newest first:

```bash
ls -dt ~/challenge_ws/runs/ROS_bags/*/metadata.yaml | xargs -n1 dirname
```

### Rename a recording

```bash
rec_rename maze_try3                       # newest bag -> maze_try3
rec_rename maze_20261114_153000 maze_try3   # a specific bag
```

Use `rec_rename` rather than `mv`. It renames the folder, the `.db3` files inside,
`metadata.yaml`, and the matching `.mp4` if you already made one, so the names stay
consistent and `ros2 bag info` / `ros2 bag play` keep working. It refuses a name that's
already used and a bag that's still being recorded.

To replay a bag (for example, to test a perception node against it):

```bash
ros2 bag play ~/challenge_ws/runs/ROS_bags/<name> \
  --qos-profile-overrides-path ~/challenge_ws/runs/ROS_bags/qos_overrides.yaml
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
python3 ~/bag_to_video.py ~/challenge_ws/runs/ROS_bags/<name>
```

This creates `~/challenge_ws/runs/ROS_bags/<name>.mp4` (H.264, real-time speed) from `/camera/color/image_raw/compressed`. Depth and lidar are not included in the video.

Optional: choose the output name and/or a different camera topic:

```bash
python3 ~/bag_to_video.py ~/challenge_ws/runs/ROS_bags/run1 ~/challenge_ws/runs/ROS_bags/front_cam.mp4 /image_raw
```

---

## 5. Watch the video / get the bag

The robot computer has no screen, so use your own computer:

- **VS Code:** click the `.mp4` in the Explorer sidebar to play it. Right-click a file or bag folder and choose **Download…** to save a copy.
- **Browser:** open **http://192.168.10.74:8080** (FileBrowser), log in, and click the file to play or download it.
- **Copy a whole bag** to your computer: `scp -r User@192.168.10.74:~/challenge_ws/runs/ROS_bags/run1 .`

---

## Quick reference

```bash
rec_list                     # presets and topics
rec_full run1                # teleop, everything (Ctrl+C to stop)
rec_maze maze_run1           # full maze_nav run: navigation + perception, no depth (section 2.2)
rec_last                     # check: every topic Count > 0
rec_rename run1_good         # rename the newest bag (or: rec_rename <old> <new>)
python3 ~/bag_to_video.py ~/challenge_ws/runs/ROS_bags/run1   # -> run1.mp4
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| A topic has `Count: 0` in `ros2 bag info` | Wrong topic name or driver not running. Re-check with `ros2 topic list` and `ros2 topic hz <topic>`. |
| `/map` or `/tf_static` has `Count: 0` | The recorder started after their one message. The `rec_*` aliases pass the QoS override for you; if you ran `ros2 bag record` by hand, add `--qos-profile-overrides-path` (section 1). If the map is still missing, save it while SLAM runs: `ros2 run nav2_map_server map_saver_cli -f ~/challenge_ws/runs/ROS_bags/<name>_map` |
| `/tf` has `Count: 0` | Fast-LIO wasn't running. Start `lite3_bringup` (from `lite3_launcher`) before recording |
| `/livox/points` missing | The Livox driver isn't running. Start the robot's lidar driver, then check `ros2 topic hz /livox/points`. |
| Depth doesn't line up with the color image | `/camera/depth/image_raw` isn't aligned. Record the aligned depth topic, or turn on `depth_registration` in the camera driver. See [data_required.md](../src/team_perception/data_required.md). |
| `Topic '...' has more than one type associated` (repeats every 0.1 s) | Another node uses the topic with a different message type, usually an `rqt_image_view` on a laptop subscribed to a `.../compressed` topic as a raw `Image`. Find it with `ros2 topic info -v <topic>` (look at each `Topic type`). Close it, or reopen it and pick the topic from the dropdown (`rqt_image_view` with no argument). Topics the recorder had already subscribed keep recording; a topic that first appears during the conflict is skipped. |
| Dropped messages / recorder warnings | Disk or CPU can't keep up. Record fewer topics, or try `--max-cache-size 1000000000`. |
| Robot won't move under Nav2 while recording; collision_monitor `Ignoring the source` | Recording raw depth delays `/livox/points` (~1.2 s seen). Stop the recording and use `rec_nav` or `rec_navdebug`. |
| `could not connect to display` / Qt platform plugin error | GUI tools (e.g. `rqt_image_view`, `rviz2`) don't work over SSH/VS Code. Convert to `.mp4` (step 4), or copy the bag to a computer with a screen. |
| `Package 'image_view' not found` | Not needed. Use `bag_to_video.py`. |
| `Topic ... not in bag` (bag_to_video.py) | The script lists the topics that are in the bag. Pass one of them as the 3rd argument. |
| `ros2: command not found` | Run `source /opt/ros/humble/setup.bash`. |
| `rec_maze: command not found` | Run `source ~/challenge_ws/runs/ROS_bags/aliases.sh` (section 1). |
| `... already exists. Pick another name.` | Give a new name, or leave the name out to get a timestamped one. |
| `Bad name '...'` | Names may only use letters, digits, `.`, `_` and `-`. Replace spaces with `_`. |
| Renamed a bag folder with `mv` and the files inside still have the old name | Harmless (`metadata.yaml` refers to the files by their own names), but to tidy up rename it back and use `rec_rename` |
| `rec_rename: command not found` | Open a new terminal or `source ~/challenge_ws/runs/ROS_bags/aliases.sh` (alias added 2026-11-14). |
| `Nav2 is running and preset ... records raw depth` | Use `rec_maze`, or stop Nav2 first. |
| Disk filling up | Check with `df -h`. Don't record uncompressed color. Delete or copy off old bag folders. |
