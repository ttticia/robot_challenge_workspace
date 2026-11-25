# Recording bags on the Lite3

How to record a run with the `rec_*` aliases, check the bag, replay it and get a video or
frames out of it. Robot computer: `192.168.10.74`, ROS 2 Humble, `ROS_DOMAIN_ID=4`.

Cleaned up 2026-11-25: everything about the old `maze_nav` stack (`rec_full`, `rec_maze`,
`rec_navdebug`, `rec_lidar`, `rec_curtain`, their topic groups) is gone. It is still in git
history (`git log -- runs/ROS_bags/ROSBAG_GUIDE.md`) and on branch `curtain_cv`.

## 1. The aliases

`~/.bashrc` on the robot computer loads them. In a new terminal, or on another machine:

```bash
source /opt/ros/humble/setup.bash
source ~/challenge_ws/runs/ROS_bags/aliases.sh
```

| Alias | What it does |
|---|---|
| `rec_test [name]` | Record an autonomous `maze_explore` run (section 2.1) |
| `rec_manual [name]` | Record a hand-driven run with the mapping launch (section 2.2) |
| `rec_curtain [name]` | Record a curtain detection test driven by hand (section 2.3) |
| `rec_img [name]` | Record the colour camera only (section 2.4) |
| `rec_list` | Print each preset and its exact topics |
| `rec_last` | `ros2 bag info` of the newest bag |
| `rec_rename [old] new` | Rename a bag (`old` = the newest bag if left out) |
| `explore_check` | Sensor-rate go / no-go before a run (`src/maze_explore/preflight.sh`) |
| `explore_go` / `explore_stop` | Start / stop the explorer (`/nav/start`, `/explore/stop`) |
| `curtain_watch` / `curtain_view` | Run `curtain_node` on every frame / show its picture with the boxes in rqt |

All four recording aliases work the same way:

```bash
rec_test                     # bag named explore_<date>_<time>
rec_test run5                # choose the name
rec_test run5 -- /qr/read    # extra topics after --
rec_test -n                  # print the ros2 bag record command, record nothing
```

- Start recording **before** the robot moves. **Ctrl+C** stops it.
- After Ctrl+C it prints the duration, the size and every topic with `Count: 0`, then asks for
  a name. Enter keeps the name; `REC_ASK_NAME=0` skips the question.
- Names: letters, digits, `.`, `_`, `-`. A name that exists is refused.
- It warns when less than 5 GB is free.
- Bags go to `~/challenge_ws/runs/ROS_bags/<name>/` (a `.db3` file and `metadata.yaml`).
  They are never committed.
- The topics of each preset are defined in [record.sh](record.sh); change them there.

## 2. Record a run

### 2.1 Autonomous run: `rec_test`

```bash
explore_check                                   # terminal 1: must say GO
ros2 launch maze_explore explore.launch.py      # terminal 1
rec_test run5                                   # terminal 2, once the launch has settled
explore_go                                      # terminal 3
```

Recorded: `/tf`, `/tf_static`, `/rosout`, compressed colour, `/cloud_registered_body`
(Fast-LIO's cloud), `/odom`, `/scan`, `/map`, `/simple_cmd`, `/cmd_vel`,
`/explore/status`, `/explore/event`, `/explore/cells`, `/maze/body_height`.

Only the colour stream comes from the robot; the rest is published on this computer. Every
topic taken from the robot is one more copy across the bridge, so the raw lidar, the IMUs
and `camera_info` are left out on purpose.

### 2.2 Hand-driven run: `rec_manual`

Drive with the handheld controller while the mapping part of the stack runs, to collect
camera frames and a map.

```bash
# Terminal 1: robot standing still and level in the start cell (this pose is the map origin)
explore_check                                   # must say GO
ros2 launch maze_explore explore.launch.py explore:=false
# Terminal 2 (wait about 10 s)
rec_manual demo2
# before stopping terminal 1, if you want the map as a file (it is also in the bag as /map):
mkdir -p ~/challenge_ws/maps && ros2 run nav2_map_server map_saver_cli -f ~/challenge_ws/maps/demo2
```

Recorded: everything `rec_test` records, plus `/camera/color/camera_info`, the raw lidar
(`/livox/lidar`, `/livox/imu`, so Fast-LIO can be re-run), `/imu/data`, `/leg_odom2`,
`/handle_state` (sticks) and `/commands/joy_raw` (controller buttons).

### 2.3 Curtain detection test: `rec_curtain`

```bash
curtain_watch                 # terminal 1: curtain_node on every frame
rec_curtain curtain_t1        # terminal 2; drive with the controller; Ctrl+C to stop
```

Recorded: `/tf`, `/tf_static`, `/rosout`, compressed colour + `camera_info`, `/curtain/seen`
(one JSON per processed frame: boxes, confidence, direction), `/curtain/status`, and
`/odom`, `/scan`, `/map`, `/maze/body_height`, `/handle_state`, which only have messages
when the mapping launch or the controller is on. No depth. `/curtain/image` is not recorded:
it is raw, and the boxes can be drawn again from `/curtain/seen`.

It warns if `curtain_node` is not running, runs `explore_check` (with `curtain_node`
allowed), and after Ctrl+C prints the rates of the camera and of `/curtain/seen` (about 3 a
second). The whole procedure is in `src/maze_explore/launch_guide.md`, "Test curtain
detection by hand".

### 2.4 Camera only: `rec_img`

```bash
rec_img frames1
```

Records `/camera/color/image_raw/compressed` and nothing else. Needs no launch, skips the
sensor check, about 1 MB per second. Use it for dataset frames (section 4).

### 2.5 What `rec_test`, `rec_manual` and `rec_curtain` check for you

- **Before:** they run `explore_check` and ask before recording on a NO-GO (`REC_CHECK=0`
  skips it). `rec_manual` also warns when the mapping launch or a needed topic is missing.
- **After:** they print each stream's average rate and longest gap, and mark slow ones
  `SLOW`. A slow `/livox/imu` (needs about 200 Hz) means `/odom`, `/map` and the transforms
  in the bag are wrong; the camera frames and raw lidar are still good. Bag `demo`
  (2026-11-23) had `/livox/imu` at 8 Hz and its map is unusable.
- **Compression:** zstd, message by message (`REC_COMPRESS=0` = off). About 0.35 GB per
  minute instead of 0.5. It saves disk only; the streams still cross the bridge
  uncompressed. `ros2 bag info` / `play` and `rosbag2_py` (`SequentialCompressionReader`)
  read these bags as usual.
- **`/map` and `/tf_static`** are published once, often before recording starts. The aliases
  pass `qos_overrides.yaml` (created on first use) so the recorder still gets them.

### 2.6 Depth and `/livox/points`: not with the mapping stack

One subscriber to raw `/camera/depth/image_raw` or to `/livox/points`, even a single
BEST_EFFORT one, slows every stream from the robot to about 10 Hz (measured 2026-11-23:
`/imu/data` 195 → 11 Hz), which breaks Fast-LIO. The three presets above refuse those
topics (`ALLOW_HEAVY=1` overrides). There is no compressed depth topic.

For depth, record a separate short teleop bag with no launch running:

```bash
ros2 bag record -o ~/challenge_ws/runs/ROS_bags/<name> /camera/depth/image_raw \
    /camera/depth/camera_info /camera/color/image_raw/compressed /camera/color/camera_info /tf_static
```

Depth is not aligned to the colour image (different intrinsics): see
[data_required.md](../../src/team_perception/data_required.md).

Also keep the robot's domain quiet while recording: no second stack, no RViz showing depth,
no `ros2 topic hz` / `echo` on a big topic, no simulator on this computer.

## 3. Check, rename, replay

```bash
rec_last                                            # newest bag: every topic you need has Count > 0
ros2 bag info ~/challenge_ws/runs/ROS_bags/<name>   # any bag
rec_rename run5_stairs                              # rename the newest bag
rec_rename run5 run5_stairs                         # rename a given bag
```

Use `rec_rename`, not `mv`: it renames the folder, the `.db3` inside, `metadata.yaml` and a
matching video, so `ros2 bag info` / `play` keep working. It refuses a name in use and a bag
that is still being recorded.

Replay, on a test domain and never on domain 4 (the recorded `/cmd_vel` would reach the
robot):

```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
ros2 bag play ~/challenge_ws/runs/ROS_bags/<name> \
    --qos-profile-overrides-path ~/challenge_ws/runs/ROS_bags/qos_overrides.yaml
```

Start the nodes you are testing **before** `bag play`, so they get the single `/map` and
`/tf_static` messages. `--topics <list>` replays only some topics.

## 4. Video and frames

```bash
# side-by-side video: camera | map with the robot's track (rec_test / rec_manual bags)
nice -n 19 python3 ~/challenge_ws/runs/ROS_bags/bag_video.py ~/challenge_ws/runs/ROS_bags/<name> <name>.webm
# frames for a dataset (any bag with a colour topic)
nice -n 19 python3 ~/challenge_ws/datasets/extract_frames.py ~/challenge_ws/runs/ROS_bags/<name> --hz 3
```

Write `.webm`: it plays in VS Code and browsers. This OpenCV has no H.264 encoder, so an
`.mp4` only opens in players like VLC. A `rec_img` bag has no map, so the right half of
the video stays empty. `extract_frames.py --help` lists its options (`--dedup`, `--start`,
`--end`, `--list`).

## 5. Get the files

The robot computer has no screen.

- **VS Code:** click a video in the Explorer to play it; right-click a file or bag folder and
  choose **Download…**.
- **Browser:** `http://192.168.10.74:8080` (FileBrowser).
- **Copy a bag:** `scp -r <user>@192.168.10.74:~/challenge_ws/runs/ROS_bags/<name> .`

## Troubleshooting

| Problem | Fix |
|---|---|
| `rec_test: command not found` | `source ~/challenge_ws/runs/ROS_bags/aliases.sh` (section 1) |
| `explore_check` prints `NOTICE the explore launch is already running` and asks `Proceed? [y/N]` | Normal for `rec_test`: it checks while the launch is up, and lists the launch's own nodes (`qr_scanner`, `task_reporter`, `junction_trigger`, `curtain_node`). Type `y` to record. If you meant to start a new run, answer `n` and stop the old launch first |
| `explore_check` says NO-GO | Do not start a run. Slow streams come from the robot or bridge side; "other nodes on domain 4" means a leftover node (find it with `ros2 node list`, stop that process) |
| A topic has `Count: 0` | It was not published: wrong name, driver or launch not running. Check with `ros2 topic list` |
| `/map` or `/tf_static` has `Count: 0` | Recorded by hand without `--qos-profile-overrides-path` (section 3), or SLAM was not running |
| `/tf` has `Count: 0` | Fast-LIO was not running: start the launch before recording |
| A stream is marked `SLOW` after recording | Section 2.5. The pose and map in that bag cannot be trusted |
| `... slows every bridge stream` and the recorder stops | You asked for depth or `/livox/points` after `--` (section 2.6) |
| `Topic '...' has more than one type associated` | Another node uses the topic with a different type, usually an `rqt_image_view` subscribed to a `.../compressed` topic as a raw image. Close it, or pick the topic from its dropdown |
| `... already exists. Pick another name.` | Give a new name, or leave it out for a timestamped one |
| `Bad name '...'` | Letters, digits, `.`, `_`, `-` only |
| Renamed a bag with `mv` | Harmless, but rename it back and use `rec_rename` to keep the file names consistent |
| `could not connect to display` | GUI tools do not work over SSH. Make a video (section 4) or copy the bag to a computer with a screen |
| Disk filling up | `df -h`; copy old bags off and delete them |
