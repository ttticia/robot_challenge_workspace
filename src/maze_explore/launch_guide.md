# maze_explore: build, launch and record test runs

For running the Lite3 with the `maze_explore` stack on the robot computer
(`192.168.10.74`, ROS 2 Humble, `ROS_DOMAIN_ID=4`). How the stack works is in
[README.md](README.md).

Status (2026-11-23): the stack has only been dry-run on the robot. Treat the first driving
runs as tests: controller in hand, someone next to the robot.

---

## 1. Build

Only needed after pulling new code, or after adding files or changing `setup.py`. Edits to
existing Python, launch and config files take effect without a rebuild.

```bash
cd ~/challenge_ws
nice -n 19 colcon build --symlink-install --packages-select maze_explore --parallel-workers 1
source install/local_setup.bash
```

- Always `--symlink-install`. A plain build breaks the links for every terminal.
- `nice -n 19` and one worker keep the build from slowing the robot's own software, which
  shares this computer's host.
- New terminals source the workspace by themselves (`~/.bashrc`). In a terminal that was
  already open, run `source ~/.bashrc` to get new aliases.

Check the logic without the robot (a few seconds):

```bash
cd ~/challenge_ws/src/maze_explore && nice -n 19 python3 -m pytest -q test
```

If the build fails with `can't copy ... doesn't exist`, a launch or config file was renamed:
`rm -rf build/maze_explore install/maze_explore` and build again.

---

## 2. Before every run

1. Robot standing in the start cell, facing roughly into the maze. It may be off-centre
   (up to about 0.3 m) and at an angle (up to about 30 degrees).
2. Nothing else running on the robot's domain, and the sensors at full rate:

   ```bash
   explore_check
   ```

   It must end with **GO**. On **NO-GO** do not run: with the lidar IMU below about 150 Hz
   the robot's position runs away within seconds. A slow feed comes from the robot or its
   host, not from this stack; wait, or ask the lab.
3. During a run, do not look at the big streams: no `ros2 topic hz` / `echo` on
   `/camera/depth/*`, `/camera/color/image_raw`, `/camera/depth/points` or `/livox/points`,
   and no RViz depth or raw-image display. A single subscriber to raw depth dropped the body
   IMU from 195 to 11 Hz (measured 2026-11-23); `explore_check` says NO-GO while one exists.
   Small topics such as `/explore/status` are fine.
4. At least 3 GB free for a bag: `df -h ~`. A test bag takes about 0.3-0.5 GB per minute.

---

## 3. Launch

Three terminals.

**Terminal 1: the stack**

```bash
ros2 launch maze_explore explore.launch.py
```

Wait for `ready: ros2 topic pub --once /nav/start std_msgs/msg/Empty` in the log. The robot
does not move before the start message.

**Terminal 2: the recording** (section 4)

```bash
rec_test run1
```

**Terminal 3: start, watch, stop**

```bash
explore_go                                         # publishes /nav/start
ros2 topic echo /explore/status --field data       # state, cell, job, action
explore_stop                                       # zero velocity, stays stopped
```

After `explore_go` the robot switches to the RUG gait and navigation mode, creeps 10 cm, lines
itself up with the walls, then explores. `explore_go` after `explore_stop` carries on.
Ctrl+C in Terminal 1 also stops it with a zero velocity. The handheld controller takes over
at any time with `lite3_manual`.

### Launch variants

| Command | Use |
|---|---|
| `explore.launch.py drive:=false auto_start:=true` | **Dry run.** Everything live, nothing sent to the robot. The log shows what it sees around its cell and what it would do. Do this first in a new arena |
| `explore.launch.py explore:=false` | Mapping only, for hand-driven runs (`rec_manual`) |
| `explore.launch.py v_max:=0.15` | Slower walking (default 0.20 m/s) |
| `explore.launch.py gait:=hstep` | H-Step gait instead of the default RUG general gear |
| `explore.launch.py auto_start:=true` | Starts without waiting for `/nav/start` |

### What the log tells you

| Line | Meaning |
|---|---|
| `floor found ...` / `floor hidden by walls; leg odometry: standing 0.27 m` | Height reference is set |
| `grid lined up: turned +3.5 deg, moved 0.12 m (quality 0.85)` | Start pose corrected from the walls. `not lined up` at the start: stop and re-place the robot |
| `job: visit cell (1, 0) side 0 ...  [in (1, 0): ahead/left/back/right = free room free wall]` | Next job, and what it sees on each side of its cell |
| `event: curtain ...`, `room left ...`, `stairs: AI mode ...` | Behaviours as they happen |
| `/odom jumped ... paused 3 s` | One jump after a stall in the lidar feed. Carries on by itself |
| `FAULT: /odom jumped ... (3 jumps in 10 s)` | Odometry ran away. Stays stopped; relaunch after `explore_check` says GO |
| `DONE: in the last cell` | Finished |

---

## 4. Record a test run

```bash
rec_test run1            # autonomous run. Ctrl+C stops; then Enter keeps the name
rec_manual demo2         # hand-driven run with explore:=false
rec_last                 # topics and counts of the newest bag
rec_rename run1_stairs   # rename the newest bag
rec_list                 # what each one records
```

- Start the recording after Terminal 1 says `ready` and **before** `explore_go`.
- Both run `explore_check` first and ask before recording on a NO-GO.
- Bags go to `~/challenge_ws/runs/ROS_bags/<name>/`.
- `rec_test` records: transforms, compressed colour, Fast-LIO's cloud, `/odom`, `/scan`,
  `/map`, the commands sent (`/cmd_vel`, `/simple_cmd`), and the explorer's
  `/explore/status`, `/explore/event`, `/explore/cells`.
  Compressed colour is the only stream it takes from the robot: every topic recorded from
  the robot is one more copy across the bridge. `rec_manual` adds the raw lidar, both IMUs,
  leg odometry and the controller, so use it only for hand-driven runs.
- Neither records depth: one raw depth subscriber slows every stream from the robot.
- Bags are zstd-compressed message by message (`REC_COMPRESS=0` turns it off). `ros2 bag`
  tools and `bag_video.py` read them as usual; reading the `.db3` with plain sqlite gives
  compressed blobs.
- After Ctrl+C the summary lists each stream's rate over its own time span and its longest
  gap. `SLOW` on `/odom` means the map and pose in that bag cannot be trusted. A longest gap
  near 1 s is a stall in the robot's feed; the explorer stands still through it.
- Stop the recorder and the stack in either order; the summary no longer depends on it.

### Hand-driven run (handheld controller), step by step

Robot standing still and level in the start cell (this pose becomes the map origin),
`ros2 node list` showing only `/lite3_domain_bridge_4`, at least 3 GB free.

```bash
# Terminal 1: mapping only (Fast-LIO, /odom, /scan, slam_toolbox), no explorer
explore_check                                   # must end with GO
ros2 launch maze_explore explore.launch.py explore:=false

# Terminal 2: wait ~10 s for the launch to settle, then record
rec_manual <name>                               # start BEFORE the robot moves
#   ... drive the run with the controller ...
#   Ctrl+C stops; Enter keeps the name

# Optional, before stopping Terminal 1: save the map as a file (it is also in the bag)
mkdir -p ~/challenge_ws/maps && ros2 run nav2_map_server map_saver_cli -f ~/challenge_ws/maps/<name>

rec_last                                        # check topics and counts
```

- On NO-GO, or `SLOW` on `/odom` or `/livox/imu` in the summary, the map and pose in the
  bag are wrong; the camera frames and raw lidar are still good.
- For depth (for example the range to a curtain for the YOLO set), record a separate short
  bag **without** the mapping launch:
  `ros2 bag record -o <name> /camera/depth/image_raw /camera/depth/camera_info /camera/color/image_raw/compressed /camera/color/camera_info /tf_static`
- Full details: `runs/ROS_bags/ROSBAG_GUIDE.md` section 2.6.

Save the map before stopping Terminal 1 if you want it as an image:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/challenge_ws/maps/run1
```

---

## 5. After a run

1. Ctrl+C in Terminal 2 (recording), then Terminal 1 (stack).
2. `ros2 node list` should show only `/lite3_domain_bridge_4`. Anything left over from the
   stack: close its terminal.
3. Note what happened next to the bag name (where it stopped, what it misread). The `job:`
   lines are in Terminal 1 and in `~/challenge_ws/logs/ros/`.

## 6. Watch a run in RViz

One view for both cases: [config/explore.rviz](config/explore.rviz). It opens top-down on
`map` and shows the map, the cells with each side coloured by kind (`/explore/cells`), the
wall scan, the lidar cloud, the path driven (`/odom`) and the `map`, `odom` and `base_link`
frames. A second saved view, "Behind the robot", is in the Views panel.

RViz needs a screen: it does not open from SSH or the VS Code terminal
(`could not connect to display`). Use the robot computer's desktop, or a laptop with ROS 2
Humble and a copy of the bag folder and of `explore.rviz`.

**Live** (robot's domain, while Terminal 1 runs):

```bash
rviz2 -d ~/challenge_ws/src/maze_explore/config/explore.rviz
```

- It may be opened before or after `explore_go`; the map appears whenever RViz starts.
- Every display in the file reads a topic made on this computer, so it adds no load on the
  feed from the robot. Do **not** add a depth, raw-image, `/camera/depth/points` or
  `/livox/points` display (section 2, point 3).
- "Height cloud (/maze/cloud)" is switched off; tick it to see heights above the floor
  (what `object` and `stairs` are judged from). It is not recorded, so it is live only.

**Replay** of a `rec_test` or `rec_manual` bag (test domain, never domain 4, or the recorded
`/cmd_vel` reaches the robot):

```bash
# Terminal A: RViz first, on the bag's clock
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
rviz2 -d ~/challenge_ws/src/maze_explore/config/explore.rviz --ros-args -p use_sim_time:=true

# Terminal B: the bag
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
ros2 bag play ~/challenge_ws/runs/ROS_bags/run3 --clock \
  --qos-profile-overrides-path ~/challenge_ws/runs/ROS_bags/qos_overrides.yaml
```

- Space in Terminal B pauses and resumes; `--rate 2` plays at double speed; `--loop` repeats
  (press Reset in RViz if the picture freezes when the bag starts over).
- `ros2 topic echo /explore/status --field data` in a third terminal (same two exports)
  shows the state and job next to the picture.
- A bag whose summary said `SLOW` on `/odom` shows a wrong map and pose. A bag without
  `metadata.yaml` (`run2`) cannot be played.
- The camera is not in this view. Make a video with `bag_video.py`
  (`runs/ROS_bags/ROSBAG_GUIDE.md` section 4).

## Troubleshooting

| Symptom | Check |
|---|---|
| Stays at `WAIT ... waiting for /map` | Fast-LIO has no lidar: `explore_check`. `waiting for map -> base_level`: `/odom` missing |
| `explore_go` does nothing | Status must be `READY`. In `FAULT`, relaunch |
| Robot does not move after `explore_go` | Handheld controller still in control: the stack sends navigation mode once at the start; `lite3_auto` sends it again |
| Turns but will not walk | `way blocked` in the log: something within 20 cm ahead from 0.30 m up |
| Stops at the stairs | `no progress on the stairs in AI mode`: take over (`lite3_ai_off`), walk it across, `explore_go` |
| `rec_test: command not found` | `source ~/.bashrc` |
| RViz: `could not connect to display` | No screen in this terminal (SSH, VS Code): section 6 |
| RViz replay: map shown, nothing else moves | RViz started without `use_sim_time:=true`, or the bag without `--clock` |
| RViz: no map | Replay: the `--qos-profile-overrides-path` option is missing. Live: the stack is still at `WAIT ... waiting for /map` |
