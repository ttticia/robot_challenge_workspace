# Launching the maze navigation stack

How to start `maze_nav` on the Lite3, what each part of the stack does, and what to check
when a run goes wrong. For the design and the example maze, see [README.md](README.md).

## 1. Before you launch

1. **Stop every other navigation stack.** `maze_nav` starts its own Fast-LIO, SLAM and Nav2.
   It must run *instead of* the lab launcher (`ros2 launch lite3_launcher lite3.launch.py`),
   never on top of it. Two stacks give two `odom → base_link` and `map → odom` publishers and
   two `/navigate_to_pose` servers. A run on 2026-11-14 failed this way: the robot walked
   forward and stopped at the first turn. Run these checks **on their own, before** the
   launch command in section 2. Each line is a separate command:

   ```bash
   ros2 node list | grep -E "slam|controller|planner|bt_navigator|fastlio|odom_relay|twist_mux"
   ```
   It must print nothing. If it lists nodes, Ctrl+C the launch that owns them and check
   again. The ROS daemon can keep listing nodes after they have exited. If names stay listed
   but `ps aux | grep -E "slam|nav2|component_container"` shows nothing, run `ros2 daemon stop`
   and check again, or use `ros2 node list --no-daemon`.

   ```bash
   ros2 topic info -v /cmd_vel
   ```
   The only subscriber should be the robot bridge, and there should be no publishers yet.

2. **Robot on the start cell, standing or lying down.** With `auto_stand:=true` (default),
   `cmd_vel_bridge` stands a lying robot up at launch, before the gait and before
   `/maze/start`: log `robot LYING DOWN … standing it up`, then `robot standing`. Stand
   (`0x21010202`) is a stand/sit **toggle**, so it is sent only when the lidar shows the robot
   lying down (base_link ≤ 0.18 m above the floor), never when it already stands (≥ 0.25 m) or
   the height is unknown (`posture unknown … stand NOT sent`, e.g. with `floor_z:=` given).
   `wall_scan` measures the floor once the robot stands. **Keep clear of the legs at launch.**
   If the robot is in fall protection, recover it with the handheld controller first.
3. **Operator next to the robot with the handheld controller.** At launch `cmd_vel_bridge`
   sends (after the stand, if any) Move mode and the RUG gait (General gear) once on `/simple_cmd` (`gait:=rug`, the
   default; `gait:=none` leaves the gait alone). **The operator sends `lite3_auto` after the
   log says `gait sent`**, before `/maze/start`: the documented order is gait request, then
   auto mode, and Move mode may take the robot out of auto. There is no gait feedback, so
   watch the robot. If the log says `gait … NOT sent`, set the gait on the handheld
   controller. Our code never sends auto or manual mode.
4. **No raw depth recording** (`/camera/depth/image_raw`) during the run. It delayed the lidar
   enough to stall Nav2 (see `ROS_bags/ROSBAG_GUIDE.md`).

## 2. Launch

```bash
cd ~/challenge_ws
colcon build --symlink-install --packages-select maze_nav   # first time / after changes (without
                                                             # --symlink-install: after EVERY change)
source install/setup.bash
ros2 launch maze_nav maze_nav.launch.py
```

First the launch checks for anything that would clash: another SLAM, Nav2 or maze_nav on
this computer (also Fast-LIO / the simulator with `bringup:=true`), teleop or `twist_mux`,
and on the whole `ROS_DOMAIN_ID` any `/cmd_vel` publisher or node using one of our names.
If it finds any, it prints a `!!!` banner listing them (with PIDs) and pauses 3 s, then
launches anyway. Ctrl+C, stop them, and relaunch.

After launch, once SLAM, TF and Nav2 are up, the robot holds still (`/maze/status` =
`PAUSED`) until you tell it to go. Start the rosbag recording first (`rec_maze`, see
`ROS_bags/ROSBAG_GUIDE.md`), then in another terminal:

```bash
ros2 topic pub --once /maze/start std_msgs/msg/Empty
```

A start sent before `PAUSED` is ignored (the log says so: `start ignored: not ready yet`);
send it again once the status reads `PAUSED`. Then it drives 10 cm straight ahead at 0.05 m/s
(`CREEPING`), with no steering and no turn on the spot, so its corners don't swing towards a
wall it can't see (the lidar sees no wall within ~0.5 m beside the robot). The lidar sees
nothing behind the robot, so the SLAM map ends at about the start pose and the robot can start
a few cm off it (log: `Robot is out of bounds of the costmap`). The creep also moves it off the
start cell's back wall. Then it scans (`SCANNING`, Nav2's `spin` behavior): 45° left, 90°
right, 45° left, which brings it back to the heading it had before the scan. A failed turn is
logged and the run carries on. Then it starts exploring.

The robot may start with its body as little as **0.15 m from a wall**. Before moving, the
explorer checks the footprint against the walls SLAM has already mapped: the creep is cut
short to keep 5 cm to a wall ahead (log `creep shortened`), and each side of the scan is cut
to what fits with 3 cm to spare (log `start scan limited by walls: 20 deg left, 32 deg
right`), still ending on the start heading. Walls behind the robot aren't mapped yet at that
point (lidar blind sector), so leave the back as the side with the most room.

Recommended first run on a new maze: `go:=false`, check the map and goals in RViz, then
relaunch without it.

| Argument | Default | Meaning |
|---|---|---|
| `bringup` | `true` | Start Fast-LIO + `odom_relay`. `false` if they already run (or for the simulator) |
| `go` | `true` | `false`: explorer picks goals and publishes them on `/maze/goal` but never sends them. The robot stays still |
| `drive` | `true` | `false`: Nav2 drives, but commands go to `/cmd_vel_bridge/cmd_vel_preview`, not `/cmd_vel` |
| `perception` | `true` | Also start team_perception's `robot.launch.py` (`qr_scanner`, `human_detector`, `poster_detector`). Don't start it separately as well (preflight warns about it). If team_perception isn't built, the run goes ahead without it, with a warning. `false` = navigation only (the simulator script sets this) |
| `posters` | `true` | With `perception`: `false` = QR codes only (no `human_detector` / `poster_detector`; saves ~0.5 core). Other perception arguments: run `robot.launch.py` by hand with `perception:=false` |
| `auto_stand` | `true` | Robot lying down at launch: send stand once, wait until it stands (15 s), then the gait. Never sent to a standing robot or if the posture is unknown. `false` = never |
| `wait_for_start` | `true` | Hold still once everything is up until `/maze/start` is published. `false` = start right away (simulator) |
| `creep_distance` | `0.10` | m driven straight ahead (no steering) after `/maze/start`, before the scan: onto the SLAM map and off the start cell's back wall so the scan turns clear it. `0` = off |
| `stairs` | `true` | `stair_detector`: stairs seen within 1.5 m, with the robot in front of them (≤ 0.35 m off their centre line) → explorer stops Nav2 (`STAIRS`), the robot faces them square and stops 0.45 m (`base_link`) from the first riser (`AT_STAIRS`), then holds until `/maze/start`. Every move is checked against walls on `/map`; blocked or over 45 s, it holds anyway as `AT_STAIRS: NOT LINED UP: <reason>` for a hand line-up. Follows `go` (`go:=false` = detect and log only). `false` = off, Nav2 drives over the stairs as floor |
| `scene_trigger` | `true` | Scene understanding: `scene_trigger` publishes `/scene/trigger` (+ `/junction_detected`) the first time the robot reaches each cell, only while the explorer runs. `false` = not started |
| `cpu_log` | `true` | `cpu_monitor`: CPU per process every 2 s to `~/challenge_ws/logs/cpu/`, summary on Ctrl+C |
| `start_scan` | `true` | After the creep, turn 45° left, 90° right, 45° left (back to the heading before the scan). `false` = off |
| `max_speed` | `0.15` | m/s for the path follower (the bridge clamps to 0.25 m/s, 0.5 rad/s anyway) |
| `band_low` / `band_high` | `0.30` / `0.43` | Wall slice, metres above the floor. Below `band_low` = walkable (ramps, mats, stairs). `band_high` must stay below the wall top (0.45 m) |
| `floor_z` | `nan` | Floor height in `odom`. `nan` = measure at launch. Set it (e.g. `-0.35`) if you have to launch while the robot is lying down |
| `map_dir` | `~/challenge_ws/maps` | Where the final map is saved |
| `udp_only` | `true` | UDP-only Fast DDS profile (Nav2 stalled on the robot with shared memory) |
| `use_sim_time` | `false` | |
| `domain_id` | `auto` | `auto`: a real run (`bringup:=true`) always uses `ROS_DOMAIN_ID=4` with `ROS_LOCALHOST_ONLY=0`, whatever the terminal is set to, so the robot bridge and RViz see it. With `bringup:=false` (simulator) the terminal's domain is kept. A number forces that domain. The launch logs which one it uses |
| `log_tags` | `false` | Terminal only: `false` drops `[INFO]` and the timestamps (`[WARN]`/`[ERROR]` tags stay, in yellow/red); `true` = ROS's full `[INFO] [<time>] [node]: msg`. Log files in `~/.ros/log` always keep the full prefix |
| `verbose` | `true` | `false` = Nav2 and slam_toolbox log warnings and errors only (no lifecycle / per-goal chatter). `maze_explorer`, `wall_scan` and `cmd_vel_bridge` always log INFO |

`stairs`, `perception` and `scene_trigger` are independent: switch off any combination.
Navigation only: `ros2 launch maze_nav maze_nav.launch.py stairs:=false perception:=false
scene_trigger:=false`. A switched-off part starts no nodes and is disconnected from the
explorer (`stairs_topic` / `look_topic` / `qr_unread_topic` blanked), so a stray publisher can't stop the robot.

### Terminal output

By default the terminal leaves out `[INFO]` and the timestamps:
`[maze_explorer]: [EXPLORING] go`. Warnings and errors keep their `[WARN]` / `[ERROR]` tag
and are shown in yellow / red.

```bash
ros2 launch maze_nav maze_nav.launch.py log_tags:=true    # full ROS prefix: [INFO] [<time>] [node]: msg
ros2 launch maze_nav maze_nav.launch.py verbose:=false    # Nav2 / slam_toolbox: warnings and errors only
```

Both only change this launch. To make the full prefix the default, set `default_value='true'`
on `log_tags` in `launch/maze_nav.launch.py` (with a `--symlink-install` build no rebuild is needed; otherwise rebuild). The log
files in `~/.ros/log` always keep the full prefix.

### Watching a run

On a laptop on the same network with `ROS_DOMAIN_ID=4`:

```bash
ros2 topic echo /maze/status          # STARTING → PAUSED → CREEPING → SCANNING → EXPLORING (⇄ TURNING: turn, or creep ahead and turn, before a goal near a wall, ⇄ LOOKING: look-around in a QR room, ⇄ FACING_QR: turn to an unreadable QR code until it is read, ⇄ STAIRS → AT_STAIRS: stair line-up, hold until /maze/start) → EXITED | NO_EXIT | STOPPED

# RViz with the maze_nav layout (config/maze_nav.rviz). If the laptop has this workspace built:
rviz2 -d $(ros2 pkg prefix maze_nav)/share/maze_nav/config/maze_nav.rviz
# otherwise copy the file over once:
scp user@group4:~/challenge_ws/src/maze_nav/config/maze_nav.rviz ~/ && rviz2 -d ~/maze_nav.rviz
```

The layout is top-down and follows the robot, with a 1 m grid (one maze cell). It shows:

| Display | Topic | Look for |
|---|---|---|
| SLAM map | `/map` | walls closing in as the robot explores |
| Walls (red squares) | `/wall_scan` | they sit on the map's walls; none = wall_scan / floor height problem |
| Footprint (cyan) | `/local_costmap/published_footprint` | the 0.64 x 0.46 m box around the robot |
| Local costmap | `/local_costmap/costmap` | an obstacle inside or touching the footprint explains "collision ahead" |
| Global costmap (off) | `/global_costmap/costmap` | 16 x 16 m rolling window |
| Global plan (green) | `/plan` | Smac path to the current goal |
| RPP collision arc (yellow) | `/lookahead_collision_arc` | where the controller checks for collisions |
| Frontier goal (magenta arrow) | `/maze/goal` | the explorer's current goal |

The robot computer has no screen, so run RViz on the laptop (`ROS_DOMAIN_ID=4`). Over Wi-Fi the
costmaps are the heaviest topics; untick them if RViz lags.

If a process dies during a run, the launch terminal prints a banner like the one below. It
doesn't print one for normal Ctrl+C shutdowns or for processes that exit cleanly.

```
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!! planner_server-5 DIED (exit code -11)
!!! Nav2 lifecycle manager will take ALL of Nav2 down in ~10 s; ... Ctrl+C and relaunch.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
```

### Stopping

Ctrl+C in the launch terminal. The bridge sends one zero `/cmd_vel` on shutdown, and the explorer
saves the map. For an emergency stop, use the handheld controller (switch to manual mode).

At the end the map is in `map_dir` as `maze_<date>_<time>.{pgm,yaml}`, with `latest.yaml`
pointing to the newest.

## 3. What the stack contains

```
/livox/points ─┬─> Fast-LIO + odom_relay ──> /odom, TF odom → base_link
               └─> wall_scan ──> /wall_scan ─┬─> slam_toolbox ──> /map, TF map → odom
                                             └─> Nav2 costmaps
/map ──> maze_explorer ──(NavigateToPose)──> bt_navigator
          bt_navigator ──> planner_server ──> path ──> controller_server ──> cmd_vel_nav
          cmd_vel_nav ──> velocity_smoother ──> nav_cmd_vel ──> cmd_vel_bridge ──> /cmd_vel ──> robot
```

| Node | Package | Does |
|---|---|---|
| `fastlio_mapping` | `fast_lio` (via `lite3_launcher`) | Lidar-inertial odometry from `/livox/lidar` + `/livox/imu`. Started when `bringup:=true` |
| `odom_relay` | `lite3_launcher` | Re-expresses Fast-LIO's pose as `/odom` and TF `odom → base_link` (base_link at z = 0 at launch, floor about 0.35 m below) |
| `wall_scan` | `maze_nav` | `/livox/points` → `/wall_scan` (LaserScan in `base_link`). Keeps only points 0.30–0.43 m above the floor, measured in `odom`, so walls (0.45 m) and hand rails are in, while ramps, mats and stairs (≤ 0.26 m) are out. Replaces the lab `/scan`, which starts at ~0.45 m and misses the maze walls |
| `slam_toolbox` | `slam_toolbox` (online async) | Builds `/map` from `/wall_scan`, publishes `map → odom`. Config `config/slam_toolbox.yaml` |
| `maze_explorer` | `maze_nav` | Every 1 s: finds frontiers (free cells next to unknown) on `/map`, sends the nearest reachable one (≥ 0.33 m from walls; frontiers narrower than 80 % of the robot width, 0.37 m, or not reachable along free space, e.g. seen through a slit, are ignored) to `/navigate_to_pose`, and blacklists each finished goal. Stops with **EXITED** when there is ≥ 1.1 m of seen-free floor within 0.8 m of the robot and ≥ 1.5 m from the start, held for 2 s. Stops with **NO_EXIT** when nothing is left to explore (cancels the running goal, sends a zero Twist and prints `nav stopped`) or goals keep failing after 3 recovery rounds (`max_recoveries`), and with **STOPPED** after 15 min or on Ctrl+C. Before a goal whose first turn on the spot would hit a mapped wall it turns the robot itself (`TURNING`; blocked both ways: creeps up to 0.6 m ahead to the spot with the most room first, `pre_turn_max_creep`). Publishes `/maze/status` and `/maze/goal` |
| `bt_navigator` | Nav2 | Runs `config/maze_bt.xml`: replan at 1 Hz → follow path. Recoveries: clear costmaps, spin 90°, wait 3 s (no BackUp, because the bridge never reverses) |
| `planner_server` | Nav2, Smac Planner 2D | Plans on the global costmap (frame `map`) and **through unknown space** (`allow_unknown`), 0.25 m goal tolerance |
| `controller_server` | Nav2, Regulated Pure Pursuit | Follows the path at `max_speed` and turns on the spot first when the heading is off by > 34°. Stops if the footprint would hit an obstacle within 1.5 s. Goal reached within 0.20 m, heading ignored |
| `behavior_server` | Nav2 | `spin` and `wait` recoveries |
| `velocity_smoother` | Nav2 | Limits acceleration, max 0.25 m/s and 0.5 rad/s, never backwards |
| `lifecycle_manager_navigation` | Nav2 | Brings the five Nav2 servers up |
| `stair_detector` | `maze_nav` | Detects stairs in the depth image (`/camera/depth_relay/image_raw` from `depth_relay`, or the raw depth if team_perception isn't built; 5 Hz, ~30 ms a frame, ~0.2 core) and, with the explorer's handshake on `/stairs/status` ↔ `/maze/status`, lines the robot up square in front of them on `nav_cmd_vel` (≤ 0.08 m/s, ≤ 0.35 rad/s), checking every move against walls on `/map`. Camera pose from `base_link → camera_link` (the robot's TF has a wrong second parent for the depth camera, `agent.md` §12). See `maze_nav/stair_detector.py` |
| `depth_relay` | `team_perception` | Takes the raw depth from the robot bridge once and passes it on at 10 Hz on `/camera/depth_relay/*`, for `stair_detector` and perception. Runs when `stairs` or `perception` is on |
| `scene_trigger` | `maze_nav` | `/scene/trigger` (JSON: cell, the way in, sides open on `/map`) + a `/junction_detected` pulse the first time the robot reaches each 1 m cell, while `/maze/status` says the run is under way. Grid from the robot's pose when Fast-LIO started: start the robot square on the start cell |
| `cpu_monitor` | `maze_nav` | CPU per process every 2 s (`cpu_log`) |
| `cmd_vel_bridge` | `maze_nav` | `nav_cmd_vel` → `/cmd_vel` with the QoS the robot bridge needs (RELIABLE + TRANSIENT_LOCAL). Clamps to 0.25 m/s and 0.5 rad/s, never backwards. Sends one stop when Nav2 goes quiet. Stops and blocks if `/livox/points` or `/odom` is silent for 0.5 s |

### Costmaps and footprint (`config/nav2_params.yaml`)

| | Local costmap | Global costmap |
|---|---|---|
| Frame / size | `odom`, rolling 3 × 3 m | `map`, grows with `/map` |
| Layers | `/wall_scan` obstacles + inflation | `/map` (static) + `/wall_scan` obstacles + inflation |
| Resolution | 0.05 m | 0.05 m |

- **Footprint:** 0.64 × 0.46 m rectangle (Lite3 including moving feet), offset 5 cm behind
  `base_link`: x from −0.37 to +0.27 m, y ±0.23 m. In a 0.9 m maze corridor that leaves 0.22 m
  each side. The farthest corner is 0.44 m from `base_link`, so turning on the spot in a cell
  centre leaves only about 1 cm to spare, and in a 0.9 m corridor it doesn't fit at all
  (0.437 m vs 0.425 m to the wall). The explorer's `TURNING` handles that before goals.
- **Inflation:** local 0.5 m / cost scaling 6, global 0.7 m / 3.5 (raised on 2026-11-16 after
  one simulator run; still to be confirmed over more runs). This keeps paths near the middle
  of the corridor.
- **No-return beams** (`inf`) are ignored, because the lidar can't see behind the robot. They
  never clear walls.

### Topics and TF

| Name | Type | From |
|---|---|---|
| `/odom`, TF `odom → base_link` | `nav_msgs/Odometry` | `odom_relay` |
| `/wall_scan` | `sensor_msgs/LaserScan` (frame `base_link`) | `wall_scan` |
| `/map`, TF `map → odom` | `nav_msgs/OccupancyGrid` | `slam_toolbox` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` (goals from `maze_explorer`) |
| `/plan` | `nav_msgs/Path` | `planner_server` |
| `cmd_vel_nav` → `nav_cmd_vel` | `geometry_msgs/Twist` | controller / behaviors → `velocity_smoother` |
| `/cmd_vel` | `geometry_msgs/Twist` | `cmd_vel_bridge` (only while Nav2 drives) |
| `/maze/status` | `std_msgs/String`, latched | `maze_explorer` |
| `/maze/goal` | `geometry_msgs/PoseStamped`, latched | `maze_explorer` |
| `/stairs/status` | `std_msgs/String`, latched | `stair_detector` (`IDLE`, `HANDOVER`, `ALIGNING`, `APPROACHING`, `SQUARING`, `AT_STAIRS`, `ABORTED`) |
| `/scene/trigger`, `/junction_detected` | `std_msgs/String` (JSON), `std_msgs/Bool` | `scene_trigger` |

## 4. When something goes wrong

| What you see | Likely cause | Do |
|---|---|---|
| `more than one action server for the action 'navigate_to_pose'`, or `unknown goal response` from bt_navigator | Another Nav2 (usually `lite3.launch.py`) is running | Stop it, relaunch (section 1) |
| `[NO_EXIT] all frontiers mapped` (then `nav stopped`) a few seconds after start | Same as above, or `/map` has no walls | Check `ros2 node list`. In RViz, `/wall_scan` should outline the walls |
| `floor seen at …, expected …` warning | Robot not standing at launch, or no floor visible | Stand up and relaunch, or pass `floor_z:=-0.35` |
| Walls missing or broken in `/map` | Band too high or floor wrong | Check the logged floor height. Try `band_low:=0.28` |
| Ramps or stairs drawn as walls | Band too low | Raise `band_low` (the top stair tread is 0.26 m) |
| Goals fail at once, planner log `Starting point in lethal space!` | The global costmap has an obstacle at the robot: its own legs (hips are in the wall band) were once mapped as a wall. `wall_scan` now drops points inside the body box (`body_back` 0.42, `body_front` 0.32, `body_half_width` 0.28 m). The explorer sets failed goals aside and, while frontiers are open, clears the costmaps and retries (`max_recoveries` 3) before `NO_EXIT` | Check `/wall_scan` in RViz for points next to the robot; widen the body box if the legs still show |
| Robot rubs a wall on turns / cuts inside corners | Path or controller cuts the corner; footprint at the limit (1 cm margin turning on the spot) | Global costmap: larger `inflation_radius` / smaller `cost_scaling_factor` (now 0.7 / 3.5) pushes the path to the corridor centre. RPP: shorter `lookahead_dist` (now 0.35) cuts corners less. Check with `test/run_sim.sh`: the sim logs the footprint's min wall clearance and writes `track.csv` |
| Robot stops at a goal and turns slowly, then the goal fails | Progress checker (0.15 m in 20 s) | Raise `movement_time_allowance` |
| `SENSOR TIMEOUT` from cmd_vel_bridge, robot frozen | `/livox/points` or `/odom` stopped | Check Fast-LIO and the lidar. Restart the launch |
| Robot stalls at the stairs | Gait, not Nav2 (Nav2 treats the stairs as floor) | Operator switches gait on the handheld controller. With `stairs:=true` it waits in `AT_STAIRS` for that: switch gait, then `/maze/start` |
| Explorer sits in `AT_STAIRS` | Waiting for the operator, on purpose | Switch gait, then `ros2 topic pub --once /maze/start std_msgs/msg/Empty` |
| `AT_STAIRS: NOT LINED UP: …` | The line-up was blocked by walls (the cell before the stairs leaves the turning circle about 1 cm) or took over 45 s | Line the robot up by hand, switch gait, then `/maze/start` |
| `TURNING … creeping … cm ahead to turn there` | A turn before a goal would hit a wall both ways; the robot creeps ahead to more room first | Normal. `… leaving it to Nav2` = no room found: watch that corner |
| `/tf_static also has base_link -> camera_depth_optical_frame` (perception) | The robot's TF gives the depth camera a second, unpitched parent | Handled (`tf_pin`); report it to whoever runs the robot's driver container (`agent.md` §12) |
| Ends `NO_EXIT` with the robot pinned on the stair handrail just past the stairs | Known since 2026-11-16 (open item in `agent.md` §12) | Take over with the handheld controller |

## 5. Without the robot

One command (runs, waits for the result, stops everything; exit 0 = EXITED, logs in `/tmp/maze_sim`):
`src/maze_nav/test/run_sim.sh`. Run it on an idle machine: if the CPU is overloaded, Nav2 misses its control rate and the result means nothing. Or by hand:

```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1        # never the robot's domain
python3 src/maze_nav/test/fake_maze_lite3.py &      # example maze in a 10 x 8 m room
ros2 launch maze_nav maze_nav.launch.py bringup:=false udp_only:=false map_dir:=/tmp/maze_maps
```

The simulator logs `OUT OF THE MAZE` once the robot is past the stairs, and `/maze/status` should end at `EXITED`.
It publishes no depth image, so the stair line-up never runs there. `89` is for these
simulator runs; other test setups (and other Claude sessions) need their own domain: check it
is empty first (`ros2 node list`), and stop only your own processes (no `pkill -f <name>`).
Unit tests: `cd src/maze_nav && python3 -m pytest -q test`.
