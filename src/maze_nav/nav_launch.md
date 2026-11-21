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
   `wall_scan` measures the floor once the robot stands and its height is steady. It accepts a floor that
   puts base_link 0.27–0.41 m above it (the Lite3 stood at 0.29 m on 2026-11-19). The `wall_scan`
   parameter `floor_start_tile` (default 0) widens that for an arena where the robot starts on a raised
   tile and the walls stand on the floor below; don't use it when the maze stands on a plate above a
   visible lab floor (the sim example maze: walls on the 0.1 m plate, so the lower layer seen is the
   lab floor outside the maze, which made every wall look raised 10 cm). Otherwise it logs
   `floor not believable yet … retrying (n/20)` and measures again. Only after 20 failed tries
   does it fall back to a guess (`FLOOR DETECTION FAILED`). **Keep clear of the legs at launch.**
   If the robot is in fall protection, recover it with the handheld controller first.
3. **Operator next to the robot with the handheld controller.** At launch `cmd_vel_bridge`
   sends (after the stand, if any) Move mode and the RUG gait General gear once on `/simple_cmd` (`gait:=rug`, the
   default; `gait:=hstep` = H-Step, the high-stepping RUG gait; `gait:=none` leaves the gait alone). **The operator sends `lite3_auto` after the
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
lite3_auto
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
ros2 topic pub --once --keep-alive 2 /maze/start std_msgs/msg/Empty
```

`--keep-alive 2` keeps the publisher up for 2 s after it sends. Without it (default 0.1 s) the
publisher can exit before the explorer has discovered it, and the start is silently lost (no
`start received` and no `start ignored` in the log), so it seemed to need sending several times.

A start sent before `PAUSED` is ignored (the log says so: `start ignored: not ready yet`);
send it again once the status reads `PAUSED`. After a start the explorer also waits until `/wall_scan` has come steadily for 3 s (`start_steady`): while the lidar or odometry from the robot bridge is late, wall_scan drops clouds and the log says `start held: /wall_scan 1.2 s old: lidar or odometry late` every 2 s. The run then starts by itself; no second `/maze/start` is needed. Then it drives 10 cm straight ahead at 0.05 m/s
(`CREEPING`), with no steering and no turn on the spot, so its corners don't swing towards a
wall it can't see (the lidar sees no wall within ~0.5 m beside the robot). The lidar sees
nothing behind the robot, so the SLAM map ends at about the start pose and the robot can start
a few cm off it (log: `Robot is out of bounds of the costmap`). The creep also moves it off the
start cell's back wall. Then it scans (`SCANNING`, Nav2's `spin` behavior, always a real turn,
never Pose mode): 30° left, 60° right, 30° left, which brings it back to the heading it had before the scan. A failed turn is
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
| `perception` | `true` | Also start team_perception's `robot.launch.py` (`qr_scanner` only: human / survivor poster detection was removed on 2026-11-19, because each camera subscriber is another copy from the robot bridge and with them the bridge fell behind: IMU 23 Hz, lidar 1.1 s late, no map). Don't start it separately as well (preflight warns about it). If team_perception isn't built, the run goes ahead without it, with a warning. `false` = navigation only (the simulator script sets this) |
| `auto_stand` | `true` | Robot lying down at launch: send stand once, wait until it stands (15 s), then the gait. Never sent to a standing robot or if the posture is unknown. `false` = never |
| `wait_for_start` | `true` | Hold still once everything is up until `/maze/start` is published. `false` = start right away (simulator) |
| `creep_distance` | `0.10` | m driven straight ahead (no steering) after `/maze/start`, before the scan: onto the SLAM map and off the start cell's back wall so the scan turns clear it. `0` = off |
| `stairs` | `true` | `stair_detector`: stairs seen within 1.5 m, with the robot in front of them (≤ 0.35 m off their centre line) → explorer stops Nav2 (`STAIRS`), the robot faces them square and stops 0.45 m (`base_link`) from the first riser (`AT_STAIRS`). Lined up, `cmd_vel_bridge` then switches to AI mode (manual → AI state → AI basic gait) and walks 1.2 m straight ahead at 0.1 m/s, measured on `/odom` (`cmd_vel_bridge` parameters `ai_walk_distance` / `ai_walk_speed`, `0` = no walk; it stops after 8 s without progress). Then it holds until `/maze/start`, which leaves AI mode. Every move is checked against walls on `/map`; blocked or over 45 s, it holds anyway as `AT_STAIRS: NOT LINED UP: <reason>` for a hand line-up. Follows `go` (`go:=false` = detect and log only). `false` = off, Nav2 drives over the stairs as floor |
| `curtains` | `true` | `curtain_detector`: a wall in front with a gap at the bottom of 5–15 cm, wider than 0.7 m, is a curtain (seen ~0.6–2.6 m from base_link with `curtain_sensors:=both`): cleared from the wall scan and the explorer's map. Within 2 m, robot in front of it → explorer stops Nav2 (`CURTAIN`), the robot walks through its middle at 70 % of `max_speed` to 0.2 m past it, turns 20° left / 40° right / 20° left, and while the lidar or depth camera stays covered creeps 5 cm and turns again, alternately; still covered after 12 steps → `AT_CURTAIN`, hold until `/maze/start`. Follows `go`. `false` = off, curtains stay walls |
| `curtain_sensors` | `both` | What sees the curtain's gap: `both` (lidar + depth camera, ~0.6–2.6 m from base_link), `lidar` (~1.2–2.6 m: its lowest beam only gets under the curtain from ~1 m) or `depth` (~0.7–2.0 m, ±40° only). Closer than ~0.55 m no sensor sees under a 5–15 cm gap. Depth comes through `depth_relay` (needs `team_perception` built); without it, lidar only |
| `curtain_max_gap` | `0.15` | m: a gap at the bottom at least this big makes it a hurdle, not a curtain (to-do spec < 15 cm). The lab test curtain hangs ~20 cm up: `curtain_max_gap:=0.25` to test with it |
| `scene_trigger` | `true` | Scene understanding: `scene_trigger` publishes `/scene/trigger` (+ `/junction_detected`) when the robot faces a dead end from the cell before it, and `/scene/junction` at junctions (the explorer then faces each way), only while the explorer runs. `false` = not started (no junction looks, no dead-end stops either) |
| `scene_understanding` | `false` | `true` = also start `room_scene` (one camera frame per dead-end view, picked by stamp inside the hold, saved to `~/challenge_ws/logs/scene/<date>/` with `rooms.jsonl`, queued for the VLM, which is **not wired yet**: results say `vlm: none`) and `task_reporter` (`/mission/task_completed`: QR text with the dead-end cell it was read in; objects/hazards once a VLM exists). Off by default: one more colour-camera copy over the robot bridge. The dead-end looks run either way and the explorer never waits for scene understanding. A checked dead end (from `/scene/trigger` or the junction follow-up) is never entered: no frontier goal anywhere in that cell, a running goal into it is cancelled, unless a confirmed curtain sits on its edge. Tilt: body pitched more than 30° from its pitch at `/maze/start` (TF `odom → base_link`, 0.5 s average) → every view of that dead end is a Pose-mode look with the body pitched back (`/maze/pose_pitch`, clamped to `pose_max_pitch_deg`, 20° unmeasured); `/scene/capture` records tilt, pitch sent and the camera's pitch |
| (room look-in) | on | Explorer parameters `room_looks` (true), `room_look_radius` 3.0, `room_look_hold` 3.0, `room_look_max` 6, `room_stair_radius` 2.0. A cell confirmed as holding an object (`block_objects`: e.g. a toy kitchen marking the room type) is a room: once the robot is within 3 m, the explorer drives to the centre of the nearest neighbouring cell with an open side into it, squares up facing in, holds 3 s, then looks at the room's left and right walls (Pose mode; survivor posters and QR codes hang there), with perception on (`/maze/perceive`) and one `/scene/capture` per view (`kind: room`). Status `JUNCTION` (`room [i, j]: ...`). It never goes in. At most 6 a run; none within 2 m of confirmed stairs (stairs can read as objects). A falsely blocked cell gets a pointless look too |
| (keepout) | on | The painted object cells are also a Nav2 KeepoutFilter mask (`/maze/keepout_mask` + `/maze/costmap_filter_info`, published by the explorer, map frame) on both costmaps, so the controller and the spin / wait recoveries treat them as lethal, not only the planner. A cell isn't painted while the robot's body may still reach into it. `keepout_mask_topic:=''` on the explorer = no mask (the filter then stays inactive) |
| `cpu_log` | `true` | `cpu_monitor`: CPU per process every 2 s to `~/challenge_ws/logs/cpu/`, summary on Ctrl+C |
| `start_scan` | `false` | Off since 2026-11-21: no turning after launch. `true` = after the creep, turn 30° left, 60° right, 30° left (back to the heading before the scan, Nav2 spins) |
| `pose_looks` | `true` | Start scan and dead-end side-wall looks in Lite3 Pose mode: cmd_vel_bridge sends manual → Pose mode → yaw axis (0x21010135) → Move mode → gait → auto mode; the feet stay put. Falls back to Nav2 spins when the bridge can't (drive:=false, sim, no transfer_interfaces). `false` = always spins |
| `pose_max_yaw_deg` | `20.0` | Body yaw at full axis; NOT measured yet. After each look cmd_vel_bridge logs `/odom yaw moved up to N deg`: set this to N |
| `pose_max_pitch_deg` | `20.0` | Body pitch (nose down) at full axis, for the curtain inspection's Pose-mode look (`/maze/pose_pitch`, axis 0x21010130); NOT measured yet. Check by eye (or /imu/data pitch) during the first inspection |
| `max_speed` | `0.15` | m/s for the path follower (the bridge clamps to 0.25 m/s, 0.5 rad/s anyway) |
| `band_low` / `band_high` | `0.30` / `0.45` | Wall slice, metres above the floor. Below `band_low` = walkable (ramps, mats, stairs). `band_high` must not exceed the wall top (0.45 m). It was 0.43 until 2026-11-20: with the lidar ~0.125 m above `base_link`, side walls only enter the band beyond (base_link + 0.125 − band_high) / tan 6.8°, i.e. 0.45 m at a 0.36 m body height with 0.43 vs 0.29 m with 0.45, so a centred robot kept losing its side walls |
| `floor_z` | `nan` | Floor height in `odom`. `nan` = measure at launch. Set it (e.g. `-0.35`) if you have to launch while the robot is lying down |
| `map_dir` | `~/challenge_ws/maps` | Where the final map is saved |
| `udp_only` | `true` | UDP-only Fast DDS profile (Nav2 stalled on the robot with shared memory) |
| `fastlio_config` | `<maze_nav>/config/fastlio_mid360.yaml` | Fast-LIO settings, a copy of lite3_launcher's `mid360.yaml` with `blind` 0.5 → 0.3 m, `filter_size_surf/map` 0.5 → 0.3 m and `extrinsic_est_en` false. With 0.5 m blind Fast-LIO ignored walls closer than 0.5 m; turning in a corner on 2026-11-19 its pose slipped ~0.5 m and `cmd_vel_bridge` stopped the robot (`SENSOR FAULT … odometry diverged?`). The robot's own returns end at 0.25 m. Replaying bag `curtain_test1` (turning 0.5 m from a curtain): original drifted ~97 m, this ~9 m and then held. ~30 % instead of ~25 % of a core. Original: `fastlio_config:=$(ros2 pkg prefix lite3_launcher)/share/lite3_launcher/config/mid360.yaml` |
| `lidar_relay` | `true` | `lidar_relay` takes `/livox/points` from the robot bridge **once** and republishes it unchanged (raw bytes, same stamp) on `/livox/points_local` for `wall_scan`, `cmd_vel_bridge`, `stair_detector` and `curtain_detector`. The bridge sends a full copy to every subscribing process: on 2026-11-19, 6 copies delayed the lidar to ~1.1 s (p90) and cut the IMU to a seventh, so Fast-LIO odom lagged and `wall_scan` dropped every cloud (`no TF within 0.5 s`, costmap `/wall_scan … not updated`). With the relay: 0.13 s. `false` = every node subscribes `/livox/points` itself. Every extra `/livox/points` subscriber on domain 4 (RViz, `ros2 topic hz/delay`, probes, a bag of `/livox/points`) is another bridge copy. `lidar_relay` checks the ROS graph every 5 s and warns with their node names (`N other subscriber(s) on /livox/points: …`), then logs `other subscribers gone` once they are closed |
| `lidar_crop` | `true` | With `lidar_relay`: the relay also publishes `/livox/points_crop`, the same cloud without the Mid-360's `(0, 0, 0)` no-return points (17–22 % of a cloud), points beyond 12 m, and points outside −1.2…+1.0 m around the lidar along world up (ceiling, room above the maze). About 35–40 % of the points are left (robot bags). `wall_scan`, `stair_detector` and `curtain_detector` read it; `cmd_vel_bridge` stays on `/livox/points_local`. Up comes from the latest TF; while the tilt is changing (> 4° in 1.5 s), for 3 s after a `/maze/pose_pitch` command, or with no TF, only the no-return and range filter applies. Every 30 s the relay's report adds `crop kept N % of points`. `false` = everything on `/livox/points_local` |
| `use_sim_time` | `false` | |
| `domain_id` | `auto` | `auto`: a real run (`bringup:=true`) always uses `ROS_DOMAIN_ID=4` with `ROS_LOCALHOST_ONLY=0`, whatever the terminal is set to, so the robot bridge and RViz see it. With `bringup:=false` (simulator) the terminal's domain is kept. A number forces that domain. The launch logs which one it uses |
| `log_tags` | `false` | Terminal only: `false` drops `[INFO]` and the timestamps (`[WARN]`/`[ERROR]` tags stay, in yellow/red); `true` = ROS's full `[INFO] [<time>] [node]: msg`. Log files in `~/.ros/log` always keep the full prefix |
| (startup) | | Everything starts at once (staggered startup removed 2026-11-19: the lidar lag it was meant to fix came from bridge copies, see `lidar_relay`; order and timings: **Startup sequence** below). ~4 s after launch: `startup done (+4 s): N processes running (…)`; running, not ready (Nav2 activates a few s later; `PAUSED` = ready for `/maze/start`) |
| `verbose` | `true` | `false` = Nav2 and slam_toolbox log warnings and errors only (no lifecycle / per-goal chatter). `maze_explorer`, `wall_scan` and `cmd_vel_bridge` always log INFO |

`stairs`, `perception`, `curtains` and `scene_trigger` are independent: switch off any combination.
Navigation only: `ros2 launch maze_nav maze_nav.launch.py stairs:=false perception:=false
scene_trigger:=false curtains:=false`. A switched-off part starts no nodes and is disconnected from the
explorer (`stairs_topic` / `look_topic` / `qr_unread_topic` / `curtain_topic` / `curtains_topic` blanked), so a stray publisher can't stop the robot.

### Startup sequence

The launch is **not staggered**: all 19 processes start in the same ~0.1 s, and the order below
comes from what each one waits for, not from timers. Times are from the 2026-11-20 05:54 robot run
(full launch, perception on):

| t (s) | What happens | Waits for |
|---|---|---|
| +0 | `ros2 launch`; the domain is forced to 4 for a real run | |
| +2.3 | All processes start together: Fast-LIO, `odom_relay`, `cpu_monitor`, `lidar_relay`, `wall_scan`, slam_toolbox, `cmd_vel_bridge`, the Nav2 servers + lifecycle manager, `scene_trigger`, `maze_explorer`, `depth_relay`, `qr_scanner`, `stair_detector`, `curtain_detector` | |
| +2.6 | Lifecycle manager starts configuring Nav2 (planner, controller, behaviors, bt_navigator, velocity smoother) | |
| +3.4 | `lidar_relay` / `depth_relay` subscribe to the bridge (one copy of each stream) | robot bridge |
| +4.0 | `maze_explorer` `[STARTING]` | `/map`, TF `map->base_link`, Nav2 |
| +4.3 | `cmd_vel_bridge`: robot already standing → Move mode, then gait (hstep) | `/maze/body_height` |
| +4.9 | `wall_scan` measures the floor (`floor at odom z = ...`) and starts `/wall_scan`; slam_toolbox registers it and publishes the first `/map` + `map->odom` | Fast-LIO odom, a steady standing height |
| until +5 | global costmap logs `Invalid frame ID "map"` (normal: no map yet) | `map` frame |
| +6.3 | `startup done (+4 s): 19 processes running` (running, not ready) | |
| +6.5 | `Managed nodes are active` (Nav2 up) | |
| +11 | `[PAUSED] ready`: send `/maze/start` | |

The brief CPU spike as everything starts (818 % of 1000 % at +4 s in that run) caused no TF or
lidar lag: no `no TF within` warnings in the first 40 s of any robot run on 2026-11-20 except
05:29-05:31, when a simulator was starting on the same machine. The staggered start (5 s per
stage) was removed on 2026-11-19. It had been meant to fix exactly that lag, but the cause turned out
to be too many local subscribers each taking a copy of the lidar/camera streams over the bridge
(see `lidar_relay`, `depth_relay`), not the start order.

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
ros2 topic echo /maze/status          # STARTING → PAUSED → CREEPING → SCANNING → EXPLORING (⇄ TURNING: turn, or creep ahead and turn, before a goal near a wall, ⇄ LOOKING: look-around in a QR room, ⇄ FACING_QR: turn to an unreadable QR code until it is read, ⇄ JUNCTION: face each way on at a junction, left first, then visit any way ending in a dead end (cell before it, square up, 3 s) and take the rightmost other way, ⇄ DEAD_END: hold 0.5 s facing a dead end, then look at its left and right walls (1 s each), ⇄ STAIRS → AT_STAIRS: stair line-up, hold until /maze/start, ⇄ CURTAIN (→ AT_CURTAIN): walk through a curtain, hold until /maze/start if the sensors stay covered) → EXITED | NO_EXIT | STOPPED

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
| `wall_scan` | `maze_nav` | `/livox/points_crop` (`/livox/points_local` with `lidar_crop:=false`, `/livox/points` with `lidar_relay:=false`) → `/wall_scan` (LaserScan in `base_link`). Keeps only points 0.30–0.45 m above the floor, measured in `odom`, so walls (0.45 m) and hand rails are in, while ramps, mats and stairs (≤ 0.26 m) are out. Replaces the lab `/scan`, which starts at ~0.45 m and misses the maze walls |
| `slam_toolbox` | `slam_toolbox` (online async) | Builds `/map` from `/wall_scan`, publishes `map → odom`. Config `config/slam_toolbox.yaml` |
| `maze_explorer` | `maze_nav` | Every 1 s: finds frontiers (free cells next to unknown) on `/map`, sends the nearest reachable one (≥ 0.33 m from walls; frontiers narrower than 80 % of the robot width, 0.37 m, or not reachable along free space, e.g. seen through a slit, are ignored) to `/navigate_to_pose`, and blacklists each finished goal. Stops with **EXITED** when there is ≥ 1.1 m of seen-free floor within 0.8 m of the robot and ≥ 1.5 m from the start, held for 2 s. Stops with **NO_EXIT** when nothing is left to explore (cancels the running goal, sends a zero Twist and prints `nav stopped`) or goals keep failing after 3 recovery rounds (`max_recoveries`), and with **STOPPED** after 15 min or on Ctrl+C. Before a goal whose first turn on the spot would hit a mapped wall it turns the robot itself (`TURNING`; blocked both ways: left to Nav2 - the creep ahead to a spot with more room, `pre_turn_max_creep`, is off since 2026-11-21). Publishes `/maze/status` and `/maze/goal` |
| `bt_navigator` | Nav2 | Runs `config/maze_bt.xml`: replan at 1 Hz → follow path. Recoveries: clear costmaps, spin 90°, wait 3 s (no BackUp, because the bridge never reverses) |
| `planner_server` | Nav2, Smac Planner 2D | Plans on the global costmap (frame `map`) and **through unknown space** (`allow_unknown`), 0.25 m goal tolerance |
| `controller_server` | Nav2, Regulated Pure Pursuit | Follows the path at `max_speed` and turns on the spot first when the heading is off by > 34° (`rotate_to_heading_min_angle` 0.6, = explorer `pre_turn_min_angle`; smaller errors are arcs while walking). Lookahead 0.18 m (0.15-0.25), so it cuts corners by less than that. Angular acceleration 0.5 rad/s² (RPP, velocity smoother, spin). Stops if the footprint would hit an obstacle within 1.5 s. Goal reached within 0.20 m, heading ignored |
| `behavior_server` | Nav2 | `spin` and `wait` recoveries |
| `velocity_smoother` | Nav2 | Limits acceleration, max 0.25 m/s and 0.5 rad/s, never backwards |
| `lifecycle_manager_navigation` | Nav2 | Brings the five Nav2 servers up |
| `stair_detector` | `maze_nav` | Detects stairs in the depth image (`/camera/depth_relay/image_raw` from `depth_relay`, or the raw depth if team_perception isn't built; 5 Hz, ~30 ms a frame, ~0.2 core) and, with the explorer's handshake on `/stairs/status` ↔ `/maze/status`, lines the robot up square in front of them on `nav_cmd_vel` (≤ 0.08 m/s, ≤ 0.35 rad/s), checking every move against walls on `/map`. Camera pose from `base_link → camera_link` (the robot's TF has a wrong second parent for the depth camera, `agent.md` §12). See `maze_nav/stair_detector.py` |
| `curtain_detector` | `maze_nav` | Curtains in `/livox/points` + depth (2 Hz on the last 5 clouds / 3 depth frames) → `/curtain/curtains` (latched JSON, odom; `wall_scan` and the explorer clear them) and, with the explorer's handshake on `/curtain/status` ↔ `/maze/status`, walks the robot through on `nav_cmd_vel` (0.7 × `max_speed`, ≤ 0.35 rad/s), checked against walls on `/map`. Depth (`/camera/depth_relay/image_raw` + `camera_info`, only through the relay) for detection at 0.6–2 m (`curtain_sensors`) and the covered-lens check. See `maze_nav/curtain_detector.py` |
| `depth_relay` | `team_perception` | Takes the raw depth from the robot bridge once and passes it on at 10 Hz on `/camera/depth_relay/*`, for `stair_detector` and perception. Runs when `stairs` or `perception` is on |
| `scene_trigger` | `maze_nav` | `/scene/trigger` (JSON, `kind: dead_end`: dead-end cell, robot cell, sides on `/map`, side-wall middles) + a `/junction_detected` pulse when the robot faces a dead end from the cell before it; `/scene/junction` (JSON: open ways on, map headings) near the centre of each new junction cell; while `/maze/status` says the run is under way. Grid from the robot's pose when Fast-LIO started: start the robot square on the start cell |
| `cpu_monitor` | `maze_nav` | CPU per process every 2 s (`cpu_log`) |
| `lidar_relay` | `maze_nav` | `/livox/points` → `/livox/points_local` (raw, one bridge copy for all local readers; `lidar_relay`) and `/livox/points_crop` (`lidar_crop`). Logs clouds/s and median delay every 30 s; warns when no clouds come. If it dies, walls, stairs and curtains go blind and `cmd_vel_bridge` stops the robot |
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
| `/curtain/status`, `/curtain/curtains` | `std_msgs/String`, latched (status; JSON) | `curtain_detector` (`IDLE`, `HANDOVER`, `FACING`, `APPROACHING`, `THROUGH`, `WIGGLING`, `CREEPING`, `CHECKING`, `PASSED`, `STUCK`, `ABORTED`) |
| `/maze/floor_z` | `std_msgs/Float32`, latched | `wall_scan` (floor height in `odom`, once measured) |
| `/scene/trigger`, `/junction_detected`, `/scene/junction` | `std_msgs/String` (JSON), `std_msgs/Bool`, `std_msgs/String` (JSON) | `scene_trigger` |

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
| `SENSOR STALE` from cmd_vel_bridge, robot stops briefly | `/livox/points` or `/odom` more than 0.5 s late (robot bridge busy) | Nothing: it drives on by itself once both have been steady for 1 s (`sensor steady again`). Often it means extra copies of the lidar or camera streams on the bridge (RViz, a bag, `ros2 topic hz`) |
| `ODOM JUMP` from cmd_vel_bridge, robot stops ~3 s | Fast-LIO's `/odom` jumped > 0.3 m in one message or moved > 1 m/s (pose slipped, often turning close to a wall) | Nothing: once `/odom` has had no new jump for 2 s and cloud + odom are fresh for 1 s more, it clears both costmaps and drives on (`… forwarding Nav2 again; costmaps cleared`). The SLAM map may still be off after a big jump: watch RViz |
| `SENSOR FAULT` from cmd_vel_bridge, robot frozen | More than 3 `/odom` jumps within 60 s (`… jumps in 60 s (odometry diverged?)`: Fast-LIO diverged), or `/livox/points` or `/odom` silent for more than 3.5 s | Check Fast-LIO and the lidar. Restart the launch. `odom_jump_limit:=0` on cmd_vel_bridge = latch at the first jump (old behaviour) |
| Robot stalls at the stairs | Gait, not Nav2 (Nav2 treats the stairs as floor) | Operator switches gait on the handheld controller. With `stairs:=true` it waits in `AT_STAIRS` for that: switch gait, then `/maze/start` |
| Explorer sits in `AT_STAIRS` | Waiting for the operator, on purpose | Switch gait, then `ros2 topic pub --once --keep-alive 2 /maze/start std_msgs/msg/Empty` |
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
