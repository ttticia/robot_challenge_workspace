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

2. **Robot standing on the start cell.** `wall_scan` measures the floor height at launch.
   If the robot was lying down, stand it up and relaunch.
3. **Operator next to the robot with the handheld controller.** The operator sends `lite3_auto`.
   Our code never switches modes.
4. **No raw depth recording** (`/camera/depth/image_raw`) during the run. It delayed the lidar
   enough to stall Nav2 (see `ROS_bags/ROSBAG_GUIDE.md`).

## 2. Launch

```bash
cd ~/challenge_ws
colcon build --symlink-install --packages-select maze_nav   # first time / after changes
source install/setup.bash
ros2 launch maze_nav maze_nav.launch.py
```

First the launch checks for anything that would clash: another SLAM, Nav2 or maze_nav on
this computer (also Fast-LIO / the simulator with `bringup:=true`), teleop or `twist_mux`,
and on the whole `ROS_DOMAIN_ID` any `/cmd_vel` publisher or node using one of our names.
If it finds any, it prints a `!!!` banner listing them (with PIDs) and pauses 3 s, then
launches anyway. Ctrl+C, stop them, and relaunch.

After launch, once SLAM, TF and Nav2 are up, the robot holds still (`/maze/status` =
`PAUSED`) until you tell it to go. Start the rosbag recording first (`rec_nav`, see
`ROS_bags/ROSBAG_GUIDE.md`), then in another terminal:

```bash
ros2 topic pub --once /maze/start std_msgs/msg/Empty
```

A start sent before `PAUSED` is ignored (the log says so); send it again. Next it turns 90° left, 180° right and 90° back
to its start heading so SLAM sees the start area, then starts exploring.

Recommended first run on a new maze: `go:=false`, check the map and goals in RViz, then
relaunch without it.

| Argument | Default | Meaning |
|---|---|---|
| `bringup` | `true` | Start Fast-LIO + `odom_relay`. `false` if they already run (or for the simulator) |
| `go` | `true` | `false`: explorer picks goals and publishes them on `/maze/goal` but never sends them. The robot stays still |
| `drive` | `true` | `false`: Nav2 drives, but commands go to `/cmd_vel_bridge/cmd_vel_preview`, not `/cmd_vel` |
| `wait_for_start` | `true` | Hold still once everything is up until `/maze/start` is published. `false` = start right away (simulator) |
| `start_scan` | `true` | Turn 90° left and right on the start cell before exploring. Skipped with `go:=false` |
| `max_speed` | `0.15` | m/s for the path follower (the bridge clamps to 0.25 m/s, 0.5 rad/s anyway) |
| `band_low` / `band_high` | `0.30` / `0.43` | Wall slice, metres above the floor. Below `band_low` = walkable (ramps, mats, stairs). `band_high` must stay below the wall top (0.45 m) |
| `floor_z` | `nan` | Floor height in `odom`. `nan` = measure at launch. Set it (e.g. `-0.35`) if you have to launch while the robot is lying down |
| `map_dir` | `~/challenge_ws/maps` | Where the final map is saved |
| `udp_only` | `true` | UDP-only Fast DDS profile (Nav2 stalled on the robot with shared memory) |
| `use_sim_time` | `false` | |

### Watching a run

On a laptop on the same network with `ROS_DOMAIN_ID=4`:

```bash
ros2 topic echo /maze/status          # STARTING → PAUSED → SCANNING → EXPLORING → EXITED | NO_EXIT | STOPPED
rviz2                                  # Fixed frame: map. Add /map, /wall_scan (LaserScan),
                                       # /maze/goal (Pose), /plan, /local_costmap/costmap
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
| `maze_explorer` | `maze_nav` | Every 1 s: finds frontiers (free cells next to unknown) on `/map`, sends the nearest reachable one (≥ 0.33 m from walls) to `/navigate_to_pose`, and blacklists each finished goal. Stops with **EXITED** when there is ≥ 1.1 m of seen-free floor within 0.8 m of the robot and ≥ 1.5 m from the start, held for 2 s. Stops with **NO_EXIT** when nothing is left to explore (cancels the running goal, sends a zero Twist and prints `nav stopped`) or 3 goals fail in a row, and with **STOPPED** after 15 min or on Ctrl+C. Publishes `/maze/status` and `/maze/goal` |
| `bt_navigator` | Nav2 | Runs `config/maze_bt.xml`: replan at 1 Hz → follow path. Recoveries: clear costmaps, spin 90°, wait 3 s (no BackUp, because the bridge never reverses) |
| `planner_server` | Nav2, Smac Planner 2D | Plans on the global costmap (frame `map`) and **through unknown space** (`allow_unknown`), 0.25 m goal tolerance |
| `controller_server` | Nav2, Regulated Pure Pursuit | Follows the path at `max_speed` and turns on the spot first when the heading is off by > 34°. Stops if the footprint would hit an obstacle within 1.5 s. Goal reached within 0.20 m, heading ignored |
| `behavior_server` | Nav2 | `spin` and `wait` recoveries |
| `velocity_smoother` | Nav2 | Limits acceleration, max 0.25 m/s and 0.5 rad/s, never backwards |
| `lifecycle_manager_navigation` | Nav2 | Brings the five Nav2 servers up |
| `cmd_vel_bridge` | `maze_nav` | `nav_cmd_vel` → `/cmd_vel` with the QoS the robot bridge needs (RELIABLE + TRANSIENT_LOCAL). Clamps to 0.25 m/s and 0.5 rad/s, never backwards. Sends one stop when Nav2 goes quiet. Stops and blocks if `/livox/points` or `/odom` is silent for 0.5 s |

### Costmaps and footprint (`config/nav2_params.yaml`)

| | Local costmap | Global costmap |
|---|---|---|
| Frame / size | `odom`, rolling 3 × 3 m | `map`, grows with `/map` |
| Layers | `/wall_scan` obstacles + inflation | `/map` (static) + `/wall_scan` obstacles + inflation |
| Resolution | 0.05 m | 0.05 m |

- **Footprint:** 0.69 × 0.46 m rectangle: the Lite3 (0.64 m including moving feet) plus
  5 cm extra at the back. x from −0.37 to +0.32 m, y ±0.23 m. In a 0.9 m maze corridor that
  leaves 0.22 m each side. The rear corners are 0.44 m from `base_link`, just under the
  corridor's 0.45 m half width, so turning on the spot in a cell centre leaves only about
  1.5 cm to spare.
- **Inflation:** radius 0.45 m, cost scaling 6. This keeps paths in the middle of the corridor.
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

## 4. When something goes wrong

| What you see | Likely cause | Do |
|---|---|---|
| `more than one action server for the action 'navigate_to_pose'`, or `unknown goal response` from bt_navigator | Another Nav2 (usually `lite3.launch.py`) is running | Stop it, relaunch (section 1) |
| `[NO_EXIT] all frontiers mapped` (then `nav stopped`) a few seconds after start | Same as above, or `/map` has no walls | Check `ros2 node list`. In RViz, `/wall_scan` should outline the walls |
| `floor seen at …, expected …` warning | Robot not standing at launch, or no floor visible | Stand up and relaunch, or pass `floor_z:=-0.35` |
| Walls missing or broken in `/map` | Band too high or floor wrong | Check the logged floor height. Try `band_low:=0.28` |
| Ramps or stairs drawn as walls | Band too low | Raise `band_low` (the top stair tread is 0.26 m) |
| Robot rubs a wall on turns | Footprint at the limit (1 cm margin) | Increase `inflation_radius`, or shrink or re-centre the footprint |
| Robot stops at a goal and turns slowly, then the goal fails | Progress checker (0.15 m in 20 s) | Raise `movement_time_allowance` |
| `SENSOR TIMEOUT` from cmd_vel_bridge, robot frozen | `/livox/points` or `/odom` stopped | Check Fast-LIO and the lidar. Restart the launch |
| Robot stalls at the stairs | Gait, not Nav2 (Nav2 treats the stairs as floor) | Operator switches gait on the handheld controller |

## 5. Without the robot

One command (runs, waits for the result, stops everything; exit 0 = EXITED, logs in `/tmp/maze_sim`):
`src/maze_nav/test/run_sim.sh`. Run it on an idle machine: if the CPU is overloaded, Nav2 misses its control rate and the result means nothing. Or by hand:

```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1        # never the robot's domain
python3 src/maze_nav/test/fake_maze_lite3.py &      # example maze in a 10 x 8 m room
ros2 launch maze_nav maze_nav.launch.py bringup:=false udp_only:=false map_dir:=/tmp/maze_maps
```

The simulator logs `OUT OF THE MAZE` once the robot is past the stairs, and `/maze/status` should end at `EXITED`.
Unit tests: `cd src/maze_nav && python3 -m pytest -q test`.
