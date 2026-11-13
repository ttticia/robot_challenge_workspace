# nav_straight

Starts SLAM, then Nav2 walks the Lite3 straight ahead and stops before the first obstacle.
Fully autonomous once launched. It does **not** switch the robot to auto mode; the
operator does that.

```
lite3_bringup (lite3_launcher)   Fast-LIO + odom_relay + /scan    -> /odom, odom->base_link, /scan
slam_toolbox (mapping)           /scan                            -> /map, map->odom
straight_goal                    /odom                            -> straight Path on /follow_path
Nav2 controller_server (RPP)     path + voxel costmap (/livox/points) -> cmd_vel_nav
Nav2 velocity_smoother           cmd_vel_nav                      -> cmd_vel_smoothed
Nav2 collision_monitor           cmd_vel_smoothed + /livox/points -> nav_cmd_vel
cmd_vel_bridge                   nav_cmd_vel                      -> /cmd_vel (robot QoS)
straight_goal (at the end)                                        -> /simple_cmd manual mode
```

Only the controller part of Nav2 runs: no planner, no bt_navigator, no recovery
behaviours. The goal is a fixed straight line, so Nav2 never plans *around* an obstacle;
it stops in front of it. Everything runs in the `odom` frame, so SLAM corrections don't
bend the line.

The two Python nodes are glue only:
- `straight_goal` builds the line from the current pose, sends it to `/follow_path`, and
  sends manual mode when the action ends.
- `cmd_vel_bridge` is needed because the robot's bridge only hears a RELIABLE +
  TRANSIENT_LOCAL `/cmd_vel` (robot_info.md §6.2) and Nav2 publishes VOLATILE. It also
  clamps to 0.25 m/s / 0.5 rad/s and stops on a sensor dropout.

## Run on the robot

```bash
# 0. Check nothing else is running / driving
ros2 node list                       # no other slam / nav2 / maze / fastlio
ros2 topic info -v /cmd_vel          # only the bridge's subscription, no publishers

# 1. Build (once)
cd ~/challenge_ws && colcon build --symlink-install --packages-select nav_straight
source install/setup.bash

# 2. Dry run first: SLAM + Nav2, but commands go to /cmd_vel_bridge/cmd_vel_preview
ros2 launch nav_straight nav_straight.launch.py drive:=false

# 3. Real run: robot standing, path clear, operator holds the handheld STOP, then
lite3_auto
ros2 launch nav_straight nav_straight.launch.py
```

- **Stand the robot up before you launch.** `odom_relay` takes the height at launch as
  z = 0 and assumes the floor is 0.35 m below. Launched while lying down, the floor is
  only ~0.08 m below `base_link`, lands in the obstacle band, and the run stops at once
  with `Travelled 0.00 m`. Standing it up after launch doesn't fix it; relaunch.
- **Give it time.** Startup takes about 3–5 s. `Timed out waiting for transform from
  base_link to odom` repeats until Fast-LIO and `odom_relay` are up; that's normal. Wait
  for `Nav2 ready; starting in 3 s`, then the countdown.
- **Don't record raw depth while driving.** With `ros2 bag record` of
  `/camera/depth/image_raw` running, `/livox/points` arrived ~1.2 s late, collision_monitor
  ignored it (`timestamps differ on 1.2 seconds. Ignoring the source.`) and held the robot
  at zero. Normal lag is ~0.14 s. See [ROSBAG_GUIDE.md](../team_perception/ROSBAG_GUIDE.md).
- Dry run check: `ros2 topic echo /cmd_vel_bridge/cmd_vel_preview` shows `linear.x: 0.1`
  with small `angular.z` corrections. The robot doesn't move, so the progress checker
  aborts after 10 s; that's expected in a dry run.
- Logs: each node also writes `~/.ros/log/<node>_<pid>_*.log` (Python nodes:
  `python3_<pid>_*.log`). Or `ros2 launch … 2>&1 | tee ~/challenge_ws/runs/straight_$(date +%H%M).log`
  and `grep -E '\[(straight_goal|controller_server)'`.

RViz layout: `rviz2 -d $(ros2 pkg prefix nav_straight)/share/nav_straight/config/nav_straight.rviz`.

`straight_goal` waits for `/odom` and for Nav2 to be active, counts down `start_delay`
seconds, then sends the path. The run ends (robot stopped, then manual mode) when:

| Reason | Who stops it | Log line (`straight_goal`) |
|---|---|---|
| Obstacle within `stop_distance` of the front edge | RPP: `detected collision ahead!` → abort | `STOPPED: Nav2 aborted: obstacle ahead…` |
| Obstacle already that close at the start | same, on the first cycle | same, `Travelled 0.00 m` |
| Walked `max_distance` (±0.10 m goal tolerance) | goal checker | `STOPPED: reached max_distance` |
| No cloud or odometry for 0.5 s | `cmd_vel_bridge` stops and latches; `straight_goal` cancels | `STOPPED: sensor timeout` |
| Something appears inside 0.35 m (backstop) | collision_monitor holds 0; progress checker aborts after 10 s | `STOPPED: Nav2 aborted…` |
| Ctrl+C | goal cancelled, bridge sends zero | `STOPPED: shutdown (Ctrl+C)` |

If it doesn't move:

| Symptom | Cause | Fix |
|---|---|---|
| `Timed out waiting for transform … odom` for a few seconds | normal startup | wait for `Nav2 ready` |
| `detected collision ahead!` at once, `Travelled 0.00 m`, nothing in front | robot was lying down at launch (floor in the obstacle band) | stand up, relaunch |
| `driving: 0.01 m done…` but no motion; collision_monitor `Ignoring the source` | lidar arriving > 0.5 s late (heavy rosbag recording) | stop the recording / drop raw depth |
| `cmd_vel_bridge` says `DRY RUN` | `drive:=false` | relaunch without it |
| Moves in RViz/preview but not the robot | not in auto mode | operator sends `lite3_auto` |

Nav2 Humble's FollowPath has no error code, so "aborted" lumps obstacle / held /
controller error together. The `controller_server` log line says which.

SLAM keeps running after the stop. To save the map, call
`ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/user/challenge_ws/maps/straight'}}"`
before Ctrl+C.

## Launch arguments

| Argument | Default | |
|---|---|---|
| `drive` | `true` | `false` = dry run |
| `max_speed` | `0.10` | m/s; RPP `desired_linear_vel`. Bridge clamps to 0.25 |
| `stop_distance` | `0.50` | m between the robot's front edge and the obstacle; sets RPP's lookahead |
| `max_distance` | `5.0` | m; length of the line (must be > 0) |
| `start_delay` | `3.0` | s |
| `bringup` | `true` | `false` if Fast-LIO / `/scan` already runs |
| `slam` | `true` | `false` = no slam_toolbox |
| `udp_only` | `true` | UDP-only Fast DDS (Nav2 lifecycle stalled on the robot without it) |

Everything else (footprint, heights, slow/stop zones, gains) is in
[config/nav2_params.yaml](config/nav2_params.yaml). The collision_monitor zones are fixed
in that file and don't follow `stop_distance`.

## How obstacles are detected and the stop works

`/scan` from the lab launcher only covers 0.45–0.85 m above the floor and would miss boxes
and steps, so Nav2 reads the raw `/livox/points`:

- **Local costmap, `VoxelLayer`** (odom frame, floor at z = −0.35): marks points from
  0.10 m to 1.05 m above the floor (`max_obstacle_height: 0.70`; 16 voxels × 0.06 m, the
  VoxelLayer maximum). It was 0.75 m until 2026-11-13, which ignored table tops: the robot
  stopped 0.6 m from the table legs but only 0.3 m (head) from the overhanging top.
  Footprint ±0.32 × ±0.30 m (body with feet + 7 cm side margin).
- **RPP collision check**: each cycle it projects the footprint forward along the line up
  to the carrot (`lookahead_dist` = `stop_distance`) and aborts if it touches a lethal
  cell. So the robot stops about `stop_distance` before the obstacle (in simulation
  0.59–0.64 m for 0.50: one or two costmap cells early).
- **collision_monitor** on the live cloud: half speed within 1.2 m of the front edge,
  emergency stop within 0.35 m.
- **Heading**: pure pursuit on the start line holds heading and steers back onto it.

**Lidar blind zone.** The Mid-360's lowest ray is −7°. With the 15° nose-down mount that's
about −22° straight ahead, so a 10 cm box drops out of view at about 0.8 m clearance. The
voxel layer clears in 3D, so rays to things behind the box pass over it and the box stays
in the costmap: the RPP stop still works. collision_monitor has no memory and won't see
it, so it only slows down. The flip side: **a low obstacle that is already within about 1 m
when the run starts is not seen.** Start with at least 1 m of clear floor ahead.

## Test without the robot

Use a spare domain that nothing else is on (check `ros2 node list` is empty first):

```bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1      # never domain 4
python3 src/nav_straight/test/fake_cloud_lite3.py --ros-args -p scene:=low_box &
ros2 launch nav_straight nav_straight.launch.py bringup:=false slam:=false start_delay:=1
```

Scenes: `wall` (3 m ahead), `low_box` (15 cm box 2 m ahead), `open`. The simulator adds
0.05 rad/s of yaw drift to exercise the heading hold. Unit tests:
`python3 -m pytest src/nav_straight/test`.

Simulator results with Nav2 (2026-11-13, domain 91):

| Scene | Result |
|---|---|
| wall | RPP stop after 2.08 m, 0.59 m clearance; slowed at 1.2 m |
| low_box | RPP stop after 1.04 m, 0.64 m true clearance (box out of view for the last ~0.2 m) |
| open, `max_distance:=1` | goal reached at 0.91 m (goal tolerance 0.10) |
| Ctrl+C mid-run | goal cancelled, zero Twist + manual mode |
| simulator killed mid-run | bridge stop within 0.5 s, `STOPPED: sensor timeout`, manual mode |

Lateral offset at the stop was about 10 cm under the simulator's constant yaw drift: pure
pursuit leaves a steady offset where the old custom driver kept it under 6 cm. The old
custom driver is archived in `src/archive/nav_straight_python/`.

After raising the band to 1.05 m: wall scene unchanged (RPP stop after 2.08 m).

## Real robot results (2026-11-13)

| Run | Result |
|---|---|
| Launched while lying down | `detected collision ahead!` on the first cycle, 0.00 m: floor 0.08 m below `base_link` |
| Dry run, standing | preview `linear.x 0.10`, `angular.z` 0.004–0.013 rad/s |
| Real run towards a table (band 0.75 m) | walked 0.74 m at 0.10 m/s, heading −0.3° at start, RPP stop. Lidar: legs/front 0.6 m from the front edge; measured 0.3 m from the head to the overhanging table top. Manual mode sent |
| Real run while recording raw depth | no motion: lidar 1.2 s late, collision_monitor ignored it |

Not yet tried on the robot: the 1.05 m band, a low box (10–20 cm), a full 5 m run.
