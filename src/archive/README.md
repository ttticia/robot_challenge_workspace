# navigation

Build a map of a space with the Lite3: slam_toolbox makes the map, Nav2 drives to
goals you set in RViz, and the map is saved automatically.

## Run

```bash
cd ~/challenge_ws
colcon build --packages-select navigation --symlink-install
source install/setup.bash
ros2 launch navigation mapping.launch.py
```

Stop any other SLAM/Nav2 stack first (for example `lite3_mission` or `lite3.launch.py`).
Two stacks on the same ROS domain fight over `/map`, TF and `/cmd_vel`.
The robot must already be standing in its walking mode (as for teleop).

Then set goals in RViz on your laptop (same `ROS_DOMAIN_ID`, Fixed Frame `map`,
**2D Goal Pose**), or drive with teleop and let SLAM map as you go:

```bash
ros2 launch navigation mapping.launch.py nav2:=false   # SLAM + map saving only
```

## Where the map is saved

`~/challenge_ws/maps/map_<date>_<time>.pgm` + `.yaml`, rewritten every 30 s and once
more when you press Ctrl+C. `~/challenge_ws/maps/latest.yaml` always points to the newest
map. Both formats are what Nav2's `map_server` loads, for example:

```bash
ros2 launch lite3_launcher lite3.launch.py map:=$HOME/challenge_ws/maps/latest.yaml
```

## Arguments

| Argument | Default | Meaning |
|---|---|---|
| `bringup` | `true` | Start Fast-LIO, odom relay and `/scan` from `lite3_launcher` |
| `nav2` | `true` | Start Nav2 |
| `drive` | `true` | `false` = Nav2 plans but nothing reaches `/cmd_vel` (dry run) |
| `map_dir` | `~/challenge_ws/maps` | Save folder |
| `map_name` | `map_<date>_<time>` | File name without extension |
| `save_interval` | `30.0` | Seconds between saves; `0` = only on shutdown |
| `udp_only` | `true` | UDP-only Fast DDS (Nav2 activation stalled on the robot without it) |

## Nodes

| Node | Role |
|---|---|
| `slam_toolbox` | `/scan` + odometry → `/map` and `map → odom` |
| Nav2 (`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`) | Plans through known and unknown space and outputs `/nav_cmd_vel` |
| `cmd_vel_bridge` | `/nav_cmd_vel` → `/cmd_vel` with RELIABLE + TRANSIENT_LOCAL QoS (needed by the Lite3 bridge), clamps to 0.25 m/s and 0.5 rad/s, and sends one stop when Nav2 goes quiet |
| `map_autosaver` | Writes `/map` to disk periodically and on shutdown |

## Tuning

- `config/nav2_params.yaml`: speeds, footprint, inflation, goal tolerance.
- `config/slam_toolbox.yaml`: `use_scan_matching` is `false` (Fast-LIO odometry is
  trusted, as in `lite3_launcher`). Set it to `true` if walls look doubled or smeared.
- The floor appearing as obstacles means `min_height` in
  `lite3_launcher/config/pointcloud_to_scan.yaml` needs tuning.
