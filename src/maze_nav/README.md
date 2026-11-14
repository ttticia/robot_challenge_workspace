# maze_nav: get the Lite3 through the maze with SLAM + Nav2

The robot starts inside the maze, maps it with `slam_toolbox` while Nav2 drives it to
the nearest unexplored opening, and stops when it is out (open floor next to it, away from
the start). Nothing about the layout is hard-coded. The example maze
(`src/team_perception/nav/example maze.step`) is used for the simulator and the tests.

```bash
colcon build --symlink-install --packages-select maze_nav && source install/setup.bash
ros2 launch maze_nav maze_nav.launch.py                 # real run
ros2 launch maze_nav maze_nav.launch.py go:=false       # robot standing still: goals on /maze/goal only
ros2 launch maze_nav maze_nav.launch.py drive:=false    # Nav2 drives, nothing reaches /cmd_vel
```

**Before a real run:** robot **standing on the start cell** (the floor height is measured
at launch), area clear, operator holding the handheld controller, `lite3_auto` sent by the
operator, nothing else running SLAM/Nav2 or publishing `/cmd_vel` (`ros2 node list`). Watch
`/maze/status`. The run ends with `EXITED`, `NO_EXIT` or `STOPPED`, and the map is saved to
`~/challenge_ws/maps/maze_<date>_<time>.{pgm,yaml}` (`latest.yaml` points to the newest).

## The example maze

Decoded from the STEP file (`tools/step_to_boxes.py` → `test/example_maze.json`). It is
1 m cells on 0.1 m floor tiles, with walls 0.05 m thick and **0.45 m tall**.

Cell centres in metres, start cell at (0, 0), its opening facing +y:

| | x = −3 | x = −2 | x = −1 | x = 0 |
|---|---|---|---|---|
| **y = 2** | | | dead end | |
| **y = 1** | | studded mat (0.1 m) | 4 ramps to 0.15 m (a bowl) | ribbed mat (0.12 m) |
| **y = 0** | **stairs = exit** (3 × 0.12 m steps, rails 0.87 m apart) | plain | outside the maze (a notch) | **START** |

Route: start → north onto the ribbed mat → west over the ramps (the dead end is north of
them) → west onto the studded mat → south → west over the stairs → out into the room.
There is a ~0.1 m slit at the wall corner between the start cell and the ramps that shows
the outside notch to the lidar. That frontier can't be cleared, which is why finished
goals are blacklisted.

## How it works

| Node | Does |
|---|---|
| Fast-LIO + `odom_relay` (from `lite3_launcher`) | `/odom`, `odom → base_link`. Skip with `bringup:=false` |
| `wall_scan` | `/livox/points` → `/wall_scan`: keeps only points **0.30–0.43 m above the maze floor** (in `odom`, not `base_link`), so walls (0.45 m) and rails are kept while ramps, mats and stairs (≤ 0.26 m) are dropped. The floor height is measured at startup |
| `slam_toolbox` (online async) | `/wall_scan` → `/map`, `map → odom` |
| `maze_explorer` | Frontier exploration on `/map`: picks the nearest reachable frontier (by path length along free space) where the footprint fits (≥ 0.33 m from walls) and sends it to `/navigate_to_pose`. Every finished goal is blacklisted. Stops on exit, when nothing is left (`NO_EXIT`), after 3 failed goals in a row, or after `max_runtime` |
| Nav2: Smac 2D planner (plans through unknown), Regulated Pure Pursuit (turns on the spot), behaviors (spin, wait), `bt_navigator` (`config/maze_bt.xml`), `velocity_smoother` | goal → `nav_cmd_vel` |
| `cmd_vel_bridge` | `nav_cmd_vel` → `/cmd_vel` with the robot QoS, clamped to 0.25 m/s and 0.5 rad/s, never backwards, stops and latches if `/livox/points` or `/odom` goes stale |

**Why a separate scan.** The lab `/scan` (`lite3_launcher`) keeps 0.1–0.5 m above
`base_link`, which is about 0.45–0.85 m above the floor, so it misses 0.45 m walls entirely.
A band fixed to `base_link` would also ride up and down with the body on the ramps and stairs.

**Exit test.** "Out" means there is a disk of floor SLAM has *seen* free, with radius
≥ 1.1 m, within 0.8 m of the robot and ≥ 1.5 m from the start, held for 2 s. Maze corridors
never have more than ~0.45 m. Tune with `exit_open_radius`, `exit_near_radius`,
`exit_min_displacement`.

## Tuning on the robot

| Symptom | Fix |
|---|---|
| Walls missing or broken up in `/map` | Check the logged floor height (`floor at odom z = …`). Widen the band down (`band_low:=0.28`) if stairs aren't in the maze; `band_high` must stay below the wall top (0.45 m) |
| Stairs or terrain drawn as walls | Raise `band_low` (stair top tread is 0.26 m) |
| Launched lying down | Floor measured wrong: stand up and relaunch, or pass `floor_z:=-0.35` |
| Robot rubs walls in corridors | `footprint` / `inflation_radius` in `config/nav2_params.yaml` |
| Stalls on stairs | The gait, not Nav2: the operator switches gait on the handheld controller; Nav2 treats the stairs as flat floor |
| Explores outside the maze after leaving | Exit test too strict for the room: lower `exit_open_radius` |

## Testing without the robot

One command (runs, waits for the result, stops everything; exit 0 = EXITED, logs in `/tmp/maze_sim`):
`src/maze_nav/test/run_sim.sh`. Run it on an idle machine: if the CPU is overloaded, Nav2 misses its control rate and the result means nothing. Or by hand:

```bash
python3 -m pytest -q src/maze_nav/test       # logic + offline exploration of the example maze

export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1        # never on the robot's domain
python3 src/maze_nav/test/fake_maze_lite3.py &      # example maze in a 10 x 8 m room
ros2 launch maze_nav maze_nav.launch.py bringup:=false udp_only:=false map_dir:=/tmp/maze_maps
```

The simulator logs `OUT OF THE MAZE` when the robot is past the stairs, and the explorer
should report `[EXITED]`. Result on 2026-11-14: exited about 50 s after the start (start → ribbed mat →
ramps → studded mat → stairs → room) with one blacklisted goal. The map showed clean walls
and no terrain or stairs. The simulator has no lidar noise, no occlusion and no real gait,
so treat the robot run as the real test.
