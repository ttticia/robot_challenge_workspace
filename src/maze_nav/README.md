# maze_nav: get the Lite3 through the maze with SLAM + Nav2

The robot starts inside the maze, maps it with `slam_toolbox` while Nav2 drives it to
the nearest unexplored opening, and stops when it is out (open floor next to it, away from
the start). Nothing about the layout is hard-coded. The example maze
(`src/maze_nav/tools/example maze.step`) is used for the simulator and the tests.

```bash
colcon build --symlink-install --packages-select maze_nav && source install/setup.bash
ros2 launch maze_nav maze_nav.launch.py                 # real run
ros2 launch maze_nav maze_nav.launch.py go:=false       # robot standing still: goals on /maze/goal only
ros2 launch maze_nav maze_nav.launch.py drive:=false    # Nav2 drives, nothing reaches /cmd_vel
ros2 launch maze_nav maze_nav.launch.py stairs:=false perception:=false scene_trigger:=false   # navigation only
```

Optional parts, each on by default and each switched off on its own: `stairs` (stair
line-up), `perception` (team_perception: QR codes, posters, look-around and QR facing),
`scene_trigger` (scene understanding), `cpu_log`. A switched-off part starts no nodes, and
the explorer stops listening to its topic, so a stray publisher can't stop the robot. If one
dies mid-run, navigation carries on.

**Before a real run:** robot **standing on the start cell** (the floor height is measured
at launch), area clear, operator holding the handheld controller. The launch sets the RUG gait (General gear) on
`/simple_cmd`; the operator sends `lite3_auto` once the log says `gait sent` (`gait:=none` skips the gait), nothing else running SLAM/Nav2 or publishing `/cmd_vel` (`ros2 node list`). Watch
`/maze/status`. Once it reads `PAUSED`, start the recording (`rec_nav`), then send
`ros2 topic pub --once /maze/start std_msgs/msg/Empty`; the robot waits for this
(`wait_for_start:=false` skips it). It then drives 10 cm straight ahead (`CREEPING`) and turns 45° left, 90° right and 45° left again (`SCANNING`) before exploring. The run ends with `EXITED`, `NO_EXIT` or `STOPPED`, and the map is saved to
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
| `maze_explorer` | Frontier exploration on `/map`: picks the nearest reachable frontier (by path length along free space) where the footprint fits (≥ 0.33 m from walls) and sends it to `/navigate_to_pose`. Every finished goal is blacklisted. Stops on exit, when nothing is left (`NO_EXIT`), when goals keep failing after 3 recovery rounds, or after `max_runtime`. Extra states on `/maze/status`: `TURNING` (it turns the robot itself before a goal when Nav2's turn on the spot would hit a mapped wall; blocked both ways, it first creeps up to `pre_turn_max_creep` 0.6 m ahead to the spot with the most room), `LOOKING` (perception asked for a look-around in a QR room), `FACING_QR` (turns to face a QR code seen but not read), `STAIRS` / `AT_STAIRS` (stair line-up, below) |
| Nav2: Smac 2D planner (plans through unknown), Regulated Pure Pursuit (turns on the spot), behaviors (spin, wait), `bt_navigator` (`config/maze_bt.xml`), `velocity_smoother` | goal → `nav_cmd_vel` |
| `stair_detector` | Stairs from the depth camera (risers: ≥ 2 faces each one rise tall, a tread apart). Confirmed within 1.5 m, with the robot in front of them (≤ 0.35 m off their centre line, `trigger_max_offset`) → `HANDOVER`: the explorer cancels Nav2 (status `STAIRS`), then it turns the robot to face the stairs, drives onto their centre line, stops with `base_link` 0.45 m from the first riser and squares up (±2.5°) → `AT_STAIRS`. The explorer holds until `/maze/start` (switch gait first). Every turn and drive is checked against walls on `/map` (`wall_check`, footprint + 3 cm, else 1 cm); when the line-up is blocked or takes over 45 s it still stops and holds, as `AT_STAIRS: NOT LINED UP: <reason>`, for a hand line-up. The camera pose comes from `base_link → camera_link` (the robot's TF has a second, wrong parent for the depth camera; `agent.md` §12). Status on `/stairs/status`, per-frame hits on `/stairs/detection`, RViz pose on `/stairs/pose`. `stairs:=false` = off |
| `scene_trigger` | `/scene/trigger` (JSON: cell, the way it came in, which sides look open) + a `/junction_detected` pulse the first time the robot reaches each 1 m cell, start cell included, only while `/maze/status` says the run is under way. The cell grid starts at the robot's pose when Fast-LIO started. `scene_trigger:=false` = off |
| `depth_relay` (team_perception) | One copy of the raw depth from the robot bridge, passed on at 10 Hz to `stair_detector` and perception (`/camera/depth_relay/*`). Runs when `stairs` or `perception` is on |
| `cpu_monitor` | CPU per process to `~/challenge_ws/logs/cpu/` (`cpu_log:=false` = off) |
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
| Stalls on stairs | The gait, not Nav2: the operator switches gait on the handheld controller; Nav2 treats the stairs as flat floor. With `stairs:=true` the robot stops square in front of them first (`AT_STAIRS`): switch gait, then `ros2 topic pub --once /maze/start std_msgs/msg/Empty` |
| Never stops at the stairs | `ros2 topic echo /stairs/detection` while facing them: nothing = depth/TF missing (`waiting for TF` in the log) or the stairs are out of the 0.06–0.22 m rise (≥ 0.075 m on average) / 0.10–0.50 m tread range; `confirmed: false` = frames disagree. `stair_detector` only looks while `/maze/status` is `EXPLORING`, `TURNING`, `LOOKING` or `FACING_QR`, and only takes over in front of the stairs |
| Stops at the stairs with `NOT LINED UP` | Walls on the map left no room to turn or square up (the cell before the stairs leaves the turning circle about 1 cm). Line the robot up by hand, then `/maze/start` |
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
so treat the robot run as the real test. It also publishes no depth image, so stairs are
never detected there: test the stair line-up on the robot or with a rendered-depth rig.
Since 2026-11-16 some runs end `NO_EXIT` with the robot pinned on the stair handrail just
past the stairs; see the open items in `agent.md` §12.
