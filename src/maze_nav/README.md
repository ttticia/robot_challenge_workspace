# maze_nav: get the Lite3 through the maze with SLAM + Nav2

The robot starts inside the maze, maps it with `slam_toolbox` while Nav2 drives it to
the nearest unexplored opening, and, once every cell of the `maze_cols` x `maze_rows` grid
(default 5 x 3, 1.02 m cells) is explored, parks in the cell inside the exit, facing out
(`TO_EXIT` → `AT_EXIT`; `maze_cols:=0` = the old walk-out exit, `EXITED` on open floor).
Only the grid size is assumed; walls, exit and features are found on the way.

**Changed on 2026-11-22.** Where this README and `agent.md` §3 disagree, `agent.md` and
`src/bt.md` are current. In short: no start scan, `LOOKING` removed, junction scan with
sideways centring, a stairs / curtain candidate queue, dead ends wait for the AI verdict,
the AI-mode stair walk carries on by itself, and the default gait is H-Step. What still
needs testing: `src/test_checklist.md`. The example maze
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
`scene_trigger` (scene understanding), `curtains` (curtain pass-through), `cpu_log`. A switched-off part starts no nodes, and
the explorer stops listening to its topic, so a stray publisher can't stop the robot. If one
dies mid-run, navigation carries on.

**Before a real run:** robot **standing on the start cell**, square to it (the floor height is measured
at launch; not standing → an ERROR every 5 s and startup waits, `wait_for_stand`), area clear, operator holding the handheld controller. The launch sets the H-Step gait on
`/simple_cmd` (`gait:=rug` = RUG General gear); the operator sends `lite3_auto` once the log says `gait sent` (`gait:=none` skips the gait), nothing else running SLAM/Nav2 or publishing `/cmd_vel` (`ros2 node list`). Watch
`/maze/status`. Once it reads `PAUSED`, start the recording (`rec_nav`), then send
`ros2 topic pub --once /maze/start std_msgs/msg/Empty`; the robot waits for this
(`wait_for_start:=false` skips it). It then drives 10 cm straight ahead (`CREEPING`) and starts exploring (the start scan, `SCANNING`, only runs with `start_scan:=true`). The run ends with `AT_EXIT`, `NO_EXIT`, `STOPPED` or (no grid, or outside it) `EXITED`, and the map is saved to
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
| `wall_scan` | `/livox/points` → `/wall_scan`: keeps only points **0.30–0.45 m above the maze floor** (in `odom`, not `base_link`), so walls (0.45 m) and rails are kept while ramps, mats and stairs (≤ 0.26 m) are dropped. The floor height is measured at startup |
| `slam_toolbox` (online async) | `/wall_scan` → `/map`, `map → odom` |
| `maze_explorer` | Frontier exploration on `/map`: picks the nearest reachable frontier (by path length along free space) where the footprint fits (≥ 0.33 m from walls) and sends it to `/navigate_to_pose`. Every finished goal is blacklisted. Stops on exit, when nothing is left (`NO_EXIT`), when goals keep failing after 3 recovery rounds, or after `max_runtime`. Extra states on `/maze/status`: `TURNING` (it turns the robot itself before a goal when Nav2's turn on the spot would hit a mapped wall; blocked both ways, it first creeps up to `pre_turn_max_creep` 0.6 m ahead to the spot with the most room), `FACING_QR` (turns to face a QR code seen but not read), `JUNCTION` (scene_trigger saw a junction: for each adjacent unvisited cell, left-most first, faces it, moves sideways onto the junction cell's centre line, holds 1 s with QR on and one `/scene/capture`; dead-end ways aren't visited; then works the stairs / curtain candidate queue, then leaves by the nearest unexplored way), `DEAD_END` (facing a dead end from the cell before it: holds 0.5 s, turns the camera to the middle of the dead end's left wall, then its right wall, 1 s at each, never drives into it; with `scene_understanding:=true` waits for the AI verdict on `/scene/result`, 15 s + 1 retry, room → blacklisted, stairs / curtain → candidate queue), `TO_EXIT` / `AT_EXIT` (grid explored: drives to the launch cell, faces the exit, stands still), `STAIRS` / `AT_STAIRS` (stair line-up, below), `CURTAIN` / `AT_CURTAIN` (walking through a curtain, below) |
| Nav2: Smac 2D planner (plans through unknown), Regulated Pure Pursuit (turns on the spot), behaviors (spin, wait), `bt_navigator` (`config/maze_bt.xml`), `velocity_smoother` | goal → `nav_cmd_vel` |
| `stair_detector` | Stairs from the depth camera (risers: ≥ 2 faces each one rise tall, a tread apart). Confirmed within 1.5 m, with the robot in front of them (≤ 0.35 m off their centre line, `trigger_max_offset`) → `HANDOVER`: the explorer cancels Nav2 (status `STAIRS`), then it turns the robot to face the stairs, drives onto their centre line, stops with `base_link` 0.45 m from the first riser and squares up (±2.5°) → `AT_STAIRS`. Lined up, `cmd_vel_bridge` switches to AI mode and walks 1.3 m forward at 0.1 m/s on `/odom` (`ai_walk_distance`, `0` = off), then leaves AI mode by itself (back to auto mode) and publishes `DONE` on `/maze/ai_status`; the explorer resumes. A walk that ended early holds until `/maze/start`. Every turn and drive is checked against walls on `/map` (`wall_check`, footprint + 3 cm, else 1 cm); when the line-up is blocked or takes over 45 s it still stops and holds, as `AT_STAIRS: NOT LINED UP: <reason>`, for a hand line-up. The camera pose comes from `base_link → camera_link` (the robot's TF has a second, wrong parent for the depth camera; `agent.md` §12). Status on `/stairs/status`, per-frame hits on `/stairs/detection`, RViz pose on `/stairs/pose`. `stairs:=false` = off |
| `curtain_detector` | Curtains in `/livox/points` (`maze_nav/curtains.py`): a wall in front whose lowest face point is 5–15 cm above the local floor, with floor returns just behind it (beams went under), over more than 0.7 m. Works from ~1.1 m (the lidar must see the floor at it) to 2.6 m. Confirmed (3 hits in 3 s) curtains go on `/curtain/curtains` (latched JSON, odom): `wall_scan` drops their points (SLAM and the costmaps clear them) and the explorer clears them on its copy of `/map`; goals at a curtain are sent to 0.9 m in front of its middle first. Within 2 m, robot in front (≤ 0.35 m off its centre line) → `HANDOVER`: explorer stops Nav2 (`CURTAIN`); the robot faces the middle, walks through at 70 % of `max_speed` until `base_link` is 0.20 m past it, turns 20° left / 40° right / 20° left, holds 0.8 s and checks the sensors (lidar: > 30 % of returns within 0.5 m or far returns < 40 % of usual; depth: < 25 % of pixels beyond 0.3 m). Blocked: 5 cm creep and the turn sequence, alternately, checking after each, up to 12 steps, then `STUCK` → explorer `AT_CURTAIN` holds until `/maze/start`. Clear → `PASSED`. Moves are checked against walls on `/map` (curtain free). Crosses a passed curtain back only for an explorer goal beyond it. `wall_scan` publishes no scan while the lidar is covered within 1 m of a curtain. `curtains:=false` = off |
| `scene_trigger` | Scene understanding: `/scene/trigger` (JSON, `kind: dead_end`: the dead-end cell, the robot's cell, its sides, the map points of its side-wall middles) + a `/junction_detected` pulse when the robot, in the cell before a dead end, faces it (its other three sides walls on `/map`), once per dead end. Junctions: `/scene/junction` (JSON: cell, headings of the 2+ open ways on) near the centre of each new cell with 2+ ways on besides the way in, for the explorer's `JUNCTION` look. Only while `/maze/status` says the run is under way. The cell grid starts at the robot's pose when Fast-LIO started. `scene_trigger:=false` = off |
| `depth_relay` (team_perception) | One copy of the raw depth from the robot bridge, passed on at 10 Hz to `stair_detector` and perception (`/camera/depth_relay/*`). Runs when `stairs` or `perception` is on |
| `cpu_monitor` | CPU per process to `~/challenge_ws/logs/cpu/` (`cpu_log:=false` = off) |
| `cmd_vel_bridge` | `nav_cmd_vel` → `/cmd_vel` with the robot QoS, clamped to 0.25 m/s and 0.5 rad/s (sideways `linear.y` ≤ `max_lateral` 0.10 m/s, for junction centring), never backwards, stops and latches if `/livox/points` or `/odom` goes stale |
| `room_scene` + `task_reporter` (scene_understanding package) | Dead-end views → VLM queue, verdict on `/scene/result`, resend on `/scene/retry`; `/mission/task_completed`. `scene_understanding:=true` only; the VLM backend isn't wired yet |

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
| Walls missing or broken up in `/map` | Check the logged floor height (`floor at odom z = …`). Widen the band down (`band_low:=0.28`) if stairs aren't in the maze; `band_high` must not exceed the wall top (0.45 m) |
| Stairs or terrain drawn as walls | Raise `band_low` (stair top tread is 0.26 m) |
| Launched lying down | Floor measured wrong: stand up and relaunch, or pass `floor_z:=-0.35` |
| Robot rubs walls in corridors | `footprint` / `inflation_radius` in `config/nav2_params.yaml` |
| Stalls on stairs | The gait, not Nav2: the operator switches gait on the handheld controller; Nav2 treats the stairs as flat floor. With `stairs:=true` the robot stops square in front of them first (`AT_STAIRS`): switch gait, then `ros2 topic pub --once /maze/start std_msgs/msg/Empty` |
| Never stops at the stairs | `ros2 topic echo /stairs/detection` while facing them: nothing = depth/TF missing (`waiting for TF` in the log) or the stairs are out of the 0.06–0.22 m rise (≥ 0.075 m on average) / 0.10–0.50 m tread range; `confirmed: false` = frames disagree. Stairs seen in a junction or dead-end look are only queued (`/stairs/candidate`) and checked after the scan, so `AT_STAIRS` can come a while later. It only takes over in front of the stairs |
| Stops at the stairs with `NOT LINED UP` | Walls on the map left no room to turn or square up (the cell before the stairs leaves the turning circle about 1 cm). Line the robot up by hand, then `/maze/start` |
| Curtain not found (`curtain N found` never logged) | It must be wider than 0.7 m with a 5–15 cm gap, seen from 1.1–2.6 m ahead. A gap under 5 cm reads as a wall (`min_gap`, `maze_nav/curtains.py`); a heavy curtain that touches the floor can't be told from a wall by the lidar |
| A wall taken for a curtain | Stray returns behind it: raise `min_behind` / `min_gap` in `CurtainParams`, or `curtains:=false` |
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
so treat the robot run as the real test. Curtains: `maze:=src/maze_nav/test/curtain_maze.json`
(a 0.9 m corridor with a curtain 2 m ahead of the start, 8 cm gap, out into a room) turns on
lidar occlusion and lays the curtain over the lidar while it is 0–0.45 m past it. Result on
2026-11-19: found at 1.9 m, walked through within 2 cm of its middle, two creeps and a
second turn sequence until the lidar was clear, `PASSED`, then `EXITED`. The plain example
maze has no occlusion (walls are see-through), yet still finds no curtains (walls read a 3–4
cm gap, below `min_gap`). Since 2026-11-22 the simulator also renders a depth camera (640 x 480
at 5 Hz), but at that setting it delays odometry by about 1.5 s and no map forms, so runs
so far used `depth_rate:=0` (no stair detection). The 5 x 3 `challenge_*` mazes test the grid
exit (`agent.md` §9).
Since 2026-11-16 some runs end `NO_EXIT` with the robot pinned on the stair handrail just
past the stairs; see the open items in `agent.md` §12.
