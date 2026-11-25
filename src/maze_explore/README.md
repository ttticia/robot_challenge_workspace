# maze_explore

Navigation and exploration for the Lite3 in the challenge maze (branch `exploration-v2`).
The robot maps with SLAM, treats the maze as a grid of 1.02 m cells, and walks from cell
centre to cell centre. It does not use Nav2.

Status (2026-11-23): dry-run on the robot (no driving) and one simulated run. **It has not
driven the real robot yet.** Curtain and stairs recognition are untested on the robot.

## Run it

```bash
explore_check                                   # must say GO (lidar IMU near 200 Hz)
ros2 launch maze_explore explore.launch.py      # terminal 1, robot standing in the start cell
rec_test run1                                   # terminal 2 (optional bag)
explore_go                                      # terminal 3: start exploring
explore_stop                                    # stop: zero velocity, stays stopped
```

Put the robot in the start cell facing roughly into the maze. It may be off-centre (up to
about 0.3 m) and at an angle (up to about 30 degrees): the walls are used to line up.

Useful launch arguments: `drive:=false` (decides and logs, sends nothing), `explore:=false`
(mapping only, for hand-driven bags), `v_max:=0.15`, `gait:=rug` (RUG general gear; the default is `hstep`, H-Step, since 2026-11-24).

Navigation only: `plain:=true` turns off the scene understanding trigger, stairs, curtains and
terrain handling in one go. Each also has its own switch: `scene_understanding:=false`,
`stairs:=false` (no depth check, no AI mode; stairs are a blocked cell), `curtains:=false` (a
curtain is a wall: not faced, not pushed), `terrain:=false` (uneven cells are walked into head
first). `task_reporter` is not touched by `plain` (`report:=false` turns it off).

Watch `ros2 topic echo /explore/status --field data` and the launch log (`job: ...` lines).
RViz, live or on a replayed bag: `rviz2 -d config/explore.rviz` shows `/map` and `/explore/cells`
(cell sides coloured by what they are) with the scan, cloud and path (`launch_guide.md` section 6).

## How it works

```
 lidar ─ Fast-LIO ─ /odom ─┐
        /cloud_registered_body ─ wall_scan ─ /scan ─ slam_toolbox ─ /map, map->odom
                                     └─ /maze/cloud (heights above the floor) ─┐
                                                                    maze_explorer ─ /cmd_vel
                                                                                  └ /simple_cmd
```

### 1. Where the robot is, and the map
- **Fast-LIO + odom_relay** (lab packages) give `/odom` from the lidar and its IMU.
- **wall_scan** levels each cloud with the body's roll and pitch and keeps the points
  0.32-0.48 m above the floor. That band is under the 0.53 m wall tops and above all terrain
  (0.25 m at most), so terrain never looks like a wall. The result is `/scan`.
- **slam_toolbox** builds `/map` from `/scan` and corrects drift (`map -> odom`).

### 2. The grid
The explorer lays a grid of 1.02 m cells over the map. At the start it creeps 10 cm forward,
then turns and shifts the grid until the cell borders sit on the walls in the map
(`grid.fit_grid`, up to 30 degrees and 0.40 m). The map after the creep is often too thin
for that fit, so it is tried again at every planning step until it works once (log: `grid
lined up with the walls`); sides taken as walls before that are judged again. After that the
fit is repeated in small steps (3 degrees, 0.08 m). The start cell is cell (0, 0).

### 3. What is on each side of a cell (`sense.py`)
For every side of every visited cell, from the map and from a height map of the lidar
returns:

| Kind | How it is recognised | What the robot does |
|---|---|---|
| wall | wall-high returns along the cell border | nothing |
| free | border open, next cell empty | may walk in |
| room | border open, next cell closed on its other three sides | looks in, never enters |
| object | border open, something 0.38 m or taller (above terrain) inside the next cell: at least 8 of the 5 cm squares of its interior (0.25 m clear of the borders), each seen 3 times | looks in like a room, **never enters**, also with `enter_rooms:=true` |
| curtain | like a wall, but returns continue above wall height (0.65 m+) | faces it; crosses last |
| stairs | border open, next cell blocked by something that rises like steps | crosses last, in AI mode |
| blocked | border open, next cell blocked by something else | stays out |
| exit | border open and no walls for 2 m beyond | marks this cell as the last cell |
| unknown | not seen well enough | turns to face it; still unknown = wall |

### 4. What to do next (`plan.py`)
1. A curtain beside the current cell that has not been faced yet: turn square to it, hold 2 s.
2. The **nearest** unexplored thing, counted in cells to walk: an open way into an unvisited
   cell, or a room not looked into yet. Rooms are viewed from the cell in front: face the
   room, then 25 degrees left, then 25 degrees right, 1.5 s each.
3. Only when nothing in 2 is left: the nearest curtain or stairs not crossed yet.
4. Nothing left: walk to the last cell (the one with the exit), face the exit, stop.

The start cell is closed as soon as the robot has left it.

### 5. Moving (`explorer.py`)
- One step = turn on the spot to the grid direction, then walk to the next cell centre while
  holding the heading and sliding sideways onto the centre line. Never backwards.
- **Front check**: something from 0.30 m up, 6-20 cm ahead of the body, stops the step; if it
  stays, that way is marked as a wall and the robot returns to the cell centre.
- **Curtain**: square up in the cell in front, then walk straight through to the next centre.
  While the fabric lies over the lidar no scans go to SLAM. No progress for 8 s = give up and
  mark it a wall.
- **Stair check**: before stepping into a cell where the lidar shows something 0.28 m or
  higher, and before crossing stairs, the robot stands still facing it and looks with the
  depth camera for up to 3 s (`stair_levels.py` in the `stairs` package, `src/stairs`; the detector from `~/stairs_detect`: treads
  at 12 / 24 / 36 cm and the triangle handrails). Only a confirmed look makes it stairs.
  The depth stream is on only for those seconds, because it slows every other stream.
- **Stairs**: square up, then manual mode -> AI state -> AI basic gait, walk two cells
  straight (over the stairs cell), then AI off -> manual -> move mode -> walking gait -> auto mode.
- **Start**: move mode -> walking gait (H-Step by default) -> auto mode, creep 10 cm, line the grid up.

### 6. Safety stops
- `/odom` jumping (0.4 m, or faster than 1.5 m/s): **FAULT**, latched until relaunch. This is
  what a diverged Fast-LIO looks like (slow `/livox/imu`).
- Pose or lidar older than 1 s: stands still until fresh.
- No progress on the stairs: stops and waits for the operator.
- Ctrl+C or `explore_stop`: one zero velocity.

## Automatic scene understanding

`explore.launch.py` starts the VLM observer by default during exploration.
The observer uses the explorer's centre, left and right holds for room and
object looks, puts the three RGB views into one 2 x 2 image (left | centre over
right | empty, 1024 x 768, the server's limit) and submits one short prompt
(`LOOK_PROMPT` in `scene_prompt.py`) that asks only for what the missions score:
scene type, hand tools, hazard sign, survivor (no QR). The reported position is the
cell the explorer says it looked into. Each view is taken 0.6 s into its hold.

While the robot holds a look, `qr_scanner` (team_perception, started here, `qr:=false` =
off) reads QR codes from the same camera at the same time; its camera subscription exists
only during the look. Do not also run `team_perception`'s `robot.launch.py`.

Each analysed look goes out on `/scene/result`. `task_reporter` (a package of its own,
also started here, `report:=false` = off) turns those and the QR text on `/qr/read`
into the five mission reports and POSTs each one, as `group_id`
(`group04_grp4`), to `report_url` (default `http://192.168.10.141:8001/task_report_B`,
course B). Every report has the seven keys and a position. The QR report needs no VLM.
The server's reply, with its per-field evaluation, is printed in the launch terminal.
Individual frames, the stitched JPEG and `results.jsonl` are saved below
`~/challenge_ws/logs/scene/junctions_*`. Use `scene_understanding:=false` to
run without the VLM or `scene_camera_topic:=...` to select another RGB topic.

The observer reads `<scene_camera_topic>/compressed` (JPEG, about 75 KB a frame). Do not
point it at the raw image on the robot (`scene_image_transport:=raw`): a raw image subscriber
slows the lidar IMU on its way to this computer and Fast-LIO diverges within a minute (both
runs on 2026-11-24 morning ended in `FAULT` in the start cell this way). Nothing is saved
until a room or object look has finished all three views.

The camera is subscribed only during a look: from the moment the explorer's status shows a
`look` job in the current cell (the turn to the centre view) until 2 s after it ends. The
terminal shows `Camera on for an explorer look` and `Camera off`.

## Events for perception
`/explore/event` (String) says when the robot is posed for a look, e.g.
`room left cell (3, 0) side 1`, `curtain cell (1, 1) side 0`, `exit: stopped in the last cell`.

## Files
- `maze_explore/grid.py`, `sense.py`, `plan.py`, `robot.py`: logic without ROS (`pytest test`).
- `maze_explore/wall_scan.py`, `explorer.py`: the two nodes.
- `config/`: Fast-LIO and slam_toolbox settings. `preflight.sh`: `explore_check`.
- `sim/run_sim.sh [maze]`: the stack against a simulated Lite3 (mazes in `maze_examples/sim`).

## Known gaps
- Tuning values for curtain and stairs heights come from the arena description, not from
  robot data.
- A tall object or person right behind a wall, or a 0.9 m frame wall, can still be read as a
  curtain. The obstacle stop is now active during a curtain push as well (`push_stop`, default
  true): something knee-high 6-20 cm ahead stops the push and the side becomes a wall. The
  fabric of a real curtain triggers it too, so **curtains are not crossed for now**; set the
  node parameter `push_stop` to false to push again.
- The object rule looks at height only. Something lower than 0.38 m in a cell is walked into,
  and a person standing in a cell closes it.
- Whether the Lite3 follows `/cmd_vel` in AI state is still unverified on the robot.
- **Curtain seen by the camera** (on by default since 2026-11-25; `curtain_cv:=false` = off): `curtain_node` runs the
  curtain YOLO model (branch `curtain_cv`, `models/curtain_int8_320x256_openvino_model`) on the
  colour camera outside room checks and publishes `/curtain/seen`. The explorer places each
  sighting on the first closed edge in its direction, verifies it by facing that edge for 3 s
  from a cell centre, and then treats the edge as a curtain even if it maps as a wall.
- **Curtain edges are latched** (2026-11-25): once read as a curtain an edge stays one (both sides), even if the fabric later moves and the map shows it open or as a plain wall. A failed crossing or the first grid fit undoes it.
- **Curtains are crossed sideways** (2026-11-25): quarter turn on the spot, sideways to the
  centre of the cell behind, quarter turn back on the spot.
- **Terrain** (2026-11-24, simulator only so far): a cell is uneven when 15 or more of the 5 cm
  squares of its interior stand 0.06 m or more above the floor (below object height;
  `terrain_min`, `terrain_squares`). The robot enters it **sideways**: at the centre of the cell
  before it turns on the spot until its body is square to the way in (the nearer quarter turn),
  then steps sideways to the next centre at 0.10 m/s (`side_speed`), with its length centred in
  the opening (`side_offset`). The obstacle stop then looks to the side it steps towards.
  It publishes `terrain: sideways into cell ...` on `/explore/event`. No special gait.
