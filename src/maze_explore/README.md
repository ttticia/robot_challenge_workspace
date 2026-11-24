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
(mapping only, for hand-driven bags), `v_max:=0.15`, `gait:=rug`.

Watch `ros2 topic echo /explore/status --field data` and the launch log (`job: ...` lines).
In RViz add `/map` and `/explore/cells` (cell sides coloured by what they are).

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
- **Stairs**: square up, then manual mode -> AI state -> AI basic gait, walk two cells
  straight (over the stairs cell), then AI off -> manual -> move mode -> H-Step -> auto mode.
- **Start**: move mode -> H-Step -> auto mode, creep 10 cm, line the grid up.

### 6. Safety stops
- `/odom` jumping (0.4 m, or faster than 1.5 m/s): **FAULT**, latched until relaunch. This is
  what a diverged Fast-LIO looks like (slow `/livox/imu`).
- Pose or lidar older than 1 s: stands still until fresh.
- No progress on the stairs: stops and waits for the operator.
- Ctrl+C or `explore_stop`: one zero velocity.

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
- Terrain gets no special speed or gait.
