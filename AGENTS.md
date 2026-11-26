# challenge_ws: guide for agents and teammates

Read this before changing anything here. It records the current strategy for the maze run,
how the code is laid out, how to build and test it, and what the old stack taught us about
the robot and the arena.

Last updated 2026-11-26 (night, after the revert below). Competition day is **27 November 2026**.
Keep this file current when the strategy or the interfaces change.

> **Reverted to `ff405d8` (user, night of 2026-11-26).** `exploration-v2` was reset from
> `e35dff5` to `ff405d8`. The three later commits (`2a913ae`, `0019738`, `e35dff5`) are on
> branch `backup/pre-revert-e35dff5`; the uncommitted work on top of them (it included
> `bt.md`, QR approach off by default, `walls_plain`, `through_done`, `outside_arena`,
> `try_unknown`, `turn_centre`, the head-on curtain crossing, `fault_resume`, `stall_retry`)
> is in `git stash` as "pre-revert to ff405d8". None of those exist in the code now, and
> this file describes `ff405d8` only. Changed since the revert: `look_angle` 45 -> 35 deg;
> the end stop (centred, facing the open edge, 0.10 m from it; row 6 = end cell); the
> stairs line-up and the stairs-sequence fixes after the 19:05 run (section 3.6, Stairs).
> `origin/exploration-v2` still has `2a913ae` and `0019738` until it is force-pushed.

- Robot: DEEP Robotics **Lite3** quadruped, Livox **Mid-360** lidar (tilted 15°), Orbbec
  RGB-D camera. ROS 2 **Humble**, Ubuntu 22.04, Python packages (`ament_python`).
- Rules: `SUTD_Legged_Robot_Challenge_Booklet_2026_v6.md`. Arena facts: `maze_info.md` and
  `maze_examples/`. Robot interface (topics, QoS, `/simple_cmd` codes): `robot_info.md`.

## 1. Where things stand

On 2026-11-23 the exploration stack was restarted from scratch on branch `exploration-v2`.

| | |
|---|---|
| New stack | `src/maze_explore` (v2), on branch `exploration-v2`. |
| Old stack | `maze_nav` (slam_toolbox + Nav2 + frontier explorer). Removed here in `044e17c`; still on branch `navigation` (`4303e33`) with its docs (`agent.md`, `skills.md`, `src/bt.md`, `src/to_do.md`). Use it for reference only. |
| Kept | `src/team_perception`, `src/scene_understanding`, `runs/ROS_bags` tools, booklet, `robot_info.md`, `maze_info.md`, `maze_examples/`. |

State on the evening of 2026-11-26 (the day before the competition):

- The stack drives the real robot. Best run so far, 05:57 on 2026-11-26: stairs, one
  curtain and several uneven cells crossed, the last row reached, two possible exits found;
  it ended in an odometry `FAULT` before the end cell.
- **Every robot run of 2026-11-26 ended in an odometry fault or was stopped by hand.** Each
  jump followed a late sensor feed (section 3.6, safety table). Unproven suspects: the
  flat -> H-Step gait switch just before a sideways step onto uneven floor (three runs),
  and speeds above 0.25 m/s.
- Most of what was added on 2026-11-26 is unit-tested only; each entry in section 3.6 says
  so. Simulated end to end that day: the end stop, the gait switch, a curtain crossing.
- Scene understanding, QR reading and mission reports are connected to the explorer
  (section 5). `poster_detector` is not.

## 2. What scores (short form of the booklet)

| Task | Points |
|---|---|
| Each uneven-terrain cell crossed autonomously (4 cells) | 5 each |
| Stairs crossed autonomously | 10 |
| Movable obstacle (curtain) passed autonomously | 15 |
| M01 object (always `hand_tools`), M02 hazard sign (type varies), M03 survivor with QR, M05 survivor without QR (label / scene type 5, position 5) | 10 each |
| M04 QR message decoded | 5 |
| Stop autonomously inside the final cell | 10 |
| Drone landing | up to 50 |
| Autonomy bonus: reach the final cell with at most 1 / 2 / 3 manual interventions, and at least one valid mission report | +20 / +10 / +5 |
| Drone kept mounted for the whole ground run | +10 |
| Major collision | −10 |

Rules that shape the design:

- No pre-built map. The layout changes on the morning of the competition.
- **AI gait is allowed only for the stairs.** Enter it at most one cell before them and
  leave it at most one cell after. No points for any other cell crossed in AI mode.
- A manual intervention gives 0 points for that obstacle. A mission reached with manual
  help is capped at 8 points.
- Each restart is a new attempt with mission scores and reports reset; the best ground
  attempt counts. The drone launch may be started at any time by carrying the robot to the
  launch cell, but ends the ground mission (booklet update 2026-11-24).
- "Inside the final cell" means all four legs inside it, nothing touching the frame.
- Reported positions use the arena frame: origin at the centre of the start cell, +X along
  the start heading, +Y to the robot's left, cell centres 1.02 m apart. A position counts if
  it is within 0.5 m of the true cell centre on both axes. The latest valid report per
  `task_id` is the one scored.
- Reports go by HTTP POST to `http://192.168.10.141:8001/task_report_A` or `..._B` (booklet
  section E): the course follows `maze_side`, L = A, R = B (section 5).

## 3. Current strategy: `maze_explore`

The idea: treat the maze as a **grid of 1.02 m cells** and move **cell centre to cell
centre**. There is no Nav2, no costmap and no path planner. The robot classifies the four
sides of each cell it stands in, picks the nearest unexplored thing, turns on the spot and
walks straight to the next centre.

### 3.1 Pipeline

```
Fast-LIO + odom_relay   /odom, TF odom -> base_link
        |  /cloud_registered_body
wall_scan               /scan (points 0.32-0.48 m above the floor), /maze/cloud (heights
        |               above the floor), TF odom -> base_level (no roll, no pitch)
slam_toolbox            /map, TF map -> odom
        |
maze_explorer           /cmd_vel, /simple_cmd, /explore/status, /explore/event, /explore/cells
```

| File (`src/maze_explore/maze_explore/`) | Job | ROS? |
|---|---|---|
| `wall_scan.py` | Lidar cloud → levelled, floor-referenced scan and cloud | yes |
| `explorer.py` | State machine, action queue, driving, mode switches | yes |
| `grid.py` | Cell grid, `fit_grid` (lines the grid up with the mapped walls) | no |
| `sense.py` | What is on each side of a cell, from `/map` and a height map | no |
| `plan.py` | Which job next (`next_job`) | no |
| `robot.py` | `/simple_cmd` codes and mode sequences | no |

Keep `grid.py`, `sense.py` and `plan.py` free of ROS so they can be unit-tested.

### 3.2 Why the scan is floor-referenced

Walls are 0.53 m tall and terrain reaches about 0.25 m. `wall_scan` keeps only points
0.32–0.48 m above the **floor**, in a gravity-levelled frame, so terrain does not look like
a wall and the body's pitch on terrain does not tilt the slice. The floor height is measured
once at the start with the robot standing still, then follows odometry z. While the robot
pushes through a curtain the explorer publishes `/maze/blind` = True and `wall_scan` stops
sending `/scan`, because the fabric lies over the lidar and SLAM would match against it.

### 3.3 The grid

- Confirmed by the user (2026-11-26): the start cell is always (0, 0); the robot always
  starts facing the start cell's only way out, along the LONG edge of the grid (the 5 rows);
  cells in front have positive x, cells to the right negative y.
- Origin: centre of the start cell. Direction 0 = the start heading, then 1 = left,
  2 = back, 3 = right. This is the same frame the judges use, so a cell `(i, j)` reports as
  `pos = [i * 1.02, j * 1.02]`.
- After the first 0.10 m creep the grid is fitted to the mapped walls (up to 30° and
  0.40 m). That first map is often too thin (`run1`: 37 wall points, the grid stayed 6° and
  0.27 m off all run), so the wide fit is retried at every planning step until it works
  once. From then on it is nudged before every planning step (up to 3° and 0.08 m).
- The start cell can be any of the three cells of the first row (user, 2026-11-26), but
  **its left, right and back sides are taken as walls, always** (user, later the same day):
  nothing is sensed, faced or latched there, so the start costs no turns and a tall wall
  beside it cannot read as a curtain (05:30 run: side 3 latched as a lidar curtain, side 1
  was faced for 7 s and still unknown). The only way out of it is ahead, and it is closed
  once the robot has left it. If a start side is really open, the cells behind it are
  reached another way or not at all.

### 3.4 What each side of a cell can be (`sense.py`)

| Kind | Meaning | How it is told |
|---|---|---|
| `wall` | wall-high obstacle on the edge | ≥ 40 % of the edge occupied in `/map` |
| `curtain` | as a wall, but it carries on above wall height | tall returns (≥ 0.65 m) along ≥ 50 % of the edge |
| `unknown` | edge not seen well enough | resolved by facing it; still unknown = wall |
| `exit` | open, no walls for 2 m beyond, and the cell is the last cell: a stub that sticks out of the maze, walled on both sides of the way out and not walled behind (user, 2026-11-25; open floor alone read as an exit in the 09:38 run) | never from the start cell. An exit that later stops reading as one is withdrawn |
| `object` | open, something taller than terrain stands inside the cell behind (toy set, prop) | height map: ≥ 8 squares of the interior (0.25 m clear of the borders) at ≥ 0.38 m, each seen 3 times. Checked before `room` |
| `room` | open, the cell behind is closed on its other three sides (dead end) | |
| `stairs` | open, the cell behind is blocked by something that rises like steps | height map: top 0.33–0.60 m, front 0.08–0.42 m; then confirmed by the depth-camera stair check (section 3.6) |
| `blocked` | open, the cell behind is blocked by something else | |
| `free` | open, the cell behind can be walked into | |

### 3.5 Job order (`plan.py`)

Order set by the user on 2026-11-25: explore everything reachable first, dead ends first;
curtains and stairs last. Cells are plain, uneven terrain, dead ends / rooms, curtain,
stairs, start and the last cell.

1. **visit / look**: the nearest unexplored thing: an open way into an unvisited cell (plain
   or uneven terrain), or a room or object cell not looked into yet (an object cell is never
   entered). Nearest = fewest cells to walk, then looks before visits (every room / prop
   beside a cell is looked at before the robot walks on; 11:41 run, two rooms were passed
   and the robot had to come back), then the way that looks most like a dead end (most walls
   already seen around the cell behind it), then least turning.
   Possible stairs beside the current cell are checked with the depth camera at once, before any other job (a check, not a crossing): confirmed = `stairs` (crossed last), not stairs = a prop = `object` (looked at, never entered). Before, such a cell ended as `blocked` and was never shown to scene understanding.
2. **greet**: only when nothing in 1 is left, and only for the curtain that 3 is about to
   cross, from the cell beside it: face it squarely, publish an event, first camera look
   (3 s). Until 2026-11-26 every curtain beside the cell was greeted before stairs too:
   in the 04:42 run the robot, with confirmed stairs ahead in the start cell, turned to two
   lidar curtains at its sides first and the odometry failed before the stairs' turn.
3. **stairs / curtain**: only when nothing in 1 is left: the nearest one not crossed yet; at
   the same distance stairs come before a curtain.
4. **exit**: nothing left at all: go to the cell that has the way out, do the end stop (section 3.6, `DONE`).
5. **no exit**: not believed at once (user, 2026-11-26 night; unit-tested only,
   `test/test_no_exit.py`): first every wall to an untravelled cell that is not solidly
   mapped (under 40 % of the edge occupied, or a wall only by a resolve timeout or a
   given-up step) is faced once more, nearest first (`recheck_soft_wall`,
   `no_exit_walls:=false` = off); then the nearest tall edge to an untravelled cell
   (`cv_tall_min`, 20 % curtain-high) is tried as a curtain, camera confirmation or none
   (`force_tall_curtain`, `curtain_last:=false` = off): fabric maps as a wall, and a
   curtain is the one 'wall' the robot can walk through. The push keeps its guards (leg
   stall, 8 s); failing, the edge is a wall for good (`push_failed`) and the next is
   tried. Each edge is faced / tried once. Only when nothing is left: the navigation
   pauses and the robot lies down where it stands (`STOPPED`, not `DONE`; `lie_down:=false` = pause standing).
   `/nav/start` stands it up again and it plans afresh. **Not when the robot is lost**: with
   the pose more than 0.75 m (`lost_dist`) from the nearest known cell centre it pauses
   standing and says so (09:06 run: 2.37 m from every known cell, it called it 'no exit
   found' and lay down).

Junction rules (user, 2026-11-26). At one junction the order is room, path, stairs, curtain.
A job beside the current cell beats one that needs walking back through explored cells;
among those, the fewest cells re-crossed. A room / object cell that has been looked at is
explored (no second look from another side). An open edge into a cell the robot has
travelled through is never checked. A look is called off when, at its third view, the cell
no longer reads as a room / object (a curtain, stairs or a way on has shown up).
Between several ways on at one junction (same distance): a way already seen to be a dead end
first; then the one furthest from the far row, where the end cell is (lowest row along the
start heading; user, 2026-11-26), so the robot works towards the exit; then most walls
already seen; then least turning.

**Cells that lead nowhere new are not entered, path cells and junction cells alike** (user,
2026-11-26 night; unit-tested only; `through_cells`): an open cell not visited yet is skipped when every other side of it
is a mapped wall or an open way into a cell the robot has already been in. An unknown side,
or one open into a cell not visited, keeps it on the list: ALL its ways out must lead to
known cells. **Since 2026-11-26 night the skipped cells stay out for good** (user: mark empty cells as
explored; 19:21 run: (2, -1) was entered after all, costing two crossings of an uneven cell
before the curtain job). `through_enter:=true` brings the old rule back (entered once
nothing else is left). Uneven cells are always crossed (they score). `skip_through:=false`
= every open cell is entered.

So curtains and stairs are crossed **last**, after everything reachable on this side has
been explored, and the robot ends the run standing in the final cell.

### 3.6 How each thing is handled (`explorer.py`)

- **Start**: stand still, centred in the start cell, facing into the maze. The explorer
  waits for `/map`, `map -> base_level` and `/maze/cloud` (`WAIT` → `READY`), then for
  `/nav/start`. It sends Move → gait → navigation mode, creeps 0.10 m, fits the grid.
- **Turning (user, 2026-11-26)**: on the spot, about the robot's own pivot where it stands
  when the turn begins, **not about the cell centre**. Measured on 17 recorded turns, the
  Lite3 turns about base_link itself (`pivot_offset` 0). Only the rotation is commanded,
  plus 0.02 m/s forward at full turn rate (`turn_forward`) against its backward creep; the
  pivot is steered back (forward and sideways only) once it has drifted more than 3 cm
  (`pivot_slack`). Until 2026-11-26 the turn steered base_link onto the cell centre when
  within 0.25 m of it: with a pose or grid that was off, the robot walked while it turned
  (03:45 run: 0.13 m/s forward, the turn hunting left and right). Getting back onto a cell
  centre is the recentre rule's job only (more than 0.28 m off, before a job; 0.20 until
  the looks were made from 0.14 m ahead of the centre, which set it off after every look). Each turn
  over 20° logs how far base_link and the pivot moved. Body outline: 0.27 m ahead of
  base_link, **0.42 m behind** (0.37 m body + 0.05 m for the bent hind legs), 0.23 m to
  each side; the rear corner sweeps 0.48 m, against 0.485 m from a cell centre to its
  walls, so a turn made off-centre has less room than that.
- **Walking**: turn to the heading, drive straight to the next centre. Top speed `v_max` in
  `config/speeds.yaml` (team limit 0.25 m/s). Sideways and heading corrections on the way. **Never backwards**
  (`linear.x` is clamped at 0: nothing looks behind).
- **Rooms: a dead end is NEVER entered (user, 2026-11-26 night; unit-tested only;
  `no_dead_end_entry`)**: the `depth_clear` rule (camera saw the floor clear, walk in) gives
  way when the cell behind is closed on its other three sides; such a cell stays a `room`,
  looked at only. 19:21 run: room (4, -2) re-read as stair-high, the camera found its floor
  clear and the robot walked into the dead end.
- **Room checks (user, 2026-11-26 night; unit-tested only, `test/test_room_checks.py`)**:
  before the views of a room / object look:
  (1) **The depth stair check runs only on potential stairs** (user, later that night):
  the lidar shows something stair-high (`raised`), or the VLM said stairs (`vlm_stairs`).
  `room_stairs:=true` = at every room, as for a few hours that evening.
  (2) **Confirmed content overrides the stair check** (user; 20:12 run: the prop room
  behind (1, 0), 7 squares at 0.65 m or more, got the check and the close recheck on a
  late feed): a room or object cell with items taller than the wall inside
  (`Sense.tall_inside` >= 4 squares at 0.65 m, the room_empty content test) gets no stair
  check and never becomes a `stairs` candidate in `kind()` — stairs are 0.45 m, under the
  0.53 m walls, so such a cell is a prop room.
  (3) **The three-point side scan** (face, then 45° left and right, `side_check_angle`,
  0 = off, 3 s each, `side_check_hold`, with the curtain camera on: `side_watch`,
  `curtain_node` watches during it) runs for cells with NOTHING confirmed inside (empty
  or unmeasured rooms: an empty dead end is where a curtain or the stairs can hide on a
  side), and is skipped for cells with confirmed tall content and when both of the room's
  side walls are measured ordinary low walls (`Sense.low_wall`: nothing hides behind one;
  the turns cost ~12 s a look and turning on a late feed is when the odometry jumps,
  19:42 run). A sighting is verified later like any other. The look's own three views
  stay at `look_angle` (35°). A look skipped (all missions done, or empty-early) still
  gets these checks (`no_views`): only the views and scene understanding are dropped.
  (4) **No curtain check during a scene check** (user, 2026-11-26 night): the look queues
  no `cv_verify` of its own (`look_curtain:=true` = as before); a candidate ahead of the
  room comes from the side scan's sightings or the usual check outside looks.
  (5) **Stairs seen during the approach** (user, 2026-11-26 night; `look_late_stairs`):
  the look's stair check runs from the cell centre, before the walk up to the room; a
  cell that shows stair-high only after the approach (0.14 m ahead, three views of lidar)
  and was never checked gets one depth check as the look ends — never a cell with
  confirmed tall content, never once the stairs are crossed.
- **Rooms**: not entered by default (`enter_rooms:=false`). From the cell before, the robot
  looks centre, 35° left and 35° right (`look_angle`; user, 2026-11-26: 25°, 30°, 45°, then 35°), holds 1.0 s each (`look_hold`; 1.5 until that evening) and publishes
  `room centre|left|right cell (i, j) side d` on `/explore/event`. For the look it first walks
  up to the room (user, 2026-11-26; unit-tested only): forward until its nose is 0.10 m
  (`look_gap`) before the edge shared with the room, base_link 0.14 m ahead of the cell
  centre. From an uneven cell too since the night of 2026-11-26 (user; in the 09:06 run two
  looks were made from the centre of a ramp cell, 0.51 m from the room); the three views
  are then turned 0.14 m off that cell's centre (`look_gap_uneven:=false` = from the centre
  there). A negative
  `look_gap` = from the cell centre. Before the look, checks run only where they can
  matter (user, 2026-11-26 evening; `look_quick:=false` = both before every look): the depth
  stair check only when the lidar shows something stair-high in that cell, the camera
  curtain check only for an edge the curtain camera has pointed at. A look took about 22 s
  in the 07:45 run, the three views only 10 of them. No terrain verdict is made for the
  looked-at cell (`look_depth:=false` = no depth check at all). Only what is IN the cell makes it a room with something in it
  (`Sense.tall_inside` stops at the cell's own borders, walls included).
- **Empty rooms (user, 2026-11-26; unit-tested only)**: at the third view of a look
  (`do_look_last`) a dead end with nothing taller than an ordinary wall in it
  (`Sense.tall_inside`: under 4 squares at 0.65 m or more in the cell and on its walls, far
  wall's top measured) is empty: the view is held with the QR scanner on, the event is
  `empty room cell …` instead of `room right …`, so scene understanding sends nothing, and
  the cell counts as looked at. Rests on the user's word that every room with content has
  something taller than the walls. `object` cells always go to the VLM. `empty_skip:=false`
  (node parameter) turns it off.
- **Looks that are not made (user, 2026-11-26; unit-tested only)**: before every job is
  chosen (`skip_looks`), (1) a `room` the lidar already shows to be empty (same test, plus
  nothing stair-high inside) counts as looked at without any turn, view or QR scan
  (**off by default since the evening of 2026-11-26, user: no room is skipped without a
  look**; `empty_early:=true` brings it back; with the grid a few cm off a board on a wall
  can fall outside the cell and the room reads empty); (2) once the scoring server has
  marked a report for each of M01-M05 as successful, every remaining room / object look is
  dropped (`missions_stop:=false` = off). task_reporter passes each answer on as
  `/mission/result` (`{task_id, ok, reply}`); ok = `status: accepted` and, where the reply
  carries an `evaluation` (practice mode), every field true. The competition reply has no
  evaluation, so there accepted is all that can be checked. The latest answer per mission
  counts. Not ok for M01 / M02: that sighting lets go of the mission and the next one seen
  elsewhere is reported in its place (`Missions.refused`). With `report:=false`, or no
  answer from the server, (2) never applies.
- **QR approach, 2026-11-26 changes (user)**: no deeper than 0.3 m into the room
  (`qr_depth`, base_link past the border; 0.5 at first, 0.3 since the same day). Turns in
  there are made about the spot the robot stands on, not the room centre. Not read at the far end: after the hold the
  robot turns slowly (`qr_sweep_rate`, 0.25 rad/s) towards the room's back wall until the
  code is read or it faces that wall, then leaves sideways as before.
- **QR approach (user, 2026-11-25; `qr_approach:=false` = off; never run on the robot)**:
  from the cell before, a code on a side wall is seen about 60° off square and cannot be
  decoded (10:54 run, 11-24: 34 x 68 px). The VLM can see it, though: the look prompt also
  asks `qr` / `qr_view` (left, ahead, right), passed on in `/scene/result`. After the three
  views the robot stands up to 4 s (`qr_wait`; 8 until 2026-11-26) for that answer. With a code in the left or
  right view it turns 75° (`qr_angle`) towards that side at the cell centre, then moves
  into the looked-at cell with its body kept at that angle (almost sideways, camera
  sweeping that wall from close up) at 0.08 m/s, up to that cell's centre or until
  something is 6-20 cm in its way, holds 1.5 s, turns 15° more and steps back out purely
  sideways. A code read (`/qr/read`) at any moment ends the move and the robot goes back.
  Nothing becomes a wall because of it. The scanner stays on throughout (the job is still
  the look). A code in the `ahead` view gets no approach. Skipped once the code is read.
  Simulated only (`test/test_qr_approach.py`; rendered views decode from 0.65 m in).
- **Curtain: crossed HEAD ON (user, 2026-11-26 night; unit-tested only)**: square up on the
  cell's centre line facing the curtain and walk straight through it at `curtain_speed`
  (0.20 m/s) with `/maze/blind` on, all the way to the centre of the cell behind. The leg
  stall stop (below) guards the head-on push too. `curtain_sideways:=true` brings back the
  sideways step of 2026-11-25 (quarter turn, then sideways through), which the next
  paragraphs describe where they say sideways. **There it turns only if the fabric still covers the
  lidar** (user, 2026-11-26 night; unit-tested only; `do_curtain_clear`): covered = the lidar's
  cloud has shrunk to under 40 % (`fabric_frac`) of what it gave just before the push. Then
  90° to the right and 90° back to the left, twice at most; clear = no turns. `/maze/blind`
  stays on meanwhile, and if the lidar is still covered after the turns nothing is added to
  the map (no scans to SLAM, no height map) until it clears, 15 s at most.
  `curtain_clear:=false` = stand 1 s and go on; `curtain_turns:=true` = the old 180° right,
  50° left. The crossing is remembered as a link so later routes can use it.
  **Latched (2026-11-25):** once an edge reads `curtain` it stays a curtain on both sides, whatever the map shows later (swaying or parted fabric must not turn it into an open way or a plain wall). A failed crossing, or the first grid fit (the cells move), undoes it.
  **Obstacle stop during the push:** off for a curtain the camera has confirmed (2026-11-25; with the stop on, the fabric itself stopped the step 3 cm in and the curtain became a wall: 10:42 run, cell (2, 2) side 0). For a curtain only the lidar has seen (`curtain_cv:=false`) `push_stop` still rules (default true = the fabric stops the robot, user 2026-11-23; `push_stop:=false` goes through). No progress for 8 s still ends any push.
  **Stall stop (user, 2026-11-25; unit-tested only):** the lidar is under the fabric during a push, so the guard is whether the robot moves. The explorer listens to the leg odometry (`/leg_odom2` pose, only during the push: it is a 200 Hz stream) and gives the push up (side = wall) when, after 2.2 s of stepping, the legs report under 3 cm in the last 1.5 s (`push_stall`, `push_stall_time`, `push_stall_grace`, `push_stall_min`). On the bags the leg pose follows Fast-LIO at about 0.85 of its distance while walking and moves 0-2 cm in 1.5 s standing; what it reads while the robot presses on something (feet slipping) has not been recorded. No leg data = the 8 s rule alone.
- **Curtain seen by the camera (2026-11-25, ON by default since the same day, user; `curtain_cv:=false` = off)**:
  `curtain_node` runs the v3 curtain YOLO model of branch `curtain_cv` (OpenVINO INT8
  320x256, `src/maze_explore/models/`, about 10 ms a frame, no ultralytics) on the compressed
  colour stream, 3 frames a second, only while the explorer runs and is NOT on a room /
  object look (the camera then belongs to scene understanding and the QR scanner). It has
  no depth: a box gives a direction. The explorer takes the first closed edge in that
  direction (`edge_along`); after 3 frames in 3 s it is a candidate. From the centre of the
  cell before it (or its own cell) the robot faces the edge for 3 s; a box across the middle
  of the image on 3 frames makes that edge a `curtain`, **also where the map and the height
  map call it a wall** (and the cell before it is then no longer a dead-end `room`). A push
  that fails takes the claim back. The model is not accepted yet (val: 20 of 23 curtains,
  false boxes on 2 of 51 frames without one).
  **Not over a wall (user, 2026-11-25).** The camera sees a curtain OVER a 0.53 m wall and
  has no depth, so such a sighting used to be pinned on that wall (09:42 run: cell (0, 0)
  side 1). An edge that is an ordinary wall (`Sense.low_wall`: wall in the map, its top
  measured along at least half of it, under 20 % of it curtain-high) collects no sightings,
  is not queued for a check, and camera hits there do not make it a curtain. An edge whose
  height is not known yet can still be claimed. Same code in the stairs copy
  (`src/stairs_crossing/patches/low_wall.patch`).
  **The camera confirms before every push (user, 2026-11-25).** With `curtain_cv` on, a
  lidar `curtain` (tall returns) is only a candidate: an aluminium-frame wall reads the same
  (bag `stairs10`, 23.4 s: 0.84 m high, no curtain box on 81 frames). The camera looks at it
  twice: during the greet (3 s instead of the 2 s hold; a miss there is not final) and,
  squared up in the cell before, just before the push. Not confirmed there, or no camera
  frames at all (`curtain_node` down), = the edge is a wall for good. `curtain_cv:=false`
  = the lidar decides alone, as before. Do not merge `curtain_cv`: it deletes the
  navigation code; bring files over one by one. Never run on the robot so far.
- **Stair check (2026-11-24)**: before walking into an unvisited cell where the lidar shows
  something 0.28 m or higher, and before any stairs crossing, the robot squares up, stands
  still and runs the teammates' depth-camera detector (package `stairs`: `src/stairs/stairs/stair_levels.py`, copied from
  `~/stairs_detect`, now kept in `archive/stairs_detect`: treads at 12 / 24 / 36 cm, plus the triangle handrails in the lidar)
  for up to 3 s. Its verdict overrides the lidar's: `stairs` only when it confirms. Depth is
  subscribed only during the check, because one raw depth subscriber slows every stream
  from the robot; odometry jumps in that window do not count towards `FAULT`. A cell that
  is `blocked` stays a stairs candidate until it has been looked at. `stair_check:=false`
  turns it off (lidar only).
  **Around the depth stream (user, 2026-11-26; unit-tested only):** `curtain_node` gives
  up its colour-camera subscription while the explorer's action is `stair_check`. After
  depth is closed the robot stands 1.5 s as before (a wait for the feed to be on time was
  added and taken out again the same day, user). The floor
  depth check is skipped when the lidar has seen 25 of the cell's 121 squares
  (`terrain_seen_min`, 40 before).
  **Floor first, stairs after (user, 2026-11-26; unit-tested only):** the first depth frames
  are read for the floor only. The stair detector runs only if that reading shows 6 or more
  squares at 0.20 m or higher in the cell (`stair_gate_min`, `stair_gate_squares`), or
  gives no usable reading in 6 frames (close up, stairs hide the floor). Three usable
  frames with nothing that high = not stairs, depth closes with the floor verdict (about
  1 s instead of up to 4.5 s). `stair_gate:=false` (node parameter) = both at once, as before.
  **The gate never vetoes where the lidar shows stair-high** (user, 2026-11-26 night,
  "missing stairs" fixes; unit-tested only): with `raised()` true the detector runs from
  the first frame, whatever the floor frames say.
  **Sharp-edge recheck (user, 2026-11-26 night; unit-tested only):** when the detector
  fails only on an edge-quality test (`no top level`, `top level alone`, `broken`,
  `too little of it seen`, `front edge straight`, `not in line` — the 19:42 run failed the
  real stairs on that one) while the lidar shows stair-high, no
  verdict is recorded: the robot creeps up (nose `look_gap` before the shared edge, as
  for a look) and checks ONCE more from there (`stair_rechecked`); that verdict is final.
  From a cell away the top tread can be sparse ('no top level at 0.36 m (17 pts)',
  threshold 25). Not for a clean miss (no flat surfaces, wrong heights, slant, lidar veto).
- **Stairs**: square up in the cell before, enter AI mode (manual → AI on → AI gait), walk
  two cells (over the stair cell to the centre of the cell after) at `ai_speed` (`config/speeds.yaml`), leave AI mode
  (AI off → manual → Move → gait → navigation mode). The stair cell is closed afterwards.
  This matches the one-cell-before, one-cell-after rule.
  **Crossed once, never again (user, 2026-11-26 night; unit-tested only,
  `test/test_stairs_once.py`).** The stairs count as crossed as soon as the AI walk is over
  (`stairs_crossed`, `stairs_done`): the stair cell is closed, the crossing link is never
  used as a route (`plan.routes` skips `stairs` links, so cells left behind the stairs are
  not gone back to), no second stairs job is planned, anything that reads as stairs after
  that is an `object` (looked at, never entered), and `go()` refuses a stairs step. A stop
  during the crossing (AI-mode stop, `/explore/stop`, a fault after the walk) is settled at
  the next `/nav/start` (`settle_ai_pending`): AI walk finished, or the robot more than a
  cell past the cell before the stairs (finished by hand) = crossed, the robot is in the
  cell after; short of that, not crossed and it may be tried again.
  **Centred before the gait change (user, 2026-11-26; unit-tested only):** the AI walk
  ends by distance just past the border of the cell after, about 0.16 m short of its
  centre. The robot now stands 3 s (`ai_settle`), the grid is re-anchored (this used to
  come after the gait change), and if the robot is more than 0.10 m (`ai_centre_tol`) off
  the centre it walks onto it, still in the AI gait: forward only at 0.20 m/s
  (`ai_centre_speed`), steering towards the centre (`do_ai_centre`). Not attempted when the
  pose is old, the centre is not ahead (within 70 deg) or is more than a cell away; a wall
  ahead or a time limit ends it. Only then AI off -> manual -> Move -> gait -> navigation.
  `ai_centre:=false` (node parameter) = off.
  **After the 19:05 run (2026-11-26 night; unit-tested only).** That run the stairs were
  straight ahead of the start; the grid was never fitted (every wide fit failed, 94-164
  wall points), the robot was squared up within the old 8 cm / 5 deg on that unfitted grid
  while the detector saw the stairs 7 cm to the side, the walk (counted as 0.55 m/s x 3.5 s
  = 1.93 m) ended about 0.6 m short on the stair cell, and a 1.16 m pose jump right after
  it dropped the whole queue: no centring walk, no AI leave sequence, and the robot,
  still in the AI gait, was sent back towards the start. Now:
  (1) **lined up on the stairs** (`stair_line`, `align` drive): the depth stair check keeps
  the stairs' centre line in the map (`stair_lines`); before AI mode the robot steps onto
  it within 3 cm (`stair_align_tol`; sideways steps at 0.04 m/s or more) and turns to within
  2 deg (`stair_align_deg`), twice over. A measured line more than 10 deg
  (`stair_line_deg`) or 0.25 m (`stair_line_off`) off the cell's is not used (the cell's
  centre line then). The event `stairs: lined up, … cm off …, … deg` records the result.
  (2) **A pose jump does not drop the stairs sequence** (`ai_sequence`, from the line-up to
  the end of the AI leave sequence): the robot stands until the pose is still, then goes
  on with it. (3) **Short is believed** (`ai_short_trust`): speed x time can only
  over-count, so when the lidar puts the robot behind the counted spot (by more than
  `reanchor_min`, up to a cell), the grid is not re-anchored and `ai_centre` walks the rest
  in the AI gait. A pose ahead of the counted spot is still taken as Fast-LIO's error.
- **Terrain: uneven cells are entered HEAD ON (user, 2026-11-26 evening; unit-tested only).**
  The robot turns to the heading and walks straight in at `terrain_push_speed` (0.15 m/s,
  in `config/speeds.yaml`, the one speed for uneven floor; `side_speed` is gone), in the
  H-Step gait, and on arrival is taken as centred (`trust_step`). **It pushes** (user,
  2026-11-26 night): the obstacle stop is off for that step, because the blocks and ramps
  themselves rise into its window, and 'no progress' is called after 10 s, not 8
  (`terrain_push`, `terrain_push_time`; 09:06 run: given up 0.65 m short after 8 s).
  So a wall or prop in a cell wrongly read as uneven is not stopped for either. Odometry jumped on almost
  every sideways step onto a ramp cell that day. `terrain_sideways:=true` (node parameter)
  brings the sideways entry back; what follows describes it.
- **Terrain, sideways entry (user, 2026-11-24; run on the robot 2026-11-25 and 26)**: a cell with uneven terrain is
  entered **sideways**. Uneven = 15 or more 5 cm squares of the cell's interior at 0.06 m or
  more above the floor, below object height (`Sense.rough`). In the cell before, the robot
  turns on the spot until its body is square to the way in, then steps sideways to the next
  centre at `terrain_push_speed` (`do_strafe`); the obstacle stop looks to that side. `terrain:=false`
  walks in head first. The gait on uneven floor is H-Step (`gait:=hstep`; `rug` = RUG general
  gear); on plain floor see "Gait by floor" below.
  **Depth camera for the floor (user, 2026-11-25):** the lidar's lowest ray meets the floor
  about 1.3 m ahead, so from the cell before, the next cell's floor is mostly unseen. When
  fewer than 25 (40 until 2026-11-26) of its 121 squares have been seen (and the lidar does not already say
  uneven), the standing depth check runs before walking in (`stair_check` action, stairs
  only looked for when that check was due anyway). It bins the depth points of the next
  cell into the same 5 cm squares, takes the median height per square, and calls it uneven
  on the same 0.06 m / 15 squares rule, by majority over the frames. Its verdict beats the
  lidar's; with too little seen the lidar's stands. Costs about 2-3 s standing and a short
  raw-depth subscription per new cell. `terrain_depth:=false` (node parameter) turns it off.
  **Off by default since the evening of 2026-11-26 (user: cut the depth floor checks;
  `floor_check:=true` brings them back).** Each one slowed every stream from the robot for
  1-3 s while the feed was already late. Uneven floor is now told by the lidar alone before
  the robot walks in; the floor is still read when a stair check runs anyway.
  **Uneven confirmed = no more floor checks (user, 2026-11-26; unit-tested only):** after 3
  frames that say uneven (`terrain_confirm`, and they are the majority) the floor is not
  read again. With no stairs to look for, the check ends there and depth closes (about
  0.4 s instead of 8 frames). The cell is remembered (`rough_known`): no floor check for it
  from any other side either. A flat verdict still takes the 8 frames.
- **Exit / end cell (user, 2026-11-26; unit-tested and simulated only)**: the maze is 3 x 5 and the
  end cell is the only cell of the 6th row along the start heading (`maze_rows:=5`), open at
  its far edge towards a wide open area. An open way from row 5 into row 6 reads `exit`: it
  is not walked as a visit, only as the last job. Nothing beside or beyond the end cell is
  entered, and an `exit` the map shows anywhere else is a wall. `maze_rows:=0` = the old
  rule (exit from the map alone, section 3.4).
  **End stop (user, 2026-11-26 night, after the revert; unit-tested only; `end_stop`)**:
  the end cell is simply the one cell of the 6th row; **being anywhere in that row is
  being in the end cell**. Whenever `plan()` finds the robot there, it goes straight to the
  end stop, whatever else is left. In it: square onto the cell's centre line facing the
  open (far) edge, creep forward until the nose is 0.10 m (`end_gap`) before that edge
  (base_link 0.14 m ahead of the centre), stop **standing**, `DONE`. Something in the way
  inside the end cell = it ends where it got to, still `DONE`. The drive into the end
  cell blocked before row 6 = `STOPPED` (a possible exit: that side becomes a wall and the
  next is tried). The side-wall offset (`end_side_gap`), `end_turn` and `end_lie_down`
  are gone; `maze_side` is now used only for the report course. The red line on the floor
  (`maze_info.md`) is not used.
  **Possible exits (user, 2026-11-26; unit-tested only):** the way into the end cell can
  read wrong (open floor beyond, little for the lidar to hit). On the far side of a last-row
  cell everything that is not a mapped wall (under 40 % of the edge occupied) is a possible
  exit: `unknown` after the robot has faced it (not made a wall there), `blocked`, `object`,
  possible `stairs` (no stair check, no look). A plain open way beats a possible one; of two
  possible ones the less walled is tried first. A possible exit is walked into with the
  obstacle stop on: stopped = that side is a wall and the next one is tried.
- **`maze_side`**: `explore.launch.py` asks "R or L" in the terminal at every launch
  (`maze_side:=R|L` answers it). Used for the course the reports go to (L = A, R = B); no
  longer for where the robot ends.
- **Gait by floor (user, 2026-11-26)**: two gaits only. `normal` = the normal gait of Move
  mode (`0x21010300`, "flat gait, slow"), for flat floor; `hstep` = RUG H-Step
  (`0x21010407`), for uneven floor. Before a step from a cell that is not uneven, into a
  cell that is not uneven either: `normal`; into an uneven cell: `hstep`. Standing in an
  uneven cell nothing changes. Each change is Move → gait → navigation mode, then 1 s more
  standing (`gait_settle`), about 5.5 s in all. The run starts in `normal`.
  `gait_switch:=false` = `gait` (H-Step) for the whole run.
  **The robot must never switch to grip (user).** RUG general gear is not a gait of ours
  either (it was the plain-floor gait for an hour that night by mistake). Only `hstep` and
  `normal` can be requested (`robot.GAITS`; any other name is taken as `normal`); the grip
  code `0x21010402` and the flat medium / fast codes are in `robot.FORBIDDEN` and
  `simple_cmd` refuses them. **Open:** in the 09:06 run the controller showed grip on the
  uneven blocks, where the only gait command sent was H-Step. Whether `0x21010407` shows
  as grip when it follows the normal gait, or the robot changed by itself, is not known;
  it needs watching on the controller at the next uneven cell.
- **Speeds** (what `speeds.yaml` holds at `ff405d8`: walking 0.30 m/s, **above the team
  limit of 0.25**; into uneven cells 0.15; curtains 0.20; stairs 0.55; turning 0.6 rad/s,
  0.3 on uneven floor. The user's last word that night was walking 0.25, turning 0.7, 0.10
  into uneven cells and through curtains; the file was not changed to that. At 0.30
  that day arrivals were no less accurate than at 0.20; turns drift 2-4 cm): `src/maze_explore/config/speeds.yaml` sets normal walk (`v_max`), uneven
  terrain (`terrain_push_speed`), stairs (`ai_speed`), curtains (`curtain_speed`), QR sideways check
  (`qr_speed`), turning on the spot (`w_max`) and the gains that hold the turn on its pivot.
  Read at launch, no rebuild. `v_max` and `ai_speed` are no longer launch arguments. After
  each turn over 20° the explorer logs how far off the pivot it ended.

- **Turns on uneven terrain only at the cell centre (user, 2026-11-26; unit-tested only)**:
  off-centre the legs come down on the ends of the ramps. Standing in an uneven cell more
  than 0.10 m (`terrain_centre`) off its centre, the robot first shifts onto the centre
  WITHOUT turning (`do_shift`: forward and sideways steps at 0.08 m/s, heading kept), once
  per stay in the cell, and only then turns. Nothing drives backwards, so the part of the
  offset that lies behind the robot stays. **Not right after a sideways step onto the
  uneven cell** (user, 2026-11-26): odometry is known to slip during that step, so on arrival
  the robot is taken as centred and neither shifts nor walks back onto the centre for that
  stay (`trust_step`; 08:01 run: 0.18 m off on arrival, the shift then shuffled 8 s and
  ended 0.25 m off).
- **Depth camera against lidar (user, 2026-11-26; unit-tested only)**: when the depth check
  has read a cell's floor (60 or more of its 121 squares, `depth_clear_min`) and found
  nothing 0.20 m or higher in it, that cell is `free` even where the lidar calls it
  `blocked`, possible `stairs` or `object`: the robot walks in with the obstacle stop on,
  and the side becomes a wall only if it is stopped. Before, such a cell became an `object`
  (looked at, never entered); in the 07:45 run three of them boxed the robot in after six
  cells. Such a cell is LOOKED AT first and then walked into (a poster or sign on its walls
  must still reach scene understanding). A `room` (dead end) is still looked at, not
  entered. `depth_clear:=false` = as before.
- **Realign on arrival (user, 2026-11-26; unit-tested only)**: in every cell, once the robot
  has stopped at the centre and its pose has been still for 0.7 s (it waits up to 4 s), the
  cell grid is fitted to the mapped walls again with wider limits than the small
  correction before each planning step: 8 deg and 0.20 m (`arrive_fit_deg`, `arrive_fit_m`)
  against 3 deg and 0.08 m, which was refused as soon as the grid had drifted further than
  that. Not while scans to SLAM are off. It moves the CELLS onto the map; the SLAM map
  itself cannot be moved. `arrive_fit:=false` = off.
- **Slower turns on uneven floor (user, 2026-11-26; unit-tested only)**: in an uneven cell
  the turn rate is capped at 0.3 rad/s (`terrain_w_max` in `config/speeds.yaml`;
  `w_max`, 0.6, elsewhere). 07:42 run: the sensor feed went late and Fast-LIO diverged during a 0.5 rad/s
  turn on a tile.
- **Start-cell guards (2026-11-26, after the 03:29 run; unit-tested only)**: in that run the
  first grid fit put the start centre 0.3-0.4 m behind the robot, the recentre rule walked
  it at the back wall three times and the third time it pushed into the wall. (1) While the
  robot is in the start cell a grid fit may move the start centre at most 0.12 m
  (`start_u_max`) along the start heading from where the robot was put down; beyond that
  the centre stays at the placed spot along the heading (sideways and angle still from the
  walls). So **place the robot centred fore and aft**. The back wall cannot be the
  reference: the lidar has no returns behind the robot there (bag
  `explore_20261125_100238`: 0 points behind, about 2000 on each side wall in 30 scans).
  (2) A walk back onto a cell centre that is blocked is not tried again from that cell,
  and in the start cell the wide grid fit is made once more. (3) The obstacle stop
  remembers what it saw for 60 s (`block_memory`): Fast-LIO drops returns within 0.30 m of
  the lidar, so a 0.53 m wall straight ahead is visible only while the nose is 14-20 cm
  from it. Not covered: a drive that STARTS with the nose already closer than 14 cm to a
  wall the stop has never seen.

- **False curtains (user, 2026-11-26; unit-tested only):** with `curtain_cv` on, the lidar alone proposes no curtain. A tall
  edge (tall walls read exactly like curtains: 04:42 and 05:30 runs, beside the start
  cell and beside cell (1, 0)) is a wall until the curtain camera has pointed at it at
  least once (`camera_pointed`); then the usual checks follow. Cost: a real curtain the
  camera never fires on while the robot moves about is never looked at squarely.
  `cv_first:=false` = as before. With `curtain_cv:=false` the lidar decides alone.
- **False stairs (user, 2026-11-26; unit-tested only):** a depth-camera STAIRS verdict counts only if the lidar's height map
  shows something 0.38 m or higher in that cell (`stair_lidar_top`; stairs are 0.45 m,
  terrain under 0.30 m). 04:42 run: "stairs 0.70 m, levels 0.12 / 0.25 / 0.38" ahead of
  the start, in a cell later walked through; what the camera saw there is not known (no
  depth on record). `stair_agree:=false` = the camera alone, as before.
  **What the lidar has not measured it cannot reject (user, 2026-11-26 night; unit-tested
  only):** with the 0.35-0.65 m band of that cell unmeasured in the height map (stairs seen
  only through a doorway), the camera decides alone; the lidar rejects only what it has
  measured low.
  **VLM backup (user, 2026-11-26 night; unit-tested only):** the look prompt also asks
  `stairs` (a climbable staircase; toys and shelves are false). A yes on `/scene/result`
  reopens the depth check for that edge (`vlm_stairs`, kind() makes it a `stairs`
  candidate again, a `False` verdict is dropped); the depth detector keeps the last word.
  Never once the stairs are crossed.
  **Square to the grid (user, 2026-11-26 night):** stairs the depth camera sees at more than
  15° to the grid direction (`stair_slant_max`) are not stairs; real stairs stand with their
  steps parallel to the cell's edge (10:39 run: the play kitchen was read as stairs).
- **Re-anchor after an odometry jump: OFF by default (`jump_fix:=true` = on).** First real
  use, 05:57 run: 2 s after a 0.32 m jump the pose read 0.38 m / 9.7° off, the grid was
  turned by that, the map did not turn with it (slam_toolbox kept matching scans to the
  walls it had), and the cells ended up 10° askew to every mapped wall; the wall fit
  could not bring them back. What it does when on: the robot stands from the first jump on, so where
  it believed itself just before is where it still is. Once Fast-LIO has settled (no jump
  for `odom_settle`, pose still for 1 s, reported tilt under 25°) and the map pose is more
  than 0.10 m / 4° off that, the cell grid is moved and turned by the difference, a few
  medium wall fits near the robot refine it, and what was known of the OTHER cells is
  kept as it was (`self.kept`; the map of them no longer lines up). The height map and
  the obstacle memory start afresh. Not done when Fast-LIO has not settled after 6 s
  (it has diverged) or the pose is more than 1.0 m / 30° off. Known weak point: walls
  mapped before the jump stay in `/map` beside the new ones near the robot.
  `jump_fix:=false` = off.

- **Against map drift (user, 2026-11-26; unit-tested only)**: (1) SLAM gets no scans while
  odometry is not to be trusted (`/maze/blind`, as under a curtain): during a burst of
  odometry jumps and 1 s after it, while the sensor feed is late, and for the whole AI walk
  over the stairs; the explorer's pose then comes from the held `map -> odom` and the
  newest odometry, and the height map takes no clouds. `map_blind:=false` = off. (2) Tried
  and taken back the same day: grid corrections from the walls within 2 m only
  (`grid_fit_radius`). On the robot the correction was then refused 9 times out of 16
  (07:45 run; 4 of 19 with 4 m in the 05:57 run), the grid drifted and cells' own walls
  read as objects inside them. It is 4 m again.

- **Fewer readjustments at looks (user, 2026-11-26 night; unit-tested only)**: squaring up
  onto a centre line now accepts 8 cm and 5° (`square_tol`, `square_deg`; was 4 cm / 3°,
  and the robot shuffled sideways again and again before a look). A curtain-camera sighting
  is checked (a turn and 3 s of looking) only once the lidar shows at least 20 % of that
  edge curtain-high (0.65 m and more; `cv_tall_min`); an edge measured low is dropped, one
  not measured yet waits.
- **Fewer false curtains (user, 2026-11-26 night; unit-tested only)**: (1) the camera needs
  5 frames (`cv_verify_hits`, was 3) at confidence 0.6 (`cv_conf`, was 0.5) to confirm; it
  looks 4 s (`cv_verify_time`, was 3). A real curtain the camera does not confirm becomes
  a wall for good, so watch the first real crossing. (2) Floor-gap test (`Sense.solid_base`,
  `HeightMap.base_hits`): a curtain hangs from about 0.9 m and leaves about 0.15 m free
  under it, so an edge with lidar returns 0.06-0.12 m above the floor along half of it or
  more (`base_share_max`) stands on the floor. **Used only when the lidar decides alone
  (`curtain_cv:=false`)**: a raised platform behind a real curtain fills that gap too
  (user, same night), so the test never overrules the camera or stops a camera check.

Safety behaviour:

| Condition | Reaction |
|---|---|
| `/odom` moves more than 0.15 m (`odom_jump_min`; 0.25 for a few hours on 2026-11-26, when a 0.18 m slip during a sidestep onto a ramp cell went unnoticed) + 0.6 m/s × the time since the last message | stand until it has been steady for 1.5 s (`odom_settle`; was 3 s: user, "pausing for too long"), then carry on. Jumps that follow each other within 3 s (`odom_burst_gap`) are one burst and do not add up (user, 2026-11-26: allow some jumping; 05:03 run: /odom swung ±0.3 m sideways for 1.7 s on a late feed, 10 jumps, then held, but 3 jumps in 10 s was a `FAULT` then). `FAULT` (stop for good, relaunch) only when it does not come back: a burst still going after 6 s of standing (`odom_burst_max`), a pose more than 1.5 m from where the burst began (`odom_net_max`), or one jump over 2 m. Unit-tested on the 05:03 trace only. **On uneven floor more is allowed** (user, 2026-11-26; `odom_terrain`, default 2): standing in an uneven cell, or on a step into or out of one, a step of `/odom` counts as a jump only from twice the size (0.30 m), and a burst may last and the pose may end twice as far (12 s, 3.0 m) before `FAULT`; one jump over 2 m is a `FAULT` anywhere |
| Sensor feed late: `/odom` arrives more than 0.30 s (`feed_late`; 0.15 until the 05:30 run, where 11 of 15 pauses began under 0.30 s) later than usual after its stamp (2026-11-26) | stand still until it has been on time for 0.5 s (`feed_settle`); at most 6 s in one go, then carry on for 10 s at 0.10 m/s at most (`feed_slow`; 05:03 run: it went on at 0.30 m/s and odometry swung 5 s later). Every odometry jump on record followed a late feed while the robot was moving (bags of 11-25 and 11-26: 12 late episodes moving, 2 jumps; 24 standing, none). `feed_pause:=false` (node parameter) = off |
| The map pose moves 0.5 m or more between two ticks (`pose_jump_max`; 2026-11-26 night, unit-tested only) | everything planned is dropped; the robot stands until the pose has been still for 2 s (12 s at most), lines the grid up with the walls again and plans afresh. 10:39 run: a 45° turn "moved" the robot 1.04 m and what it planned from there took the play kitchen for stairs. A slip of a whole cell cannot be seen by the wall fit (walls line up with cell borders either way), so this stops acting on a jumped pose but cannot always put it right |
| Pose or lidar cloud older than 1 s | stand still until fresh (the action clock waits too) |
| Something knee-high or taller 6–20 cm ahead while walking, and while pushing (`push_stop`) | stop; after 8 ticks give up the step |
| No progress for 8 s (12 s in AI mode) | give up the step |
| A turn that does not finish | given up after the time it should take at its top rate, half as much again, plus 6 s (`turn_timeout`): about 10 s for a quarter turn. It was 25 s whatever the turn (09:06 run: six turns that went nowhere cost 2.5 minutes) |
| A step given up | the side becomes a wall, robot creeps back to the cell centre. In AI mode: `STOPPED`, take over by hand (`lite3_ai_off`), then `/nav/start` |
| `/explore/stop` | `STOPPED`; `/nav/start` resumes, and first sends Move → gait → navigation mode again (about 4.5 s standing), so a controller takeover or `lite3_ai_off` in between needs no `lite3_auto` (2026-11-25) |

### 3.7 Topics

| Topic | Type | Notes |
|---|---|---|
| `/nav/start`, `/explore/stop` | `std_msgs/Empty` | in |
| `/explore/status` | `String` | `STATE cell=… job=[…] act=… why`, 10 Hz. States: `WAIT`, `READY`, `RUN`, `DONE`, `STOPPED`, `FAULT` |
| `/explore/event` | `String` | `curtain …`, `room centre\|left\|right …`, `stairs: …`, `terrain: …`, `exit …`, each with cell and side. Meant for perception and logging |
| `/explore/cells` | `MarkerArray` | visited cells and the kind of each side, frame `map` (RViz) |
| `/maze/blind` | `Bool` | True while pushing through a curtain |
| `/curtain/seen`, `/curtain/status` | `String` | `curtain_node` (on by default; `curtain_cv:=false` = off): JSON per camera frame with the curtain boxes and their direction; `UNAVAILABLE: why` / `IDLE` / `RUNNING` |
| `/cmd_vel` | `Twist` | RELIABLE + TRANSIENT_LOCAL, 0.5 s lifespan. One zero, then silence, so teleop works again |
| `/simple_cmd` | `MotionSimpleCMD` | mode and gait requests, 1.5 s apart |

## 4. Build, run, test

```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash     # transfer_interfaces, livox_ros_driver2
source ~/ros2_ws/install/local_setup.bash      # lite3_launcher (odom_relay), fast_lio
cd ~/challenge_ws
colcon build --symlink-install                 # ALWAYS --symlink-install
source install/setup.bash
```

`~/.bashrc` on the robot computer already sources these and sets `ROS_DOMAIN_ID=4`. A plain
`colcon build` (without `--symlink-install`) replaces the links with copies and breaks every
other session's edits. `~/test_scene_understanding/install` uses `~/challenge_ws/install`
as an underlay, so do not delete `install/`.

Robot:

```bash
ros2 launch maze_explore explore.launch.py                 # full stack
ros2 launch maze_explore explore.launch.py explore:=false  # mapping only (hand-driven)
ros2 launch maze_explore explore.launch.py drive:=false    # decides, sends nothing
ros2 topic pub --once /nav/start std_msgs/msg/Empty    # go
ros2 topic pub --once /explore/stop  std_msgs/msg/Empty    # stop
```

Navigation only: `plain:=true` = no scene understanding trigger (`junction_trigger`), no
stairs (check and crossing; stairs are a blocked cell), curtains are walls, uneven cells are
walked into head first. Single switches: `scene_understanding`, `stairs`, `curtains`,
`terrain`, `qr_approach` (needs `scene_understanding` and `qr`). `plain` also turns `curtain_cv` (on by default) off. `task_reporter` keeps running (`report:=false` stops it).

`maze_side` (user, 2026-11-26): the launch asks "maze on the right (R) or on the left (L)?"
in the terminal before anything starts; `maze_side:=R|L` answers it. Not asked with
`explore:=false`. Without a terminal and without the argument the launch refuses to start.
It reaches the explorer as parameter `maze_side` (`self.maze_side`, only logged there) and
task_reporter, for the report course.

Other launch arguments: `auto_start`, `gait` (`hstep` / `rug`, for uneven terrain), `flat_gait`,
`gait_switch`, `maze_rows`, `enter_rooms`,
`stand_height`, `cloud_topic`, `fastlio`. Defaults are in `launch/explore.launch.py`.

Simulator (no robot needed, cannot reach the robot):

```bash
src/maze_explore/sim/run_sim.sh [maze=challenge_a.json] [timeout_s=900] [log_dir] [domain=89]
```

It runs `sim/fake_maze_lite3.py` and the launch on a spare ROS domain, localhost only, and
exits 0 when the explorer reports `DONE`. Mazes are in `maze_examples/sim/`
(`challenge_a*.json`, `challenge_b*.json`, `maze_curtains.json`, `maze_stairs.json`,
`curtain_maze.json`, `extru_maze.json`, …). Logs: `launch.log`, `sim.log`, `status.log`,
`events.log` in the log directory. The docstring in `fake_maze_lite3.py` still names
`maze_nav` paths. The simulator has no colour camera, so with `curtain_cv` on (the default)
no curtain is ever confirmed and every curtain becomes a wall: pass `curtain_cv:=false` to
test curtain crossings there.

**Sessions start simulators through the queue (2026-11-26):**
`src/maze_explore/sim/queue_sim.sh <tag> [maze] [timeout_s] [launch args...]`. It holds a
lock (`logs/sim/lock`), so runs follow each other with no gap and never overlap, uses
domain 97, writes its logs to `logs/sim/<date-time>-<tag>/` and one line per run to
`logs/sim/index.tsv`. A run of the same maze, arguments and code (HEAD + uncommitted `src`
changes) is not repeated: the script prints the earlier result (`FORCE=1` runs anyway).
Tell the other session what you are about to run, so one run covers both sets of edits.

Unit tests: `nice -n 19 python3 -m pytest src/<package>/test -q`. `team_perception` and
`scene_understanding` and `task_reporter` have tests; `maze_explore` has `test/test_*.py`
(logic, exit stop, start guards, odometry jumps, curtains, QR approach, stairs).

## 5. Perception and reporting

| Package | Nodes | State |
|---|---|---|
| `team_perception` | `depth_relay`, `qr_scanner`, `human_detector`, `poster_detector` | Guides: `launch_guide.md`, `perception_guide.md`, `data_required.md`. `challenge_ws` is the master copy; do not re-sync from Robot-Dog-Grp4 |
| `scene_understanding` | `junction_trigger`, `room_scene` | **Connected to `maze_explore` (2026-11-24)**, started by `explore.launch.py`. `junction_trigger` takes the three views of each room / object look (compressed colour, subscribed only during the look), sends them as one 2 x 2 image with `LOOK_PROMPT` (scene type, hand tools, hazard, survivor; no QR) to the lab VLM (`192.168.10.141:8000`, images at most 1024 x 768, 2 s cooldown) and publishes `/scene/result`. `scene_understanding:=false` turns it off. `room_scene` is the old path, not started |
| `task_reporter` (own package, user 2026-11-24) | `task_reporter` | Takes `/scene/result` (scene understanding) and `/qr/read` (`qr_scanner`), maps them to the five missions (`missions.py`), checks the format (`missions.check`) and POSTs each report as `group04_grp4` to `http://192.168.10.141:8001/task_report_A` or `..._B`: **the course follows `maze_side`, L = course A, R = course B** (user, 2026-11-26; `missions.course_url`; a `report_url` ending in `_A` / `_B` overrides it; with neither, nothing is posted and the node logs an error; practice mode, live since 2026-11-24; the reply carries the per-field evaluation). **A report the server cannot take yet is kept and sent again every 5 s** (user, 2026-11-26 night; `retry_period`): HTTP 409 ("No active competition slot", every report of the 10:39 run) and server errors count as not delivered, not as a wrong answer, so the sighting is not dropped; a newer report for the same mission replaces the waiting one. Tested against a local fake server. M01, M02, M03 and M05 carry `pos`; **M04 (the QR text) is sent with `pos` null**: the server rejects a QR report with a position (08:01 run, 2026-11-26, HTTP 422; until then every report carried one). `pos` is always the centre of the cell the robot is LOOKING AT (user, 2026-11-25), taken from the explorer's look events and from its status while the job is a look, so both coordinates are whole multiples of 1.02; never where the robot stands. One answer per mission: M01 only for the object the mission asks for (`m01_object`, default `hand_tools`; a setting because the missions may still change, user 2026-11-24), M02 for a hazard sign (`m02_hazard`, default any); the first sighting holds it. Everything else seen is recorded on `/mission/sightings`, not submitted. Imports nothing from the other packages. `report:=false` turns it off |
| QR during looks | `qr_scanner` (in `team_perception`) | `explore.launch.py` starts one (`qr:=false` = off) with `on_demand`: its camera is subscribed, and every frame fully scanned, only while `/explore/status` shows a look job, the same moment scene understanding takes its views. Do not run `robot.launch.py` as well (a second scanner). It ends with `maze_explore` (user, 2026-11-24): the launch interrupts it when the `maze_explorer` process exits, and it stops itself when the launch dies in any way (`EXIT_WITH_PARENT`, plus a parent check every 2 s). 2026-11-24 10:54 frame: the code on the survivor poster was about 35 px wide from the cell before and the decoder did not even locate it |

**The gap to close.** These packages were written against the old explorer and still
listen to its topics, which nothing publishes now:

| They expect | From the old stack | New stack offers |
|---|---|---|
| `/maze/perceive`, `/maze/status` (`run_gate.py`) | when to run image work | `/explore/event`, `/explore/status` |
| `/maze/grid` (`poster_detector`) | cell grid for survivor positions | nothing yet |
| `/scene/capture` (`room_scene`) | room views to analyse | `room centre\|left\|right` events (used by `junction_trigger` and `task_reporter`) |
| `/qr/unread`, `/perception/look_request` | asks the explorer to face something | not subscribed |

Scene understanding and reporting are bridged (table above). Still open: `poster_detector`
and the `/qr/unread` / look-request path.

Decisions from the user that still hold:

- All perception runs on the robot. No offboard computer.
- A survivor is a poster. Report the **cell centre**, not the poster's position. Track
  posters in `odom` and place them with the latest `map <- odom`; never track in `map`
  (it jumps on SLAM corrections).
- QR codes appear only in rooms with survivors, on the left or right wall. QR positions are
  not reported. **Booklet update 2026-11-24:** there are exactly two survivors and one QR
  code. The survivor WITH the QR code is M03 and its decoded text is M04 (reported exactly
  as decoded, e.g. "Please help me."); the survivor WITHOUT one is M05. The QR text is no
  longer a hint to another survivor's room.
  While no code has been read: a room where the VLM saw a survivor AND a QR code is M03 at
  once; failing that, the first survivor found is M03 and the second M05 (user,
  2026-11-25). A code read later puts M03 in its room and both go out again.
- Detect QR codes and survivors only at dead ends (rooms); plain QR reading may run always.
- The exit is a plain opening, never a curtain or stairs.

## 6. Working rules

**Several agent sessions share this workspace and this computer.**

- Another session may be editing the same package. Message it before you change a file it
  is working on. Never `pkill` by name; kill only your own PIDs.
- ROS domains: **4** = the real robot. **89** = the user's simulators. **117 / 121** = other
  sessions. Check that a domain is empty before using it, and say which one you use.
- The robot's own stack (Fast-LIO and the rest) runs on this computer. Run simulators and
  offline jobs with `nice -n 19` and `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`. Queue
  simulator runs instead of running several at once.
- Our nodes never talk to the hardware. `lite3_domain_bridge_4` republishes sensors into
  domain 4 and forwards `/cmd_vel` and `/simple_cmd`. Do not start the manufacturer
  `transfer` launch here.
- Git: the user pushes. Commit only when asked; hand over the push command and PR text.
  `robot_challenge_workspace` is the main repo; PRs go to `main`.
- Never commit bags (`*.db3`), `runs/`, `maps/`, `logs/`, `build/`, `install/`, `log/`.
- Logs go to `~/challenge_ws/logs` (`ros/` through the `~/.ros/log` link). A daily cron job
  deletes anything older than 7 days there, except `cpu/` and `junctions/`. Since
  2026-11-25 every `explore.launch.py` run has one folder, `logs/ros/<date>-<time>-…/`, with
  `launch.log` and each node's log named after the node (`maze_explorer.log`, …). Logs of
  nodes started with `ros2 run` stay at the top of `logs/ros/` as `<node>_<date-time>_<pid>.log`.
- Bags: `runs/ROS_bags/ROSBAG_GUIDE.md`, aliases in `runs/ROS_bags/aliases.sh`. `rec_test` and
  `rec_lean` are one recorder since 2026-11-26 (user). Since that night only the small
  topics: tf, `/odom`, `/scan`, commands, `/explore/*`, scene / QR / mission results,
  `/curtain/seen`. No cloud, no `/map` (rebuild it from `/scan`), no `/rosout`, no camera, no
  `/livox/imu`. A few MB a minute, at nice 19 / idle disk priority, uncompressed. Add a
  topic for one run after `--`. With `/livox/imu` in the preset (one run, 04:42, bag `run1`)
  the feed was late 26 % of the run and the IMU arrived at 109 Hz: practice runs only. Hand-driven
  data run: `explore_check`, `explore.launch.py explore:=false`, then `rec_manual <name>`
  (guide 2.2). The recorder prints sensor rates after recording (no check before), compresses with zstd, and
  refuses depth.
- Python on the robot: apt `cv2` 4.5.4 only (no pip OpenCV), numpy pinned to 1.24.4,
  OpenVINO 2024.6 for YOLO.

## 7. Lessons from the old stack

These are facts about the robot and the arena. They cost real runs to learn; keep them in
mind when changing `maze_explore`.

Arena and body:

- Cells are 1.02 m. Walls are 0.53 m tall; the lab's own `/scan` passes over them. Some
  walls have 10–25 cm gaps. The aluminium-frame ("extru") walls have see-through side
  windows 10 cm wide.
- The Lite3 stands about **0.29 m** high (base_link above the floor), not 0.35. The
  simulator uses 0.35.
- Curtains hang from about 0.9 m, with a gap of about 15 cm at the bottom and a 1–3 cm
  split. Stairs: pyramid, 3 levels, 15 cm rise, 20 cm tread, reflective metal, railings.
- Terrain tiles are raised about 10 cm; rubble up to 15 cm.

Sensors and odometry:

- The bridge sends one full copy of a stream per subscribing process and saturates past
  about 4 copies. Use **one subscriber per heavy stream** and republish locally.
- **Nothing keeps raw depth open (2026-11-24).** `depth_relay` is on demand by default
  (`depth_on_demand:=true`): it subscribes only while the explorer holds a room look
  (`/explore/event` starting with `room` / `object`) or on `/scene/capture`, and closes 6 s
  after the last one. The stair check opens depth itself for at most 3 s (1.6 s when no
  frame looks like stairs). `archive/stairs_detect/stair_watch.py` still keeps depth and
  `/livox/points` open: do not run it during a run.
- **One subscriber to raw depth (`/camera/depth/image_raw`), even BEST_EFFORT, slows every
  bridge stream to about 10 Hz** (measured 2026-11-23: `/imu/data` 195 → 11 Hz). The same
  goes for `ros2 topic hz` / `echo` on a big topic or RViz showing depth during a run.
  There is no compressed depth or compressed lidar topic; bag compression saves disk only.
- Fast-LIO needs `/livox/imu` near 200 Hz. At 8-20 Hz its pose runs away by kilometres
  within seconds. Run `explore_check` before every run.
- Fast-LIO with `blind: 0.5` lost walls closer than 0.5 m and its odometry jumped. Keep
  `blind: 0.3` (`config/fastlio_mid360.yaml`).
- Near lidar returns from the robot's own body show up as obstacles. Crop them.
- **No height-map updates while the robot turns (2026-11-25).** The explorer places each
  cloud with its last tick's pose, a little older than the cloud; at turning speed that
  swings the walls of the next cell into its middle. 10:02 run: the terrain cell ahead of
  the start (real heights 0.2-0.3 m) read as an `object` during the first turn and set off
  a look and scene understanding in the first cell. Clouds are skipped while /odom turns
  faster than 0.2 rad/s and for 0.3 s after (`heights_turn_max`, `heights_settle`).
  Replayed on bag `explore_20261125_100238`: 12 object squares before, 0 after.
- **The map pose stops while `/maze/blind` is on (2026-11-26).** No `/scan`, so slam_toolbox
  stops renewing `map -> odom` and the newest `map -> base_level` stays at the moment the
  crossing began. The explorer's pose froze and its stale-pose rule then held the robot for
  good, 0.18 m into the push (simulator, `challenge_a`, `push_stop:=false`). During a
  crossing the pose is now the held `map -> odom` with the newest `odom -> base_level`
  (`read_pose_blind`). Crossing completed in the simulator after that; not run on the robot.
- Fast-LIO z drifts slowly; do not trust odometry height over a long run without
  re-measuring the floor.
- **Never do a blocking TF lookup in a callback.** A 0.1 s blocking lookup once left a node
  5 s behind. Use `lookup_transform(..., Time())` and handle the exception.
- The camera has a second, unpitched TF parent in the robot's `tf_static` that is 20° wrong.
  Use `camera_link`.
- Fast DDS over shared memory stalled on this machine: use `config/fastdds_udp.xml`
  (UDP only, 4 MB buffers) in the simulator.

Motion:

- The Lite3 does not turn below about 0.1 rad/s. Command at least 0.15–0.2 rad/s
  (`min_spin`), or a slow final turn never finishes.
- AI state cannot be entered from navigation mode: go to manual first (`robot.py`).
- Pose-mode yaw: about 16° is the full range; the robot buzzed at the limit in a tight cell.
  Side looks are done by turning the body 25° instead.
- Lying down to look under a curtain was refused when odometry height had drifted.
- `/cmd_vel` and `/simple_cmd` need RELIABLE + TRANSIENT_LOCAL QoS to reach the bridge.

This computer's heat (2026-11-26):

- **It overheats.** An i5-13420H box in a container: 67-70 C idle; four busy cores take it
  to its 100 C limit within 3 s and it is throttled (9 of 12 s in a test; 13 % of the time
  between 07:34 and 08:08). Fast-LIO then falls behind, the sensor feed reads late and the
  odometry jumps. In the 08:37 run the feed was 0.77 s late 5 s after the start, before the
  robot had moved.
- **Checked before every launch** (`heat_gate` in `explore.launch.py`, and in
  `explore_check`): 75 C or more, being throttled right now, or RViz / a simulator / a test
  run going = NO-GO with the reasons; the launch then asks before it goes on.
  `heat_check:=false` skips it. By hand: `python3 -m maze_explore.heat`.
- **Logged during every run**: `heat.csv` in the run's log folder
  (`logs/ros/<date>-<time>-.../`), one line every 2 s: temperature, ms throttled in those
  2 s, load, turbo.
- **RViz is the biggest avoidable load**: software-rendered over the remote desktop, about
  half a core plus the remote desktop itself. Close it during runs.
- **Turbo boost is on and cannot be turned off from here**: the setting belongs to the host
  (`/sys/devices/system/cpu/intel_pstate/no_turbo`, write refused in the container). On the
  host, as root: `echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo`.

Measuring:

- This computer is a container on a shared host. The `_total_` CPU figure includes other
  tenants: add up the per-process rows instead.
- A simulator started during a robot run starved odometry once. Do not start one mid-run.

## 8. Open items

- [ ] Run `maze_explore` in the simulator on every maze in `maze_examples/sim/` and record
      the results.
- [x] First robot runs: done 2026-11-25 and 26 (section 1).
- [ ] One robot run that reaches the end cell without an odometry fault. Try
      `gait_switch:=false` and `v_max` 0.20 to test the two suspects of section 1.
- [x] Unit tests for `grid.py`, `sense.py`, `plan.py` (`test/test_logic.py`).
- [ ] Connect perception and scene understanding to `/explore/event` and the cell grid
      (section 5).
- [x] Mission reports reach the scoring server (practice mode, since 2026-11-24). `room_scene`
      is the old path and is not started; `junction_trigger` does the VLM work.
- [x] Decide whether the curtain YOLO model confirms a curtain before the robot pushes:
      yes, with `curtain_cv` on (user, 2026-11-25; section 3.6).
- [ ] `/livox/imu` reaches this computer at 10-20 Hz instead of 200 (2026-11-23 afternoon,
      nothing subscribed). Robot or bridge side; no mapping or autonomous run until
      `explore_check` says GO.
- [ ] Depth cannot be recorded with the mapping stack running. A `compressedDepth`
      publisher on the robot side of the bridge would fix it (ask the lab).
