# stairs_detect: our own stair detector (23 Nov 2026)

New, standalone files. Nothing in lite3_nav or maze_nav is changed.

| File | What |
|---|---|
| `stair_levels.py` | The algorithm (numpy only) + `Confirm` (voting) |
| `stair_watch.py` | ROS 2 node: depth + IMU in, `/stairs/status` out. Never drives |
| `replay_bag.py` | Runs the algorithm on a recorded bag on the laptop (no ROS) |

## How it works

It looks for the challenge stairs specifically (3 steps up, 12 cm rise, 20 cm tread):

1. Depth image -> points in a level frame (the IMU's roll/pitch removed; no odometry needed).
2. Keep the flat (horizontal) points. Floor = lowest flat layer near 0.35 m below base_link;
   close up, where the floor is out of view, it tries one or two rises lower and keeps the
   floor that puts the most points on 12 / 24 / 36 cm.
3. Treads = flat points at 12, 24 and 36 cm (+-4 cm). **The 36 cm top must be there**, plus at
   least one lower tread one tread (20 cm) nearer. Rough cells (8 cm) and the block field
   (<= ~22 cm) never reach 36 cm.
4. Stair heading = the angle that makes the tread edges sharpest; the bottom riser distance
   from the treads' far edges (never cut off by the camera view); lateral = middle of the treads.
5. `Confirm`: CONFIRMED once 3 of the last 5 frames (within 0.8 s) agree within 8 cm / 8 deg /
   10 cm; stays CONFIRMED 1.5 s after the last agreeing frame.

## Results

Real bags (`build/stairs_bags_real`, cell next to the stairs):

| Shot | Frames with stairs | Distance (base_link -> bottom riser) | Heading | Lateral |
|---|---|---|---|---|
| far_a0 | 83 / 83 | 0.72 (0.71-0.72) | -1 deg | -0.04 |
| mid_a0 | 80 / 80 | 0.58 (0.58-0.58) | 0 | 0.00 |
| near_a0 | 57 / 62 | 0.26 (0.26-0.27), bottom inferred | -1 | -0.03 |
| far_a30 (turned left) | 75 / 75 | 0.68 | -22 | +0.19 |
| far_a-30 (turned right) | 82 / 82 | 0.66 | +23 | -0.26 |
| far_left20 | 78 / 78 | 0.74 | -1 | -0.23 |
| far_right20 | 64 / 64 | 0.71 | 0 | +0.20 |

- Walk-ins: no detection before the robot turns into the cell; CONFIRMED ~2 s after the turn,
  never before. Confirmed within 0.2-0.4 s on the still shots.
- Sim (Tricia's renderer, arena terrain): block field 0 / 120 false hits; raised 8 cm cell,
  two raised cells, slat frame, wedge ramp: 0 false hits. Up-and-over stairs at 0.3-1.2 m,
  0 and +-23 deg: distance and heading exact (0.3 m square-on misses one frame).
- ~11 ms a frame on a laptop CPU (5 Hz on the robot is plenty).

Known: close up (bottom tread out of view) single frames can be off by a tread; voting drops
them. Beyond ~1.4 m the top tread is too sparse (not needed: the robot is never further than
the neighbouring cell when it matters).

## Run on the robot (detect only, never drives)

Copy the folder to the robot (File Browser `http://192.168.10.74:8080`, or
`scp -r /c/GitHub/robot_challenge_workspace/stairs_detect user@192.168.10.74:~/` from a
laptop Git Bash). On the robot:

```bash
source /opt/ros/humble/setup.bash
cd ~/stairs_detect && python3 stair_watch.py
```
Second terminal: `ros2 topic echo /stairs/status`. Point the robot at the stairs from the
neighbouring cell: within a second it should read `CONFIRMED: stairs 0.7x m at ...`.
`-p log_every_frame:=true` prints every frame's result and why.

## Offline on the laptop

```bash
pip install rosbags zstandard
python3 stairs_detect/replay_bag.py build/stairs_bags_real/stairs_walkin1
```

## Tread check (23 Nov, against the block obstacle)

Each tread must be one unbroken surface across the middle 0.8 m, with a straight front edge.
Shiny spots with no depth are allowed, but a strip holding a flat surface of *another* height
(a cube of the block obstacle) counts against it: more than 3 of 16 such strips rejects the
frame (`tread k broken: another height in n of 16 strips`). The front edge must also be straight
(within 4 cm) for at least 0.3 m. Photo-like block obstacle in sim: 200/360 views were taken for
stairs before, 2/360 after. Real recordings: mid, near, 30 deg and both walk-ins still confirm,
at the same moments as before. The lidar rails check is now off by default
(`-p require_rails:=true` turns it back on).

## 24 Nov retune (mid2 / near2 / obstacle snapshots)

Today's depth on the top tread was much sparser (1-3 points per 5 cm strip instead of 8-15), so
the straight-edge check failed half the frames at mid and near. Now: holes up to 6 strips are
bridged, edge tolerance 3 cm, at most 2 foreign strips, top level >= 0.35 m wide. Votes count
within 1.2 s (was 0.8 s, too short for 3 of 5 at the node's 5 Hz). Replay at 5 Hz: mid2 and
near2 confirm from 0.4 s, real obstacle 0 hits, sim block obstacle 3/180 single frames.

## Full run with AI mode over the stairs (24 Nov, copies, originals untouched)

- `stair_nav.py`: copy of maze_nav/stair_detector.py, stairs found by stair_levels.py; keeps the
  explorer hand-over, approach (stops 0.45 m before the first step), squaring up, AT_STAIRS.
- `cmd_vel_bridge_imu.py`: copy of maze_nav/cmd_vel_bridge.py; on AT_STAIRS: manual mode, AI
  state, AI basic gait, walk forward, then AI exit + auto mode. The walk ends when the IMU has
  seen the body tilt one way (up), the other (down), then level for 1 s (+0.10 m), capped at
  2.2 m (original: 1.3 m by odometry only).
- `maze_nav_stairs.launch.py`: copy of maze_nav.launch.py running those two instead.

    ros2 launch ~/stairs_detect/maze_nav_stairs.launch.py      # same arguments as maze_nav.launch.py

Don't run stair_watch.py at the same time: both publish /stairs/status.

## On the robot's maze_explore stack (24 Nov)

The robot runs maze_explore (branch exploration-v2), not maze_nav: the maze_nav copies above
(stair_nav.py, cmd_vel_bridge_imu.py, maze_nav_stairs.launch.py) are not used there.
maze_explore already crosses stairs: lidar height rule -> square up in the cell before ->
AI mode -> drive to the centre of the cell after (SLAM) -> AI exit, auto.

- `explorer_stairs.py`: copy of maze_explore/explorer.py; before AI mode, squared up, the depth
  camera is subscribed for up to 4 s and stair_levels must CONFIRM stairs within 1.0 m and
  20 deg; otherwise that side becomes a wall and exploring goes on. Offline on the 24 Nov bags:
  mid2 confirmed in 1.0 s, near2 in 0.7 s, block obstacle rejected (0/12 frames).
- `explore_stairs.launch.py`: copy of explore.launch.py running it (stairs_check:=false = off).

    ros2 launch ~/stairs_detect/explore_stairs.launch.py drive:=false auto_start:=true

## stairs_crossing package (25 Nov, in ~/challenge_ws/src on the robot)

The 24 Nov copies above are replaced by the ROS package `stairs_crossing/` (this folder). It is
based on the team's code of 25 Nov, where `stair_levels.py` now lives in the team package
`stairs` and maze_explore already runs a depth stair check. Team packages are untouched.

- `explorer_stairs`: copy of maze_explore/explorer.py. The changes are listed at the top of the
  file: `object` sides that are stair-high get the depth check, the handrail check is off, an
  unconfirmed stair-high side is `blocked`, AUTO is sent after the AI gait (so /cmd_vel drives
  it), and the AI walk uses leg odometry and the IMU with /odom jumps ignored (Fast-LIO
  diverged on the steps in run7).
- `launch/explore_stairs.launch.py`: the team's explore.launch.py with that explorer, plus the
  arguments stair_rails, ai_auto, ai_speed and ai_wz.
- `stair_watch` (detector only; imports stair_levels from the `stairs` package) and `keyboard_teleop`.

Install:

    scp -r stairs_crossing user@192.168.10.74:~/challenge_ws/src/
    cd ~/challenge_ws && nice -n 19 colcon build --symlink-install --packages-select stairs_crossing --parallel-workers 1
    source install/setup.bash

Run:

    ros2 launch stairs_crossing explore_stairs.launch.py 2>&1 | tee ~/challenge_ws/logs/stairs_runN.log

The full lab procedure is in the Obsidian note `notes/stairs-crossing-lab-test-guide.md`.
The loose files in this folder (explorer_stairs.py, explore_stairs.launch.py, ai_auto_helper.py,
keyboard_teleop.py) are older stand-alone versions. Use the package.

### 25 Nov evening: working on the real stairs (run10)

AI walk = 1.88 m at 0.5 m/s, timed by speed x time (leg odometry read about half the real
distance in the AI gait), with a wall stop. Then back to H-Step + navigation, and exploring goes on.
`stairs_crossing/integration/maze_explore_stairs.patch` (+ README) puts all of this into the
team's maze_explore when its owner applies it.
