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
