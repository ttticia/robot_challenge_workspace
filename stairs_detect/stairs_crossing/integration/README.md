# Merging the stairs crossing into maze_explore

`maze_explore_stairs.patch` holds every change of `stairs_crossing` (explorer_stairs.py and
explore_stairs.launch.py), written against the team's `maze_explore` as copied from the robot
on 2026-11-25 (~06:20 UTC). Once it is merged, the team's own launch crosses the stairs and
`stairs_crossing` is no longer needed. The owner of maze_explore decides whether to apply it.
Nothing is applied automatically.

Tested on the real stairs, 2026-11-25 (run10). The lidar called the side stairs/object, the
depth check confirmed it, AI mode was entered, the robot walked 1.88 m at 0.5 m/s and ended
in the cell after the stairs, then went back to H-Step + navigation mode and exploring went on.

## What it changes (explorer.py, all marked "stairs copy")

| # | Change | Why (run) |
|---|---|---|
| 1 | `object` sides that are stair-high (`raised()`) get the depth stair check, like `blocked` ones | our 3 x 12 cm stairs reach the 0.32-0.48 m band only on the top tread and mapped as `object` (runs 1-5) |
| 2 | a stair-high `free` side the depth check did not confirm becomes `blocked` | the robot climbed the stairs in H-Step (run7 attempt) |
| 3 | stair check 4.5 s (was 3.0), and it also passes with `stair_min_hits` 5 hits over the whole look | 6 of 28 frames, but never 3 of 5 in 1.2 s (run8) |
| 4 | `stair_rails` default false | the handrail check rejected the real stairs (23 Nov) |
| 5 | AUTO (0x21010C03) sent after AI_GAIT (`ai_auto`) | in manual mode the AI gait ignores /cmd_vel: the robot stood still (24 Nov); AUTO does not leave the AI state |
| 6 | new `do_ai_walk` for mode `ai`: straight at `ai_speed` 0.5 m/s for 1.5 cells + 0.35 m by speed x time, wall stop after 1.0 m, /odom jumps and the lidar pose ignored meanwhile, then the normal AI leave sequence always runs | 0.15 m/s with pose steering crept then thrashed (run6); Fast-LIO diverged on the steps -> FAULT (run7); leg odometry read ~half the distance -> into the wall, timeout left AI mode on (run9) |
| 7 | `reanchor` after the crossing (`ai_reanchor`): the cell grid is moved/turned so the believed pose sits where the walk put the robot (walked distance straight on from the cell before), then one medium wall fit (10 deg / 0.25 m, walls within 1.6 m) | Fast-LIO drifts on the steps, so after the crossing the robot was not where it thought it was (25 Nov) |

`ai_wz` (IMU heading hold) and `ai_distance:=leg` are optional and off. Their subscriptions are
only made when they are on.

## launch/explore.launch.py

New arguments, passed to maze_explorer: stair_rails, ai_auto, ai_speed, ai_wz, ai_walk_cells,
ai_body_clear, ai_distance, ai_wall_min, ai_reanchor, reanchor_min, stair_min_hits, stair_check_time.

## Apply

    cd ~/challenge_ws/src
    git apply --check ~/challenge_ws/src/stairs_crossing/integration/maze_explore_stairs.patch
    git apply ~/challenge_ws/src/stairs_crossing/integration/maze_explore_stairs.patch
    cd ~/challenge_ws && nice -n 19 colcon build --symlink-install --packages-select maze_explore --parallel-workers 1

`--check` fails if maze_explore changed since the copy. Then merge by hand from
`stairs_crossing/stairs_crossing/explorer_stairs.py`: search for "stairs copy".
The paths in the patch are `maze_explore/...`, relative to `~/challenge_ws/src`. If src is not a
git repo, use `patch -p1 --dry-run < ...` and then `patch -p1 < ...`.

## Apply on the robot (one command)

**Status: applied on the robot on 25 Nov (team version of 18:35). Backup in `~/maze_explore_backup/`.**

`apply_to_maze_explore.sh` does it all: it checks the patch fits (dry run), backs up the two team
files to `~/maze_explore_backup/` (outside challenge_ws, so colcon never sees two copies), patches,
rebuilds maze_explore and shows the stairs arguments. Running it twice does nothing the second time.

```
bash ~/challenge_ws/src/stairs_crossing/integration/apply_to_maze_explore.sh
```

Undo (team files back as they were):

```
bash ~/challenge_ws/src/stairs_crossing/integration/apply_to_maze_explore.sh --undo
```

If the dry run fails, the team files changed since 25 Nov 18:35: nothing is touched; send the two files
so the patch can be redone.
