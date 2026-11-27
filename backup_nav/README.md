# backup_nav: alternative exploration stack

A copy of the `src/` packages at commit `ff405d8` (maze_explore, scene_understanding,
stairs, stairs_crossing, task_reporter, team_perception), plus `maze_examples/sim` and
the `runs/ROS_bags` scripts. It is built as its own workspace. The main workspace skips
it (`COLCON_IGNORE`), so the package names do not clash.

## The one change from ff405d8

**Empty rooms are entered, and their sides are checked for stairs.** At the third view
of a look, a room that reads empty (`do_look_last`) is still marked as looked at.
Then the robot:

1. walks in until base_link is `room_enter` (0.5 m) past the edge it shares with the
   room. The obstacle stop stays on, and nothing becomes a wall if the walk ends short.
2. faces each of the room's two side edges in turn and runs the depth stair check
   there (`room_side_stairs`). It skips sides already checked and sides into cells it
   has visited.
3. stays in the room if stairs are found: the room counts as entered, `kind()` treats
   it as `free`, and the stairs are crossed from it later, last as usual. If no stairs
   are found, it turns round and walks back to the cell it looked from.

If the robot gets less than `room_enter_min` (0.35 m) into the room, it makes no turns
in there and comes straight back out. Set `room_enter:=0` to get plain ff405d8
behaviour. Both are node parameters of `maze_explorer`. Test:
`src/maze_explore/test/test_room_enter.py`.

## Curtain changes

- **Curtains are wider than 0.70 m** (`curtain_width_min`). `Sense.tall_width` measures
  the longest run of curtain-high lidar returns (0.65 m and up) along the whole edge, and
  bridges one-square gaps for the fabric's split. A lidar-only curtain needs to be wider
  than 0.70 m, or it is a wall. A camera confirmation (`do_cv_verify`) is also refused
  when the lidar shows the edge 0.70 m wide or less, and that includes an edge the lidar
  hasn't measured at that height (width 0). During the greet the edge stays a candidate;
  at the final check before the push it becomes a wall.
- **After the crossing: centre, then turn 180°** (`curtain_flip`, default true). The
  robot steps sideways through to the centre of the cell behind. It then centres with
  forward and sideways steps only (`shift`) and turns 180° as two quarter turns to the
  right. The fabric check (`curtain_clear`) follows. `/maze/blind` stays on throughout.
  `curtain_flip:=false` gives ff405d8 behaviour.

## Build and run

```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash
cd ~/challenge_ws/backup_nav
colcon build --symlink-install --base-paths src
source install/local_setup.bash      # last, so it overrides the main workspace
ros2 launch maze_explore explore.launch.py
```

`~/.bashrc` already sources `~/challenge_ws/install`. Both workspaces hold packages
with the same names, and the one sourced last wins. So source
`backup_nav/install/local_setup.bash` last in every terminal that should run the backup.
To check which copy is active, run `ros2 pkg prefix maze_explore`; it should print
`.../backup_nav/install/maze_explore`. Logs still go to `~/challenge_ws/logs`.

Unit tests (they import this copy's source, not the main workspace's build):

```bash
cd ~/challenge_ws/backup_nav
PYTHONPATH=$PWD/src/maze_explore:$PWD/src/stairs:$PYTHONPATH python3 -m pytest src/maze_explore/test -q
```
