# Skills: recipes for recurring jobs

Step-by-step recipes for jobs that come up again and again in this workspace. `agent.md` explains
the why; this file is the how. Paths are relative to the workspace root (`~/challenge_ws` on
the robot; `challenge_files/` in the Robot-Dog-Grp4 mirror).

When you work out a new recipe or find one here is wrong, fix it here in the same change.

---

## 1. Build after a change

```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash && source ~/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select maze_nav team_perception
source install/setup.bash
```

- On the robot computer `install/` is a copy, not symlinks. Rebuild after changing any code,
  config or launch file, or the next run uses the old version.
- A new executable must also go in `setup.py` `entry_points` (see `odom_guard.py`: written,
  not an entry point, so `ros2 run` can't find it).

## 2. Unit tests

```bash
(cd src/maze_nav && python3 -m pytest -q test)
(cd src/team_perception && python3 -m pytest -q test)
```

Pure logic lives in ROS-free modules (`core.py`, `stairs.py`, `curtains.py`, `stair_check.py`,
`curtain_branch.py`, `scene_cells.py`, `*_utils.py`). Put new logic there and test it there;
the nodes stay thin.

## 3. Simulator run

1. Pick a free domain: `ROS_DOMAIN_ID=89 ros2 node list` must be empty. If it isn't, another
   session has a run going: queue behind it, or use another domain (4th argument).
2. Build (recipe 1), then:
   ```bash
   src/maze_nav/test/run_sim.sh 600 /tmp/maze_sim_<you>                              # example maze
   src/maze_nav/test/run_sim.sh 600 /tmp/maze_sim_<you> challenge_exit_left.json 117  # any maze, domain 117
   ```
   Mazes and what each tests: agent.md §9. Before a robot run, run all five `challenge_*`
   mazes.
3. Exit 0 = parked in the maze's launch cell (`AT_EXIT`) without leaving the maze; mazes
   without a launch cell pass on `AT_EXIT` or `EXITED`. Read
   `<log_dir>/launch.log` (explorer states, failed goals, deaths) and `<log_dir>/sim.log`
   (`OUT OF THE MAZE`, wall clearance). Map in `<log_dir>/maps`, track in `track.csv`.
4. Never `pkill -f <node>`: it kills other sessions' runs. `run_sim.sh` cleans up its own
   process groups; if it leaves something, kill only those PIDs.

## 4. Simulator by hand (to watch it in RViz or change sim parameters)

```bash
export ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1        # any unused domain; check `ros2 node list`
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/maze_nav/config/fastdds_udp.xml
python3 src/maze_nav/test/fake_maze_lite3.py --ros-args \
    -p maze:=$PWD/src/maze_nav/test/challenge_exit_far.json -p track_csv:=/tmp/track.csv &
ros2 launch maze_nav maze_nav.launch.py bringup:=false udp_only:=true floor_z:=-0.35 \
    wait_for_start:=false perception:=false map_dir:=/tmp/maze_maps \
    maze_cols:=4 maze_rows:=3 cell_size:=1.02          # the maze file's "grid"
```

Sim parameters: `depth_rate` (5 Hz, 0 = no depth camera), `depth_width` (640; keep it, the stair
detector's strides are in pixels), `occlusion` (auto = on with curtains or extru walls),
`yaw_drift`. Say which domain you are using in your session notes.

## 5. Regenerate test mazes

```bash
cd src/maze_nav
python3 test/make_challenge_mazes.py                                 # challenge_*.json
python3 test/make_presentation_mazes.py                              # maze_curtains.json + maze_stairs.json
python3 tools/step_to_boxes.py "tools/example maze.step" > test/example_maze.json
python3 -m pytest -q test/test_challenge_mazes.py test/test_presentation_mazes.py
```

Fix mazes in their scripts, never in the JSON (`test_files_match_the_generator` checks).
The presentation-maze heights and the challenge mazes' interior walls are made up; change
`PASSAGES` / `EXITS` in `make_challenge_mazes.py` once the real layout is known.

## 6. Real robot run

Before:
1. `ros2 node list` on domain 4: no other SLAM, Nav2, `maze_nav` or `/cmd_vel` publisher
   (`ros2 topic info -v /cmd_vel`). The launch's preflight warns too, but check first.
2. Robot on the start cell, square to it, standing or lying. Someone holds the handheld
   controller for the whole run.
3. Rebuild if anything changed (recipe 1).

Run:
```bash
ros2 launch maze_nav maze_nav.launch.py          # add switches as needed (agent.md §3)
# wait for "gait sent" in the log, then:
lite3_auto
# wait for /maze/status = PAUSED, then start recording and go:
rec_maze
ros2 topic pub --once /maze/start std_msgs/msg/Empty
```

During: watch `/maze/status`. `AT_STAIRS` and `AT_CURTAIN` hold until another `/maze/start`.
After: Ctrl+C the launch, `lite3_manual` (or `lite3_ai_off` if it ended in AI mode), stop
`rec_maze`, and read `~/challenge_ws/logs/cpu/run_<time>_summary.txt`. Note anything new in
`src/to_do.md`.

## 7. Record and inspect bags

```bash
rec_list                     # presets and their topics
rec_maze                     # maze_nav run, no depth (never record raw depth while Nav2 drives)
rec_full <name>              # teleop only: everything incl. depth, ~1.6 GB/min (check df -h)
rec_last                     # ros2 bag info of the newest bag
rec_rename [old] new
python3 ~/bag_to_video.py ~/challenge_ws/ROS_bags/<name>
```

The presets miss `/stairs/*`, `/scene/*`, `/qr/read`, `/qr/unread`, `/maze/perceive` and
`/perception/look_request`. Add the ones you need after `--` (`ROS_bags/ROSBAG_GUIDE.md` §1).
Bags are git-ignored; copy them off the robot.

## 8. Test the curtain detector on a bag

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
nice -n 19 python3 src/maze_nav/tools/curtain_bag_replay.py ROS_bags/<bag> \
    --max-gap 0.15 0.25 --every 5 --lidar --csv /tmp/curtain.csv
nice -n 19 python3 src/maze_nav/tools/curtain_yolo_frames.py ROS_bags/<bag> \
    ~/challenge_ws/datasets/curtain --every 3 --max-gap 0.25 --lidar   # draft YOLO labels
```

Needs no `/tf`, so teleop bags without Fast-LIO work.

## 9. Add an optional feature to maze_nav

Every optional part follows the same rule (agent.md §3), so a dead or switched-off part can't
stop navigation:
1. A launch switch (`DeclareLaunchArgument`, documented in the launch file's docstring).
2. The explorer topic parameter it feeds is blanked when the switch is off.
3. A `DEATH_HINTS` entry in `maze_nav.launch.py` saying what is lost if it dies.
4. Test every on/off combination by launching it.

Also, before adding a node: read agent.md §12 *CPU budget*. Take lidar from
`/livox/points_crop` or `_local` and depth from `/camera/depth_relay/*`, never the raw bridge
topics. Use `tf_pin` (or `camera_link`) for camera TF, and set `OPENBLAS_NUM_THREADS=1` in
numpy-heavy processes.

## 10. Refresh the Robot-Dog-Grp4 mirror (`challenge_files/`)

`challenge_files/` must equal the team repo's `navigation` branch, plus `src/bt_diagram.md` and
`SUTD_Legged_Robot_Challenge_Booklet_2026_v5.md`. From the Robot-Dog-Grp4 checkout:

```bash
git remote add workspace https://github.com/ttticia/robot_challenge_workspace.git  # once
git fetch workspace navigation
K=$(mktemp -d)
cp challenge_files/src/bt_diagram.md challenge_files/SUTD_Legged_Robot_Challenge_Booklet_2026_v5.md "$K"/
git rm -rq challenge_files && rm -rf challenge_files && mkdir challenge_files
git archive workspace/navigation | tar -x -C challenge_files
cp "$K"/bt_diagram.md challenge_files/src/ && cp "$K"/SUTD_*.md challenge_files/
git add challenge_files
# verify: only the two extra files may differ
diff <(git ls-tree -r workspace/navigation | awk '{print $3, $4}') \
     <(git ls-files -s challenge_files | awk '{print $2, $4}' | sed 's| challenge_files/| |' \
       | grep -v -e ' src/bt_diagram.md$' -e ' SUTD_Legged') && echo IDENTICAL
```

Don't `git merge` the navigation branch into Robot-Dog-Grp4. The histories are unrelated, and
a merge puts a second copy at the repo root, which also breaks `colcon build` with duplicate
packages. Don't `rsync dir/` either: the trailing slash flattens the layout.

If you changed `agent.md` or `skills.md` here, copy them to the team repo too, or the next
refresh overwrites them.
