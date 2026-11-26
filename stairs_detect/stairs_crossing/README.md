# stairs_crossing

Rey's stairs crossing for the maze run. It runs the team's whole stack (`maze_explore`
explore.launch.py) with only the explorer swapped for `explorer_stairs`, a copy of
maze_explore's explorer (team code, 25 Nov). Its changes are listed at the top of
`stairs_crossing/explorer_stairs.py`. No other package is changed. Working on the real stairs
since 25 Nov (run10).

    ros2 launch stairs_crossing explore_stairs.launch.py [drive:=false auto_start:=true] + team arguments

Since 25 Nov evening the changes are merged into the team's maze_explore on the robot
(integration/apply_to_maze_explore.sh), so `ros2 launch maze_explore explore.launch.py` is the
competition launch and this one is a fallback with the same arguments.
    ros2 run stairs_crossing stair_watch       # detector only, never drives
    ros2 run stairs_crossing keyboard_teleop   # /cmd_vel teleop (lite3_auto first)

Stairs arguments (defaults):

| Argument | Default | What |
|---|---|---|
| ai_speed | 0.5 | m/s in the AI walk |
| ai_walk_cells | 1.5 | cells from the centre of the cell before to the border of the cell after |
| ai_body_clear | 0.35 | m past that border before AI mode ends (1.88 m in all) |
| ai_distance | time | `time` = speed x time; `leg` = leg odometry (reads about half in the AI gait) |
| ai_wall_min | 1.0 | m walked before a wall just ahead ends the walk |
| ai_wz | 0.0 | IMU heading hold, rad/s (0 = off, no IMU subscription) |
| ai_auto | true | send AUTO after the AI gait so /cmd_vel drives it |
| ai_reanchor | true | after the stairs, move the cell grid to where the walk put the robot, then a medium wall fit |
| reanchor_min | 0.15 | m of pose error before it re-anchors (smaller is left alone) |
| stair_check_time | 4.5 | s of depth-camera looking |
| stair_min_hits | 5 | the check also passes with this many hits over the look |
| stair_rails | false | handrails must also show in the lidar |

Needs the team packages maze_explore and stairs (stair_levels). Build:

    cd ~/challenge_ws && nice -n 19 colcon build --symlink-install --packages-select stairs_crossing --parallel-workers 1

Merging into maze_explore: `integration/README.md`. Lab procedure: Obsidian
`notes/stairs-crossing-lab-test-guide.md`. Run history: `notes/stairs-crossing-runs-2026-11-25.md`.
