# stairs_crossing

Rey's stairs crossing for the maze run. It runs the team's whole stack (`maze_explore`
explore.launch.py) with only the explorer swapped for `explorer_stairs`, a copy of
maze_explore's explorer (team code, 25 Nov) whose changes are listed at the top of
`stairs_crossing/explorer_stairs.py`. No other package is changed.

    ros2 launch stairs_crossing explore_stairs.launch.py [drive:=false auto_start:=true]
        [ai_speed:=0.25] [ai_wz:=0.15] [ai_auto:=true] [stair_rails:=false]   # + all team arguments
    ros2 run stairs_crossing stair_watch       # detector only, never drives
    ros2 run stairs_crossing keyboard_teleop   # /cmd_vel teleop (lite3_auto first)

Needs the team packages maze_explore and stairs (stair_levels). Build:

    cd ~/challenge_ws && nice -n 19 colcon build --symlink-install --packages-select stairs_crossing --parallel-workers 1

Lab procedure: Obsidian `notes/stairs-crossing-lab-test-guide.md`.
