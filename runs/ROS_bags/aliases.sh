# Rosbag recording aliases. Source from ~/.bashrc:
#   source ~/challenge_ws/runs/ROS_bags/aliases.sh
#
# Each takes an optional bag name, plus "-- topics..." to add topics, e.g.
#   rec_test run3
#   rec_manual demo2 -- /camera/depth/image_raw /camera/depth/camera_info
# After Ctrl+C you can type a new name (Enter keeps it). Rename later with rec_rename.
# Topics: `rec_list`, or ROSBAG_GUIDE.md section 2.
# The maze_nav-era aliases (rec_full, rec_maze, rec_navdebug, rec_lidar, rec_curtain) and
# their presets in record.sh were removed 2026-11-23.

_REC=~/challenge_ws/runs/ROS_bags/record.sh

alias rec_test="$_REC explore"           # real autonomous test run of maze_explore (explore.launch.py)
alias rec_manual="$_REC manual"          # hand-driven run with maze_explore mapping (explore:=false)
alias rec_list="$_REC --list"            # show presets and their topics
alias rec_rename="$_REC --rename"        # rec_rename [old] new  (old = newest bag if left out)

# Info for the newest bag
alias rec_last='ros2 bag info "$(ls -dt ~/challenge_ws/runs/ROS_bags/*/metadata.yaml | head -1 | xargs dirname)"'

# maze_explore: sensor-rate go / no-go, start and stop the explorer
alias explore_check=~/challenge_ws/src/maze_explore/preflight.sh
alias explore_go='ros2 topic pub --once /nav/start std_msgs/msg/Empty'
alias explore_stop='ros2 topic pub --once /explore/stop std_msgs/msg/Empty'
