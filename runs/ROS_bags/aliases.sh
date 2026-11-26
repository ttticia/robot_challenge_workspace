# Rosbag recording aliases. Source from ~/.bashrc:
#   source ~/challenge_ws/runs/ROS_bags/aliases.sh
#
# Each rec_* takes an optional bag name, plus "-- topics..." to add topics, e.g.
#   rec_test run3
#   rec_test run3 -- /qr/read
# After Ctrl+C you can type a new name (Enter keeps it). Rename later with rec_rename.
# Topics: `rec_list`, or ROSBAG_GUIDE.md section 2.

_REC=~/challenge_ws/runs/ROS_bags/record.sh

alias rec_test="$_REC explore"           # autonomous run of maze_explore (explore.launch.py): local topics only, no camera, no IMU
alias rec_lean="$_REC explore"           # the same recorder as rec_test (one preset since 2026-11-26)
alias rec_manual="$_REC manual"          # hand-driven run with maze_explore mapping (explore:=false)
alias rec_curtain="$_REC curtain"        # curtain detection test by hand (curtain_node running, always:=true)
alias rec_img="$_REC img"                # camera only: /camera/color/image_raw/compressed
alias rec_list="$_REC --list"            # show presets and their topics
alias rec_rename="$_REC --rename"        # rec_rename [old] new  (old = newest bag if left out)

# Info for the newest bag
alias rec_last='ros2 bag info "$(ls -dt ~/challenge_ws/runs/ROS_bags/*/metadata.yaml | head -1 | xargs dirname)"'

# Curtain detection by hand: the node on every frame, and its picture with the boxes
alias curtain_watch='ros2 run maze_explore curtain_node --ros-args -p always:=true'
alias curtain_view='ros2 run rqt_image_view rqt_image_view /curtain/image'

# maze_explore: sensor-rate go / no-go, start and stop the explorer
alias explore_check=~/challenge_ws/src/maze_explore/preflight.sh
alias explore_go='ros2 topic pub --once /nav/start std_msgs/msg/Empty'
alias explore_stop='ros2 topic pub --once /explore/stop std_msgs/msg/Empty'
