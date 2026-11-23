# Rosbag recording aliases. Source from ~/.bashrc:
#   source ~/challenge_ws/runs/ROS_bags/aliases.sh
#
# Each takes an optional bag name, plus "-- topics..." to add topics, e.g.
#   rec_maze maze_try3
#   rec_full -- /camera/aligned_depth_to_color/image_raw
# After Ctrl+C you can type a new name (Enter keeps it). Rename later with rec_rename.
# Presets and topics: ROSBAG_GUIDE.md section 2, or `rec_list`.

_REC=~/challenge_ws/runs/ROS_bags/record.sh

alias rec_full="$_REC full"              # teleop, everything incl. depth
alias rec_maze="$_REC maze"              # full maze_nav run: nav + perception + explorer, no depth
alias rec_navdebug="$_REC navdebug"      # lidar, localization, velocity chain
alias rec_lidar="$_REC lidar"            # raw sensors for Fast-LIO / SLAM
alias rec_curtain="$_REC curtain"        # curtain YOLO training bags (teleop), incl. depth
alias rec_list="$_REC --list"            # show presets and their topics
alias rec_rename="$_REC --rename"        # rec_rename [old] new  (old = newest bag if left out)

# Info for the newest bag
alias rec_last='ros2 bag info "$(ls -dt ~/challenge_ws/runs/ROS_bags/*/metadata.yaml | head -1 | xargs dirname)"'
