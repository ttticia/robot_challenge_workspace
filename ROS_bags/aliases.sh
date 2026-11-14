# Rosbag recording aliases. Source from ~/.bashrc:
#   source ~/challenge_ws/ROS_bags/aliases.sh
#
# Each takes an optional bag name, plus "-- topics..." to add topics, e.g.
#   rec_nav maze_try3
#   rec_full -- /camera/aligned_depth_to_color/image_raw
# After Ctrl+C you can type a new name (Enter keeps it). Rename later with rec_rename.
# Presets and topics: ROSBAG_GUIDE.md section 2, or `rec_list`.

_REC=~/challenge_ws/ROS_bags/record.sh

alias rec_full="$_REC full"              # teleop, everything incl. depth
alias rec_nav="$_REC nav"                # Nav2 driving, no depth
alias rec_perception="$_REC perception"  # QR / poster replay, incl. depth
alias rec_detections="$_REC detections"  # perception results only, small
alias rec_navdebug="$_REC navdebug"      # lidar, localization, velocity chain
alias rec_lidar="$_REC lidar"            # raw sensors for Fast-LIO / SLAM
alias rec_video="$_REC video"            # compressed colour camera only
alias rec_list="$_REC --list"            # show presets and their topics
alias rec_rename="$_REC --rename"        # rec_rename [old] new  (old = newest bag if left out)

# Info for the newest bag
alias rec_last='ros2 bag info "$(ls -dt ~/challenge_ws/ROS_bags/*/metadata.yaml | head -1 | xargs dirname)"'
