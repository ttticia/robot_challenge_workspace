#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == robot ]]; then
  domain=${2:?Supply the hardware ROS domain}
  shift 2
  source "$root/scripts/env.sh" robot "$domain"
  frame=map
  sim=false
  config=maze_robot.rviz
else
  source "$root/scripts/env.sh" local
  frame=odom
  sim=true
  config=maze.rviz
fi
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export QT_IM_MODULE=compose
export XMODIFIERS=@im=none
exec rviz2 -d "$root/assets/$config" -f "$frame" --ros-args -p use_sim_time:="$sim" "$@"
