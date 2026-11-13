# Source with 'local' for isolated simulation, or 'robot DOMAIN' for hardware.
_maze_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
if [[ -f $_maze_root/install/setup.bash ]]; then source "$_maze_root/install/setup.bash"; fi
if [[ ${1:-local} == local ]]; then
  export ROS_DOMAIN_ID=${MAZE_SIM_DOMAIN:-31}
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_LOCALHOST_ONLY=0
  export CYCLONEDDS_URI="file://$_maze_root/assets/cyclonedds-local.xml"
else
  if [[ ! ${2:-} =~ ^[0-9]+$ ]] || (( 10#${2:-0} > 232 )); then
    echo 'Usage: source scripts/env.sh robot DOMAIN_ID' >&2
    return 2
  fi
  export ROS_DOMAIN_ID=$2
  export ROS_LOCALHOST_ONLY=0
  unset CYCLONEDDS_URI
fi
