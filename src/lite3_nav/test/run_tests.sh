#!/usr/bin/env bash
# lite3_nav tests only: explicit loopback DDS profile on domain 88, isolated from the sim (89) and robot (4).
# Uses the team's lite3_mission source next to this package (unchanged) for the shared helpers.
set -eo pipefail
package_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
export PYTHONPATH="$package_root:$package_root/../lite3_mission${PYTHONPATH:+:$PYTHONPATH}"
export CYCLONEDDS_URI="file://$package_root/test/cyclonedds-local.xml"
if [ -e /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
else
  # Cyclone DDS not installed (Rey's WSL): Fast DDS restricted to loopback instead.
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1
fi
# Domain 88: never the sim's 89 (a running sim must not see test leases) nor the robot's 4.
export ROS_DOMAIN_ID=88
export LITE3_ROS_TESTS=1
python3 -m pytest -q -p no:cacheprovider "$package_root/test" "$@"
