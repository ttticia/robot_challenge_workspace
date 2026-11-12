#!/usr/bin/env bash
# Local tests only: explicit loopback DDS profile, isolated from robot domain 4.
set -eo pipefail
package_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
export PYTHONPATH="$package_root:$package_root/../team_perception${PYTHONPATH:+:$PYTHONPATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$package_root/test/cyclonedds-local.xml"
export ROS_DOMAIN_ID=89
export LITE3_ROS_TESTS=1
python3 -m pytest -q "$package_root/test" "$package_root/../team_perception/test/test_qr_utils.py" "$@"
