#!/usr/bin/env bash
# Local tests only: explicit loopback DDS profile, isolated from robot domain 4.
set -eo pipefail
package_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
export PYTHONPATH="$package_root:$package_root/../team_perception${PYTHONPATH:+:$PYTHONPATH}"
export RMW_IMPLEMENTATION="${LITE3_TEST_RMW:-rmw_fastrtps_cpp}"
if [ "$RMW_IMPLEMENTATION" = rmw_fastrtps_cpp ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$package_root/test/fastdds-local.xml"
elif [ "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp ]; then
    export CYCLONEDDS_URI="file://$package_root/test/cyclonedds-local.xml"
else
    echo "Unsupported test middleware: $RMW_IMPLEMENTATION" >&2
    exit 1
fi
export ROS_DOMAIN_ID=89
export LITE3_ROS_TESTS=1
python3 -m pytest -q "$package_root/test" "$package_root/../team_perception/test/test_qr_utils.py" "$@"
