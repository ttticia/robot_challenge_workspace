#!/usr/bin/env bash
# Read-only graph and interface information. Run in the already-sourced robot shell.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/scripts/env.sh" robot "${1:?Usage: inspect_robot.sh DOMAIN_ID}"
ros2 topic list -t --no-daemon --spin-time 2
ros2 topic info /simple_cmd -v
ros2 interface show transfer_interfaces/msg/MotionSimpleCMD
ros2 topic info /cmd_vel -v
ros2 topic info /map -v
