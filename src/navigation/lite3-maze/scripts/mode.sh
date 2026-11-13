#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/scripts/env.sh" local
case "${1:-}" in
  auto) mode=lite3_auto ;;
  manual|stop) mode=lite3_manual ;;
  *) echo 'Usage: mode.sh {auto|manual}' >&2; exit 2 ;;
esac
exec ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: $mode}"
