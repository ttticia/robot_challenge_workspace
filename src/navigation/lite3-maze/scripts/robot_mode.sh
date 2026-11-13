#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-}" in
  auto|manual) mode=$1 ;;
  *) echo 'Usage: robot_mode.sh {auto|manual} [DOMAIN_ID]' >&2; exit 2 ;;
esac
source "$root/scripts/env.sh" robot "${2:-4}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/assets/fastdds-udp.xml"
exec /usr/bin/python3 "$root/scripts/robot_mode.py" "$mode"
