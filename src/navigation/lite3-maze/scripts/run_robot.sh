#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! ${1:-} =~ ^[0-9]+$ ]]; then echo 'Usage: run_robot.sh DOMAIN_ID [launch args]' >&2; exit 2; fi
domain=$1
shift
lock_dir="${XDG_CACHE_HOME:-$HOME/.cache}/lite3-maze"
mkdir -p "$lock_dir"
exec 9>"$lock_dir/robot-domain-$domain.lock"
if ! flock -n 9; then
  echo "Maze navigation is already running on domain $domain. Stop that launch before starting another." >&2
  exit 1
fi
source "$root/scripts/env.sh" robot "$domain"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/assets/fastdds-udp.xml"
# Keep inference and numerical libraries from exhausting the robot's CPU cores.
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
python=${MAZE_PYTHON:-$root/.venv/bin/python}
if [[ ! -x $python && -x /home/vicdeo/ir_challenge/.venv/bin/python ]]; then python=/home/vicdeo/ir_challenge/.venv/bin/python; fi
if [[ ! -x $python ]]; then python=/usr/bin/python3; fi
echo 'Waiting for lite3_auto. Starting the course SLAM includes; stack:=false uses an existing stack.'
exec ros2 launch lite3_maze maze.launch.py stack:=true perception_python:="$python" "$@"
