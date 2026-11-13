#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/scripts/env.sh" local
# The existing environment already contains the pinned vendor MuJoCo dependencies.
python=${MAZE_PYTHON:-$root/.venv/bin/python}
if [[ ! -x $python && -x /home/vicdeo/ir_challenge/.venv/bin/python ]]; then
  python=/home/vicdeo/ir_challenge/.venv/bin/python
fi
if [[ ! -x $python ]]; then echo 'Run scripts/setup_sim.sh first' >&2; exit 1; fi
model="$root/assets/maze_rough.xml"
rviz=1
args=()
while (( $# )); do
  case "$1" in
    --flat) model="$root/assets/maze_flat.xml"; shift ;;
    --no-rviz) rviz=0; shift ;;
    *) args+=("$1"); shift ;;
  esac
done
mkdir -p "$root/logs/current"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  for _ in {1..30}; do
    live=0
    for pid in "${pids[@]}"; do if kill -0 "$pid" 2>/dev/null; then live=1; fi; done
    if (( ! live )); then break; fi
    sleep .1
  done
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid "$python" -u "$root/scripts/mujoco_sim.py" --ros --controller --model "$model" "${args[@]}" > "$root/logs/current/physics.log" 2>&1 & pids+=($!)
setsid ros2 launch lite3_maze maze.launch.py simulation:=true perception_python:="$python" > "$root/logs/current/navigation.log" 2>&1 & pids+=($!)
if (( rviz )); then
  setsid "$root/scripts/run_rviz.sh" > "$root/logs/current/rviz.log" 2>&1 & pids+=($!)
fi
echo 'MuJoCo + maze program running. RViz shows the live map, path, footprint, state and visited cells.'
echo 'In another terminal run ./scripts/mode.sh auto to simulate lite3_auto.'
echo "Logs: $root/logs/current"
wait -n "${pids[@]}"
