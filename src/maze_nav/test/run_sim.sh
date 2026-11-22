#!/bin/bash
# Run maze_nav against a simulated maze, wait for the outcome, then stop everything.
#
#   src/maze_nav/test/run_sim.sh [timeout_s=600] [log_dir=/tmp/maze_sim] [maze=example_maze.json] [domain=89]
#
# maze: a JSON file for fake_maze_lite3.py (a bare name is looked up in src/maze_nav/test/).
# Its "grid" key (cols, rows, cell_size, optional launch_cell) sets the explorer's maze grid;
# without one (example_maze.json, object_maze.json: their exit is the stairs, not a challenge
# exit) maze_cols:=0, the old walk-out exit. Uses localhost only, so it can never reach the robot; pick another domain
# when another session's sim is running (`ROS_DOMAIN_ID=<d> ros2 node list` must be empty).
# Exit 0: with a launch_cell in the maze file, the explorer parked there (AT_EXIT) and the sim
# never saw the robot out of the maze; without one, EXITED or AT_EXIT.
# Logs: <log_dir>/launch.log, sim.log; map in <log_dir>/maps.
TIMEOUT=${1:-600}
LOG=${2:-/tmp/maze_sim}
MAZE=${3:-example_maze.json}
DOMAIN=${4:-89}
PKG=$(cd "$(dirname "$0")/.." && pwd)
# The colcon workspace: the nearest folder above with install/ (~/challenge_ws on the robot;
# the Robot-Dog-Grp4 checkout nests the packages deeper).
WS=$PKG
while [ "$WS" != / ] && [ ! -f "$WS/install/setup.bash" ]; do WS=$(dirname "$WS"); done
[ -f "$WS/install/setup.bash" ] || { echo "no install/setup.bash above $PKG: colcon build first"; exit 2; }
[ -f "$MAZE" ] || MAZE="$PKG/test/$MAZE"
[ -f "$MAZE" ] || { echo "no maze file: $MAZE"; exit 2; }
MAZE=$(realpath "$MAZE")
read -r COLS ROWS CELL LAUNCH_CELL < <(python3 -c '
import json, sys
g = json.load(open(sys.argv[1])).get("grid", {"cols": 0, "rows": 0, "cell_size": 1.0})
c = g.get("launch_cell")
print(g["cols"], g["rows"], g["cell_size"], f"[{c[0]},{c[1]}]" if c else "-")' "$MAZE")
LAUNCH_CELL=${LAUNCH_CELL//,/, }
echo "maze $(basename "$MAZE"): grid ${COLS} x ${ROWS} of ${CELL} m, expected launch cell ${LAUNCH_CELL}, domain ${DOMAIN}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u                                   # after sourcing: ROS setup uses unset vars
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
# Same UDP-only profile as the robot, for the sim too: with Fast DDS defaults (shared memory +
# UDP, small buffers) only ~3 of the sim's 10 clouds/s arrived and cmd_vel_bridge's sensor
# watchdog tripped on the first motion command.
export FASTRTPS_DEFAULT_PROFILES_FILE="$PKG/config/fastdds_udp.xml"
mkdir -p "$LOG" && rm -rf "$LOG/maps" "$LOG"/*.log

# Each in its own process group (setsid), so cleanup can signal every node, even ones whose
# ros2 launch parent has already exited.
setsid python3 -W ignore "$PKG/test/fake_maze_lite3.py" --ros-args -p maze:="$MAZE" \
    -p track_csv:="$LOG/track.csv" > "$LOG/sim.log" 2>&1 &
SIM=$!
sleep 3
# floor_z: the sim's walls stand on the start cell's floor, 0.35 m (STAND) below base_link at
# the start. Measured, the floor came out as the lab floor OUTSIDE the maze (-0.45: the maze
# sits on a 0.1 m plate and sim walls are see-through), which made every wall look raised 10 cm
# and set off curtain inspections (2026-11-21).
setsid ros2 launch maze_nav maze_nav.launch.py bringup:=false udp_only:=true floor_z:=-0.35 \
    map_dir:="$LOG/maps" wait_for_start:=false perception:=false \
    maze_cols:="$COLS" maze_rows:="$ROWS" cell_size:="$CELL" > "$LOG/launch.log" 2>&1 &
LAUNCH=$!

end=$((SECONDS + TIMEOUT))
until grep -q -E "\[(AT_EXIT|EXITED|NO_EXIT|STOPPED)\]|process has died" "$LOG/launch.log" \
        || [ $SECONDS -ge $end ]; do
    sleep 2
done

# SIGTERM to the whole groups (background jobs of a script ignore SIGINT). Killing only the
# ros2 launch PID used to leave its nodes running on domain 89 (wall_scan alone ~150% CPU).
# setsid makes the job its own group leader, so its PID is the group ID.
kill -TERM -- -$LAUNCH -$SIM 2>/dev/null
for _ in $(seq 15); do
    pgrep -g $LAUNCH,$SIM > /dev/null || break
    sleep 1
done
kill -KILL -- -$LAUNCH -$SIM 2>/dev/null
wait $LAUNCH $SIM 2>/dev/null
if pgrep -g $LAUNCH,$SIM > /dev/null; then
    echo "WARNING: sim processes still running:"; pgrep -a -g $LAUNCH,$SIM
fi

grep -E "floor|\[maze_explorer\]: \[|frontier goal|failed|process has died|Traceback" \
    "$LOG/launch.log" | cut -c1-170
grep "OUT OF" "$LOG/sim.log"
if [ "$LAUNCH_CELL" = "-" ]; then
    grep -q -E "\[(AT_EXIT|EXITED)\]" "$LOG/launch.log"
    exit
fi
grep -F "[AT_EXIT]" "$LOG/launch.log" | head -1 | cut -c1-200
grep -qF "in launch cell $LAUNCH_CELL" "$LOG/launch.log" || { echo "FAIL: not parked in launch cell $LAUNCH_CELL"; exit 1; }
if grep -q "OUT OF THE MAZE" "$LOG/sim.log"; then echo "FAIL: the robot left the maze"; exit 1; fi
