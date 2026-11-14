#!/bin/bash
# Run maze_nav against the simulated example maze, wait for the outcome, then stop everything.
#
#   src/maze_nav/test/run_sim.sh [timeout_s=600] [log_dir=/tmp/maze_sim]
#
# Uses ROS_DOMAIN_ID=89 + localhost only, so it can never reach the robot.
# Exit 0 = explorer reported EXITED. Logs: <log_dir>/launch.log, sim.log; map in <log_dir>/maps.
TIMEOUT=${1:-600}
LOG=${2:-/tmp/maze_sim}
WS=$(cd "$(dirname "$0")/../../.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u                                   # after sourcing: ROS setup uses unset vars
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
unset FASTRTPS_DEFAULT_PROFILES_FILE
mkdir -p "$LOG" && rm -rf "$LOG/maps" "$LOG"/*.log

python3 -W ignore "$WS/src/maze_nav/test/fake_maze_lite3.py" > "$LOG/sim.log" 2>&1 &
SIM=$!
sleep 3
ros2 launch maze_nav maze_nav.launch.py bringup:=false udp_only:=false \
    map_dir:="$LOG/maps" start_pause:=0 > "$LOG/launch.log" 2>&1 &
LAUNCH=$!

end=$((SECONDS + TIMEOUT))
until grep -q -E "\[(EXITED|NO_EXIT|STOPPED)\]|process has died" "$LOG/launch.log" \
        || [ $SECONDS -ge $end ]; do
    sleep 2
done

# SIGTERM: background jobs of a script ignore SIGINT, so ros2 launch would never stop.
kill -TERM $LAUNCH $SIM 2>/dev/null
for _ in $(seq 15); do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done
kill -KILL $LAUNCH 2>/dev/null
wait $LAUNCH $SIM 2>/dev/null

grep -E "floor|\[maze_explorer\]: \[|frontier goal|failed|process has died|Traceback" \
    "$LOG/launch.log" | cut -c1-170
grep "OUT OF" "$LOG/sim.log"
grep -q "\[EXITED\]" "$LOG/launch.log"
