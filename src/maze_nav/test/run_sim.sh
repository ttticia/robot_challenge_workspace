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
# Same UDP-only profile as the robot, for the sim too: with Fast DDS defaults (shared memory +
# UDP, small buffers) only ~3 of the sim's 10 clouds/s arrived and cmd_vel_bridge's sensor
# watchdog tripped on the first motion command.
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/maze_nav/config/fastdds_udp.xml"
mkdir -p "$LOG" && rm -rf "$LOG/maps" "$LOG"/*.log

# Each in its own process group (setsid), so cleanup can signal every node, even ones whose
# ros2 launch parent has already exited.
setsid python3 -W ignore "$WS/src/maze_nav/test/fake_maze_lite3.py" --ros-args -p track_csv:="$LOG/track.csv" > "$LOG/sim.log" 2>&1 &
SIM=$!
sleep 3
# floor_z: the sim's walls stand on the start cell's floor, 0.35 m (STAND) below base_link at
# the start. Measured, the floor came out as the lab floor OUTSIDE the maze (-0.45: the maze
# sits on a 0.1 m plate and sim walls are see-through), which made every wall look raised 10 cm
# and set off curtain inspections (2026-11-21).
setsid ros2 launch maze_nav maze_nav.launch.py bringup:=false udp_only:=true floor_z:=-0.35 \
    map_dir:="$LOG/maps" wait_for_start:=false perception:=false > "$LOG/launch.log" 2>&1 &
LAUNCH=$!

end=$((SECONDS + TIMEOUT))
until grep -q -E "\[(EXITED|NO_EXIT|STOPPED)\]|process has died" "$LOG/launch.log" \
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
grep -q "\[EXITED\]" "$LOG/launch.log"
