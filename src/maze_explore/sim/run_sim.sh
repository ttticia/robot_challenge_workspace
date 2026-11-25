#!/bin/bash
# Run the whole stack against the simulated Lite3 and wait for the outcome.
#
#   src/maze_explore/sim/run_sim.sh [maze=challenge_a.json] [timeout_s=900] [log_dir] [domain=89]
#
# maze: a file in maze_examples/sim (or a path). Uses localhost only on a spare ROS domain, so
# it can never reach the robot; the domain must be empty. Exit 0 = the explorer reported DONE.
# Logs: <log_dir>/launch.log, sim.log, status.log, events.log.
MAZE=${1:-challenge_a.json}
TIMEOUT=${2:-900}
LOG=${3:-/tmp/maze_explore_sim}
DOMAIN=${4:-89}
PKG=$(cd "$(dirname "$0")/.." && pwd)
WS=$PKG
while [ "$WS" != / ] && [ ! -f "$WS/install/setup.bash" ]; do WS=$(dirname "$WS"); done
[ -f "$MAZE" ] || MAZE="$WS/maze_examples/sim/$MAZE"
[ -f "$MAZE" ] || { echo "no maze file: $MAZE"; exit 2; }
source /opt/ros/humble/setup.bash
source "$HOME/lite3_ws/install/local_setup.bash"
source "$WS/install/local_setup.bash"
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PKG/config/fastdds_udp.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
# The robot's own stack runs on this computer, and a simulator here takes the CPU it needs
# (2026-11-25 07:26: a sim running through robot bag run5 left the camera at 12.6 Hz and the
# lidar at 8.7 Hz, against 29.7 / 10 in run4). The sim never runs Fast-LIO (fastlio:=false),
# so a fastlio_mapping process means a robot launch: do not start, and stop if one appears.
robot_stack() { pgrep -x fastlio_mapping >/dev/null; }
if robot_stack; then
    echo "A robot launch is running (fastlio_mapping): not starting the simulator. Run it when the robot is done."
    exit 4
fi
if [ -n "$(timeout 15 ros2 node list --no-daemon 2>/dev/null)" ]; then
    echo "ROS_DOMAIN_ID=$DOMAIN is busy: use another domain"; exit 3
fi
mkdir -p "$LOG" && rm -f "$LOG"/*.log
shift 4 2>/dev/null
nice -n 19 python3 "$PKG/sim/fake_maze_lite3.py" --ros-args -p maze:="$(realpath "$MAZE")" -p depth_rate:=0.0 > "$LOG/sim.log" 2>&1 &
SIM=$!
sleep 3
nice -n 19 ros2 launch maze_explore explore.launch.py fastlio:=false cloud_topic:=/livox/points \
    auto_start:=true stand_height:=0.35 "$@" > "$LOG/launch.log" 2>&1 &
LAUNCH=$!
ros2 topic echo /explore/event std_msgs/msg/String --field data > "$LOG/events.log" 2>&1 &
EV=$!
cleanup() {   # the launch's nodes are its children: stop them by parent before the launch itself
    kill -INT $LAUNCH 2>/dev/null; kill $EV $SIM 2>/dev/null
    for i in 1 2 3 4 5 6 7 8; do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done
    pkill -9 -P $LAUNCH 2>/dev/null; kill -9 $LAUNCH $SIM 2>/dev/null
}
trap cleanup EXIT
END=$((SECONDS + TIMEOUT))
RESULT=timeout
while [ $SECONDS -lt $END ]; do
    sleep 5
    kill -0 $LAUNCH 2>/dev/null || { RESULT="launch died"; break; }
    robot_stack && { RESULT="stopped: a robot launch started (fastlio_mapping)"; break; }
    S=$(grep -aoE "\]:? (DONE|FAULT|STOPPED): .*" "$LOG/launch.log" | tail -1)
    [ -n "$S" ] && { RESULT="$S"; break; }
done
echo "result after ${SECONDS}s: $RESULT"
grep -a "OUT OF THE MAZE" "$LOG/sim.log" | tail -1
[[ "$RESULT" == *DONE* ]]
