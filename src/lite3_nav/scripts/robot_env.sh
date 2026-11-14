# Real-robot terminal setup. SOURCE it in a fresh terminal (do not execute):
#   source ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_env.sh
# Sets domain 4 + UDP-only Fast DDS for this terminal only and runs a read-only check
# (address, ping, topic list). It publishes nothing. Never start the simulator in this terminal.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "robot_env.sh must be sourced: source ${0}" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/challenge_ws/install/setup.bash

export ROS_DOMAIN_ID=4
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix lite3_nav)/share/lite3_nav/config/fastdds_udp.xml"
case "$PS1" in *"[ROBOT d4]"*) ;; *) PS1="\[\e[1;31m\][ROBOT d4]\[\e[0m\] $PS1" ;; esac

lite3_robot_quick_check() {
  local group_ip=${LITE3_GROUP_IP:-192.168.10.74} shared_ip=${LITE3_SHARED_IP:-192.168.10.62}
  echo "== ROBOT terminal: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION"
  echo "   FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
  local addr
  addr=$(ip -4 -o addr show 2>/dev/null | awk '{print $4}' | grep '^192\.168\.10\.' | head -1)
  if [ -n "$addr" ]; then echo "[ok]   own address on lab network: $addr"
  else echo "[FAIL] no 192.168.10.x address: join the robot/lab network first"; fi
  for ip in "$group_ip" "$shared_ip"; do
    if ping -c1 -W1 "$ip" >/dev/null 2>&1; then echo "[ok]   ping $ip"
    else echo "[FAIL] ping $ip"; fi
  done
  local topics expected found=0 missing=()
  topics=$(timeout 8 ros2 topic list --no-daemon 2>/dev/null)
  expected=(/imu/data /livox/points /scan /odom /tf /camera/color/image_raw/compressed /cmd_vel)
  for t in "${expected[@]}"; do
    if grep -qx -- "$t" <<<"$topics"; then found=$((found+1)); else missing+=("$t"); fi
  done
  echo "[info] $(wc -l <<<"${topics:-}" | tr -d ' ') topics visible; expected $found/${#expected[@]}"
  [ ${#missing[@]} -gt 0 ] && echo "[info] missing: ${missing[*]}"
  echo "Next (read-only): bash $(ros2 pkg prefix lite3_nav)/share/lite3_nav/scripts/robot_check.sh"
}
lite3_robot_quick_check
