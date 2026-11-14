#!/usr/bin/env bash
# Read-only robot data check: topic rates and /cmd_vel endpoints. Publishes nothing.
# Run in a terminal where robot_env.sh was sourced (domain 4), e.g.
#   bash ~/challenge_ws/install/lite3_nav/share/lite3_nav/scripts/robot_check.sh
if [ "${ROS_DOMAIN_ID:-}" != "4" ]; then
  echo "ROS_DOMAIN_ID is '${ROS_DOMAIN_ID:-unset}', not 4: source robot_env.sh first" >&2
  exit 1
fi
for topic in /imu/data /livox/points /odom /scan; do
  echo "== ros2 topic hz $topic (5 s)"
  out=$(timeout -s INT 5 ros2 topic hz "$topic" 2>&1 | grep -E 'average rate|min:|does not appear' | tail -2)
  echo "${out:-   no messages within 5 s}"
done
echo "== ros2 topic info -v /cmd_vel"
timeout 8 ros2 topic info -v /cmd_vel --no-daemon
