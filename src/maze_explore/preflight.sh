#!/bin/bash
# Go / no-go before a run: are the robot's sensor streams arriving at full rate, and is the
# robot's ROS domain free of other stacks?   (alias: explore_check)
# Fast-LIO diverges within seconds when /livox/imu arrives slowly (2026-11-23: 8-20 Hz instead
# of 200 gave a pose kilometres off), so do not start a run on a NO-GO.
source /opt/ros/humble/setup.bash
export PYTHONUNBUFFERED=1      # else `ros2 topic hz` can die at the timeout with its output unflushed (reads as 0 Hz)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-4}
ok=1
rate() { timeout 6 ros2 topic hz "$1" 2>&1 | grep 'average rate' | tail -1 | awk '{print int($3)}'; }
check() {   # topic, minimum Hz
    r=$(rate "$1"); r=${r:-0}
    if [ "$r" -ge "$2" ]; then echo "  ok     $1  ${r} Hz"; else echo "  NO-GO  $1  ${r} Hz (needs >= $2)"; ok=0; fi
}
echo "sensor rates:"
check /livox/imu 150
check /livox/lidar 9
check /imu/data 150
check /camera/color/image_raw/compressed 20
others=$(timeout 6 ros2 node list 2>/dev/null | grep -v -e lite3_domain_bridge -e _ros2cli -e transform_listener_impl \
         -e laser_mapping -e odom_relay -e wall_scan -e slam_toolbox -e maze_explorer -e rosbag2_recorder)
# Any subscriber to raw depth or /livox/points (even one, even BEST_EFFORT) slows EVERY bridge
# stream to ~10 Hz (2026-11-23: /imu/data 195 -> 11 Hz). Nodes show above; this catches the rest.
for t in /camera/depth/image_raw /camera/depth/points /livox/points; do
    n=$(timeout 6 ros2 topic info "$t" 2>/dev/null | sed -n 's/Subscription count: //p')
    if [ "${n:-0}" -gt 0 ]; then echo "  NO-GO  $t has $n subscriber(s): stop them (RViz depth display, ros2 topic hz/echo, a depth node)"; ok=0; fi
done
if [ -n "$others" ]; then echo "  NO-GO  other nodes on domain $ROS_DOMAIN_ID:"; echo "$others" | sed 's/^/           /'; ok=0
else echo "  ok     no other stacks on domain $ROS_DOMAIN_ID"; fi
if [ $ok = 1 ]; then echo "GO"; else echo "NO-GO: slow streams come from the robot / bridge side (or its host being busy), not from this stack. Wait or ask the lab, then check again."; exit 1; fi
