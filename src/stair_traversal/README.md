# Lite3 stair traversal

The launch starts the junction observer and stair routine together. A forward
`Metal stairs` result triggers a depth measurement. The routine projects the
depth into odometry coordinates, looks for a run of ascending flat treads and
a full-width upper landing, and waits for three consistent estimates. If the
depth, transforms, or odometry are stale or the landings cannot be seen, it holds.
Metal surfaces can yield missing depth readings.

The Lite3-AI user manual documents AI mode for flat terrain and RUG gaits for
stairs. This routine requests RUG General, walks straight ahead, stops at the
measured top, and reverses to the starting landing. It checks distance and
height at each endpoint and returns to manual control at the bottom. Body
odometry and visible landing geometry cannot directly verify foot contact.
[Lite3-AI user manual, §§3.6.1–3.6.2](https://www.deeprobotics.us/wp-content/uploads/2025/08/Jueying-Lite3-AI-User-Manual-V1.0.3-0.pdf)

## Preview from the repository root

```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash
colcon build --packages-select scene_understanding stair_traversal
source install/setup.bash
ros2 launch stair_traversal scene_stairs.launch.py
```

The default inputs are `/camera/depth/image_raw`, `/camera/depth/camera_info`
and `/odom`. The depth must be rectified with matching projection calibration
and time-stamped TF to `odom`. Override the corresponding `depth_topic`,
`camera_info_topic` or `odom_topic` launch arguments if needed.

If navigation doesn't publish `/junction_detected`, trigger a preview from
another sourced terminal:

```bash
ros2 topic pub --once /junction_detected std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /junction_detected std_msgs/msg/Bool '{data: false}'
```

## Motion

First validate the depth estimate in preview with the robot stationary and
square at the bottom. The routine measures a straight corridor from that
heading and does not turn or approach the staircase. Verify the chosen gait,
reverse descent on the actual stairs, and the controller's stop on command loss.
Ensure the stair node has exclusive publication on `/cmd_vel` and `/simple_cmd`
before triggering the scene observation.

```bash
ros2 launch stair_traversal scene_stairs.launch.py execute:=true \
  aligned_start:=true reverse_descent_verified:=true
```

Abort from another terminal:

```bash
ros2 service call /stair_traversal/abort std_srvs/srv/Trigger '{}'
```

Restart the launch to arm another run. Hardware traversal and command
acceptance remain unverified.
