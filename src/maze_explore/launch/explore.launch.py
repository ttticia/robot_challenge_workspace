"""The whole stack.

    ros2 launch maze_explore explore.launch.py                 # robot
    ros2 launch maze_explore explore.launch.py explore:=false  # mapping only (hand-driven runs)
    ros2 launch maze_explore explore.launch.py drive:=false    # explorer decides, sends nothing

  fast_lio + odom_relay   lidar odometry: /odom, odom -> base_link   (fastlio:=false in the sim)
  wall_scan               levelled lidar slice: /scan, /maze/cloud, odom -> base_level
  slam_toolbox            /scan -> /map, map -> odom
  maze_explorer           decides, drives (/cmd_vel) and switches modes (/simple_cmd)

Start it with the robot standing still, centred in the start cell, facing into the maze.
The explorer waits for:  ros2 topic pub --once /nav/start std_msgs/msg/Empty
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ros2 launch writes each node's parameters to a temporary file. Keep those in the workspace
# too (user, 2026-11-24: no files outside ~/challenge_ws at run time); the daily log clean-up
# removes them after a week.
import tempfile  # noqa: E402
_TMP = os.path.expanduser('~/challenge_ws/logs/tmp')
os.makedirs(_TMP, exist_ok=True)
tempfile.tempdir = _TMP
os.environ['TMPDIR'] = _TMP

ARGS = {
    'fastlio': ('true', 'run Fast-LIO and odom_relay (false: something else publishes /odom)'),
    'explore': ('true', 'run maze_explorer (false = mapping only)'),
    'drive': ('true', 'false: velocity goes to /explore/cmd_vel_preview, no mode requests'),
    'auto_start': ('false', 'start exploring without waiting for /nav/start'),
    'cloud_topic': ('/cloud_registered_body', 'cloud for wall_scan (sim: /livox/points)'),
    'gait': ('rug', 'walking gait: rug (RUG general gear) or hstep (H-Step)'),
    'scene_understanding': ('true', 'analyse stitched room and object looks with the VLM'),
    'scene_camera_topic': ('/camera/color/image_raw', 'RGB image topic for the VLM'),
    'scene_image_transport': ('compressed', 'compressed = <scene_camera_topic>/compressed '
                                            '(JPEG); raw only where no compressed topic exists'),
    'enter_rooms': ('false', 'walk into rooms instead of looking in from the cell before'),
    'stair_check': ('true', 'confirm stairs with the depth camera before crossing (depth is '
                            'only subscribed for the ~3 s of each check)'),
    'v_max': ('0.20', 'top walking speed, m/s'),
    'stand_height': ('0.30', 'base_link height used when the floor cannot be measured'),
}


def generate_launch_description():
    config = os.path.join(get_package_share_directory('maze_explore'), 'config')
    c = {k: LaunchConfiguration(k) for k in ARGS}
    env = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'}

    def val(name, kind):
        return ParameterValue(c[name], value_type=kind)
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()] + [
            Node(package='fast_lio', executable='fastlio_mapping', output='screen',
                 condition=IfCondition(c['fastlio']),
                 parameters=[os.path.join(config, 'fastlio_mid360.yaml')],
                 remappings=[('/Odometry', '/fastlio/odometry')]),
            Node(package='lite3_launcher', executable='odom_relay.py', name='odom_relay',
                 output='screen', condition=IfCondition(c['fastlio']),
                 parameters=[{'input_topic': '/fastlio/odometry', 'output_topic': '/odom',
                              'odom_frame': 'odom', 'base_frame': 'base_link',
                              'lidar_frame': 'livox_frame', 'lidar_init_frame': 'lidar_init',
                              'publish_tf': True}]),
            Node(package='maze_explore', executable='wall_scan', name='wall_scan', output='screen',
                 additional_env=env,
                 parameters=[{'cloud_topic': c['cloud_topic'],
                              'stand_height': val('stand_height', float)}]),
            Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                 output='screen', parameters=[os.path.join(config, 'slam_toolbox.yaml')]),
            Node(package='scene_understanding', executable='junction_trigger',
                 output='screen', condition=IfCondition(PythonExpression([
                     "'", c['explore'], "'.lower() == 'true' and '",
                     c['scene_understanding'], "'.lower() == 'true'",
                 ])),
                 parameters=[{'enable_vlm': True, 'explore_event_topic': '/explore/event',
                              'junction_scan_enabled': False,
                              # compressed: a raw image subscriber slows the lidar IMU
                              # on its way here and Fast-LIO diverges (08:19, 08:37 runs)
                              'image_topic': c['scene_camera_topic'],
                              'image_transport': c['scene_image_transport'],
                              'log_rate': False,
                              'save_dir': os.path.expanduser('~/challenge_ws/logs/scene')}]),
            Node(package='maze_explore', executable='maze_explorer', name='maze_explorer',
                 output='screen', additional_env=env, condition=IfCondition(c['explore']),
                 parameters=[{'drive': val('drive', bool), 'auto_start': val('auto_start', bool),
                              'gait': c['gait'], 'enter_rooms': val('enter_rooms', bool),
                              'stair_check': val('stair_check', bool),
                              'v_max': val('v_max', float)}]),
        ])
