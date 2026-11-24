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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'fastlio': ('true', 'run Fast-LIO and odom_relay (false: something else publishes /odom)'),
    'explore': ('true', 'run maze_explorer (false = mapping only)'),
    'drive': ('true', 'false: velocity goes to /explore/cmd_vel_preview, no mode requests'),
    'auto_start': ('false', 'start exploring without waiting for /nav/start'),
    'cloud_topic': ('/cloud_registered_body', 'cloud for wall_scan (sim: /livox/points)'),
    'gait': ('hstep', 'walking gait: hstep or rug'),
    'enter_rooms': ('false', 'walk into rooms instead of looking in from the cell before'),
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
            Node(package='maze_explore', executable='maze_explorer', name='maze_explorer',
                 output='screen', additional_env=env, condition=IfCondition(c['explore']),
                 parameters=[{'drive': val('drive', bool), 'auto_start': val('auto_start', bool),
                              'gait': c['gait'], 'enter_rooms': val('enter_rooms', bool),
                              'v_max': val('v_max', float)}]),
        ])
