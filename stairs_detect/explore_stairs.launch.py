"""COPY of maze_explore/launch/explore.launch.py (24 Nov, Rey: copy, don't edit the originals).
Identical, except maze_explorer runs stairs_detect/explorer_stairs.py (depth check of the
stairs before AI mode) instead of the installed maze_explore explorer. Same node name, topics
and arguments, plus stairs_check:=false to switch the check off.

    ros2 launch ~/stairs_detect/explore_stairs.launch.py drive:=false auto_start:=true

Original docstring follows.
The whole stack.

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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    'gait': ('rug', 'walking gait: rug (RUG general gear) or hstep (H-Step)'),
    'enter_rooms': ('false', 'walk into rooms instead of looking in from the cell before'),
    'v_max': ('0.20', 'top walking speed, m/s'),
    'stand_height': ('0.30', 'base_link height used when the floor cannot be measured'),
    'stairs_check': ('true', 'depth camera check of the stairs before AI mode (stairs_detect)'),
}
STAIRS_DIR = os.path.dirname(os.path.realpath(__file__))


def explorer_copy(context, c, env):
    """python3 stairs_detect/explorer_stairs.py as node maze_explorer, arguments evaluated here."""
    import tempfile
    import yaml
    from launch.actions import ExecuteProcess
    if c['explore'].perform(context).lower() not in ('true', '1'):
        return []
    b = lambda n: c[n].perform(context).lower() in ('true', '1')   # noqa: E731
    params = {'drive': b('drive'), 'auto_start': b('auto_start'), 'gait': c['gait'].perform(context),
              'enter_rooms': b('enter_rooms'), 'v_max': float(c['v_max'].perform(context)),
              'stairs_check': b('stairs_check')}
    fd, path = tempfile.mkstemp(prefix='maze_explorer_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump({'maze_explorer': {'ros__parameters': params}}, f)
    return [ExecuteProcess(cmd=['python3', '-u', os.path.join(STAIRS_DIR, 'explorer_stairs.py'),
                                '--ros-args', '-r', '__node:=maze_explorer', '--params-file', path],
                           name='maze_explorer', output='screen', additional_env=env)]


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
            OpaqueFunction(function=explorer_copy, args=[c, env]),
        ])
