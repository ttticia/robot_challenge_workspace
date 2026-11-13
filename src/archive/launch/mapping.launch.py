"""Build a map with slam_toolbox while Nav2 drives to goals you click in RViz.

    ros2 launch navigation mapping.launch.py

Pipeline:
  Fast-LIO + odom_relay + pointcloud_to_laserscan (lite3_launcher)
                                                      -> /odom, odom->base_link, /scan
  slam_toolbox                                                      -> /map, map->odom
  Nav2: planner, controller, behaviors, bt_navigator, smoother      -> /nav_cmd_vel
  cmd_vel_bridge  (QoS fix + speed clamp + stop on idle)            -> /cmd_vel (robot)
  map_autosaver   (every save_interval s and on Ctrl+C)
                                                      -> <map_dir>/<map_name>.pgm/.yaml

Arguments:
  bringup:=false   sensors/odometry already running elsewhere
  nav2:=false      SLAM only (drive by teleop)
  drive:=false     run Nav2 but never publish /cmd_vel (dry run)
  map_dir:=...     default ~/challenge_ws/maps
  map_name:=...    default map_<date>_<time>
  save_interval:=  seconds between saves (0 = only on shutdown)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server',
              'bt_navigator', 'velocity_smoother']


def generate_launch_description():
    config = os.path.join(get_package_share_directory('navigation'), 'config')
    nav2_params = os.path.join(config, 'nav2_params.yaml')
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': arg('use_sim_time')}

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[nav2_params, sim_time], remappings=list(remappings))

    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='true',
                              description='Start Fast-LIO, odom relay and /scan (lite3_launcher)'),
        DeclareLaunchArgument('nav2', default_value='true', description='Start Nav2'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='false = Nav2 runs but nothing is sent to /cmd_vel'),
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/challenge_ws/maps')),
        DeclareLaunchArgument('map_name', default_value='',
                              description='Empty = map_<date>_<time>'),
        DeclareLaunchArgument('save_interval', default_value='30.0',
                              description='Seconds between map saves; 0 = only on shutdown'),
        DeclareLaunchArgument('udp_only', default_value='true',
                              description='UDP-only Fast DDS; Nav2 lifecycle stalled on the '
                                          'robot without it (see archive/HARDWARE_NAVIGATION.md)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(config, 'fastdds_udp.xml'),
                               condition=IfCondition(arg('udp_only'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('lite3_launcher'), 'launch', 'include',
                'lite3_bringup.launch.py'])),
            condition=IfCondition(arg('bringup'))),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen',
             parameters=[os.path.join(config, 'slam_toolbox.yaml'), sim_time]),

        Node(package='navigation', executable='map_autosaver', name='map_autosaver',
             output='screen',
             parameters=[{'map_dir': ParameterValue(arg('map_dir'), value_type=str),
                          'map_name': ParameterValue(arg('map_name'), value_type=str),
                          'save_interval_sec': arg('save_interval')}, sim_time]),

        GroupAction(condition=IfCondition(arg('nav2')), actions=[
            # The controller feeds the velocity smoother; the smoother and the recovery
            # behaviors publish /nav_cmd_vel, which cmd_vel_bridge forwards to the robot.
            nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
            nav2_node('nav2_planner', 'planner_server'),
            nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'nav_cmd_vel')]),
            nav2_node('nav2_bt_navigator', 'bt_navigator'),
            nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                      [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'nav_cmd_vel')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'autostart': True, 'node_names': NAV2_NODES}, sim_time]),
            Node(package='navigation', executable='cmd_vel_bridge', name='cmd_vel_bridge',
                 output='screen',
                 parameters=[{'drive_enabled': arg('drive')}, sim_time]),
        ]),
    ])
