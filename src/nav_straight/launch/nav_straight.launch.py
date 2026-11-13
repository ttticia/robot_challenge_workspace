"""Start SLAM and let Nav2 walk the Lite3 straight ahead until an obstacle is near.

    ros2 launch nav_straight nav_straight.launch.py

Pipeline:
  lite3_bringup (lite3_launcher): Fast-LIO + odom_relay + pointcloud_to_laserscan
                                              -> /odom, odom->base_link, /scan
  slam_toolbox (online async, mapping)        -> /map, map->odom
  straight_goal                               -> straight path on /follow_path
  Nav2 controller_server (RPP, voxel costmap) -> cmd_vel_nav
  Nav2 velocity_smoother                      -> cmd_vel_smoothed
  Nav2 collision_monitor (live cloud)         -> nav_cmd_vel
  cmd_vel_bridge (QoS, clamp, sensor watchdog)-> /cmd_vel; straight_goal -> /simple_cmd manual

Before launching: robot standing, area clear, operator holding the handheld
controller, `lite3_auto` sent by the operator. Nothing else publishing /cmd_vel.

Arguments:
  bringup:=false   Fast-LIO / /scan already running (or a simulator)
  slam:=false      don't start slam_toolbox
  drive:=false     dry run: commands go to /cmd_vel_bridge/cmd_vel_preview
  max_speed:=0.10  stop_distance:=0.50  max_distance:=5.0  start_delay:=3.0
  udp_only:=true   UDP-only Fast DDS (Nav2 lifecycle stalled on the robot without it)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

NAV2_NODES = ['controller_server', 'velocity_smoother', 'collision_monitor']


def generate_launch_description():
    config = os.path.join(get_package_share_directory('nav_straight'), 'config')
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}
    drive = ParameterValue(arg('drive'), value_type=bool)

    nav2_params = RewrittenYaml(
        source_file=os.path.join(config, 'nav2_params.yaml'),
        param_rewrites={
            'use_sim_time': arg('use_sim_time'),
            'desired_linear_vel': arg('max_speed'),
            'lookahead_dist': arg('stop_distance'),
            'min_lookahead_dist': arg('stop_distance'),
            'max_lookahead_dist': arg('stop_distance'),
        },
        convert_types=True)

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[nav2_params], remappings=list(remappings))

    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='true',
                              description='Start Fast-LIO, odom relay and /scan (lite3_launcher)'),
        DeclareLaunchArgument('slam', default_value='true', description='Start slam_toolbox'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='false = dry run, nothing is sent to /cmd_vel'),
        DeclareLaunchArgument('max_speed', default_value='0.10',
                              description='m/s (clamped to 0.25 by cmd_vel_bridge)'),
        DeclareLaunchArgument('stop_distance', default_value='0.50',
                              description='m between the robot front and the obstacle '
                                          '(RPP lookahead)'),
        DeclareLaunchArgument('max_distance', default_value='5.0',
                              description='m; length of the straight path (> 0)'),
        DeclareLaunchArgument('start_delay', default_value='3.0',
                              description='s countdown before moving'),
        DeclareLaunchArgument('udp_only', default_value='true',
                              description='UDP-only Fast DDS profile'),
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
             output='screen', condition=IfCondition(arg('slam')),
             parameters=[os.path.join(config, 'slam_toolbox.yaml'), sim_time]),

        # Nav2: controller -> smoother -> collision monitor -> nav_cmd_vel
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': NAV2_NODES}, sim_time]),

        Node(package='nav_straight', executable='cmd_vel_bridge', name='cmd_vel_bridge',
             output='screen', parameters=[{'drive': drive}, sim_time]),

        Node(package='nav_straight', executable='straight_goal', name='straight_goal',
             output='screen',
             parameters=[sim_time, {
                 'drive': drive,
                 'max_distance': ParameterValue(arg('max_distance'), value_type=float),
                 'start_delay': ParameterValue(arg('start_delay'), value_type=float),
             }]),
    ])
