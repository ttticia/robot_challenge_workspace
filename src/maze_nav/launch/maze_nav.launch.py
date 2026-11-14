"""Explore a maze with SLAM + Nav2 until the Lite3 is out.

    ros2 launch maze_nav maze_nav.launch.py

Pipeline:
  Fast-LIO + odom_relay (lite3_launcher)        -> /odom, odom->base_link
  wall_scan                                     -> /wall_scan (walls only, floor-referenced)
  slam_toolbox (online async)                   -> /map, map->odom
  maze_explorer (frontiers + exit detection)    -> /navigate_to_pose goals
  Nav2: planner (Smac 2D), controller (RPP), behaviors, bt_navigator, velocity_smoother
                                                -> nav_cmd_vel
  cmd_vel_bridge (QoS, clamp, sensor watchdog)  -> /cmd_vel

Before launching: robot STANDING on the start cell (wall_scan measures the floor height at
startup), operator holding the handheld controller, `lite3_auto` sent by the operator,
nothing else publishing /cmd_vel or running SLAM/Nav2 (`ros2 node list`). The launch checks
the last part itself first (maze_nav/preflight.py) and warns if anything clashes.

Arguments:
  bringup:=false   Fast-LIO / odom already running (or a simulator)
  go:=false        explorer picks goals but doesn't send them (watch /maze/goal in RViz)
  drive:=false     Nav2 runs but nothing reaches /cmd_vel (cmd_vel_bridge preview topic)
  wait_for_start:=true  hold still once everything is up until
                   `ros2 topic pub --once /maze/start std_msgs/msg/Empty`
  start_scan:=true turn 90 deg left and right on the start cell before exploring
  max_speed:=0.15  band_low:=0.30  band_high:=0.43  floor_z:=nan (measure)
  map_dir:=~/challenge_ws/maps   udp_only:=true   use_sim_time:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

NAV2_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
              'velocity_smoother']


def preflight(context):
    # Runs before any node below starts, so none of ours are in the way yet.
    import launch.logging
    from maze_nav.preflight import run
    flag = lambda n: LaunchConfiguration(n).perform(context).lower() in ('true', '1')  # noqa: E731
    run(context, flag('bringup'), flag('wait_for_start'),
        launch.logging.get_logger('maze_nav.preflight'))
    return []


def generate_launch_description():
    config = os.path.join(get_package_share_directory('maze_nav'), 'config')
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}

    nav2_params = RewrittenYaml(
        source_file=os.path.join(config, 'nav2_params.yaml'),
        param_rewrites={'desired_linear_vel': arg('max_speed')},   # only rewrites existing keys
        convert_types=True)
    # Humble's bt_navigator always loads a through-poses tree too; its default one needs the
    # BackUp server, which isn't started here, so both point at maze_bt.xml.
    bt_xml = os.path.join(config, 'maze_bt.xml')
    extra = {'bt_navigator': {'default_nav_to_pose_bt_xml': bt_xml,
                              'default_nav_through_poses_bt_xml': bt_xml}}

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[nav2_params, sim_time, extra.get(executable, {})],
                    remappings=list(remappings))

    lite3_base = PathJoinSubstitution([FindPackageShare('lite3_launcher'), 'launch', 'include',
                                       'Base'])

    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='true',
                              description='Start Fast-LIO and odom_relay (lite3_launcher)'),
        DeclareLaunchArgument('go', default_value='true',
                              description='false = explorer only publishes /maze/goal'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='false = nothing is sent to /cmd_vel'),
        DeclareLaunchArgument('wait_for_start', default_value='true',
                              description='hold still once ready until /maze/start (std_msgs/Empty)'),
        DeclareLaunchArgument('start_scan', default_value='true',
                              description='turn 90 deg left and right before exploring'),
        DeclareLaunchArgument('max_speed', default_value='0.15',
                              description='m/s (clamped to 0.25 by cmd_vel_bridge)'),
        DeclareLaunchArgument('band_low', default_value='0.30',
                              description='m above the floor; below = terrain/stairs, ignored'),
        DeclareLaunchArgument('band_high', default_value='0.43',
                              description='m above the floor; must stay below the wall top'),
        DeclareLaunchArgument('floor_z', default_value='nan',
                              description='odom z of the floor; nan = measure at startup'),
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/challenge_ws/maps')),
        DeclareLaunchArgument('udp_only', default_value='true',
                              description='UDP-only Fast DDS profile'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(config, 'fastdds_udp.xml'),
                               condition=IfCondition(arg('udp_only'))),
        OpaqueFunction(function=preflight),

        # Fast-LIO + odom_relay only; lite3_bringup would also start a /scan we don't use.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([lite3_base,
                                                                'fast_lio.launch.py'])),
            condition=IfCondition(arg('bringup'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([lite3_base,
                                                                'odom_relay.launch.py'])),
            condition=IfCondition(arg('bringup'))),

        Node(package='maze_nav', executable='wall_scan', name='wall_scan', output='screen',
             parameters=[sim_time, {
                 'band_low': ParameterValue(arg('band_low'), value_type=float),
                 'band_high': ParameterValue(arg('band_high'), value_type=float),
                 'floor_z': ParameterValue(arg('floor_z'), value_type=float),
             }]),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen',
             parameters=[os.path.join(config, 'slam_toolbox.yaml'), sim_time]),

        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'nav_cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': NAV2_NODES,
                          'bond_timeout': 10.0}, sim_time]),

        Node(package='maze_nav', executable='cmd_vel_bridge', name='cmd_vel_bridge',
             output='screen',
             parameters=[sim_time, {'drive': ParameterValue(arg('drive'), value_type=bool)}]),

        Node(package='maze_nav', executable='maze_explorer', name='maze_explorer',
             output='screen',
             parameters=[sim_time, {
                 'go': ParameterValue(arg('go'), value_type=bool),
                 'wait_for_start': ParameterValue(arg('wait_for_start'), value_type=bool),
                 'start_scan': ParameterValue(arg('start_scan'), value_type=bool),
                 'map_dir': arg('map_dir'),
             }]),
    ])
