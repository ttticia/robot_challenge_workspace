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
startup), operator holding the handheld controller (sends `lite3_auto` once the log says "gait sent"),
nothing else publishing /cmd_vel or running SLAM/Nav2 (`ros2 node list`). The launch checks
the last part itself first (maze_nav/preflight.py) and warns if anything clashes.

Arguments:
  bringup:=false   Fast-LIO / odom already running (or a simulator)
  go:=false        explorer picks goals but doesn't send them (watch /maze/goal in RViz)
  drive:=false     Nav2 runs but nothing reaches /cmd_vel (cmd_vel_bridge preview topic)
  gait:=rug        cmd_vel_bridge sends Move mode + RUG gait (General) once at startup;
                   none = leave the gait alone. Send lite3_auto AFTER "gait sent"
  wait_for_start:=true  hold still once everything is up until
                   `ros2 topic pub --once /maze/start std_msgs/msg/Empty`
  creep_distance:=0.10  m towards the open frontier after start, onto the SLAM map and off the
                   back wall (0 = off)
  start_scan:=true turn 45 deg left, 90 right, 45 left after the creep, before exploring
  max_speed:=0.15  band_low:=0.30  band_high:=0.43  floor_z:=nan (measure)
  map_dir:=~/challenge_ws/maps   udp_only:=true   use_sim_time:=false
  domain_id:=auto  auto = 4 (the robot's domain, ROS_LOCALHOST_ONLY=0) with bringup:=true,
                   whatever the terminal has; with bringup:=false (simulator) the terminal's
                   ROS_DOMAIN_ID. A number forces that domain
  log_tags:=false  terminal only: drop `[INFO]` and the timestamps (`[WARN]`/`[ERROR]` stay,
                   coloured); true = ROS's full `[INFO] [<time>] [node]: msg`. Log files are
                   always full
  verbose:=true    false = Nav2 and slam_toolbox log warnings/errors only (no lifecycle and
                   per-goal chatter); maze_explorer, wall_scan and cmd_vel_bridge keep INFO
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

NAV2_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
              'velocity_smoother']


ROBOT_DOMAIN = '4'   # lite3_domain_bridge_4, RViz / external computer: robot_info.md


def ros_domain(context):
    """Pick ROS_DOMAIN_ID for every node of this launch (and the preflight graph check).

    Real runs must be on the robot's domain, or the robot bridge never sees /cmd_vel and
    nobody else sees /map: a terminal left on a test domain (89 for the simulator) would
    silently put the whole run there.
    """
    import launch.logging
    log = launch.logging.get_logger('maze_nav.domain')
    want = LaunchConfiguration('domain_id').perform(context).strip().lower()
    real = LaunchConfiguration('bringup').perform(context).lower() in ('true', '1')
    have = os.environ.get('ROS_DOMAIN_ID', '0')
    if want == 'auto':
        if not real:
            log.info(f'ROS_DOMAIN_ID={have} (from the terminal; bringup:=false)')
            return []
        want = ROBOT_DOMAIN
    if not want.isdigit():
        raise RuntimeError(f'domain_id:={want!r}: use auto or a number')
    if want != have:
        log.warning(f'ROS_DOMAIN_ID={want} for this run (terminal has {have}): use '
                    f'ROS_DOMAIN_ID={want} in other terminals / RViz to see it')
    else:
        log.info(f'ROS_DOMAIN_ID={want}')
    localhost = '0' if want == ROBOT_DOMAIN else os.environ.get('ROS_LOCALHOST_ONLY', '0')
    os.environ['ROS_DOMAIN_ID'] = want             # preflight runs in this process
    os.environ['ROS_LOCALHOST_ONLY'] = localhost
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', want),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', localhost)]


def preflight(context):
    # Runs before any node below starts, so none of ours are in the way yet.
    import launch.logging
    from maze_nav.preflight import run
    flag = lambda n: LaunchConfiguration(n).perform(context).lower() in ('true', '1')  # noqa: E731
    run(context, flag('bringup'), flag('wait_for_start'),
        launch.logging.get_logger('maze_nav.preflight'))
    return []


# What a dead process means for the run, for the banner below.
DEATH_HINTS = {
    'planner_server': 'Nav2 lifecycle manager will take ALL of Nav2 down in ~10 s; the explorer '
                      'stops as STOPPED. Ctrl+C and relaunch.',
    'controller_server': 'Nav2 is going down; the robot stops. Ctrl+C and relaunch.',
    'behavior_server': 'Nav2 is going down; the robot stops. Ctrl+C and relaunch.',
    'bt_navigator': 'Nav2 is going down; the robot stops. Ctrl+C and relaunch.',
    'velocity_smoother': 'Nav2 is going down; nothing reaches /cmd_vel. Ctrl+C and relaunch.',
    'lifecycle_manager': 'Nav2 is unmanaged. Ctrl+C and relaunch.',
    'async_slam_toolbox_node': 'no more /map or map->odom: navigation is blind. Ctrl+C.',
    'fastlio_mapping': 'no odometry: everything downstream stops. Ctrl+C.',
    'wall_scan': 'no /wall_scan: SLAM and costmaps get no walls. Ctrl+C.',
    'cmd_vel_bridge': 'nothing reaches /cmd_vel; the robot stops. Ctrl+C.',
    'maze_explorer': 'no more goals; Nav2 finishes the current one. Ctrl+C.',
}


# Optional `[process-N] ` and colour code, then ROS's `[LEVEL] [<epoch time>] `.
_LEAD = r'^((?:\[[^\]]+\] )?(?:\x1b\[[0-9;]*m)?)'
_INFO = re.compile(_LEAD + r'\[INFO\] (?:\[\d+\.\d+\] )?')
_STAMP = re.compile(_LEAD + r'\[(DEBUG|WARN|WARNING|ERROR|FATAL)\] \[\d+\.\d+\] ')


def quiet_launch_log(context):
    """log_tags:=false: rewrite lines on the way to the terminal only.

    `[INFO] [<time>] [node]: msg` -> `[node]: msg` and `[WARN] [<time>] ...` -> `[WARN] ...`,
    for the nodes' output and launch's own messages. Wraps the screen handler's format(),
    so launch.log and the per-node files in ~/.ros/log keep the full prefix (needed to
    debug a run afterwards).
    """
    if LaunchConfiguration('log_tags').perform(context).lower() in ('true', '1'):
        return []
    import launch.logging
    handler = launch.logging.launch_config.get_screen_handler()
    if getattr(handler, '_maze_quiet', False):
        return []
    full = handler.format

    def short(record):
        line = full(record)
        line = _INFO.sub(r'\1', line, count=1)
        return _STAMP.sub(r'\1[\2] ', line, count=1)

    handler.format = short
    handler._maze_quiet = True
    return []


def death_banners():
    """Event handlers that print a loud banner when a process dies, and nothing otherwise.

    On Ctrl+C every process exits, some with -2/-15/-9 (lifecycle_manager is often
    SIGKILLed): expected, so no banner once the launch is shutting down.
    """
    shutting_down = []

    def on_exit(event, _context):
        if shutting_down or event.returncode == 0:
            return None
        name = event.action.name                     # e.g. planner_server-5
        hint = next((h for k, h in DEATH_HINTS.items() if name.startswith(k)), 'Ctrl+C.')
        bar = '!' * 78
        return LogInfo(msg=f'\n{bar}\n!!! {name} DIED (exit code {event.returncode})'
                           f'\n!!! {hint}\n{bar}')

    return [RegisterEventHandler(OnShutdown(on_shutdown=lambda *_: shutting_down.append(1))),
            RegisterEventHandler(OnProcessExit(on_exit=on_exit))]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('maze_nav'), 'config')
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}
    # Log level for the third-party nodes (Nav2, slam_toolbox); ours always log INFO.
    quiet_level = PythonExpression(
        ["'info' if '", arg('verbose'), "'.lower() in ('true', '1') else 'warn'"])
    log_args = ['--ros-args', '--log-level', quiet_level]

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
                    remappings=list(remappings), arguments=log_args)

    lite3_base = PathJoinSubstitution([FindPackageShare('lite3_launcher'), 'launch', 'include',
                                       'Base'])

    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='true',
                              description='Start Fast-LIO and odom_relay (lite3_launcher)'),
        DeclareLaunchArgument('go', default_value='true',
                              description='false = explorer only publishes /maze/goal'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='false = nothing is sent to /cmd_vel'),
        DeclareLaunchArgument('gait', default_value='rug',
                              description='sent once on /simple_cmd at startup (after Move mode): '
                                          'rug = RUG gait General gear; none = leave it'),
        DeclareLaunchArgument('wait_for_start', default_value='true',
                              description='hold still once ready until /maze/start (std_msgs/Empty)'),
        DeclareLaunchArgument('creep_distance', default_value='0.10',
                              description='m towards the open frontier after start; 0 = off'),
        DeclareLaunchArgument('start_scan', default_value='true',
                              description='after the creep: turn 45 deg left, 90 right, 45 left (back to the start heading)'),
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
        DeclareLaunchArgument('domain_id', default_value='auto',
                              description='auto = 4 with bringup:=true (real run), else the '
                                          "terminal's ROS_DOMAIN_ID; a number forces it"),
        DeclareLaunchArgument('log_tags', default_value='false',
                              description='terminal: false = no [INFO] or timestamps; true = full ROS prefix. Log files always full'),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='false = Nav2 / slam_toolbox log warnings and errors only'),

        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(config, 'fastdds_udp.xml'),
                               condition=IfCondition(arg('udp_only'))),
        # Warnings yellow, errors red, although the nodes' stdout is a pipe to ros2 launch.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        OpaqueFunction(function=quiet_launch_log),
        OpaqueFunction(function=ros_domain),        # before preflight and every node
        OpaqueFunction(function=preflight),
        *death_banners(),

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
             output='screen', arguments=log_args,
             parameters=[os.path.join(config, 'slam_toolbox.yaml'), sim_time]),

        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'nav_cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', arguments=log_args,
             parameters=[{'autostart': True, 'node_names': NAV2_NODES,
                          'bond_timeout': 10.0}, sim_time]),

        Node(package='maze_nav', executable='cmd_vel_bridge', name='cmd_vel_bridge',
             output='screen',
             parameters=[sim_time, {'drive': ParameterValue(arg('drive'), value_type=bool),
                                    'gait': arg('gait')}]),

        Node(package='maze_nav', executable='maze_explorer', name='maze_explorer',
             output='screen',
             parameters=[sim_time, {
                 'go': ParameterValue(arg('go'), value_type=bool),
                 'wait_for_start': ParameterValue(arg('wait_for_start'), value_type=bool),
                 'map_dir': arg('map_dir'),
                 'creep_distance': ParameterValue(arg('creep_distance'), value_type=float),
                 'start_scan': ParameterValue(arg('start_scan'), value_type=bool),
             }]),
    ])
