"""Live mapping + Nav2 on the real Lite3 (Quickstart stage N1: map and take goals at the same time).

The lab launcher does not accept Nav2 goals while it maps (slam:=True), so this starts our own
Nav2 on the live map. Design follows origin/navigation's src/archive (hardware_navigation,
mapping.launch.py, HARDWARE_NAVIGATION.md): Nav2 without AMCL/map_server, 2D costmaps on /scan
only, UDP-only Fast DDS, Fast-LIO's config_file launch scope isolated.

  slam:=toolbox (default)  we start slam_toolbox (online async) on /scan -> /map, map -> odom.
                           bringup:=true (default) also starts lite3_launcher's sensor bringup
                           (Fast-LIO -> /odom + odom -> base_link, /livox/points -> /scan);
                           bringup:=false if that is already running.
  slam:=launcher           something else (e.g. lite3_launcher with slam:=True) already provides
                           /map and map -> odom; we add only Nav2. Refused if a Nav2 is already up.

  wall_scan:=true (default, slam:=toolbox only)  start lite3_nav's wall_scan (borrowed from
                           maze_nav, origin/navigation 419f205): /livox/points -> /wall_scan, only
                           points band_low..band_high (0.30..0.43 m) above the floor, which it
                           measures at startup in odom (the robot must be STANDING at launch, or
                           pass floor_z). slam_toolbox then maps from /wall_scan instead of /scan,
                           so ramps, mats and stair treads stay out of the map. wall_scan:=false
                           maps from lite3_launcher's /scan (0.45..0.85 m above the floor).
  costmap_scan_topic:=/scan  laser topic of the Nav2 obstacle layers; /wall_scan if the arena
                           walls are too low for /scan (the global costmap gets walls from /map).

Nav2's final velocity goes to nav_cmd_vel_topic (default /mission/nav_cmd_vel), never /cmd_vel:
node-scoped remaps move velocity_smoother's output and behavior_server's spin/backup output, so
the lite3_nav velocity gate stays the only /cmd_vel publisher. Run the mission with
config_file:=<share>/config/robot_own_nav2.yaml (two Nav2 input publishers).

Domain guard: robot:=true (default) requires ROS_DOMAIN_ID=4; robot:=false refuses 4 (dry runs
must never reach the robot) and forces ROS_LOCALHOST_ONLY=1. Domain 89 (the simulator) is always
refused.

  ros2 launch lite3_nav robot_navigation.launch.py                        # on the robot, domain 4
  ros2 launch lite3_nav robot_navigation.launch.py robot:=false bringup:=false   # offline dry run
"""
import os
import subprocess
import tempfile
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

NAV2_NODES = ('bt_navigator', 'controller_server', 'planner_server', 'behavior_server',
              'velocity_smoother', 'smoother_server', 'waypoint_follower')


def _truthy(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def check_domain(domain, robot):
    """Return the error for an unsafe ROS domain, or None. Pure function for tests."""
    domain = (domain or '0').strip() or '0'
    if domain == '89':
        return 'ROS_DOMAIN_ID 89 is the simulator: robot_navigation must not run against it'
    if robot and domain != '4':
        return f'robot:=true needs ROS_DOMAIN_ID=4 (the Lite3), got {domain}: source robot_env.sh'
    if not robot and domain == '4':
        return 'robot:=false (dry run) must not use ROS_DOMAIN_ID 4, the real robot'
    return None


def slam_overrides(wall_scan):
    """Parameters given to slam_toolbox after slam_params_file. Pure function for tests."""
    return {'use_sim_time': False, **({'scan_topic': '/wall_scan'} if wall_scan else {})}


def nav2_params_for_scan(params_file, topic, bt_xml=''):
    """params_file unchanged for /scan and no bt_xml, else a temporary copy whose obstacle layers
    read topic and whose bt_navigator uses bt_xml for both of its default trees."""
    import yaml
    topic, bt_xml = topic.strip(), bt_xml.strip()
    if topic in ('', '/scan') and not bt_xml:
        return params_file
    with open(params_file) as f:
        data = yaml.safe_load(f)
    if topic not in ('', '/scan'):
        for name in ('local_costmap', 'global_costmap'):
            data[name][name]['ros__parameters']['obstacle_layer']['scan']['topic'] = topic
    if bt_xml:
        # Humble's bt_navigator loads a through-poses tree too; its default one needs BackUp.
        bt = data.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
        bt['default_nav_to_pose_bt_xml'] = bt['default_nav_through_poses_bt_xml'] = bt_xml
    out = tempfile.NamedTemporaryFile('w', prefix='lite3_nav_nav2_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(data, out, sort_keys=False)
    return out.name


def _running_nodes(env, timeout=10.):
    try:
        out = subprocess.run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', '3'],
                             capture_output=True, text=True, timeout=timeout, env=env).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    return {line.strip().rsplit('/', 1)[-1] for line in out.splitlines() if line.strip()}


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot = _truthy(arg('robot'))
    env = dict(getattr(context, 'environment', os.environ))
    error = check_domain(env.get('ROS_DOMAIN_ID'), robot)
    if error:
        raise RuntimeError(error)
    slam = arg('slam').strip()
    if slam not in ('toolbox', 'launcher'):
        raise RuntimeError('slam must be toolbox or launcher')
    actions = []
    if not robot:
        actions.append(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'))
        env['ROS_LOCALHOST_ONLY'] = '1'
    if _truthy(arg('check_graph')):
        # Two SLAM or Nav2 stacks on one domain corrupt /map, TF and goals (agent.md).
        nodes = _running_nodes(env)
        if nodes is None:
            actions.append(LogInfo(msg='[robot_navigation] could not list nodes; graph check skipped'))
        else:
            clash = sorted(nodes & set(NAV2_NODES))
            if slam == 'toolbox':
                clash += sorted(n for n in nodes if n.startswith('slam_toolbox'))
            if clash:
                raise RuntimeError('Already running on this domain: ' + ', '.join(clash) +
                                   '. Stop the other SLAM/Nav2 stack first (check_graph:=false to skip).')
    return actions


def _bringup(context):
    if LaunchConfiguration('slam').perform(context) != 'toolbox' or \
            not _truthy(LaunchConfiguration('bringup').perform(context)):
        return []
    try:
        course = Path(get_package_share_directory('lite3_launcher'))
    except PackageNotFoundError:
        raise RuntimeError('lite3_launcher not found: source ~/ros2_ws/install/setup.bash, '
                           'or use bringup:=false if Fast-LIO and /scan already run')
    # Scoped group: Fast-LIO's launch argument "config_file" must not leak (it crashed Fast-LIO
    # when shared with another config_file, HARDWARE_NAVIGATION.md).
    return [GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(course / 'launch/include/lite3_bringup.launch.py')),
        launch_arguments={'config_file': 'mid360.yaml',
                          'config_path': str(course / 'config')}.items())])]


def _slam(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    if arg('slam') != 'toolbox':
        return [LogInfo(msg='[robot_navigation] slam:=launcher: expecting /map and map -> odom from the lab stack')]
    wall = _truthy(arg('wall_scan'))
    actions = [Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                    output='screen',
                    parameters=[arg('slam_params_file'), slam_overrides(wall)])]
    if wall:
        actions += [LogInfo(msg='[robot_navigation] SLAM maps from /wall_scan (robot must be standing now: '
                                'wall_scan measures the floor at startup)'),
                    Node(package='lite3_nav', executable='wall_scan', name='wall_scan', output='screen',
                         parameters=[{'use_sim_time': False,
                                      'band_low': float(arg('band_low')),
                                      'band_high': float(arg('band_high')),
                                      'floor_z': float(arg('floor_z'))}])]
    return actions


def _nav2(context):
    topic = LaunchConfiguration('nav_cmd_vel_topic').perform(context).strip()
    if topic in ('', 'cmd_vel', '/cmd_vel'):
        raise RuntimeError('nav_cmd_vel_topic must not be /cmd_vel: the velocity gate owns it')
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2 / 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'autostart': 'true', 'use_composition': 'False',
                          'params_file': nav2_params_for_scan(
                              LaunchConfiguration('params_file').perform(context),
                              LaunchConfiguration('costmap_scan_topic').perform(context),
                              LaunchConfiguration('bt_xml').perform(context))}.items())
    # Node-scoped remaps: a global cmd_vel remap would override navigation_launch's own per-node
    # remaps (controller -> cmd_vel_nav -> smoother). Same fix as the simulator.
    return [LogInfo(msg=f'[robot_navigation] Nav2 velocity output -> {topic}'),
            GroupAction(scoped=True, actions=[
                SetRemap(src='velocity_smoother:cmd_vel_smoothed', dst=topic),
                SetRemap(src='behavior_server:cmd_vel', dst=topic),
                include])]


def generate_launch_description():
    share = Path(get_package_share_directory('lite3_nav'))
    arg = LaunchConfiguration
    declared = [
        DeclareLaunchArgument('robot', default_value='true',
                              description='true: real robot, requires ROS_DOMAIN_ID=4; false: dry run (not 4)'),
        DeclareLaunchArgument('slam', default_value='toolbox', description='toolbox | launcher'),
        DeclareLaunchArgument('bringup', default_value='true',
                              description="slam:=toolbox only: start lite3_launcher's Fast-LIO + /scan bringup"),
        DeclareLaunchArgument('wall_scan', default_value='true',
                              description='slam:=toolbox: start wall_scan and map from /wall_scan instead of /scan'),
        DeclareLaunchArgument('band_low', default_value='0.30', description='wall_scan: m above the floor'),
        DeclareLaunchArgument('band_high', default_value='0.43',
                              description='wall_scan: m above the floor, below the wall top'),
        DeclareLaunchArgument('floor_z', default_value='nan',
                              description='wall_scan: odom z of the floor; nan = measure at startup'),
        DeclareLaunchArgument('costmap_scan_topic', default_value='/scan',
                              description='laser topic of the Nav2 obstacle layers (/scan or /wall_scan)'),
        DeclareLaunchArgument('nav_cmd_vel_topic', default_value='/mission/nav_cmd_vel'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/nav2_robot.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=str(share / 'config/nav2_bt_no_backup.xml'),
                              description="Nav2 behaviour tree (no BackUp); '' = params_file's own"),
        DeclareLaunchArgument('slam_params_file', default_value=str(share / 'config/slam_robot.yaml')),
        DeclareLaunchArgument('fastdds_profile', default_value=str(share / 'config/fastdds_udp.xml'),
                              description='UDP-only Fast DDS; Nav2 lifecycle stalled on the robot without it'),
        DeclareLaunchArgument('check_graph', default_value='true',
                              description='refuse to start if SLAM/Nav2 nodes already run on this domain'),
    ]
    return LaunchDescription(declared + [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', arg('fastdds_profile')),
        OpaqueFunction(function=_setup),
        OpaqueFunction(function=_bringup),
        OpaqueFunction(function=_slam),
        OpaqueFunction(function=_nav2),
    ])
