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

Before launching: robot on the start cell, standing or lying down (auto_stand stands it up;
wall_scan measures the floor height once it stands), operator holding the handheld controller (sends `lite3_auto` once the log says "gait sent"),
nothing else publishing /cmd_vel or running SLAM/Nav2 (`ros2 node list`). The launch checks
the last part itself first (maze_nav/preflight.py) and warns if anything clashes.

Arguments:
  bringup:=false   Fast-LIO / odom already running (or a simulator)
  go:=false        explorer picks goals but doesn't send them (watch /maze/goal in RViz)
  drive:=false     Nav2 runs but nothing reaches /cmd_vel (cmd_vel_bridge preview topic)
  gait:=hstep      cmd_vel_bridge sends Move mode + H-Step (high-stepping RUG gait) once at
                   startup; rug = RUG gait General gear (before 2026-11-20); none = leave
                   the gait alone. Send lite3_auto AFTER "gait sent"
  perception:=true also start team_perception (robot.launch.py: qr_scanner only; human /
                   poster detection removed 2026-11-19, too many camera copies for the
                   bridge). It runs niced on all but the first 4 cores, and only at dead
                   ends (explorer /maze/perceive).
                   Depth for them and stair_detector goes through ONE team_perception
                   depth_relay (raw depth once from the bridge, 10 Hz on /camera/depth_relay/*)
  auto_stand:=true cmd_vel_bridge sends stand first, ONLY if the lidar shows the robot lying
                   down (stand is a toggle: a standing robot would sit); false = never
  wait_for_start:=true  hold still once everything is up until
                   `ros2 topic pub --once /maze/start std_msgs/msg/Empty`
  stairs:=true     stair_detector: stairs in the depth image within 1.5 m -> the explorer stops
                   Nav2, the robot turns to face them, centres, stops with base_link 0.45 m
                   from the first riser, square to the steps (AT_STAIRS), and holds until
                   /maze/start (maze_nav/stair_detector.py)
  curtains:=true   curtain_detector: a wall in front with a < 0.15 m gap at the bottom, wider
                   than 0.7 m, is a curtain: it is cleared from the wall scan and the map, the
                   explorer stops Nav2 (CURTAIN), and the robot walks through its middle at 70 %
                   of max_speed to 0.2 m past it, turns 20 deg left / 40 right / 20 left, and
                   creeps 5 cm / turns again while the lidar or depth camera stays covered
                   (STUCK -> AT_CURTAIN: hold until /maze/start) (maze_nav/curtain_detector.py)
                   Also looked for at dead ends / junction looks (detection only); same wall
                   band as wall_scan; why a wall isn't a curtain is logged ("no curtain: ...")
  curtain_sensors:=both  what sees the gap: both (lidar + depth camera, ~0.6-2.6 m from
                   base_link), lidar (~1.2-2.6 m) or depth (~0.7-2 m, +-40 deg only)
  curtain_inspect:=true  a wall with a suitable gap that isn't a confirmed curtain yet: drive
                   to the middle of the cell in front, square up, LIE DOWN and measure the
                   gap, stand up; 12-20 cm (curtain_gap_min/max) -> walk through
                   (curtain_inspect.py); false = off. curtain_inspect_mode:=pose = Pose-mode
                   look instead of lying down
  curtain_max_gap:=0.15  m: a gap at the bottom at least this big is a hurdle, not a curtain
                   (spec < 15 cm; the lab test curtain hangs ~20 cm up -> 0.25 to test with it)
  scene_trigger:=true  /scene/trigger (+ /junction_detected) when the robot faces a dead end
                   from the cell before it; /scene/junction for the explorer's junction look
                   (maze_nav/scene_understanding/scene_trigger.py)
  creep_distance:=0.10  m straight ahead (no steering) after start, onto the SLAM map and off the
                   back wall (0 = off)
  start_scan:=true turn 30 deg left, 60 right, 30 left after the creep, before exploring
  max_speed:=0.15  band_low:=0.30  band_high:=0.45  floor_z:=nan (measure)
  map_dir:=~/challenge_ws/maps   udp_only:=true   use_sim_time:=false
  fastlio_config:=<maze_nav>/config/fastlio_mid360.yaml  Fast-LIO settings: blind 0.3 m (was
                   0.5: near walls Fast-LIO lost them and its pose slipped ~0.5 m, 2026-11-19),
                   0.3 m voxels, fixed lidar-IMU extrinsic. The lab's original:
                   fastlio_config:=$(ros2 pkg prefix lite3_launcher)/share/lite3_launcher/config/mid360.yaml
  lidar_relay:=true  /livox/points crosses the bridge ONCE (lidar_relay -> /livox/points_local
                   for wall_scan, cmd_vel_bridge, stair_detector, curtain_detector). Each
                   bridge copy costs: 6 copies put the lidar ~1.1 s late and starved the IMU
                   (2026-11-19). false = every node subscribes /livox/points itself
  domain_id:=auto  auto = 4 (the robot's domain, ROS_LOCALHOST_ONLY=0) with bringup:=true,
                   whatever the terminal has; with bringup:=false (simulator) the terminal's
                   ROS_DOMAIN_ID. A number forces that domain
  log_tags:=false  terminal only: drop `[INFO]` and the timestamps (`[WARN]`/`[ERROR]` stay,
                   coloured); true = ROS's full `[INFO] [<time>] [node]: msg`. Log files are
                   always full
  (startup)        everything starts at once; ~4 s later the log says `startup done (+4 s):
                   N processes running (...)` (running, not ready: Nav2 activates a few s
                   later, PAUSED = ready for /maze/start)
  verbose:=true    false = Nav2 and slam_toolbox log warnings/errors only (no lifecycle and
                   per-goal chatter); maze_explorer, wall_scan and cmd_vel_bridge keep INFO
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

NAV2_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
              'velocity_smoother']


ROBOT_DOMAIN = '4'   # lite3_domain_bridge_4, RViz: robot_info.md


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
        launch.logging.get_logger('maze_nav.preflight'), perception=flag('perception'))
    return []


RAW_DEPTH, RAW_INFO = '/camera/depth/image_raw', '/camera/depth/camera_info'
RELAY_DEPTH, RELAY_INFO = '/camera/depth_relay/image_raw', '/camera/depth_relay/camera_info'


def depth_users(context):
    """team_perception (perception:=true) and stair_detector (stairs:=true), sharing ONE
    depth_relay: the raw depth comes from lite3_domain_bridge_4 (another VM), which sends a
    full 18 MB/s copy to every subscribing process. The relay takes one and passes 10 frames a
    second on (team_perception/depth_relay_node.py). Without team_perception built there is no
    relay and stair_detector reads the raw depth itself."""
    import launch.logging
    log = launch.logging.get_logger('maze_nav')
    flag = lambda n: LaunchConfiguration(n).perform(context).lower() in ('true', '1')  # noqa: E731
    perception, stairs, curtains = flag('perception'), flag('stairs'), flag('curtains')
    try:
        share = get_package_share_directory('team_perception')
    except Exception:                               # not built: navigate anyway
        share = None
        if perception:
            log.warning('team_perception not found (colcon build --packages-select '
                        'team_perception); running WITHOUT perception')
        perception = False
    out = []
    depth, info = RAW_DEPTH, RAW_INFO
    if share is not None and (perception or stairs or curtains):
        out.append(Node(package='team_perception', executable='depth_relay', name='depth_relay',
                        output='both',
                        parameters=[{'in_depth_topic': RAW_DEPTH, 'in_info_topic': RAW_INFO,
                                     'out_depth_topic': RELAY_DEPTH,
                                     'out_info_topic': RELAY_INFO, 'rate_hz': 10.0}]))
        depth, info = RELAY_DEPTH, RELAY_INFO
    if perception:
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py')),
            launch_arguments={'depth_relay': 'false',          # ours above
                              'depth_topic': depth,
                              'depth_camera_info_topic': info}.items()))
    if stairs:
        out.append(Node(
            package='maze_nav', executable='stair_detector', name='stair_detector',
            output='both',
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                        value_type=bool),
                         'drive': ParameterValue(LaunchConfiguration('go'), value_type=bool),
                         'lidar_topic': cloud_topic().perform(context),
                         'depth_topic': depth, 'depth_camera_info_topic': info}]))
    if curtains:
        out.append(Node(
            package='maze_nav', executable='curtain_detector', name='curtain_detector',
            output='both',
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                        value_type=bool),
                         'drive': ParameterValue(LaunchConfiguration('go'), value_type=bool),
                         'max_speed': ParameterValue(LaunchConfiguration('max_speed'),
                                                     value_type=float),
                         # Detection (0.6-2 m) and covered-lens check; only through the
                         # relay (never a 2nd raw copy). No relay -> lidar only.
                         'sensors': LaunchConfiguration('curtain_sensors'),
                         'cloud_topic': cloud_topic().perform(context),
                         'max_gap': ParameterValue(LaunchConfiguration('curtain_max_gap'),
                                                   value_type=float),
                         'inspect': ParameterValue(LaunchConfiguration('curtain_inspect'),
                                                   value_type=bool),
                         'inspect_mode': LaunchConfiguration('curtain_inspect_mode'),
                         'gap_min': ParameterValue(LaunchConfiguration('curtain_gap_min'),
                                                   value_type=float),
                         'gap_max': ParameterValue(LaunchConfiguration('curtain_gap_max'),
                                                   value_type=float),
                         # same wall band as wall_scan (curtain points drop out of it)
                         'band_low': ParameterValue(LaunchConfiguration('band_low'),
                                                    value_type=float),
                         'band_high': ParameterValue(LaunchConfiguration('band_high'),
                                                     value_type=float),
                         'depth_topic': depth if depth == RELAY_DEPTH else '',
                         'depth_camera_info_topic': info if depth == RELAY_DEPTH else ''}]))
    return out


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
    'lidar_relay': 'no /livox/points_local: wall_scan (walls), stair/curtain_detector go blind '
                   'and cmd_vel_bridge stops the robot (lidar stale). Ctrl+C, or relaunch '
                   'with lidar_relay:=false.',
    'wall_scan': 'no /wall_scan: SLAM and costmaps get no walls. Ctrl+C.',
    'cmd_vel_bridge': 'nothing reaches /cmd_vel; the robot stops. Ctrl+C.',
    'maze_explorer': 'no more goals; Nav2 finishes the current one. Ctrl+C.',
    'qr_scanner': 'perception only: no more QR codes. Navigation carries on.',
    'cpu_monitor': 'CPU logging only. Navigation carries on.',
    'curtain_detector': 'no curtain pass-through: new curtains stay walls (ones already found '
                        'stay cleared). If it died while driving, cmd_vel_bridge stops the '
                        'robot (idle).',
    'scene_trigger': 'scene understanding only: no more /scene/trigger. Navigation carries on.',
    'depth_relay': 'no depth: no QR positions, no stair line-up. Navigation '
                   'carries on.',
    'stair_detector': 'no stair line-up: Nav2 drives over the stairs as floor. If it died '
                      'while turning the robot, cmd_vel_bridge stops it (idle).',
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


# Startup summary: every process that starts is noted, and ~4 s after launch one line says
# how many run. "Running" = process up, not "ready" (Nav2 activates a few s later).
_STARTED = []


def startup_summary(settle=4.0):
    """OnProcessStart handler noting each process, and the 'startup done' line after `settle` s."""
    def on_start(event, _context):
        _STARTED.append(re.sub(r'-\d+$', '', event.action.name))   # planner_server-5 -> ...

    def report(_context):
        n = len(_STARTED)
        return [LogInfo(msg=f'startup done (+{settle:.0f} s): {n} process'
                            f'{"" if n == 1 else "es"} running ({", ".join(sorted(_STARTED))})')]
    return [RegisterEventHandler(OnProcessStart(on_start=on_start)),
            TimerAction(period=settle, actions=[OpaqueFunction(function=report)])]


def cloud_topic():
    """/livox/points_local (lidar_relay:=true) or /livox/points."""
    return PythonExpression(["'/livox/points_local' if '", LaunchConfiguration('lidar_relay'),
                             "'.lower() in ('true', '1') else '/livox/points'"])


def generate_launch_description():
    config = os.path.join(get_package_share_directory('maze_nav'), 'config')
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}
    pose_max_yaw = ParameterValue(PythonExpression(['__import__("math").radians(',
                                                    arg('pose_max_yaw_deg'), ')']),
                                  value_type=float)
    pose_max_pitch = ParameterValue(PythonExpression(['__import__("math").radians(',
                                                      arg('pose_max_pitch_deg'), ')']),
                                    value_type=float)
    # Log level for the third-party nodes (Nav2, slam_toolbox); ours always log INFO.
    quiet_level = PythonExpression(
        ["'info' if '", arg('verbose'), "'.lower() in ('true', '1') else 'warn'"])
    log_args = ['--ros-args', '--log-level', quiet_level]
    # Every lidar cloud reader takes the relay's local copy (lidar_relay.py): one copy from the
    # bridge instead of one per process. Fast-LIO keeps its own /livox/lidar.
    cloud = ParameterValue(cloud_topic(), value_type=str)

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
        return Node(package=package, executable=executable, name=executable, output='both',
                    parameters=[nav2_params, sim_time, extra.get(executable, {})],
                    remappings=list(remappings), arguments=log_args)

    lite3_base = PathJoinSubstitution([FindPackageShare('lite3_launcher'), 'launch', 'include',
                                       'Base'])

    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='true',
                              description='Start Fast-LIO and odom_relay (lite3_launcher)'),
        DeclareLaunchArgument('go', default_value='true',
                              description='false = explorer only publishes /maze/goal'),
        DeclareLaunchArgument('pose_looks', default_value='true',
                              description='dead-end side walls: yaw the body in Pose mode (feet '
                              'still) instead of Nav2 spins; false = spins (the start scan always spins)'),
        DeclareLaunchArgument('pose_max_yaw_deg', default_value='20.0',
                              description='body yaw at full Pose-mode axis (not measured yet; '
                              'cmd_vel_bridge logs what /odom saw after each look)'),
        DeclareLaunchArgument('curtain_inspect', default_value='true',
                              description='stop and inspect possible curtains (lie down and '
                              'measure the gap) before deciding'),
        DeclareLaunchArgument('curtain_inspect_mode', default_value='lie',
                              description='lie = lie down to measure the gap; pose = Pose-mode '
                              'look (pitch forward, yaw both ways)'),
        DeclareLaunchArgument('curtain_gap_min', default_value='0.12',
                              description='m: smallest gap measured lying down that confirms a '
                              'curtain'),
        DeclareLaunchArgument('curtain_gap_max', default_value='0.20',
                              description='m: largest gap measured lying down that confirms a '
                              'curtain'),
        DeclareLaunchArgument('pose_max_pitch_deg', default_value='20.0',
                              description='body pitch (nose down) at full Pose-mode axis, for '
                              'the curtain inspection (not measured yet)'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='false = nothing is sent to /cmd_vel'),
        DeclareLaunchArgument('gait', default_value='hstep',
                              description='sent once on /simple_cmd at startup (after Move mode): '
                                          'hstep = H-Step (high-stepping RUG gait); rug = RUG '
                                          'gait General gear; none = leave it'),
        DeclareLaunchArgument('perception', default_value='true',
                              description='also start team_perception robot.launch.py '
                                          '(QR codes)'),
        DeclareLaunchArgument('auto_stand', default_value='true',
                              description='robot lying at launch: send stand once, before gait'),
        DeclareLaunchArgument('wait_for_start', default_value='true',
                              description='hold still once ready until /maze/start (std_msgs/Empty)'),
        DeclareLaunchArgument('stairs', default_value='true',
                              description='stair_detector: face the stairs square and stop '
                                          'in front of them, hold until /maze/start'),
        DeclareLaunchArgument('curtains', default_value='true',
                              description='curtain_detector: clear curtains (wall with a low '
                                          'gap at the bottom) from the map and walk through '
                                          'them'),
        DeclareLaunchArgument('curtain_max_gap', default_value='0.15',
                              description='curtain detection: m, largest gap at the bottom of a '
                                          'curtain (spec 0.15; bigger = hurdle)'),
        DeclareLaunchArgument('curtain_sensors', default_value='both',
                              description='curtain detection: both (~0.6-2.6 m from base_link), '
                                          'lidar (~1.2-2.6 m) or depth (~0.7-2 m, +-40 deg)'),
        DeclareLaunchArgument('scene_trigger', default_value='true',
                              description='/scene/trigger (+ /junction_detected) at dead ends; '
                                          '/scene/junction for the junction look'),
        DeclareLaunchArgument('cpu_log', default_value='true',
                              description='log CPU per process to ~/challenge_ws/logs/cpu/'),
        DeclareLaunchArgument('creep_distance', default_value='0.10',
                              description='m straight ahead after start, no steering; 0 = off'),
        DeclareLaunchArgument('start_scan', default_value='true',
                              description='after the creep: turn 30 deg left, 60 right, 30 left (back to the start heading; Nav2 spins, not Pose mode)'),
        DeclareLaunchArgument('max_speed', default_value='0.15',
                              description='m/s (clamped to 0.25 by cmd_vel_bridge)'),
        DeclareLaunchArgument('band_low', default_value='0.30',
                              description='m above the floor; below = terrain/stairs, ignored'),
        DeclareLaunchArgument('band_high', default_value='0.45',
                              description='m above the floor; at most the wall top (0.45); see nav_launch.md'),
        DeclareLaunchArgument('floor_z', default_value='nan',
                              description='odom z of the floor; nan = measure at startup'),
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/challenge_ws/maps')),
        DeclareLaunchArgument('udp_only', default_value='true',
                              description='UDP-only Fast DDS profile'),
        DeclareLaunchArgument('fastlio_config',
                              default_value=os.path.join(config, 'fastlio_mid360.yaml'),
                              description='Fast-LIO parameter file. Default: blind 0.3 m, '
                                          '0.3 m voxels, fixed extrinsic (better near walls). '
                                          "The lab's original: <lite3_launcher share>/config/"
                                          'mid360.yaml'),
        DeclareLaunchArgument('lidar_relay', default_value='true',
                              description='lidar_relay: /livox/points crosses the bridge once '
                                          '(/livox/points_local for wall_scan, cmd_vel_bridge, '
                                          'stair/curtain_detector); false = each its own copy'),
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
        # Maths / image thread pools at 1 thread in every node: idle OpenMP / OpenBLAS / cv2
        # pools otherwise spin on all cores for each small numpy or cv2 call (system time),
        # and the lidar then reached this computer ~1 s late. Fast-LIO sets its own count
        # (omp_set_num_threads(3)); torch / OpenVINO in human_detector set theirs too.
        *[SetEnvironmentVariable(k, '1') for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                   'MKL_NUM_THREADS', 'OPENCV_FOR_THREADS_NUM')],
        OpaqueFunction(function=quiet_launch_log),
        OpaqueFunction(function=ros_domain),        # before preflight and every node
        OpaqueFunction(function=preflight),
        *death_banners(),
        *startup_summary(),

        # Fast-LIO + odom_relay only; lite3_bringup would also start a /scan we don't use.
        # Our Fast-LIO settings (config/fastlio_mid360.yaml: blind 0.3, 0.3 m voxels, fixed
        # extrinsic). lite3_launcher joins config_path + config_file; '/' + an absolute path
        # is that path, so fastlio_config can be any file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([lite3_base,
                                                                'fast_lio.launch.py'])),
            launch_arguments={'config_path': '/',
                              'config_file': arg('fastlio_config')}.items(),
            condition=IfCondition(arg('bringup'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([lite3_base,
                                                                'odom_relay.launch.py'])),
            condition=IfCondition(arg('bringup'))),
        Node(package='maze_nav', executable='cpu_monitor', name='cpu_monitor', output='both',
             condition=IfCondition(arg('cpu_log'))),
        Node(package='maze_nav', executable='lidar_relay', name='lidar_relay', output='both',
             condition=IfCondition(arg('lidar_relay'))),

        # cmd_vel_bridge with wall_scan: its stand_wait counts from its start and needs
        # wall_scan's /maze/body_height.
        Node(package='maze_nav', executable='wall_scan', name='wall_scan', output='both',
             parameters=[sim_time, {
                 'band_low': ParameterValue(arg('band_low'), value_type=float),
                 'band_high': ParameterValue(arg('band_high'), value_type=float),
                 'floor_z': ParameterValue(arg('floor_z'), value_type=float),
                 'cloud_topic': cloud,
                 'curtains_topic': ParameterValue(PythonExpression(
                     ["'/curtain/curtains' if '", arg('curtains'), "'.lower() in ('true', '1') "
                      "else ''"]), value_type=str),
             }]),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='both', arguments=log_args,
             parameters=[os.path.join(config, 'slam_toolbox.yaml'), sim_time]),
        Node(package='maze_nav', executable='cmd_vel_bridge', name='cmd_vel_bridge',
             output='both',
             parameters=[sim_time, {'drive': ParameterValue(arg('drive'), value_type=bool),
                                    'cloud_topic': cloud,     # "lidar alive" check only
                                    'gait': arg('gait'),
                                    'auto_stand': ParameterValue(arg('auto_stand'),
                                                                 value_type=bool),
                                    'pose_max_yaw': pose_max_yaw,
                                    'pose_max_pitch': pose_max_pitch}]),

        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'nav_cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='both', arguments=log_args,
             parameters=[{'autostart': True, 'node_names': NAV2_NODES,
                          'bond_timeout': 10.0}, sim_time]),

        Node(package='maze_nav', executable='scene_trigger', name='scene_trigger',
             output='both', parameters=[sim_time], condition=IfCondition(arg('scene_trigger'))),

        Node(package='maze_nav', executable='maze_explorer', name='maze_explorer',
             output='both',
             parameters=[sim_time, {
                 'go': ParameterValue(arg('go'), value_type=bool),
                 'wait_for_start': ParameterValue(arg('wait_for_start'), value_type=bool),
                 'map_dir': arg('map_dir'),
                 'creep_distance': ParameterValue(arg('creep_distance'), value_type=float),
                 'start_scan': ParameterValue(arg('start_scan'), value_type=bool),
                 'pose_looks': ParameterValue(arg('pose_looks'), value_type=bool),
                 'pose_max_yaw': pose_max_yaw,
                 # Switched-off features are disconnected, so a stray publisher on the
                 # domain (an old stair_detector, a test rig) can't stop the robot.
                 'stairs_topic': ParameterValue(PythonExpression(
                     ["'/stairs/status' if '", arg('stairs'), "'.lower() in ('true', '1') "
                      "else ''"]), value_type=str),
                 'look_topic': ParameterValue(PythonExpression(
                     ["'/perception/look_request' if '", arg('perception'),
                      "'.lower() in ('true', '1') else ''"]), value_type=str),
                 'curtain_topic': ParameterValue(PythonExpression(
                     ["'/curtain/status' if '", arg('curtains'), "'.lower() in ('true', '1') "
                      "else ''"]), value_type=str),
                 'curtains_topic': ParameterValue(PythonExpression(
                     ["'/curtain/curtains' if '", arg('curtains'), "'.lower() in ('true', '1') "
                      "else ''"]), value_type=str),
                 'qr_unread_topic': ParameterValue(PythonExpression(
                     ["'/qr/unread' if '", arg('perception'),
                      "'.lower() in ('true', '1') else ''"]), value_type=str),
             }]),

        OpaqueFunction(function=depth_users),
    ])
