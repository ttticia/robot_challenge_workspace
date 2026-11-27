"""The whole stack.

    ros2 launch maze_explore explore.launch.py                 # robot
    ros2 launch maze_explore explore.launch.py explore:=false  # mapping only (hand-driven runs)
    ros2 launch maze_explore explore.launch.py drive:=false    # explorer decides, sends nothing
    ros2 launch maze_explore explore.launch.py plain:=true     # navigation only: no scene
                                    # understanding trigger, stairs, curtains or terrain handling

  fast_lio + odom_relay   lidar odometry: /odom, odom -> base_link   (fastlio:=false in the sim)
  wall_scan               levelled lidar slice: /scan, /maze/cloud, odom -> base_level
  slam_toolbox            /scan -> /map, map -> odom
  maze_explorer           decides, drives (/cmd_vel) and switches modes (/simple_cmd)

Start it with the robot standing still, centred in the start cell, facing into the maze.
The explorer waits for:  ros2 topic pub --once /nav/start std_msgs/msg/Empty
"""
import os
import re
import signal
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.logging
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import matches_action
from launch.events.process import SignalProcess
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
    'gait': ('hstep', 'gait on uneven floor: hstep (RUG H-Step) or normal'),
    'scene_understanding': ('true', 'analyse stitched room and object looks with the VLM'),
    'scene_camera_topic': ('/camera/color/image_raw', 'RGB image topic for the VLM'),
    'report': ('true', 'task_reporter: turn room results and QR reads into the mission '
                       'reports and POST them to the scoring server'),
    'report_url': ('http://192.168.10.141:8001/task_report',
                   'scoring server. The course follows maze_side: L = ..._A, R = ..._B. '
                   'An address ending in _A or _B is used as it is'),
    'group_id': ('group04_grp4', 'our id on the scoring server'),
    'm01_object': ('hand_tools', 'the object mission M01 asks for (medical_equipment, '
                                 'hand_tools, food, water, utensils)'),
    'm02_hazard': ('', 'the hazard mission M02 asks for; empty = whichever sign is found'),
    'qr': ('true', 'qr_scanner (team_perception): reads QR codes while the robot holds a room '
                   'or object look, at the same time as scene understanding; the text goes '
                   'to task_reporter'),
    'qr_approach': ('true', 'when scene understanding sees a QR code in the left / right '
                            'view of a look: turn 75 deg to that side, step into the cell '
                            'sideways until the code is read, step back (needs '
                            'scene_understanding and qr)'),
    'scene_image_transport': ('compressed', 'compressed = <scene_camera_topic>/compressed '
                                            '(JPEG); raw only where no compressed topic exists'),
    'enter_rooms': ('false', 'walk into rooms instead of looking in from the cell before'),
    'stair_check': ('true', 'confirm stairs with the depth camera before crossing (depth is '
                            'only subscribed for the ~3 s of each check)'),
    'stairs': ('true', 'false: no stair detection and no crossing (stairs = a blocked cell, '
                       'no depth check, no AI mode)'),
    'curtains': ('true', 'false: a curtain is a wall (not faced, not pushed)'),
    'terrain': ('true', 'false: uneven cells are walked into head first, not sideways'),
    'push_stop': ('true', 'obstacle stop stays on while going through a curtain: the fabric '
                          'then stops the robot. false = really go through'),
    'look_check': ('true', 'before a room / object look: check for stairs (and, with '
                           'curtain_cv, a curtain) and call the look off if one is verified'),
    'curtain_cv': ('true', 'true: curtain_node looks for curtains in the colour camera '
                            'outside room checks; the explorer verifies each sighting and '
                            'then takes that edge as a curtain even if it maps as a wall'),
    'plain': ('false', 'true: navigation only = scene_understanding, stairs, curtains and '
                       'terrain all off, whatever their own arguments say'),
    'flat_gait': ('normal', 'gait on flat floor: normal (Move mode normal gait) or hstep. '
                               '`gait` is then the one for uneven terrain'),
    'gait_switch': ('true', 'false: `gait` for the whole run, no switching by floor'),
    'maze_rows': ('5', 'rows of the maze along the start heading; the end cell is the one '
                       'cell of the row after. 0 = find the exit from the map alone'),
    'start_stairs': ('false', 'true: one FULL stair check of the way out before leaving the '
                              'start cell (the detector runs whatever the lidar shows)'),
    'stair_rails': ('false', 'handrails must also show in the lidar'),
    'stair_min_hits': ('5', 'stair check also passes with this many hits in the whole look'),
    'stair_check_time': ('4.5', 'seconds the stair check looks'),
    'ai_wz': ('0.0', 'most IMU yaw correction in the AI walk, rad/s (0 = none)'),
    'ai_walk_cells': ('1.5', 'cells from the centre of the cell before to the border of the cell after'),
    'ai_body_clear': ('0.35', 'm walked past that border before AI mode ends'),
    'ai_distance': ('time', 'how the AI walk counts distance: time (speed x time) or leg (leg odometry)'),
    'ai_wall_min': ('1.0', 'm walked before a wall just ahead ends the AI walk'),
    'ai_auto': ('true', 'send AUTO after the AI gait so /cmd_vel drives the stairs'),
    'ai_reanchor': ('true', 'after the stairs, move the cell grid to where the walk put the robot'),
    'reanchor_min': ('0.15', 'm of pose error before the grid is re-anchored'),
    'maze_side': ('', 'maze on the right (R) or on the left (L) of the start cell; asked for '
                      'in the terminal when not given here'),
    'heat_check': ('true', 'false: no heat / load check of this computer before the launch'),
    'floor_check': ('false', 'true: a depth-camera look at the floor before walking into each new '
                             'cell (it slows every stream from the robot for 1-3 s each time)'),
    'stand_height': ('0.30', 'base_link height used when the floor cannot be measured'),
}


def name_node_logs(log_dir):
    """Rename <exe>_<pid>_<ms>.log in log_dir after the node that pid belongs to, from the
    'process started with pid' lines of launch.log. Open files can be renamed on Linux."""
    try:
        text = open(os.path.join(log_dir, 'launch.log'), errors='replace').read()
        pids = {pid: re.sub(r'\.py$', '', name) for name, pid in re.findall(
            r'\[([^\]]+?)-\d+\]: process started with pid \[(\d+)\]', text)}
        for f in os.listdir(log_dir):
            m = re.fullmatch(r'.+?_(\d+)_\d{13}\.log', f)
            if m and m.group(1) in pids:
                new = os.path.join(log_dir, pids[m.group(1)] + '.log')
                if not os.path.exists(new):
                    os.rename(os.path.join(log_dir, f), new)
    except OSError:
        pass                          # never let tidying the logs stop a shutdown
    return []


def ask_maze_side():
    """R or L (user, 2026-11-26: asked at the start of every launch, saved as maze_side).
    maze_side:=R|L on the command line answers it. Not asked where nobody can answer or it
    is not needed: --show-args, mapping only. No terminal to ask in (a script) = an error."""
    given = [a.split(':=', 1)[1] for a in sys.argv if a.startswith('maze_side:=')]
    if given:
        side = given[-1].strip().upper()
        if side not in ('R', 'L'):
            raise RuntimeError(f"maze_side:={given[-1]}: must be R or L. Nothing was started.")
        return side
    if any(a in ('-s', '--show-args', '--show-arguments', 'explore:=false') for a in sys.argv):
        return ''
    if not sys.stdin.isatty():
        raise RuntimeError('maze_side is not given and there is no terminal to ask in. Nothing '
                           'was started. Launch again with maze_side:=R or maze_side:=L.')
    while True:
        side = input('Is the maze on the right (R) or on the left (L)? ').strip().upper()
        if side in ('R', 'L'):
            return side
        print('Please type R or L.')


def heat_gate():
    """Before anything starts (user, 2026-11-26): is this computer cool and idle enough?
    Hot (75 C or more), being throttled, or RViz / a simulator / a test run going = the
    reasons are printed and the launch asks before it goes on (no terminal: it only
    warns). Under load the processor hits 100 C within seconds and is throttled, Fast-LIO
    falls behind and the odometry jumps. heat_check:=false skips it; not made for
    explore:=false or --show-args."""
    if any(a in ('-s', '--show-args', '--show-arguments', 'explore:=false', 'heat_check:=false')
           for a in sys.argv):
        return
    try:
        from maze_explore import heat
        ok, lines = heat.check()
    except Exception as e:                       # never let the check itself stop a run
        print(f'heat check skipped: {e}')
        return
    print('this computer:')
    print('\n'.join(lines))
    if ok:
        return
    if not sys.stdin.isatty():
        print('WARNING: starting anyway (no terminal to ask at). Expect a late sensor feed.')
        return
    answer = input('NO-GO for this computer. Start anyway? [y/N] ').strip().lower()
    if not answer.startswith('y'):
        raise RuntimeError('Not started: this computer is too hot or too busy. Nothing was started.')


def generate_launch_description():
    # Every package this file starts must be known to this terminal BEFORE anything is
    # started. A launch that fails part-way (11:34 on 2026-11-24: task_reporter was built
    # after the terminal had sourced the workspace) leaves the nodes it had already started
    # running with no parent, and such a leftover qr_scanner made explore_check say NO-GO.
    for package in ('maze_explore', 'scene_understanding', 'stairs', 'task_reporter', 'team_perception',
                    'fast_lio', 'lite3_launcher', 'slam_toolbox'):
        try:
            get_package_share_directory(package)
        except Exception:
            raise RuntimeError(
                f"package '{package}' is not known in this terminal. Nothing was started. "
                'Run: source ~/challenge_ws/install/setup.bash (after colcon build '
                '--symlink-install if it was never built), then launch again.') from None
    ARGS['maze_side'] = (ask_maze_side(), ARGS['maze_side'][1])
    heat_gate()
    config = os.path.join(get_package_share_directory('maze_explore'), 'config')
    c = {k: LaunchConfiguration(k) for k in ARGS}
    env = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'}

    def val(name, kind):
        return ParameterValue(c[name], value_type=kind)

    def unless_plain(name):
        """Python expression (text): the switch `name` is on and plain is off."""
        return ["('", c[name], "'.lower() == 'true' and '", c['plain'], "'.lower() != 'true')"]

    def feature(name):
        return ParameterValue(PythonExpression(unless_plain(name)), value_type=bool)
    # One folder per launch (user, 2026-11-25): every node writes its log into this launch's
    # own folder (~/challenge_ws/logs/ros/<date>-<time>-...), next to launch.log, and at the
    # end the files are renamed after the node (python3_34638_....log -> maze_explorer.log).
    log_dir = launch.logging.launch_config.log_dir
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()] + [
            SetEnvironmentVariable('ROS_LOG_DIR', log_dir),
            # this computer's temperature and throttling, one line every 2 s, into the run's
            # log folder (heat.csv): after a fault it shows whether the computer was the cause
            ExecuteProcess(cmd=['python3', '-m', 'maze_explore.heat', '--log',
                                os.path.join(log_dir, 'heat.csv')],
                           name='heat_log', output='log', condition=IfCondition(c['explore'])),
            RegisterEventHandler(OnShutdown(on_shutdown=[
                OpaqueFunction(function=lambda context: name_node_logs(log_dir))])),
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
                     "'", c['explore'], "'.lower() == 'true' and ",
                     *unless_plain('scene_understanding'),
                 ])),
                 parameters=[{'enable_vlm': True, 'explore_event_topic': '/explore/event',
                              'junction_scan_enabled': False,
                              # compressed: a raw image subscriber slows the lidar IMU
                              # on its way here and Fast-LIO diverges (08:19, 08:37 runs)
                              'image_topic': c['scene_camera_topic'],
                              'image_transport': c['scene_image_transport'],
                              'log_rate': False,
                              'save_dir': os.path.expanduser('~/challenge_ws/logs/scene')}]),
            # Reads QR codes during the same looks, on its own (no VLM): camera subscribed
            # only while the explorer's status shows a look, full scans on every frame then.
            # Do not also run team_perception's robot.launch.py: that is a second scanner.
            # It ends with maze_explore (user, 2026-11-24): interrupted when the explorer's
            # process exits (handler at the end), and EXIT_WITH_PARENT makes it stop when
            # this launch dies in any way, a crash or a kill -9 included (two scanners were
            # left running on domain 4 by launches that died at start-up, 11:35).
            (qr := Node(package='team_perception', executable='qr_scanner', name='qr_scanner',
                 exec_name='qr_scanner', output='screen', prefix='nice -n 10',
                 additional_env=dict(env, OPENCV_FOR_THREADS_NUM='1',
                                     EXIT_WITH_PARENT=str(os.getpid())),
                 condition=IfCondition(PythonExpression([
                     "'", c['explore'], "'.lower() == 'true' and ", *unless_plain('qr')])),
                 parameters=[{'image_topic': c['scene_camera_topic'],
                              'image_transport': c['scene_image_transport'],
                              'use_depth': False, 'detect': True, 'read_everywhere': False,
                              'on_demand': True, 'full_scan_every': 1, 'unread_topic': '',
                              'status_topic': '', 'perceive_topic': '',
                              'explore_status_topic': '/explore/status'}])),
            # Its own package. Both of the above send it what they find (/scene/result,
            # /qr/read); it checks, formats and submits the mission reports.
            # respawn: it must not be missing for the rest of a run if something stops it
            # (11:37 on 2026-11-24 it exited mid-run). A restarted one has forgotten earlier
            # sightings; what was already posted stays on the server.
            Node(package='task_reporter', executable='task_reporter', output='screen',
                 respawn=True, respawn_delay=1.0,
                 condition=IfCondition(PythonExpression([
                     "'", c['explore'], "'.lower() == 'true' and '", c['report'],
                     "'.lower() == 'true'"])),
                 parameters=[{'group_id': c['group_id'], 'report_url': c['report_url'],
                              'maze_side': ParameterValue(c['maze_side'], value_type=str),
                              'm01_object': c['m01_object'],
                              'm02_hazard': ParameterValue(c['m02_hazard'], value_type=str),
                              'post': True, 'missions': 'all'}]),
            # Curtains in the colour camera (YOLO), outside room checks only. Off by default.
            Node(package='maze_explore', executable='curtain_node', name='curtain_node',
                 output='screen', prefix='nice -n 10', exec_name='curtain_node',
                 additional_env=dict(env, OPENCV_FOR_THREADS_NUM='1'),
                 condition=IfCondition(PythonExpression([
                     "'", c['explore'], "'.lower() == 'true' and ", *unless_plain('curtain_cv'),
                     " and ", *unless_plain('curtains')])),
                 parameters=[{'image_topic': c['scene_camera_topic']}]),
            (explorer := Node(
                 package='maze_explore', executable='maze_explorer', name='maze_explorer',
                 output='screen', additional_env=env, condition=IfCondition(c['explore']),
                 # speeds.yaml first: all the speeds (walk, terrain, stairs, curtain, QR,
                 # turning) are set there, not by launch arguments
                 parameters=[os.path.join(config, 'speeds.yaml'),
                             {'drive': val('drive', bool), 'auto_start': val('auto_start', bool),
                              'floor_check': val('floor_check', bool),
                              'flat_gait': c['flat_gait'], 'gait_switch': val('gait_switch', bool),
                              'maze_rows': val('maze_rows', int),
                              'maze_side': ParameterValue(c['maze_side'], value_type=str),
                              'gait': c['gait'], 'enter_rooms': val('enter_rooms', bool),
                              'start_stairs': val('start_stairs', bool),
                              'stair_check': val('stair_check', bool),
                              'stairs': feature('stairs'), 'curtains': feature('curtains'),
                              'terrain': feature('terrain'),
                              'curtain_cv': feature('curtain_cv'),
                              'qr_approach': ParameterValue(PythonExpression([
                                  *unless_plain('qr_approach'), ' and ',
                                  *unless_plain('scene_understanding'), ' and ',
                                  *unless_plain('qr')]), value_type=bool),
                              'push_stop': val('push_stop', bool),
                              'look_check': val('look_check', bool),
                              'stair_rails': val('stair_rails', bool),
                              'ai_auto': val('ai_auto', bool),
                              'ai_wz': val('ai_wz', float),
                              'ai_walk_cells': val('ai_walk_cells', float),
                              'ai_body_clear': val('ai_body_clear', float),
                              'ai_distance': c['ai_distance'],
                              'ai_wall_min': val('ai_wall_min', float),
                              'ai_reanchor': val('ai_reanchor', bool),
                              'reanchor_min': val('reanchor_min', float),
                              'stair_min_hits': val('stair_min_hits', int),
                              'stair_check_time': val('stair_check_time', float)}])),
            RegisterEventHandler(OnProcessExit(target_action=explorer, on_exit=[
                EmitEvent(event=SignalProcess(signal_number=signal.SIGINT,
                                              process_matcher=matches_action(qr)))])),
        ])
