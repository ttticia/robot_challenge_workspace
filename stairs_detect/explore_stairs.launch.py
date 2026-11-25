"""explore_stairs.launch.py: Rey's copy of maze_explore/launch/explore.launch.py (team code,
2026-11-25). Identical, except that the explorer is ~/stairs_detect/explorer_stairs.py (see
its header) instead of maze_explore's maze_explorer, and two extra arguments: stair_rails, ai_auto.

    ros2 launch ~/stairs_detect/explore_stairs.launch.py [drive:=false auto_start:=true] ...
"""
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
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
    'gait': ('hstep', 'walking gait: hstep (H-Step, high stepping) or rug (RUG general gear)'),
    'scene_understanding': ('true', 'analyse stitched room and object looks with the VLM'),
    'scene_camera_topic': ('/camera/color/image_raw', 'RGB image topic for the VLM'),
    'report': ('true', 'task_reporter: turn room results and QR reads into the mission '
                       'reports and POST them to the scoring server'),
    'report_url': ('http://192.168.10.141:8001/task_report_A',
                   'scoring server (task_report_A = left arena)'),
    'group_id': ('group04_grp4', 'our id on the scoring server'),
    'm01_object': ('hand_tools', 'the object mission M01 asks for (medical_equipment, '
                                 'hand_tools, food, water, utensils)'),
    'm02_hazard': ('', 'the hazard mission M02 asks for; empty = whichever sign is found'),
    'qr': ('true', 'qr_scanner (team_perception): reads QR codes while the robot holds a room '
                   'or object look, at the same time as scene understanding; the text goes '
                   'to task_reporter'),
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
    'curtain_cv': ('false', 'true: curtain_node looks for curtains in the colour camera '
                            'outside room checks; the explorer verifies each sighting and '
                            'then takes that edge as a curtain even if it maps as a wall'),
    'plain': ('false', 'true: navigation only = scene_understanding, stairs, curtains and '
                       'terrain all off, whatever their own arguments say'),
    'v_max': ('0.20', 'top walking speed, m/s'),
    'stair_rails': ('false', 'stairs copy: handrails must also show in the lidar'),
    'ai_speed': ('0.25', 'stairs copy: walking speed on the stairs in AI mode, m/s'),
    'ai_wz': ('0.15', 'stairs copy: most yaw correction in the AI walk, rad/s (0 = none)'),
    'ai_auto': ('true', 'stairs copy: send AUTO after the AI gait so /cmd_vel drives the stairs'),
    'stand_height': ('0.30', 'base_link height used when the floor cannot be measured'),
}


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
            # stairs copy: our explorer copy in place of maze_explorer, same parameters
            (explorer := ExecuteProcess(
                 name='maze_explorer', output='screen', additional_env=env,
                 condition=IfCondition(c['explore']),
                 cmd=['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              'explorer_stairs.py'),
                      '--ros-args', '-r', '__node:=maze_explorer',
                      '-p', ['drive:=', c['drive']], '-p', ['auto_start:=', c['auto_start']],
                      '-p', ['gait:=', c['gait']], '-p', ['enter_rooms:=', c['enter_rooms']],
                      '-p', ['stair_check:=', c['stair_check']],
                      '-p', ['stairs:=', PythonExpression(unless_plain('stairs'))],
                      '-p', ['curtains:=', PythonExpression(unless_plain('curtains'))],
                      '-p', ['terrain:=', PythonExpression(unless_plain('terrain'))],
                      '-p', ['curtain_cv:=', PythonExpression(unless_plain('curtain_cv'))],
                      '-p', ['push_stop:=', c['push_stop']], '-p', ['v_max:=', c['v_max']],
                      '-p', ['stair_rails:=', c['stair_rails']], '-p', ['ai_auto:=', c['ai_auto']],
                      '-p', ['ai_speed:=', c['ai_speed']],
                      '-p', ['ai_wz:=', c['ai_wz']]])),
            RegisterEventHandler(OnProcessExit(target_action=explorer, on_exit=[
                EmitEvent(event=SignalProcess(signal_number=signal.SIGINT,
                                              process_matcher=matches_action(qr)))])),
        ])
