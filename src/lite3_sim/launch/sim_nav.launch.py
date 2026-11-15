"""One-command bringup: Gazebo + Lite3 + SLAM + Nav2 + RViz.

ros2 launch lite3_sim sim_nav.launch.py                                    # arena_1
ros2 launch lite3_sim sim_nav.launch.py world:=$(ros2 pkg prefix lite3_sim)/share/lite3_sim/worlds/arena_2.world   # other seed
Args: world, x_pose, y_pose, gui (Gazebo window, default false), rviz (default true),
      rviz_gpu (default false), camera_full (640x480@10Hz, default false), domain_id (default 89),
      nav_cmd_vel_topic (Nav2 final velocity output, default /cmd_vel),
      sensors (full | nav: navigation-only, no camera, lighter lidar; default full)
Topic/frame names match the real Lite3 (robot_info.md); see lite3_world.launch.py.
SLAM, Nav2 and RViz start once spawn_entity has finished (robot is in the world).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _is_spawn(action):
    exe = getattr(action, 'node_executable', None)
    if exe is None:
        try:
            exe = ' '.join(str(c) for c in action.cmd)
        except Exception:
            exe = ''
    return 'spawn_entity' in str(exe)


def _domain_guard(context):
    """Sim always runs isolated: ROS_DOMAIN_ID 89 (never 4, the real robot) + localhost only."""
    dom = LaunchConfiguration('domain_id').perform(context).strip()
    if dom == '4':
        raise RuntimeError('domain_id 4 is the real robot: the sim must not run on it')
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', dom),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')]


def generate_launch_description():
    pkg = get_package_share_directory('lite3_sim')
    nav2 = get_package_share_directory('nav2_bringup')
    slam = get_package_share_directory('slam_toolbox')
    params = os.path.join(pkg, 'config', 'nav2_params_lite3.yaml')
    arena = os.path.join(pkg, 'worlds')  # bundled worlds; world:= still takes any path

    def include(path, args=None, condition=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=(args or {}).items(), condition=condition)

    def nav2_with_output(context):
        """Nav2 only: send its final velocity output to nav_cmd_vel_topic (default /cmd_vel).

        Gazebo's own /cmd_vel subscriber is untouched. Remaps are node-scoped because global
        remap rules take precedence over navigation_launch's per-node ones: velocity_smoother's
        cmd_vel_smoothed (its output) and behavior_server's cmd_vel (spin/backup) move; the
        controller -> cmd_vel_nav -> smoother chain is unchanged.
        """
        nav2_include = include(os.path.join(nav2, 'launch', 'navigation_launch.py'),
                               {'use_sim_time': 'true', 'params_file': params})
        topic = LaunchConfiguration('nav_cmd_vel_topic').perform(context).strip()
        if topic in ('', '/cmd_vel', 'cmd_vel'):
            return [nav2_include]
        return [LogInfo(msg=f'[sim_nav] Nav2 velocity output -> {topic}'),
                GroupAction(scoped=True, actions=[
                    SetRemap(src='velocity_smoother:cmd_vel_smoothed', dst=topic),
                    SetRemap(src='behavior_server:cmd_vel', dst=topic),
                    nav2_include])]

    after_spawn = [
        LogInfo(msg='[sim_nav] robot spawned: starting SLAM, Nav2, RViz'),
        include(os.path.join(slam, 'launch', 'online_async_launch.py'),
                {'use_sim_time': 'true',
                 'slam_params_file': os.path.join(pkg, 'config', 'slam_lite3.yaml')}),
        TimerAction(period=4.0, actions=[OpaqueFunction(function=nav2_with_output)]),
        # RViz's map/costmap shader crashes (SIGSEGV) on WSL's D3D12 GPU driver, so by default
        # RViz renders in software (llvmpipe); Gazebo stays on the GPU. rviz_gpu:=true to undo.
        TimerAction(period=6.0, actions=[GroupAction(scoped=True, actions=[
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                                   condition=UnlessCondition(LaunchConfiguration('rviz_gpu'))),
            include(os.path.join(nav2, 'launch', 'rviz_launch.py'),
                    {'use_sim_time': 'true',
                     'rviz_config': os.path.join(pkg, 'config', 'nav2_lite3.rviz')},
                    condition=IfCondition(LaunchConfiguration('rviz')))])]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(arena, 'arena_1.world')),
        DeclareLaunchArgument('x_pose', default_value='0.75'),
        DeclareLaunchArgument('y_pose', default_value='0.75'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_gpu', default_value='false'),
        DeclareLaunchArgument('camera_full', default_value='false',
                              description='true: camera 640x480 @ 10 Hz (default 320x240 @ 5 Hz)'),
        DeclareLaunchArgument('domain_id', default_value='89',
                              description='sim ROS domain (never 4 = real robot); localhost only'),
        DeclareLaunchArgument('nav_cmd_vel_topic', default_value='/cmd_vel',
                              description='Nav2 final velocity output (e.g. /mission/nav_cmd_vel '
                                          'when a velocity gate owns /cmd_vel)'),
        DeclareLaunchArgument('sensors', default_value='full',
                              description='full, or nav = navigation-only (no camera, lighter lidar)'),
        OpaqueFunction(function=_domain_guard),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', arena),

        RegisterEventHandler(OnProcessExit(target_action=_is_spawn, on_exit=after_spawn)),

        include(os.path.join(pkg, 'launch', 'lite3_world.launch.py'), {
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'gui': LaunchConfiguration('gui'),
            'camera_full': LaunchConfiguration('camera_full'),
            'sensors': LaunchConfiguration('sensors'),
            'domain_id': LaunchConfiguration('domain_id')}),
    ])
