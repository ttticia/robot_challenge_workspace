"""Gazebo Classic + Lite3 (frozen legs, planar base) with the real robot's topic and frame names.

ros2 launch lite3_sim lite3_world.launch.py [world:=...] [gui:=true] [x_pose:=..] [y_pose:=..]
                                            [camera_full:=true] [domain_id:=89] [sensors:=full|nav]
Topics: /livox/points, /scan, /camera/color/image_raw(+/compressed), /camera/color/camera_info,
        /camera/depth/image_raw (16UC1 mm), /camera/depth/camera_info, /camera/depth/points,
        /imu/data, /odom, /cmd_vel.   TF: odom -> base_link -> livox_frame / camera_link -> ...
Runs on ROS_DOMAIN_ID 89 + ROS_LOCALHOST_ONLY=1 so it can never reach the real robot (domain 4).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def domain_guard(context):
    dom = LaunchConfiguration('domain_id').perform(context)
    if dom.strip() == '4':
        raise RuntimeError('domain_id 4 is the real robot: the sim must not run on it')
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', dom),
               SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')]
    if os.environ.get('ROS_DOMAIN_ID', '0') != dom or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        actions.append(LogInfo(msg=f'[lite3_sim] sim runs on ROS_DOMAIN_ID={dom}, localhost only. This terminal '
                                   f'has ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "unset")}; to see sim '
                                   f'topics elsewhere: export ROS_DOMAIN_ID={dom} ROS_LOCALHOST_ONLY=1'))
    return actions


def robot(context):
    pkg = get_package_share_directory('lite3_sim')
    full = LaunchConfiguration('camera_full').perform(context).lower() in ('true', '1', 'yes')
    sensors = LaunchConfiguration('sensors').perform(context).strip().lower()
    if sensors not in ('full', 'nav'):
        raise RuntimeError("sensors must be 'full' or 'nav'")
    nav_only = sensors == 'nav'
    urdf = 'lite3_sim_nav.urdf' if nav_only else ('lite3_sim_full.urdf' if full else 'lite3_sim.urdf')
    with open(os.path.join(pkg, 'urdf', urdf)) as f:
        description = f.read()
    with open(os.path.join(pkg, 'urdf', 'base_height.txt')) as f:
        z = f.read().strip()
    sim_time = {'use_sim_time': True}
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[sim_time, {'robot_description': description}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'lite3', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x_pose').perform(context),
                        '-y', LaunchConfiguration('y_pose').perform(context),
                        '-z', z, '-timeout', '120']),
        # same /scan as the lab launcher: 0.1-0.5 m above base_link, 0.4-20 m, 0.5 deg, 360 deg
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', output='screen',
             remappings=[('cloud_in', '/livox/points'), ('scan', '/scan')],
             parameters=[sim_time, {
                 'target_frame': 'base_link', 'transform_tolerance': 0.05,
                 'min_height': 0.1, 'max_height': 0.5,
                 'angle_min': -3.14159, 'angle_max': 3.14159, 'angle_increment': 0.0087266,
                 'scan_time': 0.1, 'range_min': 0.4, 'range_max': 20.0, 'use_inf': True}]),
    ] + ([LogInfo(msg='[lite3_sim] sensors:=nav: no camera topics, lidar 720x8, IMU 100 Hz')] if nav_only else [
        Node(package='lite3_sim', executable='depth_to_mm.py', name='depth_to_mm', output='screen',
             parameters=[sim_time])])


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')
    default_world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds',
                                 'turtlebot3_world.world')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('camera_full', default_value='false',
                              description='true: camera 640x480 @ 10 Hz (default 320x240 @ 5 Hz)'),
        DeclareLaunchArgument('domain_id', default_value='89'),
        DeclareLaunchArgument('sensors', default_value='full',
                              description="full (all sensors) or nav (navigation-only: no camera, "
                                          "lighter lidar, IMU 100 Hz; same topic/frame names)"),
        OpaqueFunction(function=domain_guard),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        OpaqueFunction(function=robot),
    ])
