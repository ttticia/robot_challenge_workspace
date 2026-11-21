"""Start the scene observer and the stair controller together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def stair_node(context):
    def enabled(name):
        return LaunchConfiguration(name).perform(context).lower() == 'true'

    args = [
        '--depth-topic', LaunchConfiguration('depth_topic'),
        '--camera-info-topic', LaunchConfiguration('camera_info_topic'),
        '--odom-topic', LaunchConfiguration('odom_topic'),
    ]
    if enabled('execute'):
        args.append('--execute')
        if enabled('aligned_start'):
            args.append('--aligned-start')
        if enabled('reverse_descent_verified'):
            args.append('--reverse-descent-verified')
    return [Node(package='stair_traversal', executable='stair_traversal',
                 arguments=args, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('aligned_start', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('reverse_descent_verified', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/color/image_raw'),
        OpaqueFunction(function=stair_node),
        Node(
            package='scene_understanding', executable='junction_trigger', output='screen',
            parameters=[{'enable_vlm': True, 'junction_scan_enabled': False}],
            remappings=[('/camera/image_raw', LaunchConfiguration('camera_topic'))],
        ),
    ])
