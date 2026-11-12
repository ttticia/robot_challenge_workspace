"""Real Nav2 planner/controller integration, using the adjacent kinematic fixture."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    root=Path(__file__).resolve().parent
    params=str(root/'nav2_test_params.yaml')
    standard=yaml.safe_load((Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml').read_text())
    plugins=standard['bt_navigator']['ros__parameters']['plugin_lib_names']
    return LaunchDescription([
        Node(package='nav2_controller',executable='controller_server',name='controller_server',
             parameters=[params],remappings=[('cmd_vel','/mission/nav_cmd_vel')],output='screen'),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params],output='screen'),
        Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',
             parameters=[params,{'default_nav_to_pose_bt_xml':str(root/'nav2_test_tree.xml'),
                                 'default_nav_through_poses_bt_xml':str(root/'nav2_test_through_tree.xml'),
                                 'plugin_lib_names':plugins}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='test_lifecycle_manager',
             parameters=[{'use_sim_time':True,'autostart':True,
                          'node_names':['controller_server','planner_server','bt_navigator']}],output='screen')])
