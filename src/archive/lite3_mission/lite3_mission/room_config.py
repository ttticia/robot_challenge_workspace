"""Make a separate conservative room profile from the installed lab config."""
import argparse
from copy import deepcopy
from pathlib import Path
import yaml


def room_parameters(original, bt_file):
    data = deepcopy(original)
    def params(name):
        return data.setdefault(name, {}).setdefault('ros__parameters', {})
    controller = params('controller_server')
    controller.update(controller_frequency=20.0, controller_plugins=['FollowPath'],
                      progress_checker_plugin='progress_checker', goal_checker_plugins=['goal_checker'])
    controller['progress_checker'] = dict(plugin='nav2_controller::SimpleProgressChecker',
                                         required_movement_radius=.02, movement_time_allowance=15.)
    controller['goal_checker'] = dict(plugin='nav2_controller::SimpleGoalChecker',
                                     xy_goal_tolerance=.05, yaw_goal_tolerance=.2, stateful=True)
    follow = controller.setdefault('FollowPath', {})
    if follow.get('plugin') != 'nav2_mppi_controller::MPPIController':
        raise ValueError('This room profile expects the inspected lab MPPI controller')
    follow.update(vx_max=.1, vx_min=0., vy_max=0., wz_max=.3,
                  vx_std=.05, vy_std=0., wz_std=.1, ax_max=.2, ax_min=-.2,
                  ay_max=.2, ay_min=-.2, az_max=.5, motion_model='DiffDrive',
                  model_dt=.05, time_steps=40, batch_size=1000)
    params('planner_server')['GridBased'].update(allow_unknown=False, tolerance=.05)
    params('velocity_smoother').update(smoothing_frequency=20., feedback='OPEN_LOOP',
        max_velocity=[.1, 0., .3], min_velocity=[0., 0., -.3],
        max_accel=[.2, .2, .5], max_decel=[-.2, -.2, -.5],
        deadband_velocity=[0., 0., 0.], velocity_timeout=.2, odom_topic='/odom')
    params('bt_navigator')['default_nav_to_pose_bt_xml'] = str(bt_file)
    # The lab feeds the same flattened LaserScan to both 2D and voxel layers.
    # Small negative odom z puts that scan below the voxel grid's zero floor,
    # preventing clearing. Keep the 2D obstacle layer for this 2D source.
    for name in ('local_costmap','global_costmap'):
        costmap=data.get(name,{}).get(name,{}).get('ros__parameters',{})
        voxel=costmap.get('voxel_layer',{})
        obstacle=costmap.get('obstacle_layer',{})
        vscan=voxel.get('scan',{})
        oscan=obstacle.get('scan',{})
        plugins=costmap.get('plugins',[])
        if ('obstacle_layer' in plugins and obstacle.get('enabled',True)
                and voxel.get('observation_sources')=='scan'
                and obstacle.get('observation_sources')=='scan'
                and vscan.get('data_type')==oscan.get('data_type')=='LaserScan'
                and vscan.get('topic')==oscan.get('topic')):
            costmap['plugins']=[p for p in plugins if p!='voxel_layer']
    params('slam_toolbox')['minimum_travel_distance'] = .1
    params('behavior_server').update(max_rotational_vel=.15, min_rotational_vel=.05,
                                      rotational_acc_lim=.3)
    def hardware_clock(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'use_sim_time':
                    value[key] = False
                else:
                    hardware_clock(child)
    hardware_clock(data)
    return data


def main():
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    course = Path(get_package_share_directory('lite3_launcher'))
    package = Path(get_package_share_directory('lite3_mission'))
    source = course / 'config/nav2_params.yaml'
    data = room_parameters(yaml.safe_load(source.read_text()), package / 'config/room_to_pose.xml')
    # Use the installed Nav2 version's complete BT plugin list.
    standard = Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml'
    data['bt_navigator']['ros__parameters']['plugin_lib_names'] = yaml.safe_load(
        standard.read_text())['bt_navigator']['ros__parameters']['plugin_lib_names']
    output = Path(args.output).expanduser()
    if output.resolve() == source.resolve():
        raise ValueError('Refusing to overwrite the lab configuration')
    with output.open('x') as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
    print('Created separate room profile: ' + str(output))
