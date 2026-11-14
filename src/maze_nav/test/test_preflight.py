from maze_nav.preflight import classify


def test_nav2_and_slam_binaries():
    assert classify(['/opt/ros/humble/lib/nav2_planner/planner_server', '--ros-args',
                     '-r', '__node:=planner_server']) == 'Nav2'
    assert classify(['/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node',
                     '--ros-args']).startswith('SLAM')


def test_python_entry_points_and_launch():
    assert classify(['/usr/bin/python3', '/x/install/maze_nav/lib/maze_nav/maze_explorer',
                     '--ros-args']) == 'maze_nav'
    assert classify(['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'maze_nav',
                     'maze_nav.launch.py']) == 'ros2 launch maze_nav'
    assert classify(['ros2', 'launch', 'transfer', 'transfer_launch.py']) is not None


def test_bringup_only_counts_when_we_start_it():
    sim = ['python3', '-W', 'ignore', '/x/test/fake_maze_lite3.py']
    assert classify(sim, bringup=True) == 'maze simulator'
    assert classify(sim, bringup=False) is None


def test_unrelated_and_ros_args_ignored():
    assert classify(['/usr/bin/bash']) is None
    assert classify(['/usr/bin/python3', 'my_node.py', '--ros-args', '-r',
                     '__node:=planner_server']) is None
    assert classify(['code', '/home/user/challenge_ws/src/maze_nav/maze_nav/wall_scan.py']) \
        is None                                    # editor, not the node
    assert classify(['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard']) \
        is not None
