"""Pre-launch check for maze_nav: warn about anything that would clash with this run.

Called by maze_nav.launch.py before any node starts. Warn-only: the robot waits for
/maze/start anyway, so the operator can Ctrl+C after reading the warning.

1. Local processes (/proc, this computer): another SLAM, Nav2 or maze_nav, teleop/twist_mux,
   the manufacturer `transfer` launch; with bringup:=true also Fast-LIO / odom_relay / the
   simulator (with bringup:=false those are expected to be running already).
2. ROS graph (every computer on this ROS_DOMAIN_ID): /cmd_vel publishers (robot_info.md §6.2:
   only one at a time; lite3_domain_bridge_4 only subscribes) and nodes with our node names.
"""
import os
import time

NAV2 = 'Nav2'
MAZE = 'maze_nav'
CMD = 'publishes velocity commands'

# Executable basename (".py" stripped) -> what it is
LOCAL = {
    'async_slam_toolbox_node': 'SLAM (slam_toolbox)',
    'sync_slam_toolbox_node': 'SLAM (slam_toolbox)',
    'localization_slam_toolbox_node': 'SLAM (slam_toolbox)',
    'planner_server': NAV2, 'controller_server': NAV2, 'behavior_server': NAV2,
    'bt_navigator': NAV2, 'velocity_smoother': NAV2, 'lifecycle_manager': NAV2,
    'smoother_server': NAV2, 'waypoint_follower': NAV2, 'map_server': NAV2, 'amcl': NAV2,
    'maze_explorer': MAZE, 'wall_scan': MAZE, 'cmd_vel_bridge': MAZE,
    'teleop_twist_joy_node': CMD, 'teleop_twist_keyboard': CMD, 'twist_mux': CMD,
    'transfer_launch': 'manufacturer transfer launch (clashes with lite3_domain_bridge_4)',
}
BRINGUP = {   # started by this launch with bringup:=true; expected with bringup:=false
    'fastlio_mapping': 'Fast-LIO', 'odom_relay': 'odom_relay',
    'pointcloud_to_laserscan_node': 'lite3_launcher /scan',
    'fake_maze_lite3': 'maze simulator',
}
# `ros2 launch <package> ...` of a stack that starts any of the above
LAUNCH_PACKAGES = {'maze_nav', 'lite3_launcher', 'nav2_bringup', 'slam_toolbox'}

# ROS node names that clash with ours
NODES = {'slam_toolbox', 'planner_server', 'controller_server', 'behavior_server',
         'bt_navigator', 'velocity_smoother', 'lifecycle_manager_navigation',
         'maze_explorer', 'wall_scan', 'cmd_vel_bridge', 'twist_mux'}
BRINGUP_NODES = {'laser_mapping', 'odom_relay'}


def _name(path):
    name = os.path.basename(path)
    for ext in ('.launch.py', '.py'):
        if name.endswith(ext):
            return name[:-len(ext)]
    return name


def classify(argv, bringup=True):
    """What a process with this command line is, if it clashes with maze_nav, else None.

    Only the program itself counts (or the script, for an interpreter), never other
    arguments: an editor with wall_scan.py open is not wall_scan.
    """
    if not argv:
        return None
    known = dict(LOCAL, **(BRINGUP if bringup else {}))
    prog = [argv[0]]
    if _name(argv[0]).startswith('python'):
        rest = argv[1:]
        while rest and rest[0].startswith('-'):        # python3 -W ignore script.py
            rest = rest[2:] if rest[0] in ('-W', '-X') else rest[1:]
        prog = rest[:1]
    if not prog:
        return None
    rest = argv[argv.index(prog[0]) + 1:]
    if _name(prog[0]) == 'ros2':
        if len(rest) >= 3 and rest[0] == 'launch' and rest[1] in LAUNCH_PACKAGES:
            return f'ros2 launch {rest[1]}'
        if len(rest) >= 3 and rest[0] == 'launch' and _name(rest[2]) in known:
            return known[_name(rest[2])]          # ros2 launch transfer transfer_launch.py
        if len(rest) >= 3 and rest[0] == 'run' and _name(rest[2]) in known:
            return known[_name(rest[2])]
        return None
    return known.get(_name(prog[0]))


def _ancestors(pid):
    out = set()
    while pid > 1 and pid not in out:
        out.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])   # ppid; comm may hold spaces
        except (OSError, ValueError, IndexError):
            break
    return out


def local_conflicts(bringup=True):
    """[(pid, what, command line)] of clashing processes on this computer."""
    me = _ancestors(os.getpid())
    out = []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) in me:
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        what = classify(argv, bringup)
        if what:
            out.append((int(d), what, ' '.join(argv)))
    return sorted(out)


def graph_conflicts(node, bringup=True, wait=2.0):
    """Clashing /cmd_vel publishers and node names on the ROS network."""
    time.sleep(wait)                                # DDS discovery
    names = NODES | (BRINGUP_NODES if bringup else set())
    out = []
    for info in node.get_publishers_info_by_topic('/cmd_vel'):
        out.append(f'/cmd_vel publisher: {info.node_namespace.rstrip("/")}/{info.node_name}')
    for name, ns in node.get_node_names_and_namespaces():
        if name in names:
            out.append(f'node already running: {ns.rstrip("/")}/{name}')
    return sorted(set(out))


def run(context, bringup, wait_for_start, logger):
    """Log a warning listing everything that clashes. Returns True if anything did."""
    lines = [f'  pid {pid}: {what}  ({cmd[:110]})' for pid, what, cmd in local_conflicts(bringup)]
    try:
        from launch_ros.ros_adapters import get_ros_node
        lines += [f'  {g}' for g in graph_conflicts(get_ros_node(context), bringup)]
    except Exception as e:                          # never block a launch on the check itself
        logger.warning(f'preflight: ROS graph check skipped ({e})')
    if not lines:
        logger.info('preflight: no other SLAM / Nav2 / maze_nav / /cmd_vel publisher found')
        return False
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    logger.warning(
        '\n' + '!' * 78 + '\n'
        f'  maze_nav preflight: OTHER PROCESSES ARE RUNNING (ROS_DOMAIN_ID={domain})\n'
        + '\n'.join(lines) + '\n'
        '  Two stacks fight over /map, TF and /cmd_vel. Ctrl+C now, stop them\n'
        '  (kill <pid>, or Ctrl+C their launch terminal), check `ros2 node list`, relaunch.\n'
        + ('  Launching anyway; the robot will not move until /maze/start.\n' if wait_for_start
           else '  Launching anyway (wait_for_start:=false: it starts on its own!).\n')
        + '!' * 78)
    time.sleep(3.0)                                 # readable before the node logs scroll it
    return True
