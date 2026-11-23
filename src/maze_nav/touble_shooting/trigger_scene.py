"""Manually trigger a dead-end look and/or scene understanding at the robot's current spot.

    ros2 run maze_nav trigger_scene                 # dead-end look at the cell ahead
    ros2 run maze_nav trigger_scene capture         # scene understanding only, no motion
    ros2 run maze_nav trigger_scene capture --cell 3 1 --hold 2.0

dead_end (default): the same /scene/trigger record scene_trigger sends when the robot faces
    a dead end, for the cell the robot is facing (grid from the running scene_trigger's
    cell_size / origin_* / grid_yaw_deg), marked "manual". maze_explorer then does its usual
    look: hold dead_end_hold s facing it, scan its left and right walls, one /scene/capture
    per view (if launched with scene_understanding:=true). For a manual record it does NOT
    blacklist the cell, and it runs only while EXPLORING with go:=true (a look ends in
    EXPLORING, so from PAUSED it would start the run): otherwise it logs why it ignored it.
capture: one /scene/capture straight to room_scene (no explorer, no motion): the camera
    frame from the last --hold s, as a single last view, for the cell ahead (or --cell).
    Works in any state, robot standing still; needs room_scene running
    (scene_understanding:=true). room_scene analyses each cell once: use --cell to repeat.

Both also send a MARK to telemetry_snapshot (troubleshoot:=true), so the moment gets a
snapshot and an events.log line.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from maze_nav.scene_cells import DIRECTIONS, CellGrid, side_walls, wrap

GRID_PARAMS = ('cell_size', 'origin_x', 'origin_y', 'grid_yaw_deg')
GRID_DEFAULTS = (1.0, 0.0, 0.0, 0.0)          # = scene_trigger defaults


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wait_until(node, cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return cond()


def grid_from_scene_trigger(node, log):
    """CellGrid of the running scene_trigger (so cells match its records), else defaults."""
    cli = node.create_client(GetParameters, '/scene_trigger/get_parameters')
    vals = GRID_DEFAULTS
    if cli.wait_for_service(timeout_sec=2.0):
        fut = cli.call_async(GetParameters.Request(names=list(GRID_PARAMS)))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        if fut.done() and fut.result() is not None:
            got = [v.double_value for v in fut.result().values]
            if len(got) == len(GRID_PARAMS):
                vals = got
    else:
        log('scene_trigger not running: default grid (1 m cells, origin 0 0, yaw 0)')
    size, ox, oy, yaw_deg = vals
    return CellGrid(float(size), (float(ox), float(oy)), math.radians(float(yaw_deg)))


def facing_cell(cells, xy, yaw):
    """(robot cell, direction name, cell ahead): the grid direction nearest the heading."""
    here = cells.cell(xy)
    name = min(DIRECTIONS, key=lambda n: abs(wrap(cells.heading(DIRECTIONS[n]) - yaw)))
    d = DIRECTIONS[name]
    return here, name, (here[0] + d[0], here[1] + d[1])


def stamp(ns):
    return f'{ns // 10**9}.{ns % 10**9:09d}'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('mode', nargs='?', default='dead_end', choices=('dead_end', 'capture'))
    ap.add_argument('--cell', nargs=2, type=int, metavar=('I', 'J'),
                    help='target cell instead of the one ahead (scene_trigger grid)')
    ap.add_argument('--hold', type=float, default=1.0,
                    help='capture: s of camera frames before now to pick from (default 1.0)')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_link')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None
                                                          else sys.argv)[1:])
    rclpy.init(args=argv)
    node = Node('trigger_scene')
    log = lambda m: print(f'[trigger_scene] {m}', flush=True)  # noqa: E731
    try:
        tf = Buffer()
        TransformListener(tf, node)
        status = {}
        node.create_subscription(String, '/maze/status',
                                 lambda m: status.setdefault('s', m.data),
                                 rclpy.qos.QoSProfile(
                                     depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        pose = {}

        def have_pose():
            try:
                t = tf.lookup_transform(args.map_frame, args.base_frame, Time())
            except Exception:   # noqa: BLE001 - TF not in yet
                return False
            pose['xy'] = (t.transform.translation.x, t.transform.translation.y)
            pose['yaw'] = yaw_of(t.transform.rotation)
            return True
        if not wait_until(node, have_pose, 5.0):
            log(f'no TF {args.map_frame} -> {args.base_frame} after 5 s: is the stack running '
                '(and ROS_DOMAIN_ID right)?')
            return 1
        wait_until(node, lambda: 's' in status, 1.5)
        state = status.get('s', '(no /maze/status)')
        cells = grid_from_scene_trigger(node, log)
        xy, yaw = pose['xy'], pose['yaw']
        here, dname, ahead = facing_cell(cells, xy, yaw)
        cell = tuple(args.cell) if args.cell else ahead
        centre = cells.centre(cell)
        robot = {'x': round(xy[0], 3), 'y': round(xy[1], 3),
                 'yaw_deg': round(math.degrees(yaw), 1)}
        now = node.get_clock().now().nanoseconds
        log(f'robot at ({xy[0]:.2f}, {xy[1]:.2f}) yaw {math.degrees(yaw):.0f} deg, cell '
            f'{list(here)}, facing {dname}; target cell {list(cell)}; explorer: {state}')

        if args.mode == 'dead_end':
            topic = '/scene/trigger'
            d = DIRECTIONS[dname]
            rec = {'seq': 0, 'kind': 'dead_end', 'manual': True, 'cell': list(cell),
                   'robot_cell': list(here), 'direction': dname,
                   'cell_centre': [round(v, 3) for v in centre],
                   'heading': round(cells.heading(d), 4),
                   'side_walls': {k: [round(v, 3) for v in p]
                                  for k, p in side_walls(cells, cell, d).items()},
                   'robot': robot, 'frame': args.map_frame, 'stamp': stamp(now)}
            if not state.startswith('EXPLORING'):
                log(f'WARNING: explorer is {state.split(":")[0]}; it runs a manual dead-end '
                    'look only while EXPLORING (it will log that it ignored this)')
        else:
            topic = '/scene/capture'
            rec = {'cell': list(cell), 'cell_centre': [round(v, 3) for v in centre],
                   'kind': 'dead_end', 'view': 'centre', 'last': True, 'manual': True,
                   'stamp': stamp(now), 'hold_start': stamp(now - int(args.hold * 1e9)),
                   'robot': robot, 'frame': args.map_frame}

        pub = node.create_publisher(String, topic, 10)
        if not wait_until(node, lambda: pub.get_subscription_count() > 0, 3.0):
            who = 'maze_explorer (dead_end_topic)' if args.mode == 'dead_end' else \
                'room_scene (launch with scene_understanding:=true)'
            log(f'nobody subscribes {topic}: start {who}. Not sent.')
            return 1
        if args.mode == 'dead_end' and not node.get_publishers_info_by_topic('/scene/capture'):
            log('note: no /scene/capture publisher (scene_understanding:=false): the look runs '
                'but no scene-understanding capture is sent')
        pub.publish(String(data=json.dumps(rec)))
        mark = node.create_publisher(String, '/troubleshoot/mark', 10)
        wait_until(node, lambda: mark.get_subscription_count() > 0, 0.5)
        mark.publish(String(data=f'manual {args.mode} cell {list(cell)}'))
        wait_until(node, lambda: False, 0.3)       # let both go out before shutdown
        log(f'sent {args.mode} for cell {list(cell)} on {topic}')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
