"""scene_trigger: tell scene understanding when the robot reaches a cell it hasn't been in.

The maze logic is in scene_cells.py (NewCellTrigger). This node polls TF map -> base_link at
`rate` Hz (latest transform, never blocking: a blocking lookup in a callback starves TF) and,
the first time base_link is `enter_margin` m inside a cell (the start cell included),
publishes:

  /scene/trigger      std_msgs/String, JSON:
      {"seq": 3, "cell": [1, 1], "from_cell": [1, 0], "direction": "+j",
       "openings": {"+i": "wall", "+j": "open", "-i": "open", "-j": "open"},
       "cell_centre": [1.0, 1.0], "robot": {"x": 0.98, "y": 0.86, "yaw_deg": 85.8},
       "frame": "map", "stamp": "<sec>.<nanosec>"}
      cell indices: (0, 0) = start cell, +i = the robot's heading at bringup.
      direction: the way the robot moved into the cell; from_cell null and direction ''
      for the start cell ('' also if the pose jumped more than one cell).
      openings: each side of the new cell on /map right now: open, wall or unknown.
  /junction_detected  std_msgs/Bool: true, then false straight away, so
      scene_understanding's junction_trigger (rising_edge) fires once per event unchanged.
      bool_topic:='' turns it off.

Only while the run is under way: it follows maze_explorer's /maze/status (latched) and
tracks cells only in CREEPING, SCANNING, EXPLORING or TURNING (`active_states`). Holding still before
/maze/start (STARTING, PAUSED) nothing is published, so the start cell's event comes when the
run starts, once the recording and scene understanding are up; after the end (EXITED,
NO_EXIT, STOPPED) nothing either, so no events from walking around the room. Cells entered
while it is inactive don't count as reached.

It only reads TF, /map and /maze/status, so it runs beside maze_explorer or on its own:
status_topic:='' drops the status gate (teleop, rosbag replay with use_sim_time:=true).
It never moves the robot.

    ros2 run maze_nav scene_trigger
    ros2 run maze_nav scene_trigger --ros-args -p grid_yaw_deg:=3.0 -p origin_y:=0.05
    ros2 run maze_nav scene_trigger --ros-args -p status_topic:=''     # no explorer
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import Grid
from maze_nav.scene_understanding.scene_cells import (ACTIVE_STATES, DIRECTIONS, CellGrid,
                                                      NewCellTrigger, opening, run_active)

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SceneTrigger(Node):
    def __init__(self):
        super().__init__('scene_trigger')
        p = self.declare_parameter
        p('cell_size', 1.0)            # m
        p('origin_x', 0.0)             # map (x, y) of the start cell's centre
        p('origin_y', 0.0)
        p('grid_yaw_deg', 0.0)         # map heading of the grid's +i axis
        p('enter_margin', 0.15)        # m inside a cell before it counts as reached
        p('trigger_at_start', True)    # also fire for the start cell
        p('opening_half_width', 0.35)  # m either side of the edge middle checked for a wall
        p('rate', 10.0)                # Hz pose polling
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('map_topic', '/map')
        p('trigger_topic', '/scene/trigger')
        p('bool_topic', '/junction_detected')   # '' = don't publish the Bool
        p('status_topic', '/maze/status')        # '' = always active (no maze_explorer)
        p('active_states', list(ACTIVE_STATES))  # /maze/status states that count as running
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.cells = CellGrid(float(g('cell_size')),
                              (float(g('origin_x')), float(g('origin_y'))),
                              math.radians(float(g('grid_yaw_deg'))))
        self.logic = NewCellTrigger(self.cells, enter_margin=float(g('enter_margin')),
                                    trigger_at_start=bool(g('trigger_at_start')))
        self.half_width = float(g('opening_half_width'))
        self.map_frame, self.base_frame = g('map_frame'), g('base_frame')

        self.grid = None
        self.seq = 0
        self.active_states = tuple(g('active_states'))
        self.status_topic = g('status_topic')
        self.active = not self.status_topic
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, LATCHED)
        if self.status_topic:
            self.create_subscription(String, self.status_topic, self.on_status, LATCHED)
        self.trigger_pub = self.create_publisher(String, g('trigger_topic'), 10)
        self.bool_pub = (self.create_publisher(Bool, g('bool_topic'), 10)
                         if g('bool_topic') else None)
        self.create_timer(1.0 / float(g('rate')), self.tick)
        self.get_logger().info(
            f'{self.cells.size:.2f} m cells, start cell centre ({self.cells.origin[0]:.2f}, '
            f'{self.cells.origin[1]:.2f}), grid yaw {g("grid_yaw_deg")} deg; events on '
            f'{g("trigger_topic")}' + (f' and {g("bool_topic")}' if self.bool_pub else '')
            + (f'; only while {self.status_topic} is {"/".join(self.active_states)}'
               if self.status_topic else '; always on (no status gate)'))

    def on_status(self, msg):
        active = run_active(msg.data, self.active_states)
        if active != self.active:
            self.active = active
            self.get_logger().info(f'{"tracking cells" if active else "paused"}: '
                                   f'{self.status_topic} = {msg.data.split(":", 1)[0]}')

    def on_map(self, msg):
        self.grid = Grid.from_msg(msg)

    def opening(self, cell, d):
        if self.grid is None:
            return 'unknown'
        return opening(self.grid, self.cells, cell, d, half_width=self.half_width)

    def tick(self):
        if not self.active:
            return
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            self.get_logger().info(f'waiting for TF {self.map_frame} -> {self.base_frame}',
                                   throttle_duration_sec=10.0)
            return
        xy = (t.transform.translation.x, t.transform.translation.y)
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        now = self.get_clock().now()
        event = self.logic.update(xy, yaw)
        if event is None:
            return

        self.seq += 1
        stamp = now.to_msg()
        record = {
            'seq': self.seq,
            'cell': list(event.cell),
            'from_cell': None if event.from_cell is None else list(event.from_cell),
            'direction': event.direction,
            'openings': {n: self.opening(event.cell, d) for n, d in DIRECTIONS.items()},
            'cell_centre': [round(v, 3) for v in self.cells.centre(event.cell)],
            'robot': {'x': round(xy[0], 3), 'y': round(xy[1], 3),
                      'yaw_deg': round(math.degrees(yaw), 1)},
            'frame': self.map_frame,
            'stamp': f'{stamp.sec}.{stamp.nanosec:09d}',
        }
        self.trigger_pub.publish(String(data=json.dumps(record)))
        if self.bool_pub is not None:
            self.bool_pub.publish(Bool(data=True))
            self.bool_pub.publish(Bool(data=False))
        came = (f' from {list(event.from_cell)} ({event.direction or "jump"})'
                if event.from_cell is not None else ' (start)')
        ways = [n for n, v in record['openings'].items() if v == 'open']
        self.get_logger().info(f'new cell #{self.seq}: {list(event.cell)}{came}, '
                               f'{len(self.logic.visited)} reached; open: {ways}')


def main():
    rclpy.init()
    node = SceneTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
