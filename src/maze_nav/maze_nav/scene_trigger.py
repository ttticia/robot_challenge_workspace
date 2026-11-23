"""scene_trigger: tell scene understanding when the robot faces a dead end, and the explorer
when it is at a junction.

The maze logic is in scene_cells.py (NewCellTrigger, DeadEndDetector, JunctionDetector).
This node polls TF map -> base_link at `rate` Hz (latest transform, never blocking: a
blocking lookup in a callback starves TF) and reads the sides of the cells off /map.

Dead end (scene understanding): the robot is in a cell and faces (yaw within
`dead_end_heading_tol_deg` of a grid direction) the next cell, the side between them is
open and that cell's three other sides (left, right, far side) are walls on /map. It fires
from the cell before the dead end, never from inside it, once per dead-end cell; cells the
robot has already been in don't count (e.g. the start cell). maze_explorer stops for
`dead_end_hold` s on it (steady camera view), turns the camera to the dead end's left and
right walls (`side_walls`, `dead_end_side_hold` s each) and never drives into that cell.

  /scene/trigger      std_msgs/String, JSON:
      {"seq": 3, "kind": "dead_end", "cell": [2, 1], "robot_cell": [1, 1],
       "direction": "+i",
       "openings": {"+i": "wall", "+j": "wall", "-i": "open", "-j": "wall"},
       "cell_centre": [2.0, 1.0], "heading": 0.0,
       "side_walls": {"left": [2.0, 1.5], "right": [2.0, 0.5]},
       "robot": {"x": 0.98, "y": 0.96, "yaw_deg": 2.1},
       "frame": "map", "stamp": "<sec>.<nanosec>"}
      cell / cell_centre: the dead-end cell; robot_cell: where the robot is (next to it);
      direction / heading: the way (name, map yaw in rad) from robot_cell to the dead end;
      openings: the dead end's sides; side_walls: map (x, y) of the middles of its left and
      right walls, seen from robot_cell.
      Cell indices: (0, 0) = start cell, +i = the robot's heading at bringup.
  /junction_detected  std_msgs/Bool: true, then false straight away, on each dead end, so
      scene_understanding's junction_trigger (rising_edge) fires once per event unchanged.
      bool_topic:='' turns it off.

Junction (maze_explorer only, no scene understanding): the first time the robot gets within
`junction_centre_radius` m of a cell's centre, the cell has 2+ open sides besides the way
it came in:

  /scene/junction     std_msgs/String, JSON:
      {"seq": 1, "cell": [1, 0], "from_cell": [0, 0],
       "paths": [{"dir": "+j", "heading": 1.571}, {"dir": "-j", "heading": -1.571}],
       "cell_centre": [1.0, 0.0], "robot": {...}, "frame": "map", "stamp": "..."}
      heading: map-frame yaw (rad) of each open way on. grid: {size, origin, yaw,
      opening_half_width}, the cell grid, so maze_explorer traces each way on /map. It faces
      each way, left to right (JUNCTION), then visits a way that ends in a dead end or takes
      the rightmost way. junction_topic:='' = off.

Only while the run is under way: it follows maze_explorer's /maze/status (latched) and
tracks cells only in `active_states` (CREEPING ... DEAD_END). Holding still before
/maze/start (STARTING, PAUSED) nothing is published; after the end (EXITED, NO_EXIT,
STOPPED) nothing either. Cells entered while it is inactive don't count as reached.

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

from touble_shooting.live_params import attach as attach_live_params
from maze_nav.core import Grid
from maze_nav.scene_cells import (ACTIVE_STATES, DIRECTIONS, CellGrid,
                                                      DeadEndDetector, JunctionDetector,
                                                      NewCellTrigger, opening, run_active,
                                                      side_walls)

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SceneTrigger(Node):
    def __init__(self):
        super().__init__('scene_trigger')
        p = self.declare_parameter
        p('cell_size', 1.02)           # m (same as maze_explorer's cell_size)
        p('origin_x', 0.0)             # map (x, y) of the start cell's centre
        p('origin_y', 0.0)
        p('grid_yaw_deg', 0.0)         # map heading of the grid's +i axis
        p('enter_margin', 0.15)        # m inside a cell before it counts as reached
        p('opening_half_width', 0.35)  # m either side of the edge middle checked for a wall
        p('dead_end_heading_tol_deg', 25.0)   # facing the dead end within this
        p('junction_centre_radius', 0.25)     # m from the cell centre (each axis) to check it
        p('rate', 10.0)                # Hz pose polling
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('map_topic', '/map')
        p('trigger_topic', '/scene/trigger')
        p('bool_topic', '/junction_detected')   # '' = don't publish the Bool
        p('junction_topic', '/scene/junction')  # '' = no junction events
        p('status_topic', '/maze/status')        # '' = always active (no maze_explorer)
        p('grid_topic', '/maze/grid')            # maze_explorer's wall-fitted grid; '' = off
        p('active_states', list(ACTIVE_STATES))  # /maze/status states that count as running
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.cells = CellGrid(float(g('cell_size')),
                              (float(g('origin_x')), float(g('origin_y'))),
                              math.radians(float(g('grid_yaw_deg'))))
        self.logic = NewCellTrigger(self.cells, enter_margin=float(g('enter_margin')))
        self.dead_ends = DeadEndDetector(
            self.cells, heading_tol=math.radians(float(g('dead_end_heading_tol_deg'))))
        self.junctions = JunctionDetector(self.cells,
                                          centre_radius=float(g('junction_centre_radius')))
        self.half_width = float(g('opening_half_width'))
        self.map_frame, self.base_frame = g('map_frame'), g('base_frame')

        self.grid = None
        self.seq = 0
        self.junction_seq = 0
        self.active_states = tuple(g('active_states'))
        self.status_topic = g('status_topic')
        self.active = not self.status_topic
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, LATCHED)
        if g('grid_topic'):
            self.create_subscription(String, g('grid_topic'), self.on_grid, LATCHED)
        if self.status_topic:
            self.create_subscription(String, self.status_topic, self.on_status, LATCHED)
        self.trigger_pub = self.create_publisher(String, g('trigger_topic'), 10)
        self.bool_pub = (self.create_publisher(Bool, g('bool_topic'), 10)
                         if g('bool_topic') else None)
        self.junction_pub = (self.create_publisher(String, g('junction_topic'), 10)
                             if g('junction_topic') else None)
        self.create_timer(1.0 / float(g('rate')), self.tick)
        self.get_logger().info(
            f'{self.cells.size:.2f} m cells, start cell centre ({self.cells.origin[0]:.2f}, '
            f'{self.cells.origin[1]:.2f}), grid yaw {g("grid_yaw_deg")} deg; dead ends on '
            f'{g("trigger_topic")}' + (f' and {g("bool_topic")}' if self.bool_pub else '')
            + (f', junctions on {g("junction_topic")}' if self.junction_pub else '')
            + (f'; only while {self.status_topic} is {"/".join(self.active_states)}'
               if self.status_topic else '; always on (no status gate)'))
        self.attach_live_params()

    def attach_live_params(self):
        """`ros2 param set /scene_trigger <name> <value>` while running (touble_shooting).
        The cell grid (cell_size, origin_*, grid_yaw_deg) stays startup-only: every
        visited / reported cell is keyed on it."""
        attach_live_params(self, setters={
            'enter_margin': lambda v: setattr(self.logic, 'enter_margin', float(v)),
            'opening_half_width': lambda v: setattr(self, 'half_width', float(v)),
            'dead_end_heading_tol_deg':
                lambda v: setattr(self.dead_ends, 'heading_tol', math.radians(float(v))),
            'junction_centre_radius':
                lambda v: setattr(self.junctions, 'centre_radius', float(v)),
        })

    def on_grid(self, msg):
        """maze_explorer refitted the grid to the walls: follow it (in place, so the
        trigger and detectors holding self.cells do too). Same cell size only."""
        try:
            d = json.loads(msg.data)
            if abs(float(d['size']) - self.cells.size) > 1e-6:
                return
            self.cells.origin = (float(d['origin'][0]), float(d['origin'][1]))
            self.cells.yaw = float(d['yaw'])
        except (ValueError, KeyError, TypeError, IndexError):
            self.get_logger().warn(f'bad /maze/grid message: {msg.data[:80]}')
            return
        self.get_logger().info(f'grid from maze_explorer: yaw '
                               f'{math.degrees(self.cells.yaw):+.1f} deg, cell (0, 0) at '
                               f'({self.cells.origin[0]:+.2f}, {self.cells.origin[1]:+.2f})')

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

    def stamp(self):
        s = self.get_clock().now().to_msg()
        return f'{s.sec}.{s.nanosec:09d}'

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
        robot = {'x': round(xy[0], 3), 'y': round(xy[1], 3),
                 'yaw_deg': round(math.degrees(yaw), 1)}
        entered = self.logic.update(xy, yaw)
        if entered is not None:
            self.get_logger().info(f'new cell {list(entered.cell)}, '
                                   f'{len(self.logic.visited)} reached')
        if self.grid is None:
            return
        cell = self.logic.current

        ways = self.junctions.update(xy, cell, self.logic.previous, self.opening)
        if ways and self.junction_pub is not None:
            self.junction_seq += 1
            record = {
                'seq': self.junction_seq,
                'cell': list(cell),
                'from_cell': list(self.logic.previous),
                'paths': [{'dir': n, 'heading': round(self.cells.heading(DIRECTIONS[n]), 4)}
                          for n in ways],
                'cell_centre': [round(v, 3) for v in self.cells.centre(cell)],
                # the cell grid, so maze_explorer can trace each way on /map itself
                'grid': {'size': self.cells.size, 'origin': list(self.cells.origin),
                         'yaw': self.cells.yaw, 'opening_half_width': self.half_width},
                'robot': robot, 'frame': self.map_frame, 'stamp': self.stamp(),
            }
            self.junction_pub.publish(String(data=json.dumps(record)))
            self.get_logger().info(f'junction #{self.junction_seq} at {list(cell)}: '
                                   f'ways on {ways}')

        hit = self.dead_ends.update(xy, yaw, cell, self.opening, skip=self.logic.visited)
        if hit is None:
            return
        dead, direction, sides = hit
        self.seq += 1
        record = {
            'seq': self.seq,
            'kind': 'dead_end',
            'cell': list(dead),
            'robot_cell': list(cell),
            'direction': direction,
            'openings': sides,
            'cell_centre': [round(v, 3) for v in self.cells.centre(dead)],
            'heading': round(self.cells.heading(DIRECTIONS[direction]), 4),
            'side_walls': {k: [round(v, 3) for v in xy] for k, xy in
                           side_walls(self.cells, dead, DIRECTIONS[direction]).items()},
            'robot': robot, 'frame': self.map_frame, 'stamp': self.stamp(),
        }
        self.trigger_pub.publish(String(data=json.dumps(record)))
        if self.bool_pub is not None:
            self.bool_pub.publish(Bool(data=True))
            self.bool_pub.publish(Bool(data=False))
        self.get_logger().info(f'dead end #{self.seq}: {list(dead)}, facing it ({direction}) '
                               f'from {list(cell)}')


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
