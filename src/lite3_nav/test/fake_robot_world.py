"""Stand-in for the Lite3 sensor bringup in offline dry runs of robot_navigation.launch.py.

Publishes what lite3_launcher's bringup provides on the robot: /scan (LaserScan, frame base_link)
of a static 4 m x 3 m room with a gap, odom -> base_link (static: the robot never moves) and /odom.
Run it ONLY on a spare loopback domain (the dry-run script uses 77), never on 4 or 89.
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

WALLS = [((-1.0, -1.5), (3.0, -1.5)), ((3.0, -1.5), (3.0, 1.5)), ((3.0, 1.5), (-1.0, 1.5)),
         ((-1.0, 1.5), (-1.0, 0.4)), ((-1.0, -0.4), (-1.0, -1.5))]   # 0.8 m gap behind the robot


def ray_range(angle, max_range=20.):
    dx, dy = math.cos(angle), math.sin(angle)
    best = max_range
    for (x1, y1), (x2, y2) in WALLS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = (x1 * ey - y1 * ex) / den
        u = (x1 * dy - y1 * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


class FakeWorld(Node):
    def __init__(self):
        super().__init__('fake_robot_world')
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
        tf.transform.rotation.w = 1.
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform(tf)
        self.increment = math.radians(.5)
        self.count = int(round(2 * math.pi / self.increment))
        self.ranges = [float(ray_range(-math.pi + i * self.increment)) for i in range(self.count)]
        self.create_timer(.1, self.tick)

    def tick(self):
        now = self.get_clock().now().to_msg()
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = now, 'base_link'
        msg.angle_min, msg.angle_increment = -math.pi, self.increment
        msg.angle_max = -math.pi + (self.count - 1) * self.increment
        msg.scan_time, msg.range_min, msg.range_max = .1, .4, 20.
        msg.ranges = self.ranges
        self.scan.publish(msg)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, 'odom', 'base_link'
        odom.pose.pose.orientation.w = 1.
        self.odom.publish(odom)


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '4', '89'):
        sys.exit('fake_robot_world: refusing ROS_DOMAIN_ID ' + str(os.environ.get('ROS_DOMAIN_ID')) +
                 ' (use a spare loopback domain such as 77)')
    rclpy.init()
    node = FakeWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
