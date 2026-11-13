"""Simulated Lite3 for testing without the robot.

A 7 x 4 m room with a box: publishes /scan, /odom and odom->base_link, and drives from
/cmd_vel (subscribed RELIABLE + TRANSIENT_LOCAL, like lite3_domain_bridge_4).
Always run it on a spare domain so it cannot meet the real robot:

    export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
    python3 src/navigation/test/fake_lite3.py &
    ros2 launch navigation mapping.launch.py bringup:=false map_dir:=/tmp/sim_maps
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

# Axis-aligned boxes (xmin, xmax, ymin, ymax).
# Rays hit the room walls from inside and the box from outside.
ROOM = (-2.0, 5.0, -2.0, 2.0)
BOX = (1.5, 2.0, -0.6, 0.6)
N = 720


def ray_box(px, py, dx, dy, box, inside):
    xmin, xmax, ymin, ymax = box
    with np.errstate(divide='ignore', invalid='ignore'):
        tx1, tx2 = (xmin - px) / dx, (xmax - px) / dx
        ty1, ty2 = (ymin - py) / dy, (ymax - py) / dy
    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    if inside:
        return tmax
    return np.where((tmax >= tmin) & (tmin > 0), tmin, np.inf)


class Fake(Node):
    def __init__(self):
        super().__init__('fake_lite3')
        self.x = self.y = self.th = 0.0
        self.cmd = Twist()
        self.moved = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.scan)
        self.create_timer(2.0, lambda: self.get_logger().info(
            f'pose x={self.x:.2f} y={self.y:.2f} th={self.th:.2f} moved={self.moved:.2f} '
            f'cmd vx={self.cmd.linear.x:.2f} wz={self.cmd.angular.z:.2f}'))

    def on_cmd(self, msg):
        self.cmd = msg

    def step(self):
        v, w = self.cmd.linear.x, self.cmd.angular.z
        self.x += v * math.cos(self.th) * self.dt
        self.y += v * math.sin(self.th) * self.dt
        self.th += w * self.dt
        self.moved += abs(v) * self.dt
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z = math.sin(self.th / 2)
        t.transform.rotation.w = math.cos(self.th / 2)
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist = self.cmd
        self.odom_pub.publish(o)

    def scan(self):
        a = np.linspace(-math.pi, math.pi, N, endpoint=False)
        dx, dy = np.cos(a + self.th), np.sin(a + self.th)
        r = np.minimum(ray_box(self.x, self.y, dx, dy, ROOM, True),
                       ray_box(self.x, self.y, dx, dy, BOX, False))
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.get_clock().now().to_msg(), 'base_link'
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / N
        s.angle_max = s.angle_min + s.angle_increment * (N - 1)
        s.scan_time, s.range_min, s.range_max = 0.1, 0.4, 20.0
        s.ranges = r.astype(np.float32).tolist()
        self.scan_pub.publish(s)


def main():
    rclpy.init()
    rclpy.spin(Fake())


if __name__ == '__main__':
    main()
