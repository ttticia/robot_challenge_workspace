"""Simulated Lite3 for testing nav_straight (Nav2) without the robot.

Publishes /livox/points (in livox_frame, 15 deg pitch, Mid-360 vertical FOV -7..52 deg),
/odom, odom->base_link and base_link->livox_frame, and walks from /cmd_vel
(subscribed RELIABLE + TRANSIENT_LOCAL like lite3_domain_bridge_4). Logs /simple_cmd.

Always run it on a spare domain so it cannot meet the real robot:

    export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
    python3 test/fake_cloud_lite3.py --ros-args -p scene:=low_box &
    ros2 launch nav_straight nav_straight.launch.py bringup:=false slam:=false

Scenes: wall (wall 3 m ahead), low_box (0.15 m box 2 m ahead), open (nothing).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:
    from transfer_interfaces.msg import MotionSimpleCMD
except ImportError:
    MotionSimpleCMD = None

BASE_Z = 0.35
LIDAR_T = np.array([0.18678, 0.0, 0.12498])
PITCH = math.radians(15.0)
R_LIDAR = np.array([[math.cos(PITCH), 0, math.sin(PITCH)],   # livox -> base (pitch down)
                    [0, 1, 0],
                    [-math.sin(PITCH), 0, math.cos(PITCH)]])


def grid(xs, ys, zs):
    return np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3)


def scene_points(scene):
    pts = [grid(np.arange(-1, 8, 0.05), np.arange(-2, 2, 0.05), [0.0])]   # floor
    if scene == 'wall':
        pts.append(grid([3.0], np.arange(-2, 2, 0.03), np.arange(0, 1.5, 0.03)))
    elif scene == 'low_box':
        pts.append(grid([2.0], np.arange(-0.3, 0.3, 0.02), np.arange(0, 0.15, 0.02)))
        pts.append(grid(np.arange(2.0, 2.4, 0.02), np.arange(-0.3, 0.3, 0.02), [0.15]))
    return np.concatenate(pts)


class FakeCloudLite3(Node):
    def __init__(self):
        super().__init__('fake_cloud_lite3')
        self.declare_parameter('scene', 'wall')
        self.declare_parameter('yaw_drift', 0.05)   # rad/s of uncommanded turn, tests heading hold
        self.world = scene_points(self.get_parameter('scene').value)
        self.drift = self.get_parameter('yaw_drift').value
        self.x = self.y = self.th = 0.0
        self.cmd = Twist()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos)
        if MotionSimpleCMD is not None:
            self.create_subscription(MotionSimpleCMD, '/simple_cmd', lambda m: self.get_logger()
                                     .info(f'/simple_cmd 0x{m.cmd_code & 0xFFFFFFFF:08X}'), qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/points', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id, st.child_frame_id = 'base_link', 'livox_frame'
        st.transform.translation.x, st.transform.translation.z = LIDAR_T[0], LIDAR_T[2]
        st.transform.rotation.y = math.sin(PITCH / 2)
        st.transform.rotation.w = math.cos(PITCH / 2)
        self.static_tf.sendTransform(st)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_cloud)
        self.create_timer(1.0, lambda: self.get_logger().info(
            f'pose x={self.x:.2f} y={self.y:+.3f} th={math.degrees(self.th):+.1f}deg '
            f'cmd vx={self.cmd.linear.x:.2f} wz={self.cmd.angular.z:+.2f}'))

    def on_cmd(self, msg):
        self.cmd = msg

    def step(self):
        v, w = self.cmd.linear.x, self.cmd.angular.z
        if v != 0.0:
            w += self.drift   # a walking robot doesn't go perfectly straight
        self.th += w * self.dt
        self.x += v * math.cos(self.th) * self.dt
        self.y += v * math.sin(self.th) * self.dt
        stamp = self.get_clock().now().to_msg()
        q = (0.0, 0.0, math.sin(self.th / 2), math.cos(self.th / 2))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        self.odom_pub.publish(o)
        t = TransformStamped()
        t.header = o.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)

    def publish_cloud(self):
        c, s = math.cos(self.th), math.sin(self.th)
        d = self.world - np.array([self.x, self.y, BASE_Z])
        base = np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]], 1)
        lid = (base - LIDAR_T) @ R_LIDAR          # base -> livox (R^T applied row-wise)
        rng = np.linalg.norm(lid, axis=1)
        elev = np.degrees(np.arcsin(lid[:, 2] / np.maximum(rng, 1e-9)))
        lid = lid[(rng > 0.1) & (rng < 40) & (elev > -7) & (elev < 52)].astype(np.float32)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'livox_frame'
        msg.height, msg.width = 1, len(lid)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.point_step, msg.row_step = 12, 12 * len(lid)
        msg.is_dense = True
        msg.data = lid.tobytes()
        self.cloud_pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(FakeCloudLite3())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
