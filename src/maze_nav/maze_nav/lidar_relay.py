"""lidar_relay: take /livox/points from the robot ONCE and pass it on locally.

/livox/points comes from lite3_domain_bridge_4 on another VM, and DDS sends a full copy
(~0.5 MB a cloud, ~5 MB/s) to EVERY process here that subscribes. With wall_scan,
cmd_vel_bridge, stair_detector and curtain_detector each subscribing (plus Fast-LIO's
/livox/lidar), 2026-11-19 robot runs saw clouds 1.1 s late about 10 % of the time (IMU on
time, kernel UDP receive-buffer drops), so Fast-LIO's odom and wall_scan's TF fell ~1 s behind
("no TF within 0.5 s of a cloud", costmap "/wall_scan not updated"). With the relay, one copy
crosses the bridge and the local subscribers get theirs over loopback.

The cloud is forwarded as raw bytes (not decoded in Python), unchanged: same stamp, frame and
fields. Best effort, keep last 2 (all subscribers use qos_profile_sensor_data).

  in : in_topic   (/livox/points)        sensor_msgs/PointCloud2
  out: out_topic  (/livox/points_local)  sensor_msgs/PointCloud2, the same messages

Every `report_period` s it logs the clouds it forwarded and their median delay (receive time -
header stamp), and warns when none arrived.

Every `subscriber_check` s it asks the ROS graph (no data, no subscription) who else
subscribes to in_topic. Each one is another full copy from the bridge (RViz showing
/livox/points, a bag recording it, `ros2 topic hz/echo`), which is what puts the lidar and
Fast-LIO odom late ("no TF within 0.5 s", costmap "/wall_scan … not updated"). It warns with
their node names while there are any, and says once when they are gone. Subscribers on other
machines on the domain are listed too; they cost the bridge, less this machine's link.

    ros2 run maze_nav lidar_relay
"""
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

OUT_QOS = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT)


def stamp_of(raw):
    """Header stamp (s) of a serialized message: CDR, 4-byte encapsulation, then sec, nanosec."""
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    return sec + nsec * 1e-9


class LidarRelay(Node):

    def __init__(self):
        super().__init__('lidar_relay')
        self.declare_parameter('in_topic', '/livox/points')
        self.declare_parameter('out_topic', '/livox/points_local')
        self.declare_parameter('report_period', 30.0)
        self.declare_parameter('subscriber_check', 5.0)   # s; 0 = off
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pub = self.create_publisher(PointCloud2, g('out_topic'), OUT_QOS)
        self.create_subscription(PointCloud2, g('in_topic'), self.on_cloud,
                                 qos_profile_sensor_data, raw=True)
        self.count, self.delays = 0, []
        self.period = float(g('report_period'))
        self.create_timer(self.period, self.report)
        self.in_topic, self.extra, self.warned = g('in_topic'), [], 0.0
        if float(g('subscriber_check')) > 0:
            self.create_timer(float(g('subscriber_check')), self.check_subscribers)
        self.get_logger().info(f'{g("in_topic")} -> {g("out_topic")} (raw, one copy from the '
                               'bridge for all local subscribers)')

    def on_cloud(self, raw):
        self.pub.publish(raw)
        self.count += 1
        if len(self.delays) < 1000:
            try:
                self.delays.append(time.time() - stamp_of(raw))
            except struct.error:
                pass

    def check_subscribers(self):
        """Warn about every other subscriber to in_topic (each costs a bridge copy)."""
        me = (self.get_name(), self.get_namespace())
        extra = sorted({(i.node_namespace.rstrip('/') + '/' + i.node_name).lstrip('/') or '?'
                        for i in self.get_subscriptions_info_by_topic(self.in_topic)
                        if (i.node_name, i.node_namespace) != me})
        now = time.monotonic()
        if extra and (extra != self.extra or now - self.warned > 30.0):
            self.warned = now
            self.get_logger().warn(
                f'{len(extra)} other subscriber(s) on {self.in_topic}: {", ".join(extra)}. '
                'Each is another full lidar copy from the bridge and makes the lidar and odom '
                'late: close them (RViz: show /livox/points_local; bags: record '
                '/livox/points_local)')
        elif not extra and self.extra:
            self.get_logger().info(f'{self.in_topic}: other subscribers gone')
        self.extra = extra

    def report(self):
        if self.count == 0:
            self.get_logger().warn(f'no clouds in {self.period:.0f} s: is the lidar / bridge up?')
        else:
            d = sorted(self.delays)
            self.get_logger().info(
                f'{self.count / self.period:.1f} clouds/s forwarded; delay median '
                f'{d[len(d) // 2]:.2f} s, worst {d[-1]:.2f} s' if d else
                f'{self.count / self.period:.1f} clouds/s forwarded')
        self.count, self.delays = 0, []


def main():
    rclpy.init()
    node = LidarRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
