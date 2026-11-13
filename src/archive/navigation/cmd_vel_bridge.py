"""Forward Nav2 velocity commands to the Lite3 motion bridge on /cmd_vel.

- QoS: Nav2 publishes VOLATILE, but the Lite3 domain bridge can subscribe to
  /cmd_vel as TRANSIENT_LOCAL, which never receives from a VOLATILE publisher.
  This node republishes RELIABLE + TRANSIENT_LOCAL, which works with either
  subscriber. A short lifespan stops a reconnecting bridge replaying an old command.
- Safety: speeds are clamped. When Nav2 goes quiet, one zero command is sent and
  then nothing, so keyboard/joystick teleop works again once a goal finishes.
"""
import signal
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


def clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('input_topic', '/nav_cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('drive_enabled', True)
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.max_linear, self.max_angular = get('max_linear'), get('max_angular')
        self.timeout, self.enabled = get('timeout_sec'), get('drive_enabled')
        self.last_cmd = 0.0
        self.active = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         lifespan=Duration(seconds=self.timeout))
        self.pub = self.create_publisher(Twist, get('output_topic'), qos)
        self.create_subscription(Twist, get('input_topic'), self.on_cmd, 10)
        self.create_timer(0.1, self.watchdog)
        state = 'ENABLED' if self.enabled else 'DISABLED (dry run, nothing is sent)'
        self.get_logger().info(
            f"{get('input_topic')} -> {get('output_topic')}, drive {state}, "
            f'limits {self.max_linear} m/s, {self.max_angular} rad/s')

    def on_cmd(self, msg):
        out = Twist()
        out.linear.x = clamp(msg.linear.x, self.max_linear)
        out.linear.y = clamp(msg.linear.y, self.max_linear)
        out.angular.z = clamp(msg.angular.z, self.max_angular)
        self.last_cmd, self.active = time.monotonic(), True
        self.send(out)

    def watchdog(self):
        if self.active and time.monotonic() - self.last_cmd > self.timeout:
            self.stop('Nav2 stopped sending commands')

    def stop(self, reason):
        if self.active:
            self.send(Twist())
            self.active = False
            self.get_logger().info(f'{reason}: sent stop')

    def send(self, twist):
        if self.enabled:
            self.pub.publish(twist)
        else:
            self.get_logger().info(
                f'[dry run] vx={twist.linear.x:.2f} wz={twist.angular.z:.2f}',
                throttle_duration_sec=2.0)


def main():
    stop = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = CmdVelBridge()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.stop('Shutting down')
        end = time.monotonic() + 0.3  # let the stop command go out
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
