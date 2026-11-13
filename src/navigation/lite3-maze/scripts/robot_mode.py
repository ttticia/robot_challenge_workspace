"""Send an operator mode command after discovery and verify observer receipt."""
import json
import sys
import time

import rclpy
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


def main():
    enabled = sys.argv[1] == 'auto'
    rclpy.init(args=[])
    node = rclpy.create_node('maze_operator_mode_command')
    cls = get_message('transfer_interfaces/msg/MotionSimpleCMD')
    publisher = node.create_publisher(cls, '/simple_cmd', QoSProfile(depth=1))
    received = []
    sub = node.create_subscription(String, '/maze/mode_status',
                                   lambda msg: received.append(json.loads(msg.data)), 10)
    try:
        print('Waiting for robot bridge and maze mode observer...', flush=True)
        deadline = time.monotonic() + 12.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            names = {info.node_name for info in node.get_subscriptions_info_by_topic('/simple_cmd')}
            if {'lite3_domain_bridge_4', 'maze_mode_bridge'} <= names and publisher.get_subscription_count() >= 2:
                break
        else:
            print('Mode command not sent: both subscribers were not discovered.', file=sys.stderr)
            return 1
        # Allow the matched readers to finish discovery before this one-shot event.
        settle = time.monotonic() + 1.
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=.1)
        received.clear()
        command = cls()
        command.cmd_code = 0x21010C03 if enabled else 0x21010C02
        command.size = command.type = 0
        publisher.publish(command)
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if received and received[-1].get('auto') is enabled:
                print('Maze observer confirms ' + ('AUTO' if enabled else 'MANUAL') +
                      '. Confirm the physical mode on the robot.', flush=True)
                return 0
        print('Command sent, but the maze observer did not confirm the requested mode.', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
