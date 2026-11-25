#!/usr/bin/env python3
"""Sidecar for the team's maze_explore stairs crossing. Changes no team file.

The explorer enters AI mode with MANUAL -> AI_ON -> AI_GAIT and then drives the stairs on
/cmd_vel. In manual mode the robot ignores /cmd_vel, so it stands still. Keyboard teleop in
AI mode worked after `lite3_auto`, so this node watches /simple_cmd and, 0.5 s after it sees
AI_GAIT (0x2101052A), sends AUTO (0x21010C03) once. Nothing else is sent.

Run on the robot in its own terminal, before explore_go:
  export ROS_DOMAIN_ID=4; python3 ~/stairs_detect/ai_auto_helper.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from transfer_interfaces.msg import MotionSimpleCMD

AI_GAIT, AUTO = 0x2101052A, 0x21010C03
DELAY = 0.5


class Helper(Node):
    def __init__(self):
        super().__init__('ai_auto_helper')
        # VOLATILE: don't replay an old latched AI_GAIT from an earlier run
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MotionSimpleCMD, '/simple_cmd', pub_qos)
        self.create_subscription(MotionSimpleCMD, '/simple_cmd', self.on_cmd, sub_qos)
        self.timer = None
        self.get_logger().info('waiting for AI_GAIT 0x2101052A on /simple_cmd')

    def on_cmd(self, msg):
        if msg.cmd_code == AI_GAIT and self.timer is None:
            self.get_logger().info(f'AI_GAIT seen: sending AUTO in {DELAY} s')
            self.timer = self.create_timer(DELAY, self.send_auto)

    def send_auto(self):
        self.timer.cancel()
        self.timer = None
        self.pub.publish(MotionSimpleCMD(cmd_code=AUTO, size=0, type=0))
        self.get_logger().info('/simple_cmd 0x21010C03 (AUTO) sent')


def main():
    rclpy.init()
    node = Helper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
