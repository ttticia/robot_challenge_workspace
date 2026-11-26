#!/usr/bin/env python3
"""Keyboard teleop for the Lite3 on /cmd_vel (robot must be in auto mode: lite3_auto).

Hold a key to move; let go and the robot stops within 0.6 s.
  w / s   forward / back        a / d   left / right (sideways)
  q / e   turn left / right     space   stop now
  + / -   faster / slower       x or Ctrl+C   stop and quit
Run on the robot:  export ROS_DOMAIN_ID=4; python3 ~/stairs_detect/keyboard_teleop.py
"""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

VX, VY, WZ = 0.25, 0.15, 0.5        # team limits (robot_info.md 6.2)
HOLD = 0.6                          # s without a key -> stop (covers key-repeat delay)
KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0),
        'q': (0, 0, 1), 'e': (0, 0, -1)}


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     lifespan=Duration(seconds=0.5))
    pub = node.create_publisher(Twist, '/cmd_vel', qos)
    scale, cmd, last = 0.5, (0, 0, 0), 0.0
    old = termios.tcgetattr(sys.stdin)
    print(__doc__)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                k = sys.stdin.read(1).lower()
                if k in ('x', '\x03'):
                    break
                if k == ' ':
                    cmd = (0, 0, 0)
                elif k in ('+', '='):
                    scale = min(1.0, scale + 0.1)
                elif k in ('-', '_'):
                    scale = max(0.1, scale - 0.1)
                elif k in KEYS:
                    cmd, last = KEYS[k], time.time()
                print(f'\rspeed {scale:.1f}  cmd {cmd}      ', end='', flush=True)
            if time.time() - last > HOLD:
                cmd = (0, 0, 0)
            m = Twist()
            m.linear.x, m.linear.y, m.angular.z = (cmd[0] * VX * scale, cmd[1] * VY * scale,
                                                   cmd[2] * WZ * scale)
            pub.publish(m)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        for _ in range(10):
            pub.publish(Twist())
            time.sleep(0.05)
        print('\nstopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
