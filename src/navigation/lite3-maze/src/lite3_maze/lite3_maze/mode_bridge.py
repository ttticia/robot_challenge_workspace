"""Observe course Auto/Manual events and translate RUG gait intents."""
import json
import time
from rosidl_runtime_py.utilities import get_message
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from .runtime import run

AUTO, MANUAL = 0x21010C03, 0x21010C02
STOP, MOVE = 0x21020C0E, 0x21010D06
GAITS = {'rug_general': 0x21010401, 'rug_high_step': 0x21010407}


class ModeBridge(Node):
    def __init__(self):
        super().__init__('maze_mode_bridge')
        for key, value in dict(command_topic='/simple_cmd',
                               command_type='transfer_interfaces/msg/MotionSimpleCMD',
                               send_gait_commands=False, simulation=False).items():
            self.declare_parameter(key, value)
        self.auto = False
        self.pending_gait = None
        self.last_gait = None
        self.last_sent = -float('inf')
        self.command_pub = None
        self.cls = None
        self.sub = None
        self.auto_pub = self.create_publisher(Bool, '/maze/auto_mode',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(String, '/maze/mode_status', 1)
        self.auto_pub.publish(Bool(data=False))
        self.create_subscription(String, '/robot_mode', self.string_mode, 10)
        self.create_subscription(String, '/maze/gait_intent', self.gait, 10)
        self.create_timer(.2, self.tick)

    def set_mode(self, enabled):
        if enabled and not self.auto and self.command_pub:
            msg = self.cls()
            msg.cmd_code, msg.type = MOVE, 0
            self.command_pub.publish(msg)
        self.auto = enabled
        self.auto_pub.publish(Bool(data=enabled))
        if not enabled:
            self.last_gait = None
        self.get_logger().info('lite3_auto: exploration enabled' if enabled else 'Manual: exploration paused')

    def string_mode(self, msg):
        # This optional normalized adapter is also the simulation mode switch.
        if msg.data in ('lite3_auto', 'auto'):
            self.set_mode(True)
        elif msg.data in ('lite3_manual', 'manual', 'stop'):
            self.set_mode(False)

    def command(self, msg):
        if msg.cmd_code in (AUTO, MANUAL, STOP):
            self.set_mode(msg.cmd_code == AUTO)

    def gait(self, msg):
        try:
            name = json.loads(msg.data)['gait']
            if name in GAITS:
                self.pending_gait = name
        except (ValueError, KeyError, TypeError):
            return

    def tick(self):
        if self.sub is None and not self.get_parameter('simulation').value:
            try:
                self.cls = get_message(self.get_parameter('command_type').value)
                self.sub = self.create_subscription(self.cls, self.get_parameter('command_topic').value, self.command, 10)
                if self.get_parameter('send_gait_commands').value:
                    self.command_pub = self.create_publisher(self.cls, self.get_parameter('command_topic').value, 10)
            except (ImportError, ModuleNotFoundError, AttributeError):
                pass  # Keep /robot_mode adapter available without the transfer overlay.
        if (self.auto and self.pending_gait and self.pending_gait != self.last_gait
                and time.monotonic()-self.last_sent >= 3. and self.command_pub):
            msg = self.cls()
            msg.cmd_code, msg.type = GAITS[self.pending_gait], 0
            self.command_pub.publish(msg)
            self.last_gait, self.last_sent = self.pending_gait, time.monotonic()
        self.status_pub.publish(String(data=json.dumps(dict(auto=self.auto,
            mode_command_observer_ready=self.sub is not None,
            gait_command_publisher_ready=self.command_pub is not None,
            requested_gait=self.pending_gait, sent_gait=self.last_gait))))


def main():
    run(ModeBridge)
