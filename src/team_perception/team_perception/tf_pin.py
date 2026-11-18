"""Keep chosen TF frames under their correct parent in a node's tf2 buffer.

A TF frame has one parent. Since 2026-11-18 the robot's /tf_static (from
lite3_domain_bridge_4, robot side, not ours to change) publishes
camera_depth_optical_frame under TWO parents:
  camera_depth_frame (correct: base_link -> camera_link, pitched 20 deg down, -> ...)
  base_link directly, at (0.25, 0, 0.05) with NO pitch (wrong).
tf2 keeps whichever arrived last, and live it is the wrong one. Depth points then go to
odom / map 20 deg off, and the colour <-> depth alignment is off by 20 deg too (the colour
frame still hangs off camera_depth_frame).

TfParentPin watches /tf_static, remembers the transform each pinned frame has from its
wanted parent, and writes it back into the buffer every time /tf_static arrives and
every `period` s (the node's own TransformListener may take the same message after us).
The values come from the robot's own /tf_static: nothing is hard-coded, and nothing
changes when the duplicate is gone. Each node calls attach() once, after creating its
buffer; parameter `tf_pin` (list of "child:parent", [''] = off).
"""
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_PINS = ['camera_depth_optical_frame:camera_depth_frame']
STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)


def parse_pins(entries):
    """["child:parent", ...] -> {child: parent}; blank or malformed entries are skipped."""
    pins = {}
    for e in entries or []:
        child, sep, parent = str(e).partition(':')
        if sep and child.strip() and parent.strip():
            pins[child.strip().lstrip('/')] = parent.strip().lstrip('/')
    return pins


class TfParentPin:
    """No ROS node needed (tests): feed on_static() the /tf_static messages, call apply()."""

    def __init__(self, buffer, pins, logger=None):
        self.buffer, self.pins, self.logger = buffer, dict(pins), logger
        self.wanted = {}             # child -> TransformStamped from the wanted parent
        self.warned = set()          # (child, other parent) already reported

    def on_static(self, msg):
        for t in msg.transforms:
            child = t.child_frame_id.lstrip('/')
            parent = t.header.frame_id.lstrip('/')
            if child not in self.pins:
                continue
            if parent == self.pins[child]:
                self.wanted[child] = t
            elif (child, parent) not in self.warned and self.logger is not None:
                self.warned.add((child, parent))
                self.logger.warn(f'/tf_static also has {parent} -> {child}; keeping '
                                 f'{self.pins[child]} -> {child} (tf_pin). The robot-side '
                                 'TF publishes a second parent for it')
        self.apply()

    def apply(self):
        for t in self.wanted.values():
            self.buffer.set_transform_static(t, 'tf_pin')


def attach(node, buffer, period=0.5):
    """Declare `tf_pin` on node and keep its frames pinned in buffer. Returns the pin or None."""
    node.declare_parameter('tf_pin', DEFAULT_PINS)
    pins = parse_pins(node.get_parameter('tf_pin').value)
    if not pins:
        return None
    from tf2_msgs.msg import TFMessage
    pin = TfParentPin(buffer, pins, node.get_logger())
    node.create_subscription(TFMessage, '/tf_static', pin.on_static, STATIC_QOS)
    node.create_timer(period, pin.apply)
    return pin
