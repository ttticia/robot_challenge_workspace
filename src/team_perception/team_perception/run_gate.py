"""Run a perception node only when maze_explorer wants it.

Dead ends only (user 2026-11-19): maze_explorer publishes /maze/perceive (std_msgs/Bool,
latched; explorer.py at_dead_end) - True while it looks at a dead end (DEAD_END from
scene_trigger's /scene/trigger, the dead-end parts of a JUNCTION look and its follow-up)
and while FACING_QR / LOOKING. Once a /maze/perceive message has arrived, the node runs
only while it is True. Before that (an older explorer, or perception on its own) the
pause_states rule below applies.

maze_explorer publishes its state on /maze/status (std_msgs/String, latched,
"STATE: why" or "STATE"). While that state is one of `pause_states` the node skips its
image work, which leaves the CPU to Fast-LIO / wall_scan / the stair and curtain
manoeuvres. Default pause states:
  STARTING             no map yet; also the start-up window when wall_scan is most fragile
  STAIRS, AT_STAIRS    stair_detector drives / the robot climbs in AI mode
  CURTAIN, AT_CURTAIN  curtain_detector walks through / holds for the operator
Turning, LOOKING, JUNCTION, DEAD_END, FACING_QR etc. keep perception on (scene
understanding needs it). No /maze/status ever received (perception run on its own) or
`status_topic` '' = never paused.

Two levels (user 2026-11-20, qr_scanner):
  active   the dead-end window above (/maze/perceive, else pause_states): QR detection
           (locating unread codes for FACING_QR, slow full scans) and survivor detection.
  running  pause_states only, /maze/perceive ignored: QR READING (plain decodes), which
           stays on everywhere except the stair / curtain manoeuvres.

Parameters (declared by attach()): status_topic, pause_states, perceive_topic ('' = ignore
/maze/perceive: run everywhere but pause_states).
"""
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

DEFAULT_PAUSE = ['STARTING', 'STAIRS', 'AT_STAIRS', 'CURTAIN', 'AT_CURTAIN']
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def state_of(status):
    """'STAIRS: why' -> 'STAIRS'."""
    return str(status).split(':', 1)[0].strip().upper()


class RunGate:
    """No ROS node needed (tests): feed on_status() the status strings, read `active`."""

    def __init__(self, pause_states, logger=None, name='perception'):
        self.pause = {s.strip().upper() for s in pause_states if str(s).strip()}
        self.logger, self.name = logger, name
        self.state = None              # None = no status seen: run
        self.perceive = None           # None = no /maze/perceive seen: pause_states rule

    @property
    def active(self):
        if self.perceive is not None:
            return self.perceive
        return self.state is None or self.state not in self.pause

    @property
    def running(self):
        """pause_states only: True everywhere but STARTING / stairs / curtains."""
        return self.state is None or self.state not in self.pause

    def on_perceive(self, on):
        was = self.active
        self.perceive = bool(on)
        if self.logger is not None and self.active != was:
            self.logger.info(f'{self.name} {"ON" if self.active else "off"} '
                             '(/maze/perceive: dead ends only)')

    def on_status(self, status):
        was = self.active
        self.state = state_of(status)
        if self.logger is not None and self.active != was:
            if self.active:
                self.logger.info(f'maze_explorer {self.state}: {self.name} resumed')
            else:
                self.logger.info(f'maze_explorer {self.state}: {self.name} paused '
                                 f'(pause_states)')


def attach(node, name='perception'):
    """Declare the parameters on node and subscribe; returns the RunGate."""
    node.declare_parameter('status_topic', '/maze/status')
    node.declare_parameter('pause_states', DEFAULT_PAUSE)
    node.declare_parameter('perceive_topic', '/maze/perceive')
    topic = node.get_parameter('status_topic').value
    gate = RunGate(node.get_parameter('pause_states').value, node.get_logger(), name)
    if topic:
        node.create_subscription(String, topic, lambda m: gate.on_status(m.data), LATCHED)
    perceive = node.get_parameter('perceive_topic').value
    if perceive:
        node.create_subscription(Bool, perceive, lambda m: gate.on_perceive(m.data), LATCHED)
    return gate
