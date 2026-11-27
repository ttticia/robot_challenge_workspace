"""depth_relay: take the camera's raw depth ONCE and hand it to every local depth user at a lower rate.

The depth image comes from lite3_domain_bridge_4, a separate VM, and DDS sends one full copy to
every process that subscribes: 640x480x2 B x 30 Hz = 18 MB/s each. qr_scanner, poster_detector
and maze_nav's stair_detector each subscribing made that 3 copies (~440 Mbit/s) through the
bridge that also carries /livox/points and /cmd_vel (recording raw depth once delayed the
lidar by 1.2 s, agent.md section 10). None of them needs 30 Hz: stair_detector runs at 5 Hz,
poster_detector matches depth to ~5 Hz detections within max_dt_s 0.1, qr_scanner only reads
depth when it finds a code.

So this node subscribes once (BEST_EFFORT: the bridge never waits for it) and republishes each
frame UNCHANGED (same pixels, encoding, stamp, frame_id) at most `rate_hz` times a second,
with the matching camera_info. The bridge then sends one copy, and the local copies go over
loopback at a third of the rate.

  in : in_depth_topic  (/camera/depth/image_raw)     sensor_msgs/Image
       in_info_topic   (/camera/depth/camera_info)   sensor_msgs/CameraInfo ('' = none)
  out: out_depth_topic (/camera/depth_relay/image_raw)
       out_info_topic  (/camera/depth_relay/camera_info), one per forwarded frame
       RELIABLE, keep last 2: matches BEST_EFFORT and RELIABLE subscribers, never blocks.

On demand (on_demand:=true, the launch default for depth since 2026-11-24): the raw depth
stream is NOT kept open. One subscriber to it, even this one, slows every stream from the
robot to about 10 Hz (lidar and its IMU included, which makes odometry lag and jump). So the
relay only subscribes while someone needs depth:
  - an /explore/event that starts with one of `demand_words` ("room", "object": the explorer
    is holding a look into a room), or
  - any message on `demand_capture_topic` (/scene/capture),
and closes the subscription again `demand_hold` s after the last one. on_demand:=false = the
old always-open relay. (maze_explore's stair check opens depth by itself, for ~3 s.)

Started by team_perception robot.launch.py (depth_relay:=true) and by maze_nav.launch.py
(which shares one relay between perception and stair_detector). Run only one.

Colour (compressed:=true, 2026-11-23): the same for the camera's JPEG stream
(sensor_msgs/CompressedImage, no camera_info): robot.launch.py starts it as colour_relay
(/camera/color/image_raw/compressed -> /camera/color_relay/image_raw/compressed) so
qr_scanner and human_detector (survivor posters) share one bridge copy; human / poster
detection was removed on 2026-11-19 because each colour subscriber was another copy.

    ros2 run team_perception depth_relay
    ros2 run team_perception depth_relay --ros-args -p rate_hz:=15.0
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

OUT_QOS = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE)


class RateGate:
    """Pass one frame per period, on a fixed schedule of message stamps.

    A frame passes once its stamp reaches the next slot (minus a quarter period, for stamp
    jitter); the slot then moves on by exactly one period, so 30 Hz in at rate 10 gives every
    3rd frame and exactly 10 Hz out. A plain "one period since the last one" test drifted to
    every 4th frame (7.5 Hz) with a few ms of jitter. After a gap (lost frames) the schedule
    restarts from the frame that ends it, rather than passing a burst."""

    def __init__(self, rate_hz):
        self.period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self.next = None

    def __call__(self, stamp):
        if self.period == 0.0:
            return True
        if self.next is None or stamp < self.next - 2 * self.period:   # start / clock went back
            self.next = stamp + self.period
            return True
        if stamp < self.next - 0.25 * self.period:
            return False
        self.next = self.next + self.period if stamp - self.next < self.period \
            else stamp + self.period
        return True


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class DepthRelay(Node):
    def __init__(self):
        super().__init__('depth_relay')
        p = self.declare_parameter
        p('in_depth_topic', '/camera/depth/image_raw')
        p('in_info_topic', '/camera/depth/camera_info')
        p('out_depth_topic', '/camera/depth_relay/image_raw')
        p('out_info_topic', '/camera/depth_relay/camera_info')
        p('rate_hz', 10.0)               # 0 = forward every frame
        p('stats_period', 60.0)          # s between rate log lines
        p('compressed', False)           # True: relay sensor_msgs/CompressedImage (colour)
        p('on_demand', False)            # depth: open the raw stream only while it is needed
        p('demand_event_topic', '/explore/event')
        p('demand_words', 'room,object')
        p('demand_capture_topic', '/scene/capture')
        p('demand_hold', 6.0)            # s the stream stays open after the last request
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.gate = RateGate(float(g('rate_hz')))
        self.info = None
        self.n_in = self.n_out = 0
        self.last_in = None
        self.kind = 'colour' if g('compressed') else 'depth'
        msg_type = CompressedImage if g('compressed') else Image
        self.depth_pub = self.create_publisher(msg_type, g('out_depth_topic'), OUT_QOS)
        self.info_pub = (self.create_publisher(CameraInfo, g('out_info_topic'), OUT_QOS)
                         if g('in_info_topic') else None)
        self.msg_type, self.in_topic, self.in_info = msg_type, g('in_depth_topic'), g('in_info_topic')
        self.on_demand = bool(g('on_demand'))
        self.hold = float(g('demand_hold'))
        self.words = tuple(w for w in str(g('demand_words')).replace(' ', '').split(',') if w)
        self.subs = []                   # the raw subscriptions, while open
        self.wanted_until = None
        if self.on_demand:
            if g('demand_event_topic'):
                self.create_subscription(String, g('demand_event_topic'), self.on_event, 10)
            if g('demand_capture_topic'):
                self.create_subscription(String, g('demand_capture_topic'),
                                         lambda m: self.want('capture'), 10)
            self.create_timer(0.5, self.close_if_idle)
        else:
            self.open()
        self.stats_period = float(g('stats_period'))
        self.create_timer(5.0, self.watch)
        self.stats_at = self.get_clock().now()
        self.get_logger().info(
            f'{g("in_depth_topic")} -> {g("out_depth_topic")} at <= {g("rate_hz")} Hz'
            + (f'; {g("in_info_topic")} -> {g("out_info_topic")}' if self.info_pub else ''))

    def open(self):
        if self.subs:
            return
        self.subs = [self.create_subscription(self.msg_type, self.in_topic, self.on_depth,
                                              qos_profile_sensor_data)]
        if self.in_info:
            self.subs.append(self.create_subscription(CameraInfo, self.in_info, self.on_info,
                                                      qos_profile_sensor_data))

    def on_event(self, msg):
        if msg.data.startswith(self.words):
            self.want(msg.data[:40])

    def want(self, why):
        if not self.subs:
            self.get_logger().info(f'{self.kind} stream opened ({why})')
        self.wanted_until = self.get_clock().now().nanoseconds * 1e-9 + self.hold
        self.open()

    def close_if_idle(self):
        if self.subs and self.get_clock().now().nanoseconds * 1e-9 > self.wanted_until:
            for sub in self.subs:
                self.destroy_subscription(sub)
            self.subs, self.last_in = [], None
            self.gate.next = None
            self.get_logger().info(f'{self.kind} stream closed (nobody has asked for '
                                   f'{self.hold:.0f} s)')

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        self.n_in += 1
        self.last_in = self.get_clock().now()
        if not self.gate(_stamp(msg.header)):
            return
        self.n_out += 1
        if self.info_pub is not None and self.info is not None:
            self.info.header = msg.header        # intrinsics don't change; stamp them alike
            self.info_pub.publish(self.info)
        self.depth_pub.publish(msg)

    def watch(self):
        now = self.get_clock().now()
        if self.on_demand and not self.subs:
            self.n_in = self.n_out = 0
            self.stats_at = now
            return                           # closed on purpose
        if self.last_in is None or (now - self.last_in).nanoseconds > 5e9:
            self.get_logger().warn(f'no {self.kind} images for 5 s (camera driver / bridge up?)',
                                   throttle_duration_sec=30.0)
        dt = (now - self.stats_at).nanoseconds * 1e-9
        if dt >= self.stats_period:
            self.get_logger().info(f'{self.kind} in {self.n_in / dt:.1f} Hz, out '
                                   f'{self.n_out / dt:.1f} Hz')
            self.n_in = self.n_out = 0
            self.stats_at = now


def main():
    rclpy.init()
    node = DepthRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
