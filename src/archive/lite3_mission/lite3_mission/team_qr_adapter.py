"""Adapt the existing team QR scanner's reports without replacing its decoder."""
from decimal import Decimal
import json
import math
import time


def team_qr_event(detection):
    """Convert team_perception's JSON schema; never infer missing geometry."""
    if not isinstance(detection, dict):
        raise ValueError('QR detection must be a JSON object')
    text = detection.get('text')
    if not isinstance(text, str) or not 0 < len(text) <= 2048:
        raise ValueError('QR text must contain 1–2048 characters')
    event = dict(kind='qr', label=text, confidence=1.0, source='team_perception')
    stamp = detection.get('stamp')
    if stamp is not None:
        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                or not math.isfinite(stamp) or not 0 <= stamp < 2**31):
            raise ValueError('Invalid QR capture timestamp')
        ns = int(Decimal(str(stamp)) * 10**9)
        if ns:
            event['stamp'] = dict(sec=ns // 10**9, nanosec=ns % 10**9)
    snapshot = detection.get('snapshot')
    if snapshot is not None:
        if not isinstance(snapshot, str) or len(snapshot) > 4096:
            raise ValueError('Invalid QR snapshot path')
        # Evidence metadata only: the adapter never opens or writes this path.
        event['snapshot'] = snapshot
    pixel = detection.get('pixel')
    if pixel is not None:
        if (not isinstance(pixel, list) or len(pixel) != 2
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for x in pixel)):
            raise ValueError('Invalid QR pixel coordinates')
        event['pixel'] = pixel
    position = detection.get('position')
    if position is not None:
        if (not isinstance(position, list) or len(position) != 3
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for x in position)):
            raise ValueError('Invalid QR position')
        frame = detection.get('frame')
        if isinstance(frame, str) and frame.strip() and frame != 'none' and 'stamp' in event:
            event['frame_id'] = frame
            event['position'] = position
        else:
            event['localization_warning'] = 'Position omitted: capture timestamp or frame unavailable'
    return event


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.msg import ParameterDescriptor
    from std_msgs.msg import String
    from .runtime import initialize

    class TeamQRAdapter(Node):
        def __init__(self):
            super().__init__('team_qr_adapter')
            descriptor = ParameterDescriptor(read_only=True)
            self.declare_parameter('input_topic', '/qr/detections', descriptor)
            self.declare_parameter('output_topic', '/mission/observations', descriptor)
            source = self.get_parameter('input_topic').value
            destination = self.get_parameter('output_topic').value
            if self.resolve_topic_name(source) == self.resolve_topic_name(destination):
                raise ValueError('QR adapter input and output must differ')
            self.publisher = self.create_publisher(String, destination, 10)
            self.create_subscription(String, source, self.on_detection, qos_profile_sensor_data)
            self.get_logger().info(source + ' -> ' + destination + '; existing QR scanner is unchanged')

        def on_detection(self, message):
            try:
                if len(message.data) > 65536:
                    raise ValueError('QR report exceeds 64 KiB')

                def invalid(value):
                    raise ValueError('Non-finite JSON number: ' + value)

                event = team_qr_event(json.loads(message.data, parse_constant=invalid))
                self.publisher.publish(String(data=json.dumps(event, allow_nan=False)))
            except (ValueError, TypeError, OverflowError) as exc:
                self.get_logger().warning('Rejected QR report: ' + str(exc), throttle_duration_sec=2.0)

    stop = initialize()
    node = TeamQRAdapter()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(.002)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
