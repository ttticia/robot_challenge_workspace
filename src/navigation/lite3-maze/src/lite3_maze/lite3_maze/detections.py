"""Stable placeholder outputs; optional JSON detector inputs are configurable.

An empty output explicitly says placeholder, never fabricates QR/people results.
"""
import json
from rclpy.node import Node
from std_msgs.msg import String
from .runtime import run


class Detections(Node):
    def __init__(self):
        super().__init__('maze_detections')
        self.pubs, self.inputs, self.latest = {}, {}, {}
        for kind in ('qr', 'people', 'scene'):
            self.declare_parameter(kind+'_input_topic', '')
            self.declare_parameter(kind+'_output_topic', '/maze/'+kind+'/detections')
            self.pubs[kind] = self.create_publisher(String, self.get_parameter(kind+'_output_topic').value, 10)
            topic = self.get_parameter(kind+'_input_topic').value
            self.inputs[kind] = topic
            if topic:
                if topic == self.get_parameter(kind+'_output_topic').value:
                    raise ValueError('Detection input and output must differ')
                self.create_subscription(String, topic, lambda msg, k=kind: self.receive(k, msg), 10)
        self.create_timer(1., self.heartbeat)

    def receive(self, kind, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            self.get_logger().warning('Invalid detector JSON on '+self.inputs[kind])
            return
        self.latest[kind] = self.get_clock().now().nanoseconds
        # Preserve the detector payload verbatim inside the contract.
        self.publish(kind, 'detector', payload)

    def publish(self, kind, source, payload):
        now = self.get_clock().now().nanoseconds
        self.pubs[kind].publish(String(data=json.dumps(dict(schema_version=1, kind=kind,
            source=source, stamp_ns=now, input_topic=self.inputs[kind],
            robot_frame='base_link', localized=False, payload=payload))))

    def heartbeat(self):
        now = self.get_clock().now().nanoseconds
        for kind in self.pubs:
            if kind not in self.latest or now-self.latest[kind] > 2_000_000_000:
                self.publish(kind, 'waiting_for_detector' if self.inputs[kind] else 'placeholder', [])


def main():
    run(Detections)
