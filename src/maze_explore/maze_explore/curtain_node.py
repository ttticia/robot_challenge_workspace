"""curtain_node: curtains in the colour camera (YOLO), while the robot is NOT on a room check.

ON by default since 2026-11-25 (explore.launch.py curtain_cv:=false turns it off). The model is the v3 curtain model of
branch curtain_cv (`curtain CV/model/curtain_int8_320x256_openvino_model`, copied to this
package's models/ folder); it is not accepted yet (val: 20 of 23 curtains found, false boxes
on 2 of 51 frames without one), which is why every sighting is verified by the explorer
before it counts (explorer.py, "curtain seen by the camera").

When it works: /explore/status says RUN and the job is not a look. During a room / object
look the camera belongs to scene understanding and the QR scanner, and the bridge slows down
with every extra copy of a stream; so the colour stream is subscribed ONLY outside looks and
dropped again when one starts. Compressed colour only: no depth (one raw depth subscriber
slows every stream from the robot), so a box has a direction but no distance. The explorer
finds the cell edge from the direction and its own map.

  in : <image_topic>/compressed (sensor_msgs/CompressedImage), /explore/status
  out: /curtain/seen    std_msgs/String JSON, one per processed frame, also with no box (so
                        the explorer can count frames): {stamp, detections: [{conf,
                        box: [x0, y0, x1, y1] as fractions of the image, bearing: rad to the
                        box centre, + = left of the camera axis}]}
       /curtain/status  std_msgs/String latched: UNAVAILABLE: why | IDLE | RUNNING
       /curtain/image   sensor_msgs/Image (bgr8): the processed frame with the boxes drawn
                        on it, for rqt_image_view. Only made while something subscribes to
                        it. It is a local topic: viewing it adds no load on the robot bridge
                        (viewing /camera/color/image_raw itself would).

By hand, without the explorer (teleop): every frame is processed.

    ros2 run maze_explore curtain_node --ros-args -p always:=true
    ros2 run rqt_image_view rqt_image_view /curtain/image

The model runs on OpenVINO directly (no ultralytics, no torch: they cost seconds of start-up
and memory), one CPU thread, at `rate` frames a second.

    ros2 run maze_explore curtain_node
"""
import json
import math
import os

import numpy as np

MODEL = 'curtain_int8_320x256_openvino_model'
HFOV = 2.0 * math.atan(320.0 / 462.0)   # colour camera at 640 x 480: fx 462 (robot_info.md), 69 deg


def model_path(model=MODEL):
    """A path is taken as it is; a bare name is looked for in this package's models/ folder
    (installed share, else the source tree)."""
    model = os.path.expanduser(str(model))
    if os.sep in model:
        return model
    here = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    places = [os.path.join(here, 'models', model)]
    try:
        from ament_index_python.packages import get_package_share_directory
        places.insert(0, os.path.join(get_package_share_directory('maze_explore'), 'models', model))
    except Exception:
        pass
    return next((p for p in places if os.path.isdir(p)), places[0])


def letterbox(bgr, height, width, pad=114):
    """The frame scaled to fit height x width, centred on grey, as the model was trained.
    Returns (image, scale, x offset, y offset)."""
    import cv2
    h, w = bgr.shape[:2]
    r = min(height / h, width / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    out = np.full((height, width, 3), pad, np.uint8)
    x0, y0 = (width - nw) // 2, (height - nh) // 2
    out[y0:y0 + nh, x0:x0 + nw] = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return out, r, x0, y0


def decode(pred, conf, scale, x0, y0, shape, iou=0.7):
    """Model output (5, N: centre x, centre y, width, height, score, in letterbox pixels) ->
    [((x0, y0, x1, y1) in frame pixels, score)], best first, overlapping boxes merged."""
    import cv2
    p = np.asarray(pred, np.float32)
    p = p[:, p[4] >= conf]
    if p.shape[1] == 0:
        return []
    boxes = np.stack([p[0] - p[2] / 2, p[1] - p[3] / 2, p[2], p[3]], axis=1)
    keep = cv2.dnn.NMSBoxes(boxes.tolist(), p[4].tolist(), float(conf), float(iou))
    h, w = shape[:2]
    out = []
    for i in np.asarray(keep).reshape(-1):
        x, y, bw, bh = boxes[i]
        out.append(((float(np.clip((x - x0) / scale, 0, w)), float(np.clip((y - y0) / scale, 0, h)),
                     float(np.clip((x + bw - x0) / scale, 0, w)),
                     float(np.clip((y + bh - y0) / scale, 0, h))), float(p[4, i])))
    return sorted(out, key=lambda b: -b[1])


def bearing(x_frac, hfov=HFOV):
    """Direction of an image column (0 = left edge, 1 = right edge): rad, + = to the left."""
    return math.atan((0.5 - x_frac) * 2.0 * math.tan(hfov / 2.0))


class CurtainModel:
    """The exported YOLO curtain model on OpenVINO."""

    def __init__(self, model=MODEL, conf=0.5, threads=1):
        import openvino as ov
        self.path = model_path(model)
        xml = os.path.join(self.path, 'best.xml')
        if not os.path.isfile(xml):
            raise FileNotFoundError(f'no curtain model at {self.path}')
        core = ov.Core()
        net = core.read_model(xml)
        cfg = {'INFERENCE_NUM_THREADS': int(threads)} if threads > 0 else {}
        self.net = core.compile_model(net, 'CPU', cfg)
        _, _, self.height, self.width = (int(v) for v in self.net.input(0).shape)
        self.conf = float(conf)
        self.detect(np.zeros((480, 640, 3), np.uint8))          # warm-up

    def detect(self, bgr):
        """[((x0, y0, x1, y1) pixels, confidence)] of the curtains in a BGR frame."""
        img, r, x0, y0 = letterbox(bgr, self.height, self.width)
        blob = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1)[None], np.float32) / 255.0
        pred = self.net([blob])[self.net.output(0)][0]
        return decode(pred, self.conf, r, x0, y0, bgr.shape)


def frame_report(stamp, dets, shape):
    """The /curtain/seen JSON for one frame."""
    h, w = shape[:2]
    return json.dumps({'stamp': stamp, 'detections': [
        {'conf': round(c, 3),
         'box': [round(b[0] / w, 4), round(b[1] / h, 4), round(b[2] / w, 4), round(b[3] / h, 4)],
         'bearing': round(bearing((b[0] + b[2]) / (2.0 * w)), 4)} for b, c in dets]})


def edge_along(grid, sense, pose, rel_bearing, max_range=2.6, step=0.05):
    """The cell edge a camera sighting lies on: the first edge that is NOT open in the map
    along the ray from pose = (x, y, yaw) at yaw + rel_bearing, within max_range. Returns
    (cell, d) or None. No depth is used: a curtain stops the lidar slice like a wall does, so
    the first closed edge in that direction is where it hangs."""
    x, y, yaw = pose
    a = yaw + rel_bearing
    cell = grid.cell_of(x, y)
    for i in range(1, int(max_range / step) + 1):
        nxt = grid.cell_of(x + i * step * math.cos(a), y + i * step * math.sin(a))
        if nxt == cell:
            continue
        di, dj = nxt[0] - cell[0], nxt[1] - cell[1]
        if abs(di) + abs(dj) != 1:              # through a corner post: no single edge
            return None
        d = ((1, 0), (0, 1), (-1, 0), (0, -1)).index((di, dj))
        if sense.edge(cell, d)[0] != 'open':
            return cell, d
        cell = nxt
    return None


def centred(det, conf):
    """A box that lies across the middle of the image with at least `conf`."""
    b = det.get('box') or [1, 1, 0, 0]
    return det.get('conf', 0.0) >= conf and b[0] <= 0.5 <= b[2]


def watching(status):
    """/explore/status text -> True while the explorer runs and is not on a room / object look.
    Not while it turns either: the explorer drops those frames (their heading is not known),
    so running the model on them would be wasted work. One exception inside a look: the
    explorer's own curtain check before the views (act=cv_verify)."""
    # Not while the depth camera is open (2026-11-26): raw depth already slows every stream
    # from the robot; the colour stream is given up for those seconds.
    if ' act=stair_check ' in status:
        return False
    if not status.startswith('RUN ') or ' act=turn ' in status:
        return False
    # ... except while the explorer checks for a curtain BEFORE a look's views are taken,
    # and during a room side check (act=side_watch, user 2026-11-26 night)
    return 'job=[look ' not in status or ' act=cv_verify ' in status \
        or ' act=side_watch ' in status


def main():
    import cv2
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class CurtainNode(Node):
        def __init__(self):
            super().__init__('curtain_node')
            p = self.declare_parameter
            model = p('model', MODEL).value            # bare name -> this package's models/
            conf = p('conf', 0.5).value
            threads = p('threads', 1).value
            self.rate = p('rate', 3.0).value           # frames a second through the model
            self.topic = p('image_topic', '/camera/color/image_raw').value.rstrip('/') + '/compressed'
            status_topic = p('status_topic', '/explore/status').value
            if p('always', False).value:        # no explorer (teleop): every frame
                status_topic = ''
            self.status_pub = self.create_publisher(String, '/curtain/status', latched)
            self.out_pub = self.create_publisher(String, '/curtain/seen', 10)
            self.image_pub = self.create_publisher(Image, '/curtain/image', 2)
            self.status, self.sub, self.image = None, None, None
            self.on, self.status_time = not status_topic, 0.0
            try:
                self.model = CurtainModel(model, conf, threads)
            except Exception as e:                      # no model, no OpenVINO, ...
                self.model = None
                self.get_logger().error(f'curtain model not available: {e}')
                self.set_status(f'UNAVAILABLE: {e}')
                return
            self.get_logger().info(
                f'curtain model {os.path.basename(self.model.path)} (conf {conf}, {self.rate:g} Hz) '
                + ('outside room checks' if status_topic else 'on every frame'))
            if status_topic:
                self.create_subscription(String, status_topic, self.on_status, 10)
            self.set_status('IDLE')
            self.create_timer(1.0 / float(self.rate), self.tick)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def set_status(self, s):
            if s != self.status:
                self.status = s
                self.status_pub.publish(String(data=s))

        def on_status(self, msg):
            self.on, self.status_time = watching(msg.data), self.now()

        def on_image(self, msg):
            self.image = msg                            # the newest frame only

        def show(self, bgr, dets, header):
            for b, c in dets:
                p0, p1 = (int(b[0]), int(b[1])), (int(b[2]), int(b[3]))
                cv2.rectangle(bgr, p0, p1, (0, 255, 0), 2)
                cv2.putText(bgr, f'curtain {c:.2f}', (p0[0] + 4, max(p0[1] + 18, 18)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            if not dets:
                cv2.putText(bgr, 'no curtain', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255), 2, cv2.LINE_AA)
            h, w = bgr.shape[:2]
            self.image_pub.publish(Image(header=header, height=h, width=w, encoding='bgr8',
                                         is_bigendian=0, step=3 * w, data=bgr.tobytes()))

        def tick(self):
            on = self.on and (self.status_time == 0.0 or self.now() - self.status_time < 2.0)
            if on and self.sub is None:                 # the camera is ours only outside looks
                self.sub = self.create_subscription(CompressedImage, self.topic, self.on_image,
                                                    qos_profile_sensor_data)
            elif not on and self.sub is not None:
                self.destroy_subscription(self.sub)
                self.sub, self.image = None, None
            self.set_status('RUNNING' if on else 'IDLE')
            if not on or self.image is None:
                return
            msg, self.image = self.image, None
            # Half-size JPEG decoding (320 x 240 from the camera's 640 x 480) is what the model
            # takes anyway and costs about a third of a full decode. Full size only while
            # somebody is watching /curtain/image.
            show = self.image_pub.get_subscription_count() > 0
            bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8),
                               cv2.IMREAD_COLOR if show else cv2.IMREAD_REDUCED_COLOR_2)
            if bgr is None:
                return
            dets = self.model.detect(bgr)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.out_pub.publish(String(data=frame_report(stamp, dets, bgr.shape)))
            if show:
                self.show(bgr, dets, msg.header)
            if dets:
                self.get_logger().info('curtain seen: ' + ', '.join(
                    f'{c:.2f} at {math.degrees(bearing((b[0] + b[2]) / (2.0 * bgr.shape[1]))):+.0f} deg'
                    for b, c in dets), throttle_duration_sec=2.0)

    cv2.setNumThreads(1)
    rclpy.init()
    node = CurtainNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
