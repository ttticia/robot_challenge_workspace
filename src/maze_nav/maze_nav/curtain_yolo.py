"""curtain_yolo: curtains in the colour camera with a YOLO model, placed with the depth camera.

OFF by default (maze_nav.launch.py curtain_yolo:=false): there is no trained model yet, and
no curtain dataset (to_do.md ## curtains: data collection). Once it is trained, it can replace
the lie-down gap check as the curtain verification (curtain_detector inspect_mode 'yolo',
launch curtain_inspect_mode:=yolo): the robot stands in front of a released curtain candidate
and the candidate is confirmed when this node sees a curtain box there on enough frames.

Every 1/`rate` s, while maze_explorer's /maze/status is one of `run_states` (default: the
junction / dead-end looks and CURTAIN, i.e. not while EXPLORING: CPU, and the explorer only
takes curtain candidates in its looks), the newest colour frame goes through the model. Each
box of class `class_id` with confidence >= `conf` is placed with the newest depth frame (no
older than `depth_max_age` s): depth pixels -> points in the depth optical frame -> projected
into the colour image with the colour intrinsics (colour and depth taken as co-located, as in
tools/curtain_yolo_frames.py: ~1-3 cm) -> the points inside the box (shrunk by `shrink` each
side) -> their `near_quantile` forward distance (the fabric, not what shows through the gap)
and median lateral offset in base_link -> odom (TF at the latest pose).

  in : <image_topic>/compressed (sensor_msgs/CompressedImage) or image_topic (Image),
       color_info_topic, depth_topic + depth_info_topic (the depth relay), /maze/status,
       TF base_link -> camera_link (static), odom -> base_link
  out: /curtain/yolo         std_msgs/String JSON, one per processed frame (also with no box,
                             so a verifier can count frames): {stamp, frame: odom, model,
                             detections: [{conf, box: [x0, y0, x1, y1] normalised, range,
                             lateral (base_link, m), centre: [x, y] odom or null}]}
       /curtain/yolo_status  std_msgs/String latched: UNAVAILABLE: why | IDLE | RUNNING

    ros2 run maze_nav curtain_yolo --ros-args -p model:=~/models/curtain_yolo.pt
"""
import json
import math
import os
import time

import numpy as np

CACHE = os.path.expanduser('~/.cache/team_perception')
OPENVINO_SUFFIX = '_openvino_model'


def resolve_model(model):
    """Path of a model: a bare name is looked for in ~/.cache/team_perception/."""
    model = os.path.expanduser(str(model))
    if os.sep in model:
        return model
    return os.path.join(CACHE, model)


def points_in_box(pts_opt, k, box, shrink=0.1):
    """Mask of points (N, 3, colour optical frame) whose projection with intrinsics
    k = (fx, fy, cx, cy) falls inside box (x0, y0, x1, y1, pixels) shrunk by `shrink` of its
    size on each side."""
    pts = np.asarray(pts_opt, float)
    if len(pts) == 0:
        return np.zeros(0, bool)
    fx, fy, cx, cy = k
    x0, y0, x1, y1 = box
    dx, dy = (x1 - x0) * shrink, (y1 - y0) * shrink
    z = pts[:, 2]
    ok = z > 0.05
    u = np.where(ok, fx * pts[:, 0] / np.where(ok, z, 1.0) + cx, -1.0)
    v = np.where(ok, fy * pts[:, 1] / np.where(ok, z, 1.0) + cy, -1.0)
    return ok & (u >= x0 + dx) & (u <= x1 - dx) & (v >= y0 + dy) & (v <= y1 - dy)


def place(pts_base, near_quantile=0.2, min_points=15):
    """(range, lateral) in base_link of the points inside a box: the near_quantile forward
    distance (the fabric; points seen through the gap under it are further) and the median
    lateral offset (+ = left). None with fewer than min_points."""
    p = np.asarray(pts_base, float)
    if len(p) < min_points:
        return None
    x = float(np.quantile(p[:, 0], near_quantile))
    near = p[p[:, 0] <= x + 0.15]
    return x, float(np.median(near[:, 1]))


def to_odom(xy_base, base_pose):
    """base_link (x, y) -> odom with base_pose = ((x, y), yaw)."""
    (bx, by), yaw = base_pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (bx + c * xy_base[0] - s * xy_base[1], by + s * xy_base[0] + c * xy_base[1])


def matching_hits(detections, target_xy, radius, conf):
    """Detections (one /curtain/yolo frame's list) with confidence >= conf and an odom centre
    within radius m of target_xy."""
    out = []
    for d in detections or []:
        c = d.get('centre')
        if c is None or d.get('conf', 0.0) < conf:
            continue
        if math.hypot(c[0] - target_xy[0], c[1] - target_xy[1]) <= radius:
            out.append(d)
    return out


class CurtainYolo:
    """A YOLO model (ultralytics; .pt, or an *_openvino_model directory) -> curtain boxes."""

    def __init__(self, model, conf=0.5, imgsz=416, device='cpu', threads=2, class_id=0):
        path = resolve_model(model)
        # The OpenVINO copy train_curtain_yolo.py --install exports next to the .pt: ~40 % less
        # CPU a frame on the robot (yolo11n 416: 56 vs 92 ms CPU, benchmark 2026-11-23).
        ov = path[:-3] + OPENVINO_SUFFIX if path.endswith('.pt') else None
        if ov and os.path.isdir(ov):
            path = ov
        self.openvino = path.rstrip('/').endswith(OPENVINO_SUFFIX)
        if not os.path.exists(path):
            raise FileNotFoundError(f'no model at {path} (train one: '
                                    'maze_nav/tools/train_curtain_yolo.py)')
        os.environ.setdefault('YOLO_OFFLINE', '1')
        import torch                                   # slow (~1 s): only when used
        from ultralytics import YOLO
        if threads > 0:
            torch.set_num_threads(int(threads))
        self.path, self.conf, self.imgsz, self.device = path, conf, imgsz, device
        self.class_id = int(class_id)
        self.model = YOLO(path, task='detect')
        self.detect(np.zeros((imgsz, imgsz, 3), np.uint8))   # warm-up

    def detect(self, bgr):
        """[(x0, y0, x1, y1) pixels, conf] of the curtain class."""
        r = self.model.predict(bgr, classes=[self.class_id], conf=self.conf, imgsz=self.imgsz,
                               device=self.device, verbose=False)[0]
        boxes = r.boxes.xyxy.cpu().numpy().astype(float)
        confs = r.boxes.conf.cpu().numpy()
        return [(tuple(b), float(c)) for b, c in zip(boxes, confs)]


def main():
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from maze_nav.curtains import depth_points
    from maze_nav.stairs import camera_from_link

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def tf_rt(t):
        q, a = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return r, np.array([a.x, a.y, a.z])

    class Node_(Node):
        def __init__(self):
            super().__init__('curtain_yolo')
            p = self.declare_parameter
            p('model', 'curtain_yolo.pt')       # bare name -> ~/.cache/team_perception/
            p('conf', 0.5)
            p('imgsz', 416)
            p('device', 'cpu')
            p('threads', 2)
            p('class_id', 0)
            p('rate', 2.0)                      # Hz
            p('image_topic', '/camera/color/image_raw')
            p('image_transport', 'compressed')  # 'compressed' = <image_topic>/compressed
            p('color_info_topic', '/camera/color/camera_info')
            p('depth_topic', '/camera/depth_relay/image_raw')
            p('depth_info_topic', '/camera/depth_relay/camera_info')
            p('depth_max_age', 0.5)
            p('camera_link_frame', 'camera_link')
            p('base_frame', 'base_link')
            p('odom_frame', 'odom')
            p('status_topic', '/maze/status')   # '' = always run
            p('run_states', ['JUNCTION', 'DEAD_END', 'CURTAIN'])
            p('shrink', 0.1)
            p('near_quantile', 0.2)
            g = lambda n: self.get_parameter(n).value  # noqa: E731
            self.g = g
            self.status_pub = self.create_publisher(String, '/curtain/yolo_status', latched)
            self.out_pub = self.create_publisher(String, '/curtain/yolo', 10)
            self.status = None
            try:
                self.yolo = CurtainYolo(g('model'), g('conf'), int(g('imgsz')), g('device'),
                                        int(g('threads')), int(g('class_id')))
            except Exception as e:      # no model yet, ultralytics missing, ...
                self.yolo = None
                self.get_logger().error(f'curtain YOLO not available: {e}')
                self.set_status(f'UNAVAILABLE: {e}')
                return
            self.get_logger().info(f'curtain YOLO {self.yolo.path} (conf {g("conf")}, imgsz '
                                   f'{g("imgsz")}) while /maze/status is '
                                   f'{"/".join(g("run_states")) if g("status_topic") else "anything"}')
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.image, self.k_color, self.k_depth, self.depth = None, None, None, None
            self.cam, self.explorer = None, None
            if g('image_transport') == 'compressed':
                self.create_subscription(CompressedImage, g('image_topic').rstrip('/') +
                                         '/compressed', self.on_compressed,
                                         qos_profile_sensor_data)
            else:
                self.create_subscription(Image, g('image_topic'), self.on_raw,
                                         qos_profile_sensor_data)
            self.create_subscription(CameraInfo, g('color_info_topic'),
                                     lambda m: setattr(self, 'k_color', (m.k[0], m.k[4], m.k[2], m.k[5])),
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, g('depth_info_topic'),
                                     lambda m: setattr(self, 'k_depth', (m.k[0], m.k[4], m.k[2], m.k[5])),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, g('depth_topic'), self.on_depth,
                                     qos_profile_sensor_data)
            if g('status_topic'):
                self.create_subscription(String, g('status_topic'), lambda m: setattr(
                    self, 'explorer', m.data.split(':', 1)[0].strip()), latched)
            self.set_status('IDLE')
            self.create_timer(1.0 / float(g('rate')), self.tick)

        def set_status(self, s):
            if s != self.status:
                self.status = s
                self.status_pub.publish(String(data=s))

        def on_compressed(self, msg):
            self.image = (msg.header.stamp, msg.data, True)

        def on_raw(self, msg):
            self.image = (msg.header.stamp, msg, False)

        def on_depth(self, msg):
            enc = msg.encoding
            if enc in ('16UC1', 'mono16'):
                d = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, -1)
                d = d[:, :msg.width].astype(np.float32) * 0.001
            elif enc == '32FC1':
                d = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, -1)
                d = d[:, :msg.width]
            else:
                return
            self.depth = (time.monotonic(), d)

        def decode(self):
            stamp, data, compressed = self.image
            if compressed:
                return stamp, cv2.imdecode(np.frombuffer(bytes(data), np.uint8),
                                           cv2.IMREAD_COLOR)
            img = np.frombuffer(bytes(data.data), np.uint8).reshape(data.height, data.width, -1)
            return stamp, (img[:, :, ::-1].copy() if data.encoding == 'rgb8' else img)

        def base_pose(self):
            try:
                t = self.tf.lookup_transform(self.g('odom_frame'), self.g('base_frame'), Time())
            except TransformException:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y), yaw

        def camera(self):
            if self.cam is None:
                try:
                    self.cam = camera_from_link(*tf_rt(self.tf.lookup_transform(
                        self.g('base_frame'), self.g('camera_link_frame'), Time())))
                except TransformException:
                    return None
            return self.cam

        def tick(self):
            g = self.g
            if g('status_topic') and self.explorer not in g('run_states'):
                return self.set_status('IDLE')
            if self.image is None:
                return
            self.set_status('RUNNING')
            stamp, bgr = self.decode()
            self.image = None
            if bgr is None:
                return
            h, w = bgr.shape[:2]
            dets = []
            pts_opt = None
            if self.depth is not None and self.k_depth is not None and \
                    time.monotonic() - self.depth[0] <= g('depth_max_age'):
                pts_opt = depth_points(self.depth[1], self.k_depth, np.eye(3), np.zeros(3),
                                       stride=4, max_depth=3.0)
            cam, pose = self.camera(), self.base_pose()
            for box, conf in self.yolo.detect(bgr):
                det = {'conf': round(conf, 3),
                       'box': [round(box[0] / w, 4), round(box[1] / h, 4),
                               round(box[2] / w, 4), round(box[3] / h, 4)],
                       'range': None, 'lateral': None, 'centre': None}
                if pts_opt is not None and self.k_color is not None and cam is not None:
                    sel = pts_opt[points_in_box(pts_opt, self.k_color, box, g('shrink'))]
                    pl = place(sel @ cam[0].T + cam[1], g('near_quantile'))
                    if pl is not None:
                        det['range'], det['lateral'] = round(pl[0], 3), round(pl[1], 3)
                        if pose is not None:
                            det['centre'] = [round(v, 3) for v in to_odom(pl, pose)]
                dets.append(det)
            self.out_pub.publish(String(data=json.dumps({
                'stamp': f'{stamp.sec}.{stamp.nanosec:09d}', 'frame': g('odom_frame'),
                'model': os.path.basename(self.yolo.path), 'detections': dets})))
            if dets:
                self.get_logger().info(
                    'curtain YOLO: ' + ', '.join(
                        f'{d["conf"]:.2f}' + (f' at {d["range"]:.2f} m' if d['range'] else '')
                        for d in dets), throttle_duration_sec=2.0)

    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = Node_()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
