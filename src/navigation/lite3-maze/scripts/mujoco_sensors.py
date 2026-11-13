"""Generic MuJoCo development sensors and ROS bridge; not a MID360 emulator."""
import math
import time
import queue
import threading
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField, Image, CameraInfo
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster


class Sensors(Node):
    def __init__(self, model, data, rgb=True, stereo=False):
        super().__init__('lite3_mujoco_sensors')
        self.model, self.data = model, data
        self.lidar = model.site('lidar_site').id
        self.torso = model.body('TORSO').id
        self.cmd, self.cmd_time = np.zeros(3), -math.inf
        self.create_subscription(Twist, '/cmd_vel', self.command, 1)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/points', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(Odometry, '/sim/ground_truth/odom', 10)
        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for name, pos, quat in [('lidar', (0, 0, .12), (0, 0, 0, 1)),
                                 ('imu', (0, 0, 0), (0, 0, 0, 1)),
                                 ('camera_optical', (.24, 0, .08), (-.5, .5, -.5, .5)),
                                 ('left_optical', (.24, .03, .08), (-.5, .5, -.5, .5)),
                                 ('right_optical', (.24, -.03, .08), (-.5, .5, -.5, .5))]:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base_link', name
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pos)
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, quat)
            transforms.append(tf)
        self.broadcaster.sendTransform(transforms)
        self.camera_queue = queue.Queue(maxsize=1)
        self.camera_stop = threading.Event()
        self.camera_thread = None
        self.camera_error = None
        self.cameras = ['rgb', 'left', 'right'] if stereo else ['rgb']
        self.image_pubs, self.info_pubs = {}, {}
        for name in self.cameras:
            prefix = '/camera' if name == 'rgb' else '/stereo/' + name
            self.image_pubs[name] = self.create_publisher(Image, prefix + '/image_raw', qos_profile_sensor_data)
            self.info_pubs[name] = self.create_publisher(CameraInfo, prefix + '/camera_info', qos_profile_sensor_data)
        azimuth = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        horizontal = np.column_stack((np.cos(azimuth), np.sin(azimuth), np.zeros(360)))
        self.rays = np.concatenate([horizontal * [math.cos(e), math.cos(e), 0] + [0, 0, math.sin(e)]
                                    for e in np.deg2rad([-45, -30, -15, 0, 15, 30, 50])])
        self.group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.rng = np.random.default_rng(42)
        self.acc_adr = int(model.sensor_adr[np.flatnonzero(model.sensor_type == mujoco.mjtSensor.mjSENS_ACCELEROMETER)[0]])
        self.gyro_adr = int(model.sensor_adr[np.flatnonzero(model.sensor_type == mujoco.mjtSensor.mjSENS_GYRO)[0]])
        if rgb:
            self.camera_thread = threading.Thread(target=self.camera_worker, daemon=True)
            self.camera_thread.start()

    def stamp(self, data=None):
        ns = int((self.data if data is None else data).time * 1e9)
        return Time(sec=ns // 10**9, nanosec=ns % 10**9)

    def command(self, msg):
        values = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if np.isfinite(values).all():
            self.cmd = np.clip(values, [-.45, -.4, -.4], [.4, .4, .4])
            self.cmd_time = time.monotonic()

    def fast(self):
        stamp = self.stamp()
        self.clock_pub.publish(Clock(clock=stamp))
        imu = Imu(header=Header(stamp=stamp, frame_id='imu'))
        q = self.data.qpos[3:7]
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = q
        acc = self.data.sensordata[self.acc_adr:self.acc_adr+3] + self.rng.normal(0, .015, 3)
        gyro = self.data.sensordata[self.gyro_adr:self.gyro_adr+3] + self.rng.normal(0, .001, 3)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        imu.orientation_covariance = [1e-4, 0., 0., 0., 1e-4, 0., 0., 0., 1e-4]
        self.imu_pub.publish(imu)
        truth = Odometry(header=Header(stamp=stamp, frame_id='sim_world'), child_frame_id='base_link')
        truth.pose.pose.position.x, truth.pose.pose.position.y, truth.pose.pose.position.z = self.data.qpos[:3]
        truth.pose.pose.orientation = imu.orientation
        self.truth_pub.publish(truth)

    def lidar_scan(self):
        rotation = self.data.site_xmat[self.lidar].reshape(3, 3)
        directions = (self.rays @ rotation.T).ravel()
        distances = np.empty(len(self.rays), dtype=np.float64)
        ids = np.empty(len(self.rays), dtype=np.int32)
        mujoco.mj_multiRay(self.model, self.data, self.data.site_xpos[self.lidar], directions,
                          self.group, True, self.torso, ids, distances, None, len(ids), 8.)
        valid = (distances >= .06) & (distances <= 8.)
        distances[valid] += self.rng.normal(0, .002, valid.sum())
        scan = LaserScan(header=Header(stamp=self.stamp(), frame_id='lidar'),
                         angle_min=-math.pi, angle_max=math.pi-2*math.pi/360,
                         angle_increment=2*math.pi/360, scan_time=.1, time_increment=0., range_min=.06, range_max=8.)
        horizontal = distances[3*360:4*360].copy()
        horizontal[~valid[3*360:4*360]] = math.inf
        scan.ranges = horizontal.astype(np.float32).tolist()
        self.scan_pub.publish(scan)
        points = (self.rays[valid] * distances[valid, None]).astype('<f4')
        cloud = PointCloud2(header=scan.header, height=1, width=len(points), is_bigendian=False,
                            fields=[PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                    for i, n in enumerate(('x', 'y', 'z'))],
                            point_step=12, row_step=12*len(points), is_dense=True, data=points.tobytes())
        self.cloud_pub.publish(cloud)

    def images(self):
        if self.camera_thread is None or self.camera_error or self.camera_queue.full():
            return
        snapshot = mujoco.MjData(self.model)
        mujoco.mj_copyData(snapshot, self.model, self.data)
        self.camera_queue.put_nowait(snapshot)

    def camera_worker(self):
        renderer = None
        try:
            # OpenGL context and renderer belong exclusively to this worker.
            renderer = mujoco.Renderer(self.model, height=360, width=640)
            while not self.camera_stop.is_set():
                try:
                    snapshot = self.camera_queue.get(timeout=.2)
                except queue.Empty:
                    continue
                self.render_images(renderer, snapshot)
        except Exception as exc:
            self.camera_error = str(exc)
            self.get_logger().error('Camera worker failed: ' + str(exc))
        finally:
            if renderer:
                renderer.close()

    def render_images(self, renderer, snapshot):
        for name in self.cameras:
            renderer.update_scene(snapshot, camera=name)
            pixels = renderer.render()
            header = Header(stamp=self.stamp(snapshot), frame_id='camera_optical' if name == 'rgb' else name + '_optical')
            self.image_pubs[name].publish(Image(header=header, height=360, width=640, encoding='rgb8',
                                                is_bigendian=0, step=1920, data=pixels.tobytes()))
            focal = 180 / math.tan(math.radians(25))
            info = CameraInfo(header=header, height=360, width=640, distortion_model='plumb_bob', d=[0.]*5,
                              k=[focal, 0., 320., 0., focal, 180., 0., 0., 1.],
                              r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                              p=[focal, 0., 320., -focal*.06 if name == 'right' else 0.,
                                 0., focal, 180., 0., 0., 0., 1., 0.])
            self.info_pubs[name].publish(info)

    def close(self):
        self.camera_stop.set()
        if self.camera_thread:
            self.camera_thread.join(timeout=5.)
        self.destroy_node()
