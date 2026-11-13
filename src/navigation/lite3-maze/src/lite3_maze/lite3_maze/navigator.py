#!/usr/bin/env python3
"""Sensor-driven cell exploration with a fine-grid A* local path in MuJoCo."""
import json
import math
import time
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException
from rclpy.time import Time
from .geometry import cloud_xyz, ground_height
from .navigation_core import ScanOdometry, GridMap, FrontierPlanner, rotation
from .cells import CellExplorer
from .core import Grid
from .command_stream import CommandStream


class Navigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')
        defaults = dict(localization_source='scan', map_frame='odom', base_frame='base_link',
            command_topic='/cmd_vel', scan_topic='/scan', cloud_topic='/points', imu_topic='/imu/data',
            map_topic='/map', forward_speed=.3, reverse_speed=.4, turn_speed=.4,
            footprint_length=.64, footprint_width=.46, startup_seconds=5.,
            exit_radius=.9, minimum_exit_travel=3., exit_hold_seconds=2.,
            sensor_timeout=.6, tf_timeout=.5, map_timeout=5.)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.external = self.get_parameter('localization_source').value == 'tf'
        self.frame = self.get_parameter('map_frame').value
        self.base = self.get_parameter('base_frame').value
        self.grid = GridMap(extent=8.)
        self.grid.half_length = self.get_parameter('footprint_length').value/2
        self.grid.half_width = self.get_parameter('footprint_width').value/2
        self.odometry = ScanOdometry()
        self.planner = FrontierPlanner(self.grid)
        self.cells = CellExplorer(exit_radius=self.get_parameter('exit_radius').value,
            minimum_travel=self.get_parameter('minimum_exit_travel').value,
            exit_hold=self.get_parameter('exit_hold_seconds').value)
        self.planner.forward_speed = self.get_parameter('forward_speed').value
        self.planner.reverse_speed = self.get_parameter('reverse_speed').value
        self.planner.turn_speed = self.get_parameter('turn_speed').value
        self.auto = False
        self.pose_time = self.map_time = -math.inf
        self.high_roughness_until = -math.inf
        self.declare_parameter('maximum_tilt_deg', 0.)
        self.declare_parameter('max_traversable_height_m', .16)
        self.terrain = 'flat'
        self.semantic_grid = None
        self.xy, self.yaw = np.zeros(2), 0.
        self.imu_yaw, self.tilt = 0., math.inf
        self.roll, self.pitch = 0., 0.
        self.scan_time, self.imu_time, self.cloud_time = -math.inf, -math.inf, -math.inf
        self.first_stamp = None
        self.last_stamp = None
        self.booting = True
        self.next_plan = self.next_publish = 0.
        self.command_topic = self.get_parameter('command_topic').value
        self.command_pub = self.create_publisher(Twist, self.command_topic, 1)
        self.command_stream = CommandStream(self.command_pub)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)) if not self.external else None
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 1)
        self.status_pub = self.create_publisher(String, '/maze/status', 1)
        self.terrain_pub = self.create_publisher(String, '/maze/gait_intent', 1)
        self.markers_pub = self.create_publisher(MarkerArray, '/maze/markers', 1)
        self.tf = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Bool, '/maze/auto_mode', self.mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/maze/reset', self.reset)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu, qos_profile_sensor_data)
        if not self.external:
            self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan, qos_profile_sensor_data)
        else:
            self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value, self.external_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                     self.external_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value, self.cloud, qos_profile_sensor_data)
        self.get_logger().info('Cell exploration + sensor occupancy/A*. Application tilt stop disabled by default.')

    def halt(self):
        self.command_stream.halt()

    def destroy_node(self):
        self.command_stream.close()
        return super().destroy_node()

    def mode(self, msg):
        if not msg.data and self.auto:
            self.halt()
        self.auto = msg.data

    def reset(self, request, response):
        if self.auto:
            response.success, response.message = False, 'Switch to Manual before resetting the mission'
        else:
            self.cells = CellExplorer(exit_radius=self.get_parameter('exit_radius').value,
                minimum_travel=self.get_parameter('minimum_exit_travel').value,
                exit_hold=self.get_parameter('exit_hold_seconds').value)
            self.planner.path, self.planner.goal = [], None
            response.success, response.message = True, 'Exploration history reset; map retained'
        return response

    def external_map(self, msg):
        if msg.header.frame_id != self.frame or not msg.info.width or not msg.info.height:
            return
        if (msg.info.width * msg.info.height != len(msg.data)
                or not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0):
            self.get_logger().error('Ignoring malformed occupancy map: dimensions/data or resolution invalid',
                                    throttle_duration_sec=5.)
            return
        q = msg.info.origin.orientation
        # GridMap supports axis-aligned maps. Refuse rotated maps explicitly.
        if abs(q.x)+abs(q.y)+abs(q.z) > .001:
            self.get_logger().error('Rotated map origin is unsupported; configure an axis-aligned SLAM map')
            return
        values = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        self.grid.resolution = msg.info.resolution
        self.grid.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.grid.log = np.where((values >= 0) & (values <= 25), -4., np.where(values >= 65, 4., 0.)).astype(np.float32)
        self.map_time = time.monotonic()
        self.last_stamp = msg.header.stamp
        self.booting = False

    def external_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, self.base, Time())
            age = (self.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))*1e-9
            if not -.05 <= age <= self.get_parameter('tf_timeout').value:
                return False
            t, q = tf.transform.translation, tf.transform.rotation
            self.xy = np.array([t.x, t.y])
            self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.pose_time = time.monotonic()
            return True
        except TransformException:
            return False

    def external_scan(self, msg):
        """Supplement unknown SLAM cells with fresh, finite laser observations."""
        if self.booting:
            return
        now = time.monotonic()
        if now - self.scan_time < .5:
            return
        age = (self.get_clock().now().nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds) * 1e-9
        if not -.05 <= age <= self.get_parameter('sensor_timeout').value:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        q, p = tf.transform.rotation, tf.transform.translation
        # The course laser is projected into a level base_link plane.
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(np.clip(2*(q.w*q.y-q.z*q.x), -1., 1.))
        if max(abs(roll), abs(pitch)) > math.radians(10):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        ranges = np.asarray(msg.ranges, dtype=float)[::4].copy()
        ranges[~np.isfinite(ranges) | (ranges < msg.range_min)
               | (ranges >= msg.range_max)] = np.nan
        known = self.grid.log.copy()
        self.grid.observe(np.array([p.x, p.y]), yaw, ranges, msg.angle_min,
                          msg.angle_increment*4, min(6., msg.range_max))
        # Live rays may add observations, but cannot erase occupied SLAM cells.
        occupied = known >= .5
        self.grid.log[occupied] = known[occupied]
        self.scan_time = now

    def tilt_exceeded(self):
        limit = self.get_parameter('maximum_tilt_deg').value
        return limit > 0 and self.tilt > math.radians(limit)

    def imu(self, msg):
        q = msg.orientation
        if not np.isfinite([q.x,q.y,q.z,q.w]).all() or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .1:
            return
        self.roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        self.pitch = math.asin(np.clip(2*(q.w*q.y-q.z*q.x), -1., 1.))
        self.imu_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.tilt = max(abs(self.roll), abs(self.pitch))
        self.imu_time = time.monotonic()

    def scan(self, msg):
        now = time.monotonic()
        self.scan_time, self.last_stamp = now, msg.header.stamp
        timestamp = msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec
        if self.first_stamp is None:
            self.first_stamp = timestamp
        self.booting = timestamp-self.first_stamp < self.get_parameter('startup_seconds').value
        if self.booting or now-self.imu_time > .3 or self.tilt_exceeded():
            return
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges) & (ranges > .1) & (ranges < min(6., msg.range_max))
        points3d = ranges[valid, None]*np.column_stack((np.cos(angles[valid]), np.sin(angles[valid]), np.zeros(valid.sum())))
        cr, sr, cp, sp = math.cos(self.roll), math.sin(self.roll), math.cos(self.pitch), math.sin(self.pitch)
        level = points3d @ np.array([[cp, sp*sr, sp*cr], [0., cr, -sr], [-sp, cp*sr, cp*cr]]).T
        keep = level[:, 2] > -.18  # Reject floor returns when the body pitches.
        points = level[keep, :2]
        self.xy, self.yaw = self.odometry.update(points, self.imu_yaw, timestamp)
        if not self.odometry.valid:
            return
        level_ranges = ranges.copy()
        level_angles = angles.copy()
        level_ranges[valid] = np.where(keep, np.linalg.norm(level[:, :2], axis=1), np.nan)
        level_angles[valid] = np.arctan2(level[:, 1], level[:, 0])
        self.grid.observe(self.xy, self.yaw, level_ranges, 0., 0., min(6., msg.range_max), angles=level_angles)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = msg.header.stamp, self.frame, self.base
        tf.transform.translation.x, tf.transform.translation.y = map(float, self.xy)
        tf.transform.translation.z = .30  # Flat-ground nominal height, not an estimated elevation.
        # The cameras move with roll/pitch as well as yaw. Publishing yaw alone
        # would give stereo points the wrong attitude on the recorded roughness.
        cr, sr = math.cos(self.roll/2), math.sin(self.roll/2)
        cp, sp = math.cos(self.pitch/2), math.sin(self.pitch/2)
        cy, sy = math.cos(self.yaw/2), math.sin(self.yaw/2)
        tf.transform.rotation.x = sr*cp*cy-cr*sp*sy
        tf.transform.rotation.y = cr*sp*cy+sr*cp*sy
        tf.transform.rotation.z = cr*cp*sy-sr*sp*cy
        tf.transform.rotation.w = cr*cp*cy+sr*sp*sy
        self.tf.sendTransform(tf)
        odom = Odometry(header=tf.header, child_frame_id='base_link')
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, self.xy)
        odom.pose.pose.orientation = tf.transform.rotation
        odom.pose.covariance[0] = odom.pose.covariance[7] = max(.0025, self.odometry.error**2)
        odom.pose.covariance[35] = .01
        self.odom_pub.publish(odom)

    def cloud(self, msg):
        if self.booting or (not self.external and not self.odometry.valid) or self.tilt_exceeded():
            return
        try:
            points = cloud_xyz(msg)
        except (ValueError, AssertionError, KeyError):
            return
        if self.external:
            try:
                from scipy.spatial.transform import Rotation
                tf = self.tf_buffer.lookup_transform(self.base, msg.header.frame_id, Time.from_msg(msg.header.stamp))
                q, t = tf.transform.rotation, tf.transform.translation
                points = Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[t.x,t.y,t.z]
            except (TransformException, ValueError):
                return
        if len(points) < 30:
            return
        cr, sr, cp, sp = math.cos(self.roll), math.sin(self.roll), math.cos(self.pitch), math.sin(self.pitch)
        level = points @ np.array([[cp, sp*sr, sp*cr], [0., cr, -sr], [-sp, cp*sr, cp*cr]]).T
        floor = ground_height(level)
        if floor is None:
            return
        self.cloud_time = time.monotonic()
        # Separate floor-level roughness/treads from body-height and hanging obstacles.
        # The factory high-step intent is observable; the bundled RL policy stays unchanged.
        height = level[:, 2]-floor
        ahead = (level[:, 0] > .15) & (level[:, 0] < .8) & (np.abs(level[:, 1]) < .30)
        low_mask = ahead & (height > .02) & (height < .20)
        tall = level[height >= .20, :2]
        low_points = level[low_mask, :2]
        low = height[low_mask]
        if len(tall) and len(low_points):
            # Floor-adjacent wall returns are not treads or rough ground.
            low = low[cKDTree(tall).query(low_points)[0] > .12]
        peak = float(np.quantile(low, .9)) if len(low) >= 3 else 0.
        if peak > .05:
            self.high_roughness_until = time.monotonic()+5.
        self.terrain = 'high_roughness' if time.monotonic() < self.high_roughness_until else 'rough'
        obstacles = level[(height > self.get_parameter('max_traversable_height_m').value) & (height < .70)]
        if not self.external:
            self.grid.mark_obstacles(obstacles[:, :2] @ rotation(self.yaw).T + self.xy)

    def update(self):
        now = time.monotonic()
        command = np.zeros(3)
        pose_ready = self.external_pose() if self.external else self.odometry.valid
        timeout = self.get_parameter('sensor_timeout').value
        if self.cells.complete:
            state = 'MAZE_EXIT_REACHED'
        elif not self.auto:
            state = 'WAIT_FOR_LITE3_AUTO'
        elif self.count_publishers(self.command_topic) > 1:
            state = 'COMMAND_SOURCE_CONFLICT'
        elif self.booting:
            state = 'WAIT_FOR_STAND'
        elif (not self.external and now-self.scan_time > timeout) or now-self.imu_time > timeout:
            state = 'SENSOR_TIMEOUT'
        elif self.tilt_exceeded():
            state = 'EXCESS_TILT'
        elif not pose_ready:
            state = 'LOCALIZATION_UNCERTAIN'
        elif now-self.cloud_time > timeout:
            state = 'POINT_CLOUD_TIMEOUT'
        elif self.external and now-self.map_time > self.get_parameter('map_timeout').value:
            state = 'MAP_TIMEOUT'
        else:
            if now >= self.next_plan:
                values = np.full(self.grid.log.shape, -1, dtype=np.int16)
                values[self.grid.log < -.3] = 0
                values[self.grid.log > .5] = 100
                self.semantic_grid = Grid(values, self.grid.resolution, self.grid.origin)
                self.cells.observe(self.semantic_grid, (*self.xy, self.yaw),
                                   self.get_clock().now().nanoseconds*1e-9)
                if not self.cells.complete:
                    self.planner.plan(self.xy, self.cells, self.semantic_grid, self.yaw)
                else:
                    self.planner.path, self.planner.goal = [], None
                self.next_plan = now + 1.
            if self.cells.complete:
                state = 'MAZE_EXIT_REACHED'
            else:
                command, state = self.planner.velocity(self.xy, self.yaw)
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = map(float, command)
        if self.auto:
            self.command_stream.set(twist)
        if now < self.next_publish:
            return
        self.next_publish = now+1.
        status = {'state': state, 'xy_m': self.xy.tolist(), 'yaw_rad': self.yaw,
                  'icp_rmse_m': self.odometry.error if math.isfinite(self.odometry.error) else None,
                  'known_map_cells': int(np.count_nonzero(self.grid.log)),
                  'localization': 'external SLAM map/TF' if self.external else 'IMU heading + planar scan ICP; no loop closure',
                  'goal': self.planner.goal.tolist() if self.planner.goal is not None else None,
                  'clearance_recovery': self.planner.recovering,
                  'desired_heading_rad': self.planner.desired_heading,
                  'travel_camera': 'body-forward stereo',
                  'terrain': self.terrain, 'cells': self.cells.report(),
                  'maximum_tilt_deg': self.get_parameter('maximum_tilt_deg').value,
                  'auto': self.auto, 'localization_source': self.get_parameter('localization_source').value}
        status['scan_registration'] = self.odometry.diagnostics
        self.terrain_pub.publish(String(data=json.dumps(dict(terrain=self.terrain,
            gait='rug_high_step' if self.terrain == 'high_roughness' else 'rug_general',
            factory_gait_emulated=False))))
        self.status_pub.publish(String(data=json.dumps(status)))
        print(json.dumps(status), flush=True)
        self.publish_markers(state)
        if self.last_stamp:
            grid = OccupancyGrid()
            grid.header.stamp, grid.header.frame_id = self.get_clock().now().to_msg(), self.frame
            grid.info.resolution = self.grid.resolution
            grid.info.height, grid.info.width = self.grid.log.shape
            grid.info.origin.position.x, grid.info.origin.position.y = map(float, self.grid.origin)
            grid.info.origin.orientation.w = 1.
            values = np.full(self.grid.log.shape, -1, dtype=np.int8)
            values[self.grid.log < -.3] = 0
            values[self.grid.log > .5] = 100
            grid.data = values.ravel().tolist()
            if self.map_pub:
                self.map_pub.publish(grid)
            path = Path(header=grid.header)
            for xy in self.planner.path:
                pose = PoseStamped(header=grid.header)
                pose.pose.position.x, pose.pose.position.y = map(float, xy)
                pose.pose.orientation.w = 1.
                path.poses.append(pose)
            self.path_pub.publish(path)

    def publish_markers(self, state):
        from geometry_msgs.msg import Point
        output = MarkerArray()
        def marker(ident, kind):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.frame, self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'maze', ident, kind, Marker.ADD
            m.pose.orientation.w = 1.
            m.color.a = .9
            output.markers.append(m)
            return m
        body = marker(0, Marker.LINE_STRIP)
        body.scale.x = .025
        body.color.r, body.color.g, body.color.b = 1., .65, .1
        l, w = self.grid.half_length, self.grid.half_width
        for p in np.array([[-l,-w],[-l,w],[l,w],[l,-w],[-l,-w]]) @ rotation(self.yaw).T+self.xy:
            body.points.append(Point(x=float(p[0]), y=float(p[1]), z=.08))
        text = marker(1, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = float(self.xy[0]), float(self.xy[1]), .8
        text.scale.z, text.color.r, text.color.g, text.color.b = .16, 1., 1., 1.
        text.text = state+'\n'+self.terrain+' / '+str(len(self.cells.visits))+' cells'
        cells = marker(2, Marker.CUBE_LIST)
        cells.scale.x, cells.scale.y, cells.scale.z = .92, .92, .015
        cells.color.g, cells.color.b, cells.color.a = .7, 1., .22
        if self.cells.anchor is not None:
            for c in self.cells.visits:
                p = self.cells.anchor[:2]+rotation(self.cells.anchor[2]) @ (np.asarray(c)*self.cells.size)
                cells.points.append(Point(x=float(p[0]), y=float(p[1]), z=.015))
        goal = marker(3, Marker.SPHERE)
        goal.scale.x = goal.scale.y = goal.scale.z = .14
        goal.color.g = 1.
        if self.planner.goal is not None:
            goal.pose.position.x, goal.pose.position.y = map(float, self.planner.goal)
        else:
            goal.action = Marker.DELETE
        self.markers_pub.publish(output)


def main():
    from .runtime import run
    run(Navigator)
