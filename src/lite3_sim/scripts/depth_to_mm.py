#!/usr/bin/env python3
"""Republish Gazebo's float depth (metres) in the real Orbbec format.

/camera/color/depth/image_m (32FC1, m)  ->  /camera/depth/image_raw (16UC1, mm, 0 = no data)
/camera/color/depth/camera_info_m       ->  /camera/depth/camera_info
Both re-stamped with frame camera_depth_optical_frame, as on the robot.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

FRAME = 'camera_depth_optical_frame'


class DepthToMm(Node):
    def __init__(self):
        super().__init__('depth_to_mm')
        self.pub_img = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.create_subscription(Image, '/camera/color/depth/image_m', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/depth/camera_info_m', self.on_info,
                                 qos_profile_sensor_data)

    def on_image(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().warn(f'unexpected depth encoding {msg.encoding}', throttle_duration_sec=10)
            return
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        mm = np.nan_to_num(d * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
        mm = np.clip(mm, 0, 65535).astype(np.uint16)
        out = Image(header=msg.header, height=msg.height, width=msg.width, encoding='16UC1',
                    is_bigendian=0, step=msg.width * 2, data=mm.tobytes())
        out.header.frame_id = FRAME
        self.pub_img.publish(out)

    def on_info(self, msg):
        msg.header.frame_id = FRAME
        self.pub_info.publish(msg)


def main():
    rclpy.init()
    n = DepthToMm()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
