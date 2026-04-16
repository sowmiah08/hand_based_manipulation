import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import numpy as np


class Finger3DFromPixel(Node):
    def __init__(self):
        super().__init__('finger_3d_from_pixel')

        self.bridge = CvBridge()

        self.depth_image = None
        self.camera_info = None

        # ✅ CORRECT topics (aligned depth to color)
        self.create_subscription(
            Image,
            '/camera/side_camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            '/camera/side_camera/aligned_depth_to_color/camera_info',
            self.info_callback,
            10
        )

        # 👉 your finger pixel input
        self.create_subscription(
            PointStamped,
            '/hand_pixel',
            self.pixel_callback,
            10
        )

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    def info_callback(self, msg):
        self.camera_info = msg

    def pixel_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            return

        u = int(msg.point.x)
        v = int(msg.point.y)

        h, w = self.depth_image.shape

        # 🛑 safety check
        if u < 0 or u >= w or v < 0 or v >= h:
            self.get_logger().warn("Pixel out of bounds")
            return

        Z = self.depth_image[v, u]

        if Z == 0:
            self.get_logger().warn("No depth at finger pixel")
            return

        # Convert to meters if needed
        if Z > 10:
            Z = Z / 1000.0

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        self.get_logger().info(
            f"Finger 3D (camera frame): X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Finger3DFromPixel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()