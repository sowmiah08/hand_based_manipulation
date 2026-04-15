import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np


class PixelTo3D(Node):
    def __init__(self):
        super().__init__('pixel_to_3d')

        self.bridge = CvBridge()

        self.depth_image = None
        self.camera_info = None

        # 🔥 smoothing for Z
        self.prev_z = None
        self.alpha = 0.7

        # 🔥 Subscribers
        self.create_subscription(
            Image,
            '/camera/side_camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            '/camera/side_camera/color/camera_info',
            self.info_callback,
            10
        )

        self.create_subscription(
            PointStamped,
            '/hand_pixel',
            self.pixel_callback,
            10
        )

        # 🔥 Publisher
        self.pub = self.create_publisher(PointStamped, '/hand_3d', 10)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def info_callback(self, msg):
        self.camera_info = msg

    def pixel_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            return

        x = int(msg.point.x)
        y = int(msg.point.y)

        h, w = self.depth_image.shape

        # 🔒 bounds check
        if x < 0 or y < 0 or x >= w or y >= h:
            return

        # 🔥 FIX 1: Take neighborhood median (stable depth)
        kernel_size = 3
        depth_values = []

        for i in range(-kernel_size, kernel_size + 1):
            for j in range(-kernel_size, kernel_size + 1):
                nx = x + i
                ny = y + j

                if 0 <= nx < w and 0 <= ny < h:
                    d = self.depth_image[ny, nx]
                    if d > 0:
                        depth_values.append(d)

        if len(depth_values) == 0:
            return

        depth = np.median(depth_values)

        print("Raw depth (mm):", depth)

        # 🔥 Camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # 🔥 Convert to meters
        Z = depth / 1000.0

        # 🔥 FIX 2: Smooth Z
        if self.prev_z is not None:
            Z = self.alpha * self.prev_z + (1 - self.alpha) * Z

        self.prev_z = Z

        # 🔥 Optional: filter bad values
        if Z < 0.2 or Z > 2.0:
            return

        # 🔥 Compute X, Y
        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy

        # 🔥 Publish
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "side_camera_color_optical_frame"

        pt.point.x = float(X)
        pt.point.y = float(Y)
        pt.point.z = float(Z)

        self.pub.publish(pt)

        # 🔥 Clean print
        print(f"\n📍 3D Position:")
        print(f"   X: {X:.3f} m")
        print(f"   Y: {Y:.3f} m")
        print(f"   Z: {Z:.3f} m")


def main():
    rclpy.init()
    node = PixelTo3D()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()