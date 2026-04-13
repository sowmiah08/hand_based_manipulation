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

        # bounds check
        if y >= self.depth_image.shape[0] or x >= self.depth_image.shape[1]:
            return

        depth = self.depth_image[y, x]

        if depth == 0:
            return

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        Z = depth / 1000.0  # mm → meters
        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "side_camera_color_optical_frame"

        pt.point.x = float(X)
        pt.point.y = float(Y)
        pt.point.z = float(Z)

        self.pub.publish(pt)

        print(f"3D Point: {X:.3f}, {Y:.3f}, {Z:.3f}")

def main():
    rclpy.init()
    node = PixelTo3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
