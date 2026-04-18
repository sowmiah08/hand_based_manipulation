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

        # Used the depth image aligned to the color frame so hand tracker pixel coordinates map directly to depth samples.
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

        self.create_subscription(
            PointStamped,
            '/hand_pixel',
            self.pixel_callback,
            10
        )

        self.pub = self.create_publisher(PointStamped, '/hand_3d', 10)

        self.get_logger().info('pixel_to_3d started, waiting for depth + camera_info...')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def info_callback(self, msg):
        self.camera_info = msg

    def pixel_callback(self, msg):
        if self.depth_image is None:
            self.get_logger().warn('No depth image yet', throttle_duration_sec=2.0)
            return
        if self.camera_info is None:
            self.get_logger().warn('No camera_info yet', throttle_duration_sec=2.0)
            return

        x = int(msg.point.x)
        y = int(msg.point.y)

        h, w = self.depth_image.shape[:2]

        if y >= h or x >= w or x < 0 or y < 0:
            self.get_logger().warn(
                f'Pixel ({x},{y}) out of depth bounds ({w}x{h})',
                throttle_duration_sec=2.0)
            return

        # Search nearby pixels for a valid depth value, as depth sensors return 0 at exact points.
        radius = 5
        y_min = max(0, y - radius)
        y_max = min(h, y + radius + 1)
        x_min = max(0, x - radius)
        x_max = min(w, x + radius + 1)
        region = self.depth_image[y_min:y_max, x_min:x_max]
        valid = region[region > 0]

        if valid.size == 0:
            self.get_logger().warn(
                f'No valid depth at pixel ({x},{y})',
                throttle_duration_sec=2.0)
            return

        depth = float(np.median(valid))
        print("the depth is", depth)

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        Z = depth / 1000.0  
        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "side_camera_color_optical_frame"

        pt.point.x = float(X)
        pt.point.y = float(Y)
        pt.point.z = float(Z)

        self.pub.publish(pt)

        self.get_logger().info(
            f'3D Point: {X:.3f}, {Y:.3f}, {Z:.3f}',
            throttle_duration_sec=0.5)

def main():
    rclpy.init()
    node = PixelTo3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
