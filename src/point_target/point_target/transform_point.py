import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped with tf2

class TransformPoint(Node):
    def __init__(self):
        super().__init__('transform_point')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointStamped,
            '/hand_3d',
            self.callback,
            10
        )

        self.pub = self.create_publisher(PointStamped, '/target_point', 10)

    def callback(self, msg):
        try:
            # Use the latest available transform instead of the message timestamp
            # to avoid extrapolation errors when TF lags behind the camera.
            msg_time = msg.header.stamp
            msg.header.stamp = rclpy.time.Time().to_msg()  # time 0 = latest

            transformed = self.tf_buffer.transform(
                msg,
                'base_link',
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Restore the original timestamp so downstream nodes know when
            # the point was actually observed.
            transformed.header.stamp = msg_time

            self.pub.publish(transformed)

            print(f"Target (robot frame): "
                  f"{transformed.point.x:.3f}, "
                  f"{transformed.point.y:.3f}, "
                  f"{transformed.point.z:.3f}")

        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}")

def main():
    rclpy.init()
    node = TransformPoint()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()