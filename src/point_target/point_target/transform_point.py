import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  

class TransformPoint(Node):
    def __init__(self):
        super().__init__('transform_point')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('ema_alpha', 0.25)       
        self.declare_parameter('deadband_m', 0.015)     
        self.declare_parameter('reset_jump_m', 0.25)    

        self.target_frame = self.get_parameter('target_frame').value
        self.alpha = float(self.get_parameter('ema_alpha').value)
        self.deadband = float(self.get_parameter('deadband_m').value)
        self.reset_jump = float(self.get_parameter('reset_jump_m').value)

        self._filt = None          
        self._last_published = None

        self.sub = self.create_subscription(
            PointStamped, '/hand_3d', self.callback, 10)

        self.pub = self.create_publisher(PointStamped, '/target_point', 10)

    def callback(self, msg):
        try:
            msg_time = msg.header.stamp
            msg.header.stamp = rclpy.time.Time().to_msg()  

            transformed = self.tf_buffer.transform(
                msg,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}")
            return

        raw = (transformed.point.x, transformed.point.y, transformed.point.z)

        if self._filt is None or self._dist(raw, self._filt) > self.reset_jump:
            self._filt = raw
        else:
            a = self.alpha
            self._filt = (
                a * raw[0] + (1 - a) * self._filt[0],
                a * raw[1] + (1 - a) * self._filt[1],
                a * raw[2] + (1 - a) * self._filt[2],
            )

        if (self._last_published is not None
                and self._dist(self._filt, self._last_published) < self.deadband):
            return

        transformed.point.x, transformed.point.y, transformed.point.z = self._filt
        transformed.header.stamp = msg_time
        self.pub.publish(transformed)
        self._last_published = self._filt

        self.get_logger().info(
            f"Target ({self.target_frame}): "
            f"{self._filt[0]:.3f}, {self._filt[1]:.3f}, {self._filt[2]:.3f}",
            throttle_duration_sec=0.5)

    @staticmethod
    def _dist(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def main():
    rclpy.init()
    node = TransformPoint()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
