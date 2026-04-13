import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TagToTF(Node):
    def __init__(self):
        super().__init__('tag_to_tf')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.callback,
            10
        )

        self.br = tf2_ros.TransformBroadcaster(self)

    def callback(self, msg):
        if len(msg.detections) == 0:
            return

        detection = msg.detections[0]
        tag_frame = f'tag{detection.family}:{detection.id}'

        try:
            tf = self.tf_buffer.lookup_transform(
                'side_camera_color_optical_frame',
                tag_frame,
                rclpy.time.Time(),
            )
        except tf2_ros.TransformException:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'side_camera_color_optical_frame'
        t.child_frame_id = 'base_link'
        t.transform = tf.transform

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = TagToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()