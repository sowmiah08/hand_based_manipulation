import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class HandTracker(Node):
    def __init__(self):
        super().__init__('hand_tracker')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/side_camera/color/image_raw',
            self.callback,
            10
        )

        self.pub = self.create_publisher(PointStamped, '/hand_pixel', 10)

        self.mp_hands = mp.solutions.hands.Hands()

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.mp_hands.process(frame)

        if results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark[8]  # index finger tip

            h, w, _ = frame.shape
            x = float(lm.x * w)
            y = float(lm.y * h)

            pt = PointStamped()
            pt.header.frame_id = "side_camera_color_optical_frame"
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0.0

            self.pub.publish(pt)

        cv2.imshow("hand", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = HandTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()