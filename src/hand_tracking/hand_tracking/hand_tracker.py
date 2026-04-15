import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time  

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

        # 🔥 MediaPipe config
        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.3
        )

        self.mp_draw = mp.solutions.drawing_utils

        # 🔥 smoothing
        self.prev_x = None
        self.prev_y = None
        self.alpha = 0.7

        # 🔥 NEW: time control
        self.last_sent_time = 0
        self.interval = 0.4   # ⏱️ adjust (0.3–0.5 ideal)

        # 🔥 NEW: movement threshold
        self.pixel_threshold = 5

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.mp_hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]

            self.mp_draw.draw_landmarks(
                frame,
                hand_landmarks,
                mp.solutions.hands.HAND_CONNECTIONS
            )

            lm = hand_landmarks.landmark[8]

            h, w, _ = frame.shape
            print("height=", h,"width=", w)
            x = float(lm.x * w)
            y = float(lm.y * h)
            print("(x, y)=", x, y)

            # 🔥 smoothing
            if self.prev_x is not None:
                x = self.alpha * self.prev_x + (1 - self.alpha) * x
                y = self.alpha * self.prev_y + (1 - self.alpha) * y

            # 🔥 movement threshold check
            if self.prev_x is not None:
                if abs(x - self.prev_x) < self.pixel_threshold and abs(y - self.prev_y) < self.pixel_threshold:
                    cv2.imshow("Hand Tracking", frame)
                    cv2.waitKey(1)
                    return

            self.prev_x = x
            self.prev_y = y

            # 🔴 draw fingertip
            cv2.circle(frame, (int(x), int(y)), 8, (0, 0, 255), -1)

            # 🔥 time gating
            current_time = time.time()
            if current_time - self.last_sent_time < self.interval:
                cv2.imshow("Hand Tracking", frame)
                cv2.waitKey(1)
                return

            self.last_sent_time = current_time

            # publish
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = "side_camera_color_optical_frame"
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0.0

            self.pub.publish(pt)
            print("📤 Sent:", x, y)

        cv2.imshow("Hand Tracking", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = HandTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()