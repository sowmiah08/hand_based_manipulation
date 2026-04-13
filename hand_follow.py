# this should comer in lerobot file from https://github.com/huggingface/lerobot.git
# Download the URDF from:
# https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
# Place it at: ./SO101/so101_new_calib.urdf
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower.so_follower import SO101Follower
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.utils.rotation import Rotation

# Download the URDF from:
# https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
# Place it at: ./SO101/so101_new_calib.urdf
URDF_PATH = "./SO101/so101_new_calib.urdf"
EE_FRAME = "gripper_frame_link"
MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class HandFollower(Node):
    def __init__(self):
        super().__init__('hand_follower')

        self.sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self.callback,
            10
        )

        config = SOFollowerRobotConfig(port="/dev/ttyACM1")
        self.robot = SO101Follower(config)
        self.robot.connect()

        # Limit motor speed for safe testing
        for motor in self.robot.bus.motors:
            self.robot.bus.write("Moving_Velocity", motor, 10)
            self.robot.bus.write("Acceleration", motor, 3)

        # Initialize IK solver
        self.kinematics = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name=EE_FRAME,
            joint_names=MOTOR_NAMES,
        )

        # Read initial joint positions as IK seed
        obs = self.robot.get_observation()
        self.q_current = np.array(
            [obs[f"{m}.pos"] for m in MOTOR_NAMES], dtype=float
        )

        print("✅ Robot ready (slow mode, IK enabled)")

    def callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.get_logger().info(f"Target: {x:.3f}, {y:.3f}, {z:.3f}")

        # Safety clamp on workspace bounds (meters)
        x = max(min(x, 0.25), -0.25)
        y = max(min(y, 0.25), -0.25)
        z = max(min(z, 0.4), 0.05)

        # Build desired EE pose as 4x4 transform (position only, keep current orientation)
        t_desired = np.eye(4, dtype=float)
        t_fk = self.kinematics.forward_kinematics(self.q_current)
        t_desired[:3, :3] = t_fk[:3, :3]  # preserve current orientation
        t_desired[:3, 3] = [x, y, z]

        # Solve IK
        q_target = self.kinematics.inverse_kinematics(
            self.q_current, t_desired,
            position_weight=1.0,
            orientation_weight=0.01,
        )
        self.q_current = q_target

        # Build joint-space action
        action = {f"{m}.pos": float(q_target[i]) for i, m in enumerate(MOTOR_NAMES)}
        action["gripper.pos"] = 50.0

        self.robot.send_action(action)
        self.get_logger().info("Command sent")

    def destroy_node(self):
        self.robot.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = HandFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()