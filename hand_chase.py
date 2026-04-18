import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower.so_follower import SO101Follower
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

URDF_PATH = "./SO101/so101_new_calib.urdf"
EE_FRAME = "gripper_frame_link"
MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "gripper"
]
# Wrist roll is limited to prevent the camera from being damaged by excessive twisting.
LIMITED_URDF_JOINTS = ["wrist_roll"]

IK_INNER_ITERS = 20      
MAX_DEG_PER_STEP = 25.0  

# home position
HOME_Q = np.array([
    -10.6374,
    -100.1319,
     95.2967,
     76.7473,
     16.6667
], dtype=float)


class HandFollower(Node):
    def __init__(self):
        super().__init__('hand_follower')

        self.last_target_time = time.time()

        self.sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self.callback,
            10
        )

        config = SOFollowerRobotConfig(port="/dev/ttyACM1")
        self.robot = SO101Follower(config)
        self.robot.connect()

        # limiting speed for safety
        for motor in self.robot.bus.motors:
            self.robot.bus.write("Moving_Velocity", motor, 40)
            self.robot.bus.write("Acceleration", motor, 15)

        self.kinematics = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name=EE_FRAME,
            joint_names=MOTOR_NAMES,
        )

        for j in LIMITED_URDF_JOINTS:
            self.kinematics.solver.mask_dof(j)
        self.kinematics.solver.dt = 0.05
        self.kinematics.solver.enable_joint_limits(True)
        self.kinematics.solver.enable_velocity_limits(True)
        self.kinematics.solver.add_regularization_task(1e-4)

        self.q_current = HOME_Q.copy()

        print("Moving robot to home position")
        self.move_home()
        self.timer = self.create_timer(1.0, self.idle_check)

        print("Robot ready")

    def move_home(self):
        action = {
            f"{m}.pos": float(HOME_Q[i])
            for i, m in enumerate(MOTOR_NAMES)
        }
        self.robot.send_action(action)
        self.q_current = HOME_Q.copy()

    def idle_check(self):
        # if no target for 3 sec -> go home
        if time.time() - self.last_target_time > 3.0:
            self.move_home()


    def _seed_from_robot(self) -> np.ndarray:
        #Use the current joint positions as the IK starting point.
        # Reusing the last IK values causes drift when the arm hasn't finished moving. 
        # The actual measured joint state is the most accurate starting guess.
        
        obs = self.robot.get_observation()
        return np.array([obs[f"{m}.pos"] for m in MOTOR_NAMES], dtype=float)

    def _solve_ik(self, q_seed: np.ndarray, xyz) -> np.ndarray:
        """Iteratively solve position-only IK to convergence."""
        t_desired = np.eye(4)
        t_desired[:3, 3] = xyz
        q = q_seed.copy()
        for _ in range(IK_INNER_ITERS):
            q = self.kinematics.inverse_kinematics(
                q,
                t_desired,
                position_weight=1.0,
                orientation_weight=0.0, 
            )
        return q

    def callback(self, msg):
        self.last_target_time = time.time()

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        # workspace 
        x = max(min(x, 0.25), -0.25)
        y = max(min(y, 0.25), -0.25)
        z = max(min(z, 0.40), 0.05)

        q_seed = self._seed_from_robot()
        q_target = self._solve_ik(q_seed, (x, y, z))

        # Cap per-callback to prevent big swings when the target jumps or when the seed is near a singularity.
        dq = q_target[:4] - q_seed[:4]
        max_abs = float(np.max(np.abs(dq)))
        if max_abs > MAX_DEG_PER_STEP:
            q_target[:4] = q_seed[:4] + dq * (MAX_DEG_PER_STEP / max_abs)

        self.q_current = q_target

        action = {
            f"{m}.pos": float(q_target[i])
            for i, m in enumerate(MOTOR_NAMES)
        }

        self.robot.send_action(action)

    def destroy_node(self):
        print("Returning to HOME before shutdown...")
        self.move_home()
        time.sleep(2)
        self.robot.disconnect()
        super().destroy_node()

def main():
    rclpy.init()
    node = HandFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("CTRL+C detected")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()