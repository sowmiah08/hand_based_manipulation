# Hand-Guided Teleoperation for SO-ARM100

Real-time teleoperation of the [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) robotic arm using hand tracking and depth sensing. Point with your index finger in front of a depth camera and the robot follows.

Built on ROS 2 Jazzy, MediaPipe, and [LeRobot](https://github.com/huggingface/lerobot).

## How It Works

A RealSense depth camera captures RGB and depth streams. MediaPipe detects the index fingertip in the RGB frame and publishes its pixel coordinates. Those pixels are back-projected into 3D using the depth image and camera intrinsics. An AprilTag fixed to the robot base provides the camera-to-robot transform via TF2, so the 3D point is converted into the robot's coordinate frame. Finally, an IK solver (from LeRobot) computes joint angles and sends them to the arm over serial.

```
RealSense Camera
 ├── RGB ──────────► hand_tracker ──► /hand_pixel (2D)
 ├── Depth ─────────────────────────────┐
 └── CameraInfo ────────────────────────┤
                                        ▼
                                   pixel_to_3d ──► /hand_3d (3D, camera frame)
                                                        │
 apriltag_node ──► /detections                          │
       │                                                ▼
  tag_to_tf ──► TF (camera→base_link) ──► transform_point ──► /target_point (3D, robot frame)
                                                                      │
                                                                      ▼
                                                              hand_follow.py (IK → motor commands)
```

## Packages

| Package | Node | Role |
|---|---|---|
| `hand_tracking` | `hand_tracker` | Detects index fingertip via MediaPipe, publishes `/hand_pixel` |
| `teleop_mapper` | `pixel_to_3d` | Back-projects pixel + depth into a 3D point, publishes `/hand_3d` |
| `point_target` | `transform_point` | Transforms `/hand_3d` from camera frame to `base_link` via TF2, publishes `/target_point` |
| `system_bringup` | `tag_to_tf` | Listens to AprilTag detections, broadcasts the camera→base_link TF |
| `arm_control` | — | Placeholder for future arm control nodes |
| `robot_decription` | — | Robot description / URDF assets |

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/side_camera/color/image_raw` | `sensor_msgs/Image` | RGB image from RealSense |
| `/camera/side_camera/depth/image_rect_raw` | `sensor_msgs/Image` | Aligned depth image |
| `/camera/side_camera/color/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/hand_pixel` | `geometry_msgs/PointStamped` | Fingertip pixel coordinates |
| `/hand_3d` | `geometry_msgs/PointStamped` | Fingertip in 3D (camera frame) |
| `/target_point` | `geometry_msgs/PointStamped` | Fingertip in 3D (robot frame) |
| `/detections` | `apriltag_msgs/AprilTagDetectionArray` | AprilTag detections |

## Requirements

### Hardware

- [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) robotic arm (SO101 variant)
- Intel RealSense depth camera
- AprilTag (16h5 family, 3 cm) fixed to the robot base
- USB serial connection to the arm (`/dev/ttyACM1` by default)

### Software

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- [LeRobot](https://github.com/huggingface/lerobot) (for kinematics and motor control)
- [realsense2_camera](https://github.com/IntelRealSense/realsense-ros) ROS 2 driver
- [apriltag_ros](https://github.com/christianrauch/apriltag_ros) ROS 2 package

## Setup

### 1. Build the workspace

```bash
cd hand_robot_ws
colcon build
source install/setup.bash
```

### 2. Install Python dependencies

```bash
pip install mediapipe opencv-python
```

### 3. Set up LeRobot (in a separate conda env)

```bash
conda create -n lerobot python=3.12
conda activate lerobot
pip install lerobot
```

### 4. Download the URDF

Download [`so101_new_calib.urdf`](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf) and place it at `./SO101/so101_new_calib.urdf` relative to where you run `hand_follow.py`.

## Running

### Full system launch

This starts the RealSense camera, AprilTag detector, tag-to-TF bridge, hand tracker, Pixel-to-3D and Frame transform:

```bash
ros2 launch system_bringup full_system.launch.py
```

Then in a separate terminal:

```bash

# Robot controller (requires LeRobot conda env)
conda activate lerobot
source /opt/ros/jazzy/setup.bash
python hand_follow.py
```

## Configuration

| Parameter | Location | Default | Description |
|---|---|---|---|
| Camera name | `full_system.launch.py` | `side_camera` | RealSense camera namespace |
| AprilTag family | `apriltag.launch.py` | `16h5` | Tag family for detection |
| AprilTag size | `apriltag.launch.py` | `0.03` m | Physical tag edge length |
| Serial port | `hand_follow.py` | `/dev/ttyACM1` | SO101 USB serial device |
| Workspace limits | `hand_follow.py` | x/y: ±0.25 m, z: 0.05–0.4 m | Safety clamp on target position |
| Motor velocity | `hand_follow.py` | 10 | Servo moving velocity limit |
| Motor acceleration | `hand_follow.py` | 3 | Servo acceleration limit |

## Safety

- Motor velocity and acceleration are capped on startup for safe testing.
- Target positions are clamped to a bounded workspace before IK is solved.
- The robot disconnects cleanly on `Ctrl+C`.

## Future Work

- Gesture-based gripper control (pinch to grab)
- Trajectory smoothing and filtering
- Imitation learning via LeRobot
- Multi-camera fusion

## License

Apache-2.0
