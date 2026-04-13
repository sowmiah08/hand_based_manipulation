from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/camera/side_camera/color/image_raw'),
                ('camera_info', '/camera/side_camera/color/camera_info')
            ],
            parameters=[{
                'family': '16h5',
                'size': 0.03,
                'detector.threads': 4,
                'detector.decimate': 1.0,
                'detector.blur': 0.8,
                'detector.refine': True,
                'detector.sharpening': 0.25,
                'max_hamming': 1,
            }]
        )
    ])