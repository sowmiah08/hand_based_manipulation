from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # RealSense launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'side_camera'
        }.items()
    )

    # AprilTag launch
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('system_bringup'),
                'launch',
                'apriltag.launch.py'
            )
        )
    )

    # TF broadcaster (tag → base_link)
    tag_to_tf_node = Node(
        package='system_bringup',
        executable='tag_to_tf',
        name='tag_to_tf'
    )

    # Hand tracking node
    hand_tracking_node = Node(
        package='hand_tracking',
        executable='hand_tracker',
        name='hand_tracker'
    )

    return LaunchDescription([
        realsense_launch,
        apriltag_launch,
        tag_to_tf_node,
        hand_tracking_node
    ])