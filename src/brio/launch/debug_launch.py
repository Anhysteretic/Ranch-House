from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='brio_cam_publisher',
        output='screen',
        parameters=[
            {'video_device': '/dev/video4'},
            {'image_width': 1920},
            {'image_height': 1080},
            {'pixel_format': 'yuyv'},
            {'camera_frame_id': 'camera_link'},
            {'io_method': 'mmap'},
        ]
    )

    brio_subscriber_node = Node(
        package='brio',
        executable='brio_subscriber',
        name='brio_display_subscriber',
        output='screen',
    )

    return LaunchDescription([
        usb_cam_node,
        brio_subscriber_node
    ])