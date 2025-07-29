from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='brio_stream',
            output='screen',
            parameters=[
                {'video_device': '/dev/video5'},
                {'image_width': 1920},
                {'image_height': 1080},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'camera_link'},
                {'io_method': 'mmap'},
            ]
        )
    ])