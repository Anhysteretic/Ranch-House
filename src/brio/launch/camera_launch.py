# ~/my_robot_ws/src/my_robot_bringup/launch/april_camera.launch.py
# --- FINAL SAFE VERSION ---

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file sets the camera to a stable, default AUTOMATIC state,
    then launches the ROS driver and the tuner GUI.
    """

    camera_device = '/dev/video0'

    set_camera_controls = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '-d', camera_device,
            '--set-ctrl', 'brightness=128',
            '--set-ctrl', 'contrast=128',
            '--set-ctrl', 'gain=0',
            '--set-ctrl', 'sharpness=128',
            '--set-ctrl', 'backlight_compensation=1',
            '--set-ctrl', 'white_balance_automatic=1',
            '--set-ctrl', 'auto_exposure=3',
            '--set-ctrl', 'focus_automatic_continuous=1'
        ],
        shell=False,
        output='screen'
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=[
            {'video_device': camera_device},
            {'image_width': 1280},
            {'image_height': 720},
            {'framerate': 60.0},
            {'pixel_format': 'mjpeg2rgb'},
            {'camera_frame_id': 'camera'}
        ]
    )
    
    return LaunchDescription([
        set_camera_controls,
        usb_cam_node,
    ])