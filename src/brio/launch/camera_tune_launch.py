# ~/my_robot_ws/src/my_robot_bringup/launch/april_camera.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts the usb_cam node with a complete and explicit
    set of parameters based on a known working v4l2-ctl configuration.
    It also launches the camera_tuner for live viewing.
    """
    
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=[
            {'video_device': '/dev/video5'}, 
            
            {'image_width': 1280},
            {'image_height': 720},
            {'framerate': 60.0},
            {'pixel_format': 'mjpeg2rgb'},
            {'camera_name': 'logitech_brio'},
            
            # --- User Controls (from your v4l2-ctl log) ---
            {'brightness': 50},
            {'contrast': 128},
            {'saturation': 128},
            {'white_balance_automatic': 1},
            {'gain': 0},
            {'power_line_frequency': 2},
            {'white_balance_temperature': 6560},
            {'sharpness': 128},
            {'backlight_compensation': 1},
            
            # --- Camera Controls (from your v4l2-ctl log) ---
            {'auto_exposure': 3},
            {'exposure_time_absolute': 156},
            {'exposure_dynamic_framerate': 1},
            {'pan_absolute': 0},
            {'tilt_absolute': 0},
            {'focus_absolute': 40},
            {'focus_automatic_continuous': 1},
            {'zoom_absolute': 100},
            
            # --- Logitech-specific LED Controls ---
            {'led1_mode': 3},
            {'led1_frequency': 20},
        ]
    )
    
    # This node is presumably your camera_tuner.py script for live display
    camera_tune_node = Node(
        package='brio', # Make sure 'brio' is the correct name of your package
        executable='camera_tuner',
        name='camera_tuner_display',
        output='screen'
    )

    # The launch description returns the list of nodes to start.
    return LaunchDescription([
        usb_cam_node,
        camera_tune_node
    ])