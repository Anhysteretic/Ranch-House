#!/usr/bin/env python3
# --- FINAL DEFINITIVE VERSION WITH CTRL+C CODE GENERATOR ---

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess

class CameraTunerNode(Node):
    def __init__(self):
        super().__init__('camera_tuner_display')

        # Best practice: use your permanent udev rule link
        self.device_path = '/dev/video4' # Or '/dev/video0'
        self.image_topic = '/camera/image_raw'
        self.control_window_name = 'Camera Control Panel'
        self.video_window_name = 'Live Video Feed'

        self.get_logger().info(f"Starting v4l2-based camera tuner for device '{self.device_path}'")

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        
        self.current_values = {}

        self.param_definitions = {
            'brightness': {'min': 0, 'max': 255},
            'contrast': {'min': 0, 'max': 255},
            'saturation': {'min': 0, 'max': 255},
            'sharpness': {'min': 0, 'max': 255},
            'gain': {'min': 0, 'max': 255},
            'white_balance_automatic': {'min': 0, 'max': 1},
            'white_balance_temperature': {'min': 2000, 'max': 7500},
            'backlight_compensation': {'min': 0, 'max': 1},
            'power_line_frequency': {'min': 0, 'max': 2},
            'focus_automatic_continuous': {'min': 0, 'max': 1},
            'focus_absolute': {'min': 0, 'max': 255},
            'auto_exposure': {'min': 1, 'max': 3},
            'exposure_time_absolute': {'min': 3, 'max': 2047},
            'exposure_dynamic_framerate': {'min': 0, 'max': 1},
            'zoom_absolute': {'min': 100, 'max': 500},
            'led1_mode': {'min': 0, 'max': 3},
            'led1_frequency': {'min': 0, 'max': 255}
        }
        self.create_gui()

    def create_gui(self):
        control_panel_img = np.zeros((1, 600, 3), np.uint8)
        cv2.namedWindow(self.control_window_name)
        cv2.imshow(self.control_window_name, control_panel_img)

        for name, ranges in self.param_definitions.items():
            default_val = self.get_current_param_value(name, fallback=ranges['min'])
            self.current_values[name] = default_val
            cv2.createTrackbar(name, self.control_window_name, default_val, ranges['max'],
                               lambda val, n=name: self.set_camera_parameter(n, val))
        self.get_logger().info("GUI is ready. Press Ctrl+C in the terminal to quit and generate launch code.")

    def get_current_param_value(self, param_name, fallback=0):
        try:
            result = subprocess.run(['v4l2-ctl', '-d', self.device_path, '--get-ctrl', param_name],
                                    capture_output=True, text=True, check=True)
            return int(result.stdout.strip().split(':')[1].strip())
        except Exception:
            return fallback

    def set_camera_parameter(self, param_name, value):
        self.current_values[param_name] = value
        if param_name == 'exposure_time_absolute': self._execute_v4l2_command('auto_exposure', 1)
        elif param_name == 'focus_absolute': self._execute_v4l2_command('focus_automatic_continuous', 0)
        elif param_name == 'white_balance_temperature': self._execute_v4l2_command('white_balance_automatic', 0)
        self._execute_v4l2_command(param_name, value)
    
    def _execute_v4l2_command(self, param_name, value):
        try:
            subprocess.run(['v4l2-ctl', '-d', self.device_path, '--set-ctrl', f'{param_name}={int(value)}'],
                           check=True, capture_output=True)
            self.get_logger().info(f"Successfully set '{param_name}' to {value}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to set '{param_name}': {e.stderr.strip()}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow(self.video_window_name, cv_image)
            cv2.waitKey(1) # This is crucial for the GUI to update
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def print_launch_file_code(self):
        """Generates and prints a copy-paste ready ExecuteProcess block."""
        command_list = [f"'--set-ctrl', '{name}={value}'" for name, value in self.current_values.items()]
        formatted_commands = ',\n            '.join(command_list)
        output_code = f"""
# =================================================================================
# === COPY AND PASTE THIS BLOCK INTO YOUR april_camera.launch.py FILE ===
# =================================================================================
    
    set_camera_controls = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '-d', '{self.device_path}',
            {formatted_commands}
        ],
        shell=False,
        output='screen'
    )
# =================================================================================
"""
        self.get_logger().info(output_code)

def main(args=None):
    rclpy.init(args=args)
    node = CameraTunerNode()
    try:
        # This will run until Ctrl+C is pressed
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This block executes when Ctrl+C is detected
        node.get_logger().info("\n\nCtrl+C detected! Shutting down and generating launch file code...\n")
        node.print_launch_file_code()
    finally:
        # This block ensures a clean shutdown regardless of how the spin stops
        node.get_logger().info("Cleaning up resources.")
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()