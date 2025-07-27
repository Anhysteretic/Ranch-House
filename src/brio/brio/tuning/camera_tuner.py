#!/usr/bin/env python3
# --- CORRECTED AND ROBUST VERSION ---

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

        # It's highly recommended to use your permanent udev rule link, 
        # e.g., /dev/logitech_brio_rgb, to avoid issues if the number changes.
        self.device_path = '/dev/video0'
        self.image_topic = '/camera/image_raw'
        self.control_window_name = 'Camera Control Panel'
        self.video_window_name = 'Live Video Feed'

        self.get_logger().info(f"Starting v4l2-based camera tuner for device '{self.device_path}'")

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

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
            'auto_exposure': {'min': 1, 'max': 3}, # 1=Manual, 3=Auto
            'exposure_time_absolute': {'min': 3, 'max': 2047}, # CORRECTED NAME from log
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
            # Get the current value to set the slider's initial position
            default_val = self.get_current_param_value(name, fallback=ranges['min'])
            cv2.createTrackbar(name, self.control_window_name, default_val, ranges['max'],
                               lambda val, n=name: self.set_camera_parameter(n, val))
        self.get_logger().info("Camera tuner GUI created. Press 'q' in the video window to quit.")

    def get_current_param_value(self, param_name, fallback=0):
        try:
            result = subprocess.run(
                ['v4l2-ctl', '-d', self.device_path, '--get-ctrl', param_name],
                capture_output=True, text=True, check=True
            )
            # The output is "param_name: value", so we split on ':'
            value_str = result.stdout.strip().split(':')[1].strip()
            return int(value_str)
        except Exception as e:
            # This is expected for inactive controls, it's not a critical error
            self.get_logger().debug(f"Could not get initial value for '{param_name}' (likely inactive). Using fallback. Error: {e}")
            return fallback

    def set_camera_parameter(self, param_name, value):
        # --- THIS IS THE NEW "SMART" LOGIC ---
        # If we are setting a manual value, first ensure the corresponding auto mode is OFF
        if param_name == 'exposure_time_absolute':
            self._execute_v4l2_command('auto_exposure', 1) # Set to Manual Mode
        elif param_name == 'focus_absolute':
            self._execute_v4l2_command('focus_automatic_continuous', 0) # Turn off auto focus
        elif param_name == 'white_balance_temperature':
            self._execute_v4l2_command('white_balance_automatic', 0) # Turn off auto white balance

        # Now, set the actual parameter value
        self._execute_v4l2_command(param_name, value)
    
    def _execute_v4l2_command(self, param_name, value):
        """A helper function to run a single v4l2-ctl command."""
        try:
            cmd = ['v4l2-ctl', '-d', self.device_path, '--set-ctrl', f'{param_name}={int(value)}']
            subprocess.run(cmd, check=True, capture_output=True)
            self.get_logger().info(f"Successfully set '{param_name}' to {value}")
        except subprocess.CalledProcessError as e:
            # Capture and log the specific error from v4l2-ctl
            error_message = e.stderr.strip()
            self.get_logger().error(f"Failed to set '{param_name}': {error_message}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred while setting '{param_name}': {e}")


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow(self.video_window_name, cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Quitting camera tuner...")
                self.destroy_node(); cv2.destroyAllWindows(); rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args); node = CameraTunerNode(); rclpy.spin(node)

if __name__ == '__main__':
    main()