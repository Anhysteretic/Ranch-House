#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraTunerNode(Node):
    """
    The definitive, comprehensive tuner script.
    Provides a GUI with controls for ALL camera parameters found in the v4l2 log.
    """
    def __init__(self):
        super().__init__('camera_tuner_display')

        self.cam_node_name = '/camera/usb_cam'
        self.image_topic = '/camera/image_raw'
        self.control_window_name = 'Camera Control Panel'
        self.video_window_name = 'Live Video Feed'

        self.get_logger().info(f"Starting comprehensive camera tuner for node '{self.cam_node_name}'")
        self.bridge = CvBridge()

        self.set_param_client = self.create_client(SetParameters, f'{self.cam_node_name}/set_parameters')
        self.get_param_client = self.create_client(GetParameters, f'{self.cam_node_name}/get_parameters')
        
        while not self.set_param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Service '{self.cam_node_name}/set_parameters' not available, waiting...")
        
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        
        # --- COMPLETE PARAMETER DEFINITIONS ---
        # This dictionary now matches your full v4l2-ctl output
        self.param_definitions = {
            # --- User Controls ---
            'brightness': {'min': 0, 'max': 255},
            'contrast': {'min': 0, 'max': 255},
            'saturation': {'min': 0, 'max': 255},
            'sharpness': {'min': 0, 'max': 255},
            'gain': {'min': 0, 'max': 255},
            'white_balance_automatic': {'min': 0, 'max': 1},
            'white_balance_temperature': {'min': 2000, 'max': 7500},
            'backlight_compensation': {'min': 0, 'max': 1},
            'power_line_frequency': {'min': 0, 'max': 2}, # 0:dis, 1:50Hz, 2:60Hz
            
            # --- Camera Controls ---
            'focus_automatic_continuous': {'min': 0, 'max': 1},
            'focus_absolute': {'min': 0, 'max': 255},
            'auto_exposure': {'min': 1, 'max': 3}, # 1:Manual, 3:Aperture Priority
            'exposure_time_absolute': {'min': 3, 'max': 2047},
            'exposure_dynamic_framerate': {'min': 0, 'max': 1},
            'zoom_absolute': {'min': 100, 'max': 500},
            
            # These are less common to tune, but included for completeness
            'pan_absolute': {'min': -36000, 'max': 36000},
            'tilt_absolute': {'min': -36000, 'max': 36000},

            # --- Logitech LED Controls ---
            'led1_mode': {'min': 0, 'max': 3}, # 0:Off, 1:On, 2:Blink, 3:Auto
            'led1_frequency': {'min': 0, 'max': 255},
        }

        self.create_gui()
        self.get_logger().info("GUI created. Fetching initial camera parameters in 1 second...")
        
        self.initial_param_fetch_timer = self.create_timer(1.0, self.fetch_initial_parameters)

    def create_gui(self):
        control_panel_img = np.zeros((1, 550, 3), np.uint8) # Made window slightly wider
        cv2.namedWindow(self.control_window_name)
        cv2.imshow(self.control_window_name, control_panel_img)

        for name, ranges in self.param_definitions.items():
            # Special case for pan/tilt due to large range. Can't use a trackbar.
            if name in ['pan_absolute', 'tilt_absolute']:
                self.get_logger().info(f"Control '{name}' has too large a range for a trackbar and must be set via command line.")
                continue

            cv2.createTrackbar(name, self.control_window_name, ranges['min'], ranges['max'],
                               lambda val, n=name: self.set_camera_parameter(n, val))
        self.get_logger().info("Press 'q' in the video window to quit.")

    def fetch_initial_parameters(self):
        self.destroy_timer(self.initial_param_fetch_timer)
        param_names = list(self.param_definitions.keys())
        # We can't get/set pan and tilt from the GUI, so remove them from the fetch list
        param_names.remove('pan_absolute')
        param_names.remove('tilt_absolute')
        
        req = GetParameters.Request()
        req.names = param_names
        future = self.get_param_client.call_async(req)
        future.add_done_callback(self.on_initial_parameters_fetched)

    def on_initial_parameters_fetched(self, future):
        try:
            response = future.result()
            self.get_logger().info("Successfully fetched initial parameters. Updating GUI.")
            for param in response.values:
                if param.type == ParameterType.PARAMETER_INTEGER:
                    self.get_logger().info(f"  - Setting slider for '{param.name}' to {param.integer_value}")
                    cv2.setTrackbarPos(param.name, self.control_window_name, param.integer_value)
        except Exception as e:
            self.get_logger().error(f"Failed to fetch initial parameters: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow(self.video_window_name, cv_image)
            
            # We still need waitKey to process GUI events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutting down...")
                self.destroy_node()
                cv2.destroyAllWindows()
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def set_camera_parameter(self, param_name, value):
        req = SetParameters.Request()
        param_msg = ParameterMsg()
        param_msg.name = param_name
        param_msg.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
        req.parameters = [param_msg]
        self.set_param_client.call_async(req)
        self.get_logger().info(f"Setting '{param_name}': {value}")

def main(args=None):
    rclpy.init(args=args)
    camera_tuner_node = CameraTunerNode()
    rclpy.spin(camera_tuner_node)
    
if __name__ == '__main__':
    main()