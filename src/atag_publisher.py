import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class AtagPublisher(Node):
    def __init__(self):
        super().__init__('atag_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'atag', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture frame.')
            return
        # Encode frame as PNG
        success, png_data = cv2.imencode('.png', frame)
        if not success:
            self.get_logger().warning('Failed to encode frame as PNG.')
            return
        msg = CompressedImage()
        msg.format = 'png'
        msg.data = png_data.tobytes()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AtagPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
