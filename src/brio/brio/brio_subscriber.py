import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('Brio_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Image subscriber node started. Subscribing to /usb_cam_node/image_raw')
        cv2.namedWindow("Camera Feed (Python)", cv2.WINDOW_AUTOSIZE)

    def listener_callback(self, msg: CompressedImage):
        try:
            yuyv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # bgr_image = cv2.cvtColor(yuyv_image, cv2.COLOR_YUV2BGR_YUY2)
            bgr_image = yuyv_image
            cv2.imshow("Camera Feed (Python)", bgr_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
            
def main(args=None):
    rclpy.init()
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()