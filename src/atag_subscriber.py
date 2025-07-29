import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np

class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/drone/position_estimate',
            self.pose_callback,
            10
        )
        self.bridge = CvBridge()
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.get_logger().info("AprilTag subscriber node started.")

        self.current_pose = None
        self.last_tag_pose = None  # (x, y, z) in world frame

    def pose_callback(self, msg: PoseStamped):
        # Store the latest drone pose
        self.current_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            pts = [tuple(map(int, pt)) for pt in [ptA, ptB, ptC, ptD]]
            cv2.polylines(image, [np.array(pts)], isClosed=True, color=(0,255,0), thickness=2)
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            tag_family = r.tag_family.decode("utf-8")
            cv2.putText(image, tag_family, (pts[0][0], pts[0][1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            self.get_logger().info(f"Detected tag family: {tag_family} at ({cX}, {cY})")

            # If we have a current pose, store it as the last tag pose
            if self.current_pose is not None:
                if self.last_tag_pose is None:
                    self.last_tag_pose = self.current_pose.copy()
                    self.get_logger().info(f"Stored AprilTag world position: {self.last_tag_pose}")
                else:
                    # Compute relative position to last tag
                    rel_pos = self.current_pose - self.last_tag_pose
                    self.get_logger().info(f"Relative position to last AprilTag: {rel_pos}")

        cv2.imshow("AprilTag Detection", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSubscriber()