import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import threading

class Detector(Node):
    
    def __init__(self):
        self.__init__('detector')
        
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        self.brio_sub = self.create_subscription(
            Image,
            '/image_raw/',
            self.processImage_callback,
            10
        )
        
        self.goal_pub = self.create_publisher(Int32MultiArray, '/detector/goal', 10)
        
        self.bridge = CvBridge()
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.processor_callback)
        
        self.get_logger().info('Python image processor node has been started.')
        
    def processImage_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.image_lock: self.latest_image = cv_image
        
    def processor_callback(self):
        local_image_copy = None
        with self.image_lock:
            if self.latest_image is not None:
                local_image_copy = self.latest_image
                self.latest_image = None
        if local_image_copy is None: self.get_logger.warn('No new image to process.', throttle_duration_sec=5.0)
        else:
            returnArr = self.consolidate_into_two_lines_filtered(self.get_HoughsLinesP(local_image_copy))
            msg = self.pack_numpy_array(returnArr)
            self.goal_pub.publish(msg)
    
    def consolidate_into_two_lines_filtered(self, lines, n_iterations=10, theta_weight=100, outlier_threshold_std=2.0):
        """
        Consolidates line segments into a maximum of two lines, with outlier rejection.
        Implemented using only NumPy.

        Args:
            lines (np.ndarray): NumPy array of shape (N, 1, 4) for segments [x1, y1, x2, y2].
            n_iterations (int): Number of iterations for the K-Means algorithm.
            theta_weight (int): Weight to make angular distance comparable to pixel distance.
            outlier_threshold_std (float): How many standard deviations from the mean distance
                                        to use as the outlier threshold. Lower is stricter.

        Returns:
            list: A list containing up to two consolidated line segments.
        """
        if lines is None or lines.ndim != 3 or lines.shape[2] != 4:
            print("Invalid input shape. Expected (N, 1, 4).")
            return []

        segments = lines.reshape(-1, 4)
        line_params = []

        # 1. Represent each segment in (r, θ) space
        for x1, y1, x2, y2 in segments:
            if x1 == x2 and y1 == y2: continue
            angle = np.arctan2(y2 - y1, x2 - x1)
            theta = angle + np.pi / 2
            r = x1 * np.cos(theta) + y1 * np.sin(theta)
            if r < 0:
                r, theta = -r, (theta + np.pi) % (2 * np.pi)
            line_params.append([r, theta % np.pi])

        if len(line_params) < 2: return []

        X = np.array(line_params)
        X[:, 1] *= theta_weight

        # 2. K-Means implementation with NumPy
        initial_indices = np.random.choice(len(X), 2, replace=False)
        centroids = X[initial_indices]
        for _ in range(n_iterations):
            distances_to_centroids = np.sqrt(((X - centroids[:, np.newaxis])**2).sum(axis=2))
            labels = np.argmin(distances_to_centroids, axis=0)
            new_centroids = np.array([X[labels == i].mean(axis=0) for i in range(2)])
            if np.allclose(centroids, new_centroids): break
            centroids = new_centroids

        # 3. Filter outliers based on distance to centroid
        point_distances = np.array([distances_to_centroids[labels[i], i] for i in range(len(X))])
        distance_mean = point_distances.mean()
        distance_std = point_distances.std()
        distance_threshold = distance_mean + outlier_threshold_std * distance_std

        # Create a new set of labels, marking outliers with -1
        filtered_labels = np.where(point_distances <= distance_threshold, labels, -1)

        # 4. Merge non-outlier segments for each cluster
        final_lines = []
        for i in range(2):
            cluster_indices = np.where(filtered_labels == i)[0]
            if len(cluster_indices) < 2: continue # Need at least 2 segments to form a line
                
            cluster_points = []
            for index in cluster_indices:
                x1, y1, x2, y2 = segments[index]
                cluster_points.extend([(x1, y1), (x2, y2)])
            
            points_array = np.array(cluster_points)
            
            # PCA with NumPy to find the best-fit line
            mean_point = points_array.mean(axis=0)
            centered_data = points_array - mean_point
            covariance_matrix = np.cov(centered_data, rowvar=False)
            eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
            direction_vector = eigenvectors[:, np.argmax(eigenvalues)]
            
            # Project points to find endpoints
            projected_dist = np.dot(centered_data, direction_vector)
            endpoint1 = mean_point + np.min(projected_dist) * direction_vector
            endpoint2 = mean_point + np.max(projected_dist) * direction_vector
            
            final_lines.append([int(p) for p in endpoint1] + [int(p) for p in endpoint2])

        return final_lines

    def get_HoughsLinesP(self, image):
        if image is not None: grayscale = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        else: 
            self.get_logger.warn('HoughLinesP recieved an image that has type None', throttle_duration_sec=5.0)
            return
        _, mask = cv.threshold(grayscale, 245, 255, cv.THRESH_BINARY)
        filtered = cv.bitwise_and(grayscale, grayscale, mask=mask)
        dst = cv.Canny(filtered, 50, 150, None, apertureSize=7)
        linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 100, 10)
        return linesP

    def pack_numpy_array(self, np_array: np.ndarray) -> Int32MultiArray:
        """
        Packs a 2D NumPy array into a ROS 2 Int32MultiArray message.

        Args:
            np_array: The 2D NumPy array to pack.

        Returns:
            A ROS 2 Int32MultiArray message populated with the array's data and layout.
        """
        # Create the ROS message
        msg = Int32MultiArray()

        # --- Create the layout ---
        # The layout describes the structure of the data.
        # It's a list of MultiArrayDimension objects.
        layout = MultiArrayLayout()
        
        # Get the shape of the numpy array
        rows, cols = np_array.shape
        
        # Describe the 'rows' dimension
        dim_rows = MultiArrayDimension()
        dim_rows.label = "rows"
        dim_rows.size = rows
        # The stride is the total number of elements in the array
        dim_rows.stride = rows * cols 
        layout.dim.append(dim_rows)

        # Describe the 'columns' dimension
        dim_cols = MultiArrayDimension()
        dim_cols.label = "columns"
        dim_cols.size = cols
        # The stride for the last dimension is its size
        dim_cols.stride = cols
        layout.dim.append(dim_cols)
        
        msg.layout = layout

        # --- Flatten the array and set the data ---
        # The 'data' field must be a flat, 1D list.
        msg.data = np_array.flatten().tolist()
        
        return msg