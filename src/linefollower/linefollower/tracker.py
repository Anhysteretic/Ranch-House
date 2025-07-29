import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleRatesSetpoint, VehicleAttitude
from cv_bridge import CvBridge
import cv2 as cv
import tf_transformations as tft
import numpy as np
import threading
from CoordTransforms import CoordTransforms
import math

IMAGE_HEIGHT, IMAGE_WIDTH = 720, 1280
_MAX_CLIMB, _MAX_SPEED, _MAX_YAWSPEED = 0.5, 0.5, 0.5
CENTER = np.array([IMAGE_WIDTH//2, IMAGE_HEIGHT//2])

steps = np.array([1, 0, 0, 0, 0, 1]) # 0 is left
currIndex = 0 

class Tracker(Node):
    def __init__(self):
        self.__init__('tracker')
        
        self.coord_transforms = CoordTransforms()
        
        self.latest_arr = None
        self.process_lock = threading.Lock()
        
        self.data_sub = self.create_subscription(
            Int32MultiArray,
            '/detector/goal/',
            self.goal_callback,
            10
        )
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.0
        
        self.control_lock = threading.Lock()

        # Linear setpoint velocities in downward camera frame
        self.vx__dc = 0.0
        self.vy__dc = 0.0
        self.vz__dc = 0.0

        # Yaw setpoint velocities in downward camera frame
        self.wz__dc = 0.0
        
        self.vx, self.vy, self.wz = 0, 0, 0

        self.q_bd_lned = np.array([1.0, 0.0, 0.0, 0.0])

        timer_period = 0.1
        self.process_timer = self.create_timer(timer_period, self.proccess_callback)
        self.control_timer = self.create_timer(timer_period, self.control_callback)
        
        self.get_logger().info('Python tracker node has been started.')
    
    def goal_callback(self, msg: Int32MultiArray):
        with self.process_lock: self.latest_arr = self.unpack_to_numpy_array(msg)
        
    def proccess_callback(self):
        local_latest_arr = None
        with self.process_lock:
            if self.latest_arr is not None:
                local_latest_arr = self.latest_arr
                self.latest_arr = None
        if local_latest_arr is None: self.get_logger.warn("local latest arr is None")
        else:
            with self.control_lock:
                singleSlope: bool = False
                singleSlopeVal = None
                slope1 = (local_latest_arr[0][1] - local_latest_arr[0][3]) / (local_latest_arr[0][0] - local_latest_arr[0][2])
                slope2 = (local_latest_arr[1][1] - local_latest_arr[1][3]) / (local_latest_arr[1][0] - local_latest_arr[1][2])
                
                if math.abs(slope1 - slope2) < 0.1:
                    singleSlope = True
                    singleSlopeVal = (slope1 - slope2) / 2
                
                if not singleSlope:
                    isOne = False
                    if slope1 < slope2 and math.abs(1 - slope1) < 0.1:
                        isOne = True
                    
                    closestP1 = self.get_closest_point_on_line(
                        (local_latest_arr[0][0], local_latest_arr[0][1]),
                        (local_latest_arr[0][2], local_latest_arr[0][3]),
                        (CENTER)
                    )
                    
                    p1Err = (closestP1[0] - CENTER[0], closestP1[1] - CENTER[1])
                    
                    
                    
                    closestP2 = self.get_closest_point_on_line(
                        (local_latest_arr[1][0], local_latest_arr[1][1]),
                        (local_latest_arr[1][2], local_latest_arr[1][3]),
                        (CENTER)
                    )
                    
                    
                    
                    
                
                v__bd = self.coord_transforms.static_transform((self.vx__dc, self.vy__dc, self.vz__dc), 'dc', 'bd')
                q_px4 = self.q_bd_lned              # PX4: [w, x, y, z]
                q_tf  = [q_px4[1], q_px4[2], q_px4[3], q_px4[0]]  # tf expects [x,y,z,w]
                _, _, yaw = tft.euler_from_quaternion(q_tf)        # returns (roll,pitch,yaw)
                vx_ned = v__bd[0]*np.cos(yaw) - v__bd[1]*np.sin(yaw)
                vy_ned = v__bd[0]*np.sin(yaw) + v__bd[1]*np.cos(yaw)
                _, _, wz_bd = self.coord_transforms.static_transform((0.0, 0.0, self.wz__dc), 'dc', 'bd')
                wz_bd = min(max(wz_bd, -_MAX_YAWSPEED), _MAX_YAWSPEED)
                
    
    def control_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        self.offboard_setpoint_counter += 1
        
        with self.control_lock: self.publish_trajectory_setpoint(self.vx, self.vy, self.wz)
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        
    def get_current_heading(self):
        """
        Extract current heading (yaw) from vehicle local position quaternion.
        Returns heading in radians (-pi to pi).
        """
        try:
            # Extract quaternion from vehicle local position
            q = [
                self.vehicle_local_position.q[1],  # x
                self.vehicle_local_position.q[2],  # y  
                self.vehicle_local_position.q[3],  # z
                self.vehicle_local_position.q[0]   # w (PX4 uses w-first format, tf uses w-last)
            ]
            
            # Convert quaternion to euler angles
            euler = tft.euler_from_quaternion(q)
            yaw = euler[2]  # Yaw is the third component (roll, pitch, yaw)
            
            return yaw
        except (AttributeError, IndexError):
            # Return 0 if quaternion data is not available yet
            return 0.0

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_attitude_callback(self, msg):
        self.q_bd_lned = msg.q

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        
    def publish_trajectory_setpoint(self, vx: float, vy: float, wz: float) -> None:
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), self.takeoff_height]
        if self.offboard_setpoint_counter < 100:
            msg.velocity = [0.0, 0.0, 0.0]
        else:
            msg.velocity = [vx, vy, 0.0]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = wz
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_publisher.publish(msg)
        
    def get_closest_point_on_line(self, p1, p2, p0):
        """
        Finds the closest point on an infinite line defined by p1 and p2 to point p0.

        Args:
            p1 (tuple): The first point defining the line (x1, y1).
            p2 (tuple): The second point defining the line (x2, y2).
            p0 (tuple): The external point (x0, y0).

        Returns:
            tuple: The coordinates (x, y) of the closest point on the line.
        """
        x1, y1 = p1
        x2, y2 = p2
        x0, y0 = p0

        # Calculate the vector for the line direction
        dx = x2 - x1
        dy = y2 - y1

        # Handle the case where p1 and p2 are the same point
        if dx == 0 and dy == 0:
            return p1

        # Calculate the projection parameter 't'
        # t = [(p0 - p1) . (p2 - p1)] / |p2 - p1|^2
        dot_product = (x0 - x1) * dx + (y0 - y1) * dy
        squared_length = dx**2 + dy**2
        t = dot_product / squared_length

        # Calculate the coordinates of the closest point
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return (closest_x, closest_y)
        
    def unpack_to_numpy_array(self, msg: Int32MultiArray) -> np.ndarray:
        """
        Unpacks a ROS 2 Int32MultiArray message back into a 2D NumPy array.

        Args:
            msg: The Int32MultiArray message to unpack.

        Returns:
            The reconstructed 2D NumPy array.
        """
        # Extract the dimensions from the layout
        # We expect two dimensions: rows and columns
        if len(msg.layout.dim) != 2:
            raise ValueError("Layout does not have 2 dimensions (rows, cols)")
            
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        
        # Convert the flat list of data into a NumPy array
        data_array = np.array(msg.data, dtype=np.int32)
        
        # Reshape the flat array into the original 2D shape
        reconstructed_array = data_array.reshape((rows, cols))
        
        return reconstructed_array
