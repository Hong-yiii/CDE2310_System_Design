import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped ,Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray, Int32
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState, ChangeState
from sensor_msgs.msg import Imu, LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.action import ActionClient
from enum import Enum, auto
from collections import deque
import concurrent.futures
import time
import threading
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
from collections import deque
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sklearn.cluster import KMeans
from collections import Counter
import matplotlib.pyplot as plt
import random
import datetime
import os
from PIL import Image



# constants
rotatechange = 0.15 # was 0.1
stop_distance = 0.25 # distance to stop in front of heat source

class GlobalController(Node):
    """
    GlobalController:
    - Handles high-level state management
    - Sends goals to the planner
    - Monitors state feedback
    - Manages lifecycle transitions
    - Provides hooks for future expansion
    """
    # different states the bot can take on
    class State(Enum):
        Initializing = auto()
        Exploratory_Mapping = auto()
        Goal_Navigation = auto()
        Launching_Balls = auto()
        Imu_Interrupt = auto()
        Attempting_Ramp = auto()
        Returning_Home = auto()
        Go_to_Heat_Souce = auto()

    def __init__(self):
        super().__init__('global_controller')
        ## initialize all publishers and subscribers

        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # odom
        self.odom_subscription = self.create_subscription(
        Odometry,
        'odom',
        self.odom_callback,
        10)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # occupancy grid
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            map_qos)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        # temperature sensors
        self.left_temperature = self.create_subscription(
            Float32MultiArray,
            'temperature_sensor_1',
            self.sensor1_callback,
            10)
        self.right_temperature = self.create_subscription(
            Float32MultiArray,
            'temperature_sensor_2',
            self.sensor2_callback,
            10)
        
        # IMU subscription
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # lidar subscription
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/visualization_markers', 
            10)


        # Allow for global positioning 
        try: 
            self.tfBuffer = tf2_ros.Buffer()
            self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
            self.get_logger().info("TF listener created")
        except Exception as e :
            self.get_logger().error("TF listener failed: %s" % str(e))

        
        # Ball launcher
        self.flywheel_publisher = self.create_publisher(
            Int32, 
            'flywheel', 
            10)


        #paramas ********************************************************
        self.angle_heat_scan = 7
        self.init_attempt_ramp = True
        self.temp_threshold = 27 #26 original
        self.heat_distance_max = 2.5
        self.imu_threshold = 5.0
        self.fast_explore = False
        self.use_padding = False
        self.padding = 1
        self.rate_of_placement = 4
        self.ramp_backtrack = 20
        self.imu_abs_threshold = 0.16
        self.clusters = 3
        
        #****************************************************************
        # Temperature Attributes
        self.latest_left_temp = None
        self.latest_right_temp = None
        self.unfiltered_x_y_list = []
        self.filtered_x_y_list = []
        self.heat_left_world_x_y = []
        self.heat_right_world_x_y = []
        #self.heat_left_world_x_y = self.generate_cluster((1.0, 3.0), count=5)
        #self.heat_right_world_x_y = self.generate_cluster((2.0, 3.5), count=5)
        self.previous_position = deque(maxlen=self.ramp_backtrack)

        # IMU Attributes stored as (timestamp, pitch)
        self.pitch_window = deque()
        self.ramp_location = None
        self.hit_ramped = False
        self.initial_yaw = None
        
        # For global moving average
        self.global_pitch_sum = 0.0
        self.global_pitch_count = 0
        # For recent average (last 0.3s)
        self.recent_pitch_avg = 0.0
        self.global_pitch_avg = 0.0
        # For left and right lidar data
        self.distance_left = 0.0
        self.distance_right = 0.0
        #occ map variables
        
        #heat 
        self.max_heat_locations = []
        self.normal_bfs = set()
        self.line_coords = []

        # logic attributes
        self.state = GlobalController.State.Initializing
        self.previous_state = None
        self.ball_launches_attempted = 0
        self.finished_mapping = False
        self.goal_active = False
        self.just_reached_goal = False

        self.occ_callback_called = False
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 Action Server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 Action Server available. Ready to send goals.")
        self.visited_frontiers = set()
        self.distance_to_heat = None
        self.angle_to_heat = None
        self.laser_msg = None
        self.current_goal_handle = []
        self.occ_processing = False

        # Multi Threading functionality
        self.lock = threading.Lock()
        # Triggers the fast loop at 10hz(gy == y and gx == x) or current_val
        self.fast_timer = self.create_timer(0.1, self.fast_loop)
        # Triggers the control loop at 1hz
        self.control_loop_timer = self.create_timer(1.0, self.control_loop)
        # ✅ Thread pool for heavy background tasks
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)


        self.get_logger().info('Global Controller Initialized, changing state to Exploratory Mapping')
        self.set_state(GlobalController.State.Initializing)


    def map_callback(self):
        width = self.occdata.shape[1]
        height = self.occdata.shape[0]
        image = np.zeros((height, width, 3), dtype=np.uint8)

        #self.orange_points = [(2.0, 2.0), (3.0, 1.0)]
        #self.green_points = [(4.0, 3.0), (5.0, 4.0)]
        #self.visited_frontiers = [(1.5, 1.5), (2.5, 2.5)]
        #self.own_blocked_points = [(0.5, 0.5), (1.0, 1.0)]
        # Base map coloring
        self.get_logger().info(f"Unique values in occdata: {np.unique(self.occdata)}")
        image[self.occdata == 101] = [0, 0, 0]           # Occupied - black
        image[self.occdata == 0]   = [255, 255, 255]     # Free - white
        image[self.occdata == 1]   = [127, 127, 127]     # Special Free - grey

        # Helper to mark pixels
        def mark(points, color):
            if points is not None:
                for p in points:
                    if p is None:
                        continue
                    x, y = p
                    i, j = self.world_to_grid(x, y)
                    if 0 <= i < height and 0 <= j < width:
                        image[i, j] = color

        #mark(self.normal_bfs, [255, 165, 0])      # Orange
        #mark(self.max_heat_locations, [0, 255, 0])         # Green
        #mark(self.visited_frontiers, [255, 255, 0])  # Yellow
        #mark(self.line_coords, [0, 0, 255])   # Blue

        save_dir = "/home/rex/colcon_ws/src/map_images"
        os.makedirs(save_dir, exist_ok=True)

        # Filename with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"occupancy_map_{timestamp}.png"
        save_path = os.path.join(save_dir, filename)

        # Save the image
        Image.fromarray(image).save(save_path)


    def mark_area_around_robot_as_occ(self, x, y, radius=8):
        height, width = self.occdata.shape

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                gx = x + dx
                gy = y + dy

                if 0 <= gx < width and 0 <= gy < height:
                    self.line_coords.append((gx, gy))
                    self.occdata[gy, gx] = 101

        self.get_logger().info(f"Line coords length: {len(self.line_coords)}")

    


    ## Callback handers for temperature sensors
    def sensor1_callback(self, msg):
        
        
        if msg.data and len(msg.data) == 64:
            indices = [
            18, 19, 20, 21,
            26, 27, 28, 29,
            34, 35, 36, 37,
            42, 43, 44, 45
        ]
            center_values = [msg.data[i] for i in indices]
            self.latest_left_temp = center_values

            if(self.valid_heat(self.latest_left_temp , self.temp_threshold)):
                #TODO : adjust to the real angle range of where the sensor points
                angle , distance = self.laser_avg_angle_and_distance_in_mode_bin(87- self.angle_heat_scan, 87 + self.angle_heat_scan, 0.1)
                x , y = self.calculate_heat_world(angle , distance)
                if x is None or y is None or distance > self.heat_distance_max: #removes anything more than 1
                    return
                self.heat_left_world_x_y.append([x,y])
                self.get_logger().info(f"🔥🔥🔥🔥🔥Heat source detected at right sensor at: {x}, {y}")  


    def sensor2_callback(self, msg):
        if msg.data and len(msg.data) == 64:
            indices = [
            18, 19, 20, 21,
            26, 27, 28, 29,
            34, 35, 36, 37,
            42, 43, 44, 45
        ]
            
            center_values = [msg.data[i] for i in indices]
            #self.latest_right_temp = sum(center_values) / len(center_values)
            self.latest_right_temp = center_values

            if(self.valid_heat(self.latest_right_temp ,self.temp_threshold)):
                #TODO : adjust to the real angle range of where the sensor points
                angle , distance = self.laser_avg_angle_and_distance_in_mode_bin(0 - self.angle_heat_scan, self.angle_heat_scan, 0.1)
                x , y = self.calculate_heat_world(angle , distance)

                if x is None or y is None or distance > self.heat_distance_max:
                    return
                self.heat_right_world_x_y.append([x,y])
                self.get_logger().info(f"🔥🔥🔥🔥🔥Heat source detected front sensor at: {x}, {y}")   



    def laser_callback(self, msg):
        self.laser_msg = msg

        
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def pd_control(error, prev_error, kp, kd, dt):
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        return kp * error + kd * derivative


    def normalize_angle(self ,angle):
        return math.atan2(math.sin(angle), math.cos(angle))


    def pd_control(self, error, prev_error, kp, kd, dt):
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        return kp * error + kd * derivative


    def get_lidar_distances(self):
        if self.laser_msg is None:
            self.get_logger().warn("No laser scan data available.")
            return None, None, None  # front, left, right

        scan_msg = self.laser_msg
        ranges = np.array(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        angles_deg = (np.degrees(angles) + 180) % 360 - 180  # Normalize to [-180, 180)

        # Helper to extract shortest distance in sector
        def get_shortest_in_sector(min_deg, max_deg):
            mask = (angles_deg >= min_deg) & (angles_deg <= max_deg)
            sector_ranges = ranges[mask]
            valid = np.isfinite(sector_ranges) & (sector_ranges > 0.01)
            return sector_ranges[valid].min() if np.any(valid) else None

        # Extract distances
        front_dist = get_shortest_in_sector(-5, 5)
        left_dist  = get_shortest_in_sector(85, 95)
        right_dist = get_shortest_in_sector(-95, -85)  # or (265, 275) if using 0–360 convention

        return front_dist, left_dist, right_dist

    def run_pd_until_obstacle(self, target_yaw, stop_threshold=0.2):
        prev_yaw_error = 0.0
        prev_wall_error = 0.0
        prev_time = time.time()

        kp_yaw, kd_yaw = 2.0, 0.1
        kp_wall, kd_wall = 1.0, 0.05

        while True:
            current_yaw = self.get_robot_global_position()[2]
            front_dist, left_dist, right_dist = self.get_lidar_distances()

            if front_dist <= stop_threshold:
                break

            now = time.time()
            dt = now - prev_time
            prev_time = now

            yaw_error = self.normalize_angle(target_yaw - current_yaw)
            wall_error = right_dist - left_dist

            yaw_correction = self.pd_control(yaw_error, prev_yaw_error, kp_yaw, kd_yaw, dt)
            wall_correction = self.pd_control(wall_error, prev_wall_error, kp_wall, kd_wall, dt)
            angular_z = yaw_correction + wall_correction

            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = angular_z
            self.publisher_.publish(twist)  # assumes this global publisher exists

            prev_yaw_error = yaw_error
            prev_wall_error = wall_error

            time.sleep(0.1)

        # Stop the robot
        self.publisher_.publish(Twist())

    ## method to launch balls
    def launch_ball(self):
        msg = Int32()
        msg.data = 50
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.ball_launches_attempted += 1
        self.get_logger().info(f"Ball launches attempted: {self.ball_launches_attempted}")

    ## method to launch balls
    def launch_ball(self):
        msg = Int32()
        msg.data = 50
        self.flywheel_publisher.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.ball_launches_attempted += 1
        self.get_logger().info(f"Ball launches attempted: {self.ball_launches_attempted}")

    ## callback handler for IMU
    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, pitch, _ = euler_from_quaternion(quat)

        now = self.get_clock().now().nanoseconds / 1e9  # seconds

        with self.lock:
            # Append latest value
            self.pitch_window.append((now, abs(pitch)))  # use abs to ignore direction

            # Remove old entries beyond 0.5s
            while self.pitch_window and now - self.pitch_window[0][0] > 0.5:
                self.pitch_window.popleft()

            # Update global moving average
            self.global_pitch_sum += abs(pitch)
            self.global_pitch_count += 1
            self.global_pitch_avg = self.global_pitch_sum / self.global_pitch_count
            
            #self.get_logger().info(f"Global Pitch Avg: {self.global_pitch_avg}")
            # Compute recent average for last 0.3s
            recent_values = [p for t, p in self.pitch_window if now - t <= 0.3]
            if recent_values:
                self.recent_pitch_avg = sum(recent_values) / len(recent_values)    
            #self.get_logger().info(f"Recent Pitch Avg: {self.recent_pitch_avg}")
    ## lidar callback
    def listener_callback(self, msg):
        # Convert LaserScan ranges to numpy array
        laser_range = np.array(msg.ranges)

        # Replace zeros (invalid) with nan
        laser_range[laser_range == 0] = np.nan

        # Calculate index corresponding to +30° and -30°
        angle_increment_deg = 360 / len(laser_range)
        index_pos_30 = int(round(30 / angle_increment_deg))
        index_neg_30 = int(round((360 - 30) / angle_increment_deg))

        # Extract the distances at those indices
        distance_pos_30 = laser_range[index_pos_30]
        distance_neg_30 = laser_range[index_neg_30]
        with self.lock:
            self.distance_left = distance_pos_30
            self.distance_right = distance_neg_30

    def generate_cluster(self, center, count=5, spread=0.2):
        cx, cy = center
        return [
            (cx + random.uniform(-spread, spread), cy + random.uniform(-spread, spread))
            for _ in range(count)
        ]

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])


    def occ_callback(self, msg):
        #self.get_logger().info('In occ_callback - Updating Map Metadata')
        # Store map metadata
        self.occ_processing = True
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        # create numpy array
        msgdata = np.array(msg.data)

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        #self.get_logger().info(f"Unique values in occupancy grid: {np.unique(self.occdata)}")
        self.spare_occdata = self.occdata.copy()

        # 0 -> true unknown
        # 1 -> true free space
        # 101 -> true occupied
        # 102 -> free but marked as unknown
        # 103 -> marked as visited but true unknown should not be marked as visited


        #new stuff to ignore lidar scans
        if not self.fast_explore:
            x, y = self.get_robot_grid_position()
            if x is not None and y is not None:
                self.occdata[self.occdata == 1] = 0 #reset all lidar known scans to unknown
                self.mark_area_around_robot(x, y, radius=4) #set as free

            # Safely mark visited frontiers
            for node in self.visited_frontiers:
                x, y = node
                x, y = self.world_to_grid(x, y)
                if 0 <= y < self.occdata.shape[0] and 0 <= x < self.occdata.shape[1]:
                    if self.occdata[y, x] != 101:
                        self.mark_area_mapped(x, y, self.rate_of_placement) #mark frontier as known space
        else:
            for node in self.visited_frontiers:
                x, y = node
                x , y = self.world_to_grid(x, y)
                if 0 <= y < self.occdata.shape[0] and 0 <= x < self.occdata.shape[1]:
                    if self.occdata[y, x] != 101:
                        self.occdata[y, x] = 1
        
        height, width = self.occdata.shape

        if self.use_padding:
            # Create a copy to store expanded obstacles
            expanded_occdata = self.occdata.copy()
            self.original_occdata = self.occdata.copy()

            for y in range(height):
                for x in range(width):
                    if self.occdata[y, x] == 101:  # Only use the original grid
                        for dy in range(-self.padding, self.padding + 1):
                            for dx in range(-self.padding, self.padding + 1):
                                nx, ny = x + dx, y + dy  # New x, y coordinates
                                if 0 <= ny < height and 0 <= nx < width:  # Bounds check
                                    expanded_occdata[ny, nx] = 101  # Mark as occupied
            # Apply the expanded costmap
            self.occdata = expanded_occdata
        
        self.occ_callback_called = True
        self.map_callback()

        if np.any(self.occdata == 0):
            pass
        else:
            self.get_logger().info("No unknown cells found in the occupancy grid.")

        self.occ_processing = False


    def distance(self, x1, y1,x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def reverse(self):
        msg = Twist()
        msg.linear.x = -0.1  # Negative for reverse (m/s)
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


        x , y , yaw= self.get_robot_global_position()

        while self.distance(x,y, self.ramp_location[0], self.ramp_location[1]) < 0.15:
            x , y , yaw= self.get_robot_global_position()
            time.sleep(0.2)
        self.stopbot()

    def mark_area_around_robot(self, x, y, radius=4):
        height, width = self.occdata.shape

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                gx = x + dx
                gy = y + dy

                if 0 <= gx < width and 0 <= gy < height:
                    current_val = self.occdata[gy, gx]
                    if current_val != 101:  # not occupied
                        self.occdata[gy, gx] = 1


    
    def mark_area_mapped(self, x, y, max_radius=4):
        height, width = self.occdata.shape
        visited = set()
        queue = deque()
        
        queue.append((x, y, 0))  # (grid_x, grid_y, distance)
        visited.add((x, y))

        # 8 directions: up, down, left, right + 4 diagonals
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                    (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while queue:
            gx, gy, dist = queue.popleft()

            if not (0 <= gx < width and 0 <= gy < height):
                continue

            if dist > max_radius:
                continue

            current_val = self.occdata[gy, gx]

            if current_val == 101:
                continue  # Don't mark or expand from here

            # Mark as visited (e.g., 1)
            if self.spare_occdata[gy,gx] != 0:
                self.occdata[gy, gx] = 1

            # Expand all 8 neighbors
            for dx, dy in directions:
                nx, ny = gx + dx, gy + dy
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))


    def rotate_till_occu(self):
        self.get_logger().info("Rotating till occupied space found")
        twist = Twist()
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = rotatechange
        # start rotation
        self.publisher_.publish(twist)

        while np.sum(self.occdata == 101) < 100:
            rclpy.spin_once(self)
            
        self.stopbot()
        self.get_logger().info("Starting navigation......")


    def rotate_till_path_available(self):
        self.get_logger().info("Rotating till path available")
        twist = Twist()
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = rotatechange
        # start rotation
        self.publisher_.publish(twist)

        while self.distance_to_goal is None and self.shortest_path is None:
            rclpy.spin_once(self)
        self.stopbot()
        self.get_logger().info(f"Distance to goal is {self.distance_to_goal}")
        self.get_logger().info(f"Shortest path is {self.shortest_path}")
        self.get_logger().info("Path found. Starting navigation......")


    def get_robot_grid_position(self):
        """
        Calculate the robot's (x, y) position in occupancy grid coordinates.
        Uses TF transforms if available; otherwise, falls back to odometry.
        """
        # Check if TF transform is available
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            cur_pos_x = trans.transform.translation.x
            cur_pos_y = trans.transform.translation.y
            self.robot_x = cur_pos_x
            self.robot_y = cur_pos_y
            #self.get_logger().info("Using TF transform for position.")
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF lookup failed, falling back to odometry.")
            
            # Use odometry as fallback
            if hasattr(self, 'robot_x') and hasattr(self, 'robot_y'):
                cur_pos_x = self.robot_x
                cur_pos_y = self.robot_y
            else:
                self.get_logger().error("No valid position available.")
                return None, None  # Return None if no valid data is found
        
        # Ensure map metadata is available
        if not hasattr(self, 'map_resolution') or not hasattr(self, 'map_origin_x'):
            self.get_logger().error("Map metadata not available.")
            return None, None

        # Convert world coordinates to grid indices
        grid_x = int((cur_pos_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((cur_pos_y - self.map_origin_y) / self.map_resolution)

        #self.get_logger().info(f"Robot Grid Position: ({grid_x}, {grid_y})")
        return grid_x, grid_y


    def is_frontier(self, map_data, x, y):
        """
        Check if a given cell (x, y) is a frontier. A frontier is defined as a free space
        adjacent to an unknown cell.

        :param map_data: 2D occupancy grid data.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :return: True if the cell is a frontier, False otherwise.
        """
        
        # Check if the current cell is occupied or unknown
        if map_data[y, x] == 101 or map_data[y, x] == 0:
            return False  # This cell is either occupied (101) or unknown (0)

        # Ensure that we are not considering the robot's current position
        if (x, y) == self.get_robot_grid_position():
            return False  # Exclude the robot's current position

        # Check for neighboring unknown cells (0)
        #neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected neighbors
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if 0 <= ny < map_data.shape[0] and 0 <= nx < map_data.shape[1]:
                if map_data[ny, nx] == 0:  # If a neighbor is unknown
                    return True  # This cell is a frontier

        return False  # No adjacent unknown cells, not a frontier


    def detect_closest_frontier_outside(self, robot_pos, min_distance=3):

        # Use squared distance to avoid unnecessary sqrt calculations
        queue = deque([robot_pos])
        visited = set([robot_pos])

        count = 0
        while queue:

            x, y = queue.popleft()
            count += 1
           
            visited_frontiers_grid = list(map(lambda pt: self.world_to_grid(pt[0], pt[1]), self.visited_frontiers))
            if self.is_frontier(self.occdata, x, y) and (x, y) not in visited_frontiers_grid:
                for dx in range(-1, 2):  # Covers [-1, 0, 1]
                    for dy in range(-1, 2):  # Covers [-1, 0, 1]
                        nx, ny = x + dx, y + dy
                        if 0 <= ny < self.occdata.shape[0] and 0 <= nx < self.occdata.shape[1]:
                            world_x, world_y = self.grid_to_world(nx, ny)
                            self.visited_frontiers.add((world_x, world_y))
                return (x, y)
            else:
                pass
                #self.get_logger().info(f"Skipping cell ({x}, {y}) due to distance constraint.")

            # Explore 8-connected neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and 0 <= ny < self.occdata.shape[0] and 0 <= nx < self.occdata.shape[1]:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        print(count)
        return None


    def IMU_interrupt_check(self):
        with self.lock:
            if self.global_pitch_avg > 0 and (self.recent_pitch_avg > self.imu_threshold * self.global_pitch_avg or self.recent_pitch_avg > self.imu_abs_threshold): # set as 5 * moving average
                self.get_logger().info("IMU Interrupt detected")
                return True
            else:
                return False


    def grid_to_world(self, grid_x, grid_y):

        if not hasattr(self, 'map_resolution') or not hasattr(self, 'map_origin_x'):
            self.get_logger().error("Map metadata not available.")
            return None, None
    
        world_x = self.map_origin_x + (grid_x * self.map_resolution)
        world_y = self.map_origin_y + (grid_y * self.map_resolution)

        return world_x, world_y
 

    def stopbot(self):
        self.get_logger().info('Stopping the robot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def wait_for_map(self):
        self.get_logger().info("Waiting for initial map update...")
        while self.occdata.size == 0 or not np.any(self.occdata != 0):
            self.get_logger().warn("No occupancy grid data yet, waiting...")
            rclpy.spin_once(self)
            time.sleep(1)
        self.get_logger().info("Map received. Starting Dijkstra movement.")
        


    def valid_heat(self, array , threshold = 26.0):
        if array is None:
            return False

        array = np.array(array)

        if array.size == 0:
            return False

        if np.max(array) >= threshold:
            return True

        return False


    def find_centers(self,n_centers = 2):
        
        full_list = self.heat_left_world_x_y + self.heat_right_world_x_y
        if len(full_list) == 1:
            n_centers = 1
        # Apply KMeans clustering
        kmeans = KMeans(n_clusters=n_centers, random_state=0)
        
        data = np.array(full_list)  # ← your list of (x, y)
        if(data.size == 0):
            return
        kmeans.fit(data)

        # Get the center points
        centers = kmeans.cluster_centers_
        return [tuple(pt) for pt in centers]
        #return centers


    def dijk_mover(self):
        try:
            # Get the current position of the robot
            start = self.get_robot_grid_position()

            if start[0] is None or start[1] is None:
                self.get_logger().warn("No valid robot position available.")
                return
            
            n = 0
            while n < 10:
                frontier = self.detect_closest_frontier_outside(start, min_distance=2)
                if(frontier is not None):
                    break
                n += 1
                time.sleep(0.4)
            if frontier is not None:
                world_x, world_y = self.grid_to_world(frontier[0], frontier[1])
                self.get_logger().info(f"Navigating to closest unmapped cell at {world_x}, {world_y}")
                self.nav_to_goal(world_x, world_y)
            else:
                
                self.finished_mapping = True
                self.get_logger().warn("No frontiers found. Robot is stuck!")
                

                
        except Exception as e:
            self.get_logger().error(f"Error in dijk_mover: {e}")

        finally:
            # Stop robot if needed
            self.stopbot()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info(f"[Feedback] Distance remaining: {feedback.distance_remaining:.2f}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal was rejected by Nav2.")
            self.goal_active = False
            return
        self.current_goal_handle.append(goal_handle)
        self.get_logger().info("✅ Goal accepted by Nav2.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)



    def calculate_heat_world(self , angle_to_heat , distance_to_heat):
        temp = self.get_robot_global_position()
        if(temp is None):
            self.get_logger().warn("Unable to calculate heat grid, no robot position.")
            return None , None
        x,y,yaw = temp
        if x is None or y is None or angle_to_heat is None or distance_to_heat is None:
            self.get_logger().warn("Unagle to calculate heat grid, missing data.")
            return None , None
        #angle in rad
        return self.polar_to_world_coords(angle_to_heat, distance_to_heat, x, y, yaw)
        

    def world_to_grid(self,x_world, y_world):
        
        if not hasattr(self, 'map_resolution') or not hasattr(self, 'map_origin_x'):
            self.get_logger().error("Map metadata not available.")
            return None, None
        
        origin_x = self.map_origin_x
        origin_y = self.map_origin_y
        resolution = self.map_resolution
        grid_x = int((x_world - origin_x) / resolution)
        grid_y = int((y_world - origin_y) / resolution)
        return grid_x, grid_y


    def goal_result_callback(self, future):
        try:
            self.goal_active = False
            result_msg = future.result()
            status = result_msg.status  # ✅ status is here
            result = result_msg.result  # this is the NavigateToPose_Result message

            self.get_logger().info(f"🏁 Nav2 goal finished with status: {status}")

            if status == 3:  # STATUS_SUCCEEDED
                self.reached_heat = True
                self.get_logger().info("🎯 Goal reached successfully!")
                self.just_reached_goal = True
            else:
                self.get_logger().warn(f"⚠️ Goal ended with failure status: {status}")

        except Exception as e:
            self.get_logger().error(f"❌ Exception in goal_result_callback: {e}")



    def nav_to_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"📬 Sending goal to Nav2: x={x:.2f}, y={y:.2f}")

        self.goal_active = True  # Flag to prevent multiple goals
        self.just_reached_goal = False  # Reset flag for new goal

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def drive_straight_between_walls(self, base_speed=0.1, correction_gain=0.5, max_angular=0.3):
        """
        Drives the robot forward while adjusting orientation to stay centered between left and right walls.

        :param base_speed: Constant forward linear velocity.
        :param correction_gain: Tuning factor to determine angular adjustment strength.
        :param max_angular: Maximum allowable angular velocity.
        """
        with self.lock:
            left = self.distance_left
            right = self.distance_right

        # Ignore if one of the sides is invalid (NaN)
        if np.isnan(left) or np.isnan(right):
            self.get_logger().warn("Invalid LIDAR data — cannot drive straight.")
            self.stopbot()
            return

        # Calculate error: if left is further than right, turn left slightly (and vice versa)
        error = left - right

        # Simple proportional controller to correct orientation
        angular_z = correction_gain * error

        # Clamp angular velocity to prevent oversteering
        angular_z = max(min(angular_z, max_angular), -max_angular)

        # Create and publish the twist command
        twist = Twist()
        twist.linear.x = base_speed
        twist.angular.z = angular_z
        self.publisher_.publish(twist)

        self.get_logger().info(f"Driving straight | L: {left:.2f}, R: {right:.2f}, Angular: {angular_z:.2f}")

    def publish_visualization_markers(self):
        marker_array = MarkerArray()
        marker_id = 0  # Every marker needs a unique ID

        
        # 1. 🔴 Heat sources (green cubes)
        for source in self.heat_left_world_x_y + self.heat_right_world_x_y:
            if source is None:
                continue
            x, y = source
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "heat_sources"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            marker_id += 1

        # 2. 🧱 Sealed lines (red cubes)
        for gx, gy in self.line_coords:
            wx, wy = self.grid_to_world(gx, gy)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sealed_lines"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.05
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            marker_id += 1

        # 🔄 Publish all markers
        self.marker_pub.publish(marker_array)


    def laser_avg_angle_and_distance_in_mode_bin(self, angle_min_deg=-30, angle_max_deg=30, bin_width=0.1):
        if self.laser_msg is None:
            self.get_logger().warn("No laser scan data available.")
            return None, None

        scan_msg = self.laser_msg
        ranges = np.array(scan_msg.ranges)

        # Compute angles (radians) for each scan index
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        angles_deg = np.degrees(angles)

        # Normalize angles to [-180°, 180°)
        angles_deg = (angles_deg + 180) % 360 - 180

        # Filter angles within specified field of view
        mask = (angles_deg >= angle_min_deg) & (angles_deg <= angle_max_deg)
        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]  # still in radians

        # Debug filtered range
        self.get_logger().info(
            f"📐 Filtered angle range: {angles_deg[mask].min():.1f}° to {angles_deg[mask].max():.1f}°, "
            f"Count: {np.count_nonzero(mask)}"
        )

        # Filter out invalid range readings
        valid_mask = np.isfinite(filtered_ranges) & (filtered_ranges > 0.01)
        filtered_ranges = filtered_ranges[valid_mask]
        filtered_angles = filtered_angles[valid_mask]

        if len(filtered_ranges) == 0:
            self.get_logger().warn("No valid laser points in the specified angle range.")
            return None, None

        # Bin distances into fixed-width intervals
        binned = np.floor(filtered_ranges / bin_width).astype(int)
        from collections import Counter
        mode_bin, _ = Counter(binned).most_common(1)[0]

        # Extract the mode bin values
        bin_min = mode_bin * bin_width
        bin_max = bin_min + bin_width
        in_mode = (filtered_ranges >= bin_min) & (filtered_ranges < bin_max)
        mode_distances = filtered_ranges[in_mode]
        mode_angles = filtered_angles[in_mode]

        if len(mode_angles) == 0:
            self.get_logger().warn("No points in mode bin.")
            return None, None

        # ✅ Use circular mean for angles to prevent wrap-around issues
        avg_angle_rad = np.arctan2(np.mean(np.sin(mode_angles)), np.mean(np.cos(mode_angles)))
        avg_angle_deg = (np.degrees(avg_angle_rad) + 180) % 360 - 180

        avg_distance = np.mean(mode_distances)

        self.get_logger().info(
            f"🟢 Mode distance: {avg_distance:.2f} m, Mode angle: {avg_angle_deg:.1f}°"
        )

        return avg_angle_rad, avg_distance  # radians, meters




    def polar_to_world_coords(self, avg_angle_rad, avg_distance, robot_x, robot_y, robot_yaw_rad):
        x_local = avg_distance * math.cos(avg_angle_rad)
        y_local = avg_distance * math.sin(avg_angle_rad)

        x_world = robot_x + (x_local * math.cos(robot_yaw_rad)) - (y_local * math.sin(robot_yaw_rad))
        y_world = robot_y + (x_local * math.sin(robot_yaw_rad)) + (y_local * math.cos(robot_yaw_rad))
        return (x_world, y_world)



    # =======================
    # Thread safe State Management
    # =======================
    
    def set_state(self, new_state):
        """Thread-safe state setter"""
        with self.lock:
            self.state = new_state
            self.get_logger().info(f"State changed to: {self.state}")

    def get_state(self):
        """Thread-safe state getter"""
        with self.lock:
            return self.state
        
    def get_robot_global_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tfBuffer.lookup_transform(
                'map',         # target frame (global)
                'base_link',   # source frame (robot)
                rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Convert quaternion to yaw
            rot = trans.transform.rotation
            quat = [rot.x, rot.y, rot.z, rot.w]
            (_, _, yaw) = euler_from_quaternion(quat)

            return (x, y, yaw)

        except Exception as e:
            self.get_logger().warn(f"[TF] Failed to get global robot position: {e}")
            return None


    def IMU_interrupt_check(self):
        with self.lock:
            if self.global_pitch_avg > 0 and self.recent_pitch_avg > self.imu_threshold * self.global_pitch_avg: # set as 5 * moving average
                self.get_logger().info("IMU Interrupt detected")
                return True
            else:
                return False


    def is_valid(self, neighbor, visited):
        x, y = neighbor
        return (x, y) not in visited and 0 <= y < self.occdata.shape[0] and 0 <= x < self.occdata.shape[1]

    def is_within_map(self , neighbour):
        x, y = neighbour
        return 0 <= y < self.occdata.shape[0] and 0 <= x < self.occdata.shape[1]

    def normal_bfs_from_world(self , world_x, world_y):
        grid_x , grid_y = self.world_to_grid(world_x,world_y)


        # Initialize frontier with a starting point
        frontier = deque()
        start = (grid_x, grid_y)  # Replace x and y with your starting coords
        frontier.append(start)

        # Set to keep track of visited nodes
        self.normal_bfs.add(start)

        # Example grid directions (8-connected)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),(-1, -1), (-1, 1), (1, -1), (1, 1)]
        count = 0
        self.reached_heat = False
        while frontier:
            #self.get_logger().info(f"Frontier is at: {frontier}")
            if(self.reached_heat):
                return True
            
            if not self.goal_active:
                count += 1
                current = frontier.popleft()

                x,y = current
                if(self.is_within_map(current) and self.occdata[y,x] == 1):
                    cur_x , cur_y  = self.grid_to_world(x,y)
                    self.nav_to_goal(cur_x , cur_y)

            
                for dx, dy in directions:
                    neighbor = (current[0] + dx, current[1] + dy)
                    if self.is_valid(neighbor, self.normal_bfs):
                        frontier.append(neighbor)
                        self.normal_bfs.add(neighbor)
                
                self.get_logger().info(f"Current position: {current} | Visited count: {len(self.normal_bfs)}")
        return False
                    

    def seal_line_along_facing_axis(self ,length=21):
        """
        Seals a straight axis-aligned line in the direction the robot is roughly facing.
        Direction is snapped to the nearest axis (N/S/E/W).
        """

        # Convert robot position to grid
        world_x , world_y , yaw_rad = self.get_robot_global_position()
        cx, cy = self.world_to_grid(world_x, world_y)

        # Determine axis-aligned direction
        dx = np.cos(yaw_rad)
        dy = np.sin(yaw_rad)

        if abs(dx) > abs(dy):
            # More aligned with x-axis
            direction = 'E' if dx > 0 else 'W'
        else:
            # More aligned with y-axis
            direction = 'N' if dy > 0 else 'S'

        # Pick unit vector for direction
        dir_map = {
            'N': (0, 1),
            'S': (0, -1),
            'E': (1, 0),
            'W': (-1, 0)
        }
        step_x, step_y = dir_map[direction]

        half_len = length // 2

        for i in range(-half_len, half_len + 1):
            gx = cx + i * step_x
            gy = cy + i * step_y
            if 0 <= gy < self.occdata.shape[0] and 0 <= gx < self.occdata.shape[1]:
                self.line_coords.append((gx, gy))
            else:
                return  # Abort if any part goes out of bounds

        # Check ends
        (sx, sy), (ex, ey) = self.line_coords[0], self.line_coords[-1]

        def is_connected(x, y):
            return self.occdata[y, x] == 101

        if is_connected(sx, sy) and is_connected(ex, ey):
            for x, y in self.line_coords:
                self.occdata[y, x] = 101
            self.get_logger().info(f"🧱 Sealed {direction}-axis line from ({cx},{cy})")
        else:
            self.get_logger().info("❌ Not sealing: ends not connected.")


    def adaptive_seal_line(self, length=21, max_steps=10):
        """
        Draws a line through the robot's current position and seals between first two
        occupied cells from both ends. If none found, steps forward and repeats.
        """
        world_x, world_y, yaw_rad = self.get_robot_global_position()
        dx = np.cos(yaw_rad)
        dy = np.sin(yaw_rad)

        # Determine axis-aligned direction
        if abs(dx) > abs(dy):
            direction = 'E' if dx > 0 else 'W'
        else:
            direction = 'N' if dy > 0 else 'S'

        dir_map = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
        step_x, step_y = dir_map[direction]

        for step in range(max_steps):
            self.line_coords = []
            cx, cy = self.world_to_grid(world_x, world_y)

            # Build line centered at current position
            line = []
            half_len = length // 2
            for i in range(-half_len, half_len + 1):
                gx = cx + i * step_x
                gy = cy + i * step_y
                if 0 <= gy < self.occdata.shape[0] and 0 <= gx < self.occdata.shape[1]:
                    line.append((gx, gy))
                else:
                    break  # Skip this entire step if line goes out of bounds

            # Search from both ends for the first 2 occupied cells
            start = None
            end = None
            for fwd, rev in zip(line, reversed(line)):
                if start is None and self.occdata[fwd[1], fwd[0]] == 101:
                    start = fwd
                if end is None and self.occdata[rev[1], rev[0]] == 101:
                    end = rev
                if start and end:
                    break

            if start and end:
                # Draw sealed line between start and end
                final_line = []
                x0, y0 = start
                x1, y1 = end
                dx = np.sign(x1 - x0)
                dy = np.sign(y1 - y0)
                x, y = x0, y0

                while (x, y) != (x1 + dx, y1 + dy):
                    if 0 <= y < self.occdata.shape[0] and 0 <= x < self.occdata.shape[1]:
                        self.occdata[y, x] = 101
                        final_line.append((x, y))
                    x += dx
                    y += dy

                self.line_coords = final_line
                self.get_logger().info(f"🧱 Sealed line from {start} to {end} on step {step}")
                return

            # Step forward in direction if not found
            world_x += step_x * self.map_resolution
            world_y += step_y * self.map_resolution

        self.get_logger().info("❌ No connected points found after all steps.")

    def get_8th_grid_behind(self):
        x, y = self.get_robot_grid_position()  # map coords
        _, _, yaw = self.get_robot_global_position()

        # Rotate 180°
        reverse_yaw = yaw + np.pi

        # Snap to nearest axis
        dx = np.cos(reverse_yaw)
        dy = np.sin(reverse_yaw)
        if abs(dx) > abs(dy):
            step = (1 if dx > 0 else -1, 0)
        else:
            step = (0, 1 if dy > 0 else -1)

        # Get 8th cell behind
        gx = x + 8 * step[0]
        gy = y + 8 * step[1]
        return (gx, gy)
    # =======================
    # Fast Loop (10 Hz) – Sensor Polling
    # =======================

    def fast_loop(self):
        """
        High-frequency loop (10 Hz) for real-time monitoring.
        This should NEVER block.
        """
        bot_current_state = self.get_state()
        if bot_current_state == GlobalController.State.Imu_Interrupt:

            while self.current_goal_handle:
                goal_handle = self.current_goal_handle.pop()  # or popleft()

                if goal_handle.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                    self.get_logger().info("🛑 Cancelling active goal...")
                    goal_handle.cancel_goal_async()
            self.stopbot()
            pass
        elif bot_current_state == GlobalController.State.Exploratory_Mapping:
            if(self.previous_state != bot_current_state):
                self.get_logger().info("Exploratory Mapping...")
                self.previous_state = bot_current_state
            if self.IMU_interrupt_check() and not self.hit_ramped: ## IMU interrupt is true
                self.set_state(GlobalController.State.Imu_Interrupt)
                self.get_logger().info("IMU Interrupt detected, changing state to IMU Interrupt")
            pass
        elif bot_current_state == GlobalController.State.Goal_Navigation:
            if self.IMU_interrupt_check() and not self.hit_ramped:
                self.set_state(GlobalController.State.Imu_Interrupt)
                self.get_logger().info("IMU Interrupt detected, changing state to IMU Interrupt")
            pass
        elif bot_current_state == GlobalController.State.Go_to_Heat_Souce:
            self.IMU_interrupt_check()

        

        elif bot_current_state == GlobalController.State.Launching_Balls:
            ## do nothing, waiting on controller to change state, this state should be idle
            pass
        elif bot_current_state == GlobalController.State.Attempting_Ramp:
            ## check for ramp using IMU Data (potentially), poll for when IMU is flat, so there is no pitch meaning the top of the remp has been reached
            #self.drive_straight_between_walls()
            pass



    # =======================
    # ✅ Control Loop (1 Hz) – Decision Making
    # =======================

    def control_loop(self):
        """Slower decision-making loop (1 Hz)"""
        bot_current_state = self.get_state()
        self.publish_visualization_markers()
        if bot_current_state == GlobalController.State.Initializing:
            self.initialise()
            self.set_state(GlobalController.State.Exploratory_Mapping)
        elif bot_current_state == GlobalController.State.Imu_Interrupt:
            self.get_logger().info("🛤🛤🛤🛤🛤IMU Interrupt detected from control loop, walling off are and setting alternative goal")
            self.hit_ramped = True
            self.ramp_location = self.get_robot_grid_position()
            #self.ramp_location = self.get_robot_global_position()
            #self.adaptive_seal_line()
            self.mark_area_around_robot_as_occ(self.ramp_location[0],self.ramp_location[1],7)
            self.get_logger().info(f"Marked area around robot as occupied")
            x ,y, yaw = self.previous_position[0]
            self.nav_to_goal(x,y)
            self.get_logger().info(f"Reversing")
            time.sleep(20) # to give time for control loop to choose a new path and place a "do not go" marker
            self.set_state(GlobalController.State.Exploratory_Mapping)

        elif bot_current_state == GlobalController.State.Exploratory_Mapping:
            self.get_logger().info("Exploratory Mapping (control_loop)...")
            if not self.occ_processing:
                self.dijk_mover()
                pos = self.get_robot_global_position()
                if pos is not None:
                    self.previous_position.append(pos)
            '''
            if not self.goal_active:
                self.dijk_mover()

                pos = self.get_robot_global_position()
                if pos is not None:

                    self.previous_position.append(pos)
            else:
                self.get_logger().info("🚫 Goal is active, skipping exploratory mapping.")
            '''
            ## threshold for fully maped area to cut off exploratory mapping
            if self.finished_mapping:
                ## max heat positions and set goal to the max heat position (stored in self.temp_and_location_data), store it in self.max_heat_locations
                self.get_logger().info("Finished Mapping, changing state to Goal Navigation")
                self.max_heat_locations = self.find_centers(self.clusters)
                self.get_logger().info("🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥")
                self.get_logger().info("🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥")
                self.get_logger().info(f"Max heat locations: {self.max_heat_locations}")
                self.set_state(GlobalController.State.Goal_Navigation)

        elif bot_current_state == GlobalController.State.Goal_Navigation:
            '''
            # Case 1: In the middle of a run (goal is active and not reached yet) → Do nothing
            if self.goal_active and not self.just_reached_goal:
                return  # Nothing to do yet, wait for result callback to update the flag

            # Case 2: Just reached a goal → stop, launch, then check for more
            if self.just_reached_goal:
                self.get_logger().info("🎯 Goal reached, launching ball...")
                self.stopbot()
                time.sleep(15)
                self.launch_ball()
                time.sleep(30)

                self.just_reached_goal = False
                self.goal_active = False  # Ready for next goal

            # Case 3: No goal is active → send next one if any remain
            if not self.goal_active and len(self.max_heat_locations) > 0:
                location = self.max_heat_locations.pop(0)
                self.get_logger().info(f"📬 Sending new goal to: {location}")
                self.nav_to_goal(*location)
                self.goal_active = True

            # Case 4: Nothing left → switch state
            if not self.goal_active and len(self.max_heat_locations) == 0:
                self.get_logger().info("✅ All heat goals complete. Switching to ramp state.")
                self.finished_mapping = False
                self.set_state(GlobalController.State.Attempting_Ramp)

            self.set_state(GlobalController.State.Attempting_Ramp)
            '''
            for location in self.max_heat_locations:

                world_x, world_y = location
                self.nav_to_goal(world_x, world_y)
                #result = self.normal_bfs_from_world(world_x,world_y)
                #self.get_logger().info(f"Result of BFS from world: {result}")
                #if not result:
                #    self.get_logger().info("Unable to reach heat")

                self.get_logger().info("Goal Navigation, setting state to Launching Balls")
                self.stopbot()
                time.sleep(20)
                self.stopbot()
                self.launch_ball()
                time.sleep(40)
            self.set_state(GlobalController.State.Attempting_Ramp)

        elif bot_current_state == GlobalController.State.Attempting_Ramp:
            if(self.ramp_location is None):
                self.set_state(GlobalController.State.Exploratory_Mapping)
            else:
                x,y = self.grid_to_world(self.ramp_location[0], self.ramp_location[1])
                self.nav_to_goal(x , y, self.initial_yaw)
                time.sleep(30)
                self.run_pd_until_obstacle(self.initial_yaw)
                self.stopbot()
                time.sleep(30)
                self.launch_ball()
                time.sleep(300)
            '''
            if self.init_attempt_ramp:
                self.line_coords =[] #remove blob
                self.init_attempt_ramp = False
            else:
                self.dijk_mover()

            #if not self.goal_active:
            #    self.dijk_mover()
            #else:
            #    self.get_logger().info("🚫 Goal is active, skipping exploratory mapping.")
            
            if self.finished_mapping:
                time.sleep(30)
                self.launch_ball()
                time.sleep(300)
            '''

            

            '''
            if self.ramp_location is None:
                time.sleep(300)
            else:
                #x , y = self.ramp_location
                self.nav_to_goal(x, y)
                #self.drive_straight_between_walls()
                self.stopbot()
                time.sleep(5)
                self.launch_ball()
                time.sleep(50)
            '''



    def initialise(self):
        self.wait_for_map()
        pos = self.get_robot_global_position()
        while pos is None:
            pos = self.get_robot_global_position()
            time.sleep(0.2)
        x , y , yaw = pos
        self.initial_yaw = yaw


def main(args=None):#
    rclpy.init(args=args)

    # ✅ Use MultiThreadedExecutor to support timers + background tasks
    executor = MultiThreadedExecutor(num_threads=3)

    global_controller = GlobalController()
    executor.add_node(global_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        global_controller.get_logger().info("Shutting down...")
    finally:
        global_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
