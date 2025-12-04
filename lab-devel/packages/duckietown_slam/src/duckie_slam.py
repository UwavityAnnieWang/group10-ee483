#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion
import tf2_ros
import tf.transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import math
from collections import deque

class DuckietownSLAM:
    def __init__(self):
        rospy.init_node('duckietown_slam')
        
        # Get the vehicle name from environment
        self.veh_name = os.environ['VEHICLE_NAME']
        rospy.loginfo(f"Initializing SLAM for {self.veh_name}")
        
        # SLAM Parameters
        self.scan_buffer_size = rospy.get_param('~scan_buffer_size', 10)
        self.loop_search_distance = rospy.get_param('~loop_search_distance', 2.0)
        self.loop_match_threshold = rospy.get_param('~loop_match_threshold', 0.7)
        self.min_displacement_threshold = rospy.get_param('~min_displacement', 0.1)
        self.map_update_interval = rospy.get_param('~map_update_interval', 1.0)
        self.transform_publish_period = rospy.get_param('~transform_publish_period', 0.05)
        
        # Initialize pose tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Initialize pose graph
        self.pose_graph = []  # List of (pose, scan_data) tuples
        self.loop_closure_queue = []
        self.recent_scans = deque(maxlen=self.scan_buffer_size)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize tf2 broadcaster and buffer
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Publishers
        self.lane_pub = rospy.Publisher(f'/{self.veh_name}/lane_detector_node/debug/lines', Image, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.pose_pub = rospy.Publisher('/slam_pose', PoseStamped, queue_size=1)
        
        # Subscribers
        camera_topic = f'/{self.veh_name}/camera_node/image/compressed'
        self.image_sub = rospy.Subscriber(
            camera_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=10000000
        )
        
        # Parameters for lane detection
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.lower_white = np.array([0, 0, 140])
        self.upper_white = np.array([180, 70, 255])
        
        # Map parameters
        # Initialize map with unknown (-1) state
        self.map_resolution = 0.01  # meters per pixel
        self.map_width = 1000      # pixels
        self.map_height = 1000     # pixels
        self.map_origin_x = -5.0   # Reduced from -25.0 to keep map centered
        self.map_origin_y = -5.0   # Reduced from -25.0 to keep map centered
        
        # Initialize occupancy probabilities instead of binary values
        self.map_msg = OccupancyGrid()
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = self.map_width
        self.map_msg.info.height = self.map_height
        self.map_msg.info.origin.position.x = self.map_origin_x
        self.map_msg.info.origin.position.y = self.map_origin_y
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = [-1] * (self.map_width * self.map_height)
        
        # Add probability map to store continuous values
        self.probability_map = np.zeros((self.map_height, self.map_width))

    def add_pose_to_graph(self, pose, scan_data):
        """Add new pose and associated scan data to pose graph"""
        if len(self.pose_graph) == 0:
            self.pose_graph.append((pose, scan_data))
            return
            
        # Check if we've moved enough to add new pose
        last_pose = self.pose_graph[-1][0]
        dist = np.sqrt((pose[0]-last_pose[0])**2 + (pose[1]-last_pose[1])**2)
        angle_diff = abs(pose[2] - last_pose[2])
        
        if dist > self.min_displacement_threshold or angle_diff > 0.1:
            self.pose_graph.append((pose, scan_data))
            self.recent_scans.append((pose, scan_data))

    def find_loop_closures(self, current_pose, current_scan):
        """Detect potential loop closures"""
        candidates = []
        
        # Skip recent poses to avoid trivial loop closures
        search_start = max(0, len(self.pose_graph) - self.scan_buffer_size)
        
        for i in range(search_start):
            old_pose, old_scan = self.pose_graph[i]
            dist = np.sqrt((current_pose[0]-old_pose[0])**2 + 
                          (current_pose[1]-old_pose[1])**2)
            
            if dist < self.loop_search_distance:
                # Compare scans
                match_score = self.compare_scans(current_scan, old_scan)
                if match_score > self.loop_match_threshold:
                    candidates.append((i, match_score))
        
        return candidates

    def compare_scans(self, scan1, scan2):
        """Compare two scans for loop closure detection"""
        if scan1 is None or scan2 is None:
            return 0.0
            
        # Convert line detections to point format
        def get_scan_points(scan):
            points = []
            for line in scan:
                x1, y1, x2, y2 = line[0]
                points.extend([(x1,y1), (x2,y2)])
            return np.array(points)
        
        points1 = get_scan_points(scan1)
        points2 = get_scan_points(scan2)
        
        if len(points1) == 0 or len(points2) == 0:
            return 0.0
            
        # Calculate point matching score
        min_dists = []
        for p1 in points1:
            dists = np.sqrt(np.sum((points2 - p1)**2, axis=1))
            min_dists.append(np.min(dists))
        
        return 1.0 / (1.0 + np.mean(min_dists))

    def optimize_pose_graph(self, loop_closures):
        """Optimize pose graph after loop closure detection"""
        if not loop_closures:
            return
            
        # Simple pose graph optimization
        for pose_idx, score in loop_closures:
            old_pose = self.pose_graph[pose_idx][0]
            
            # Calculate correction
            correction = np.array([
                old_pose[0] - self.current_x,
                old_pose[1] - self.current_y,
                old_pose[2] - self.current_theta
            ]) * 0.1  # Small correction factor
            
            # Update recent poses
            for i in range(len(self.pose_graph)-1, pose_idx, -1):
                pose, scan = self.pose_graph[i]
                new_pose = (
                    pose[0] + correction[0],
                    pose[1] + correction[1],
                    pose[2] + correction[2]
                )
                self.pose_graph[i] = (new_pose, scan)
            
            # Update current pose
            self.current_x += correction[0]
            self.current_y += correction[1]
            self.current_theta += correction[2]

    def publish_static_transforms(self):
        """Publish static transforms between frames"""
        camera_transform = TransformStamped()
        camera_transform.header.stamp = rospy.Time.now()
        camera_transform.header.frame_id = f"{self.veh_name}/base_link"
        camera_transform.child_frame_id = f"{self.veh_name}/camera_optical_frame"
        
        camera_transform.transform.translation.x = 0.1
        camera_transform.transform.translation.y = 0.0
        camera_transform.transform.translation.z = 0.1
        camera_transform.transform.rotation.x = 0.0
        camera_transform.transform.rotation.y = 0.0
        camera_transform.transform.rotation.z = 0.0
        camera_transform.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(camera_transform)

    def publish_transforms(self, event=None):
        """Publish dynamic transforms"""
        try:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = f"{self.veh_name}/base_link"
            
            # Set current pose
            transform.transform.translation.x = self.current_x
            transform.transform.translation.y = self.current_y
            transform.transform.translation.z = 0.0
            
            # Convert theta to quaternion
            quat = quaternion_from_euler(0, 0, self.current_theta)
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]

            # Publish transform
            self.tf_broadcaster.sendTransform(transform)
            
            # Also publish map to odom transform (identity)
            odom_transform = TransformStamped()
            odom_transform.header.stamp = rospy.Time.now()
            odom_transform.header.frame_id = "map"
            odom_transform.child_frame_id = f"{self.veh_name}/odom"
            odom_transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(odom_transform)
            
        except Exception as e:
            rospy.logerr(f"Error publishing transforms: {str(e)}")

    def lane_detection(self, cv_image):
        """Process image to detect lanes"""
        try:
            # Crop image
            height = cv_image.shape[0]
            cropped = cv_image[height//2:, :]
            
            # Convert to HSV and filter white
            hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
            white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
            
            # Apply morphological operations
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.kernel)
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.kernel)
            
            # Edge detection
            edges = cv2.Canny(cropped, 50, 150)
            combined = cv2.bitwise_and(edges, white_mask)
            
            # Hough transform
            lines = cv2.HoughLinesP(
                combined,
                rho=1,
                theta=np.pi/180,
                threshold=50,
                minLineLength=50,
                maxLineGap=10
            )
            
            return lines, cropped
        except Exception as e:
            rospy.logerr(f"Error in lane detection: {str(e)}")
            return None, None

    def convert_lanes_to_scan(self, lines, image):
        """Convert detected lane lines to LaserScan message"""
        try:
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = f"{self.veh_name}/base_link"
            
            scan_msg.angle_min = -np.pi/2
            scan_msg.angle_max = np.pi/2
            scan_msg.angle_increment = np.pi/180
            scan_msg.range_min = 0.1
            scan_msg.range_max = 5.0
            
            num_readings = int((scan_msg.angle_max - scan_msg.angle_min) 
                             / scan_msg.angle_increment)
            scan_msg.ranges = [float('inf')] * num_readings
            
            if lines is not None and image is not None:
                height, width = image.shape[:2]
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    
                    # Convert to robot frame coordinates
                    x1_r = (width/2 - x1) * self.map_resolution
                    y1_r = (height - y1) * self.map_resolution
                    x2_r = (width/2 - x2) * self.map_resolution
                    y2_r = (height - y2) * self.map_resolution
                    
                    # Calculate angles and ranges
                    angle1 = np.arctan2(y1_r, x1_r)
                    angle2 = np.arctan2(y2_r, x2_r)
                    range1 = np.sqrt(x1_r**2 + y1_r**2)
                    range2 = np.sqrt(x2_r**2 + y2_r**2)
                    
                    # Add to scan message
                    idx1 = int((angle1 - scan_msg.angle_min) / scan_msg.angle_increment)
                    idx2 = int((angle2 - scan_msg.angle_min) / scan_msg.angle_increment)
                    
                    if 0 <= idx1 < num_readings:
                        scan_msg.ranges[idx1] = min(range1, scan_msg.ranges[idx1])
                    if 0 <= idx2 < num_readings:
                        scan_msg.ranges[idx2] = min(range2, scan_msg.ranges[idx2])
            
            return scan_msg
        except Exception as e:
            rospy.logerr(f"Error converting lanes to scan: {str(e)}")
            return None

    def update_map(self, scan_msg):
        """Update occupancy grid using pose graph information"""
        if scan_msg is None:
            return

        try:
            # Update map using current pose and scan
            curr_pos = (self.current_x, self.current_y, self.current_theta)
            self.integrate_scan_to_map(scan_msg, curr_pos)
            
        except Exception as e:
            rospy.logerr(f"Error updating map: {str(e)}")

    def integrate_scan_to_map(self, scan_msg, pose):
        """Integrate a single scan into the map"""
        x, y, theta = pose
        
        # Log odds parameters
        prior = 0.5
        p_occ = 0.7  # Probability of occupied given detection
        p_free = 0.3  # Probability of occupied given free space
        
        # Convert to log odds
        lo_occ = np.log(p_occ / (1 - p_occ))
        lo_free = np.log(p_free / (1 - p_free))
        lo_prior = np.log(prior / (1 - prior))
        
        for i, r in enumerate(scan_msg.ranges):
            if r < scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment + theta
                px = x + r * np.cos(angle)
                py = y + r * np.sin(angle)
                
                # Convert to grid coordinates
                grid_x = int((px - self.map_msg.info.origin.position.x) / self.map_resolution)
                grid_y = int((py - self.map_msg.info.origin.position.y) / self.map_resolution)
                
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    # Update probability using log odds
                    self.probability_map[grid_y, grid_x] += lo_occ - lo_prior
                    
                    # Ray trace to mark free space
                    start_x = int((x - self.map_msg.info.origin.position.x) / self.map_resolution)
                    start_y = int((y - self.map_msg.info.origin.position.y) / self.map_resolution)
                    self.bresenham_line(start_x, start_y, grid_x, grid_y, lo_free - lo_prior)
                    
                    # Convert to occupancy values (0-100)
                    p = 1 - (1 / (1 + np.exp(self.probability_map[grid_y, grid_x])))
                    self.map_msg.data[grid_y * self.map_width + grid_x] = int(p * 100)

    def bresenham_line(self, x0, y0, x1, y1, value):
        """Draw a line using Bresenham's algorithm and update probabilities"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        for _ in range(n):
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                # Update free space probability
                self.probability_map[y, x] += value
                p = 1 - (1 / (1 + np.exp(self.probability_map[y, x])))
                self.map_msg.data[y * self.map_width + x] = int(p * 100)
                
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

    def mark_free_cells(self, x1, y1, x2, y2):
        """Mark cells along a ray as free using Bresenham's line algorithm"""
        grid_x1 = int((x1 - self.map_msg.info.origin.position.x) / self.map_resolution)
        grid_y1 = int((y1 - self.map_msg.info.origin.position.y) / self.map_resolution)
        grid_x2 = int((x2 - self.map_msg.info.origin.position.x) / self.map_resolution)
        grid_y2 = int((y2 - self.map_msg.info.origin.position.y) / self.map_resolution)
        
        dx = abs(grid_x2 - grid_x1)
        dy = abs(grid_y2 - grid_y1)
        x, y = grid_x1, grid_y1
        n = 1 + dx + dy
        x_inc = 1 if grid_x2 > grid_x1 else -1
        y_inc = 1 if grid_y2 > grid_y1 else -1
        error = dx - dy
        dx *= 2
        dy *= 2
        
        for _ in range(n):
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                idx = y * self.map_width + x
                if self.map_msg.data[idx] != 100:  # Don't override occupied cells
                    self.map_msg.data[idx] = 0  # Free
                
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

    def periodic_map_update(self, event):
        """Periodic callback to update and publish map"""
        self.map_msg.header.stamp = rospy.Time.now()
        self.map_msg.header.frame_id = 'map'
        self.map_pub.publish(self.map_msg)

    def image_callback(self, msg):
        """Main callback for processing camera images"""
        try:
            # Convert compressed image to CV2
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            
            # Detect lanes
            lines, cropped = self.lane_detection(cv_image)
            
            if lines is not None and cropped is not None:
                # Get current pose estimate
                current_pose = (self.current_x, self.current_y, self.current_theta)
                
                # Add to pose graph
                self.add_pose_to_graph(current_pose, lines)
                
                # Check for loop closures
                loop_closure_candidates = self.find_loop_closures(current_pose, lines)
                
                if loop_closure_candidates:
                    # Process loop closures and optimize pose graph
                    self.optimize_pose_graph(loop_closure_candidates)
                
                # Draw detected lines for visualization
                vis_image = cropped.copy()
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Publish lane detection visualization
                self.lane_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, "bgr8"))
                
                # Convert to scan and update map
                scan_msg = self.convert_lanes_to_scan(lines, cropped)
                if scan_msg is not None:
                    self.scan_pub.publish(scan_msg)
                    self.update_map(scan_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in image callback: {str(e)}")

    def __del__(self):
        """Cleanup when node is shutdown"""
        if hasattr(self, 'tf_timer'):
            self.tf_timer.shutdown()
        if hasattr(self, 'map_timer'):
            self.map_timer.shutdown()

if __name__ == '__main__':
    try:
        slam = DuckietownSLAM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass