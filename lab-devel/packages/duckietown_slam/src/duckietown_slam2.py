#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
import tf
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, TransformStamped
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

class VisualSLAMMapper:
    def __init__(self):
        self.veh = os.environ['VEHICLE_NAME']
        self.bridge = CvBridge()
        rospy.loginfo(f"Initializing SLAM Mapper for {self.veh}")
        
        # Get params
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_frame')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # Map parameters
        self.map_resolution = 0.025  # meters per pixel
        self.map_width = 400      # pixels  
        self.map_height = 400     # pixels
        self.map_origin_x = -5.0  # meters
        self.map_origin_y = -5.0  # meters
        
        # Initialize map
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map.fill(-1)  # -1 represents unknown space
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        # Publishers
        self.map_pub = rospy.Publisher(f'/{self.veh}/slam/map', OccupancyGrid, queue_size=1)
        
        # Subscribers
        self.lane_sub = rospy.Subscriber(
            f'/{self.veh}/lane_detector_node/debug/lines',
            Image,
            self.lane_callback,
            queue_size=1
        )
        
        # Timer for publishing map
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_map)
        rospy.loginfo("SLAM Mapper initialization complete")

    def lane_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo(f"Received lane image with shape {cv_image.shape}")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {str(e)}")
            return

        try:
            # Get transform from camera to map
            self.tf_listener.waitForTransform("/map", self.camera_frame, rospy.Time(0), rospy.Duration(0.1))
            (trans, rot) = self.tf_listener.lookupTransform("/map", self.camera_frame, rospy.Time(0))
            
            # Convert to euler
            roll, pitch, yaw = euler_from_quaternion(rot)
            
            # Update map with detected lanes
            self.update_map_from_lanes(cv_image, trans[0], trans[1], yaw)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {str(e)}")
            return

    def update_map_from_lanes(self, lane_image, camera_x, camera_y, camera_yaw):
        # Convert lane image to grayscale and threshold
        gray = cv2.cvtColor(lane_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        
        # Use adaptive thresholding for better lane detection
        binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY, 11, 2)
        
        # Find lane pixels
        lane_pixels = np.where(binary > 0)
        pixel_count = len(lane_pixels[0])
        rospy.loginfo(f"Found {pixel_count} lane pixels")
        
        # Image to camera frame conversion factors
        height, width = lane_image.shape[:2]
        fov_horizontal = np.pi/3  # 60 degrees
        fov_vertical = fov_horizontal * height/width
        
        updated_cells = 0
        
        # Transform lane pixels to map coordinates
        for img_y, img_x in zip(lane_pixels[0], lane_pixels[1]):
            # Convert image coordinates to angles
            angle_horizontal = ((img_x - width/2)/(width/2)) * (fov_horizontal/2)
            angle_vertical = ((height - img_y)/(height/2)) * (fov_vertical/2)
            
            # Estimate distance based on vertical angle (assuming flat ground)
            distance = 0.5 / np.tan(angle_vertical)  # 0.5m is approx camera height
            
            # Convert to camera frame coordinates
            camera_frame_x = distance * np.cos(angle_horizontal)
            camera_frame_y = distance * np.sin(angle_horizontal)
            
            # Transform to map frame
            map_x = camera_x + camera_frame_x * np.cos(camera_yaw) - camera_frame_y * np.sin(camera_yaw)
            map_y = camera_y + camera_frame_x * np.sin(camera_yaw) + camera_frame_y * np.cos(camera_yaw)
            
            # Convert to map cells
            cell_x = int((map_x - self.map_origin_x) / self.map_resolution)
            cell_y = int((map_y - self.map_origin_y) / self.map_resolution)
            
            # Update map if within bounds with confidence filtering
            if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
                # Only update if we're confident about the measurement
                if distance < 3.0:  # Only trust measurements within 3 meters
                    current_value = self.map[cell_y, cell_x]
                    if current_value == -1:  # Unknown cell
                        self.map[cell_y, cell_x] = 100
                    elif current_value < 100:  # Increase confidence with repeated observations
                        self.map[cell_y, cell_x] = min(100, current_value + 10)
                    updated_cells += 1
        
        # Log detailed debugging info
        rospy.loginfo(f"Updated {updated_cells} map cells")
        rospy.loginfo(f"Robot pose: x={camera_x:.2f}, y={camera_y:.2f}, yaw={camera_yaw:.2f}")
        if updated_cells > 0:
            rospy.loginfo(f"Average distance of measurements: {sum(distances)/len(distances):.2f}m")
        
        # Periodically clean up noise
        if updated_cells == 0:
            # Remove isolated points that might be noise
            kernel = np.ones((3,3), np.uint8)
            map_binary = (self.map == 100).astype(np.uint8)
            cleaned = cv2.morphologyEx(map_binary, cv2.MORPH_OPEN, kernel)
            self.map[map_binary != cleaned] = -1

    def publish_map(self, event):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = self.map.flatten().tolist()
        
        # Count number of occupied cells
        occupied = np.sum(self.map == 100)
        unknown = np.sum(self.map == -1)
        rospy.loginfo(f"Publishing map - Occupied cells: {occupied}, Unknown cells: {unknown}")
        
        self.map_pub.publish(map_msg)

if __name__ == '__main__':
    rospy.init_node('visual_slam_mapper')
    mapper = VisualSLAMMapper()
    rospy.spin()