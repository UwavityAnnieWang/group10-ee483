#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import tf2_ros
from tf.transformations import quaternion_from_euler
import math

class DuckietownSLAM:
    def __init__(self):
        self.node_name = "duckietown_slam_node"
        self.veh = os.environ['VEHICLE_NAME']
        
        # Initialize ROS
        rospy.init_node(self.node_name, anonymous=False)
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Initialize feature detector
        self.feature_detector = cv2.ORB_create(
            nfeatures=500,
            scaleFactor=1.2,
            nlevels=8
        )
        
        # Initialize BFMatcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Initialize state variables
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.position = np.zeros(3)  # [x, y, theta]
        
        # Initialize map
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.resolution = 0.025  # 2.5cm per pixel
        self.map.info.width = 400      # 10m / 0.025
        self.map.info.height = 400     # 10m / 0.025
        self.map.info.origin.position.x = -5.0
        self.map.info.origin.position.y = -5.0
        self.map.data = [-1] * (self.map.info.width * self.map.info.height)
        
        # Publishers
        self.pub_map = rospy.Publisher('map', OccupancyGrid, queue_size=1, latch=True)
        
        # Subscribers
        self.sub_image = rospy.Subscriber(
            f'/{self.veh}/camera_node/image/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=10000000
        )
        
        # Timer for publishing map and transforms
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        rospy.loginfo(f"[{self.node_name}] Initialized.")

    def timer_callback(self, event):
        """Publish map and transforms periodically"""
        now = rospy.Time.now()
        
        # Publish map
        self.map.header.stamp = now
        self.pub_map.publish(self.map)
        
        # Publish transform
        self.publish_transform(now)

    def publish_transform(self, timestamp):
        """Publish transform from map to base_link"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "map"
        t.child_frame_id = f"{self.veh}/footprint"
        
        # Set translation
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = 0.0
        
        # Set rotation
        q = quaternion_from_euler(0, 0, self.position[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def update_map(self, x, y, is_lane_marking=True):
        """Update occupancy grid map"""
        # Convert world coordinates to map coordinates
        map_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        map_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

        # Check bounds
        if (0 <= map_x < self.map.info.width and 0 <= map_y < self.map.info.height):
            # Mark cell as occupied or free
            idx = map_y * self.map.info.width + map_x
            if is_lane_marking:
                self.map.data[idx] = -1  # Mark as free space
            else:
                self.map.data[idx] = 100  # Mark as occupied
    
    def process_image(self, cv_image):
        """Process the camera image to detect lane markings"""
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define HSV ranges for different lane markings
        white_lower = (0, 0, 200)
        white_upper = (180, 30, 255)
        yellow_lower = (20, 100, 100) 
        yellow_upper = (40, 255, 255)
        red_lower = (0, 100, 100)
        red_upper = (10, 255, 255)
        
        # Create masks for each color
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        
        # Apply morphological operations to clean up the masks
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        white_mask = cv2.dilate(white_mask, kernel, iterations=2)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=2)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)
        
        # Use Hough transform to detect lines in each mask
        white_lines = cv2.HoughLinesP(white_mask, 1, np.pi/180, 50, minLineLength=20, maxLineGap=5)
        yellow_lines = cv2.HoughLinesP(yellow_mask, 1, np.pi/180, 50, minLineLength=20, maxLineGap=5)
        red_lines = cv2.HoughLinesP(red_mask, 1, np.pi/180, 50, minLineLength=20, maxLineGap=5)
        
        # Return the detected lines
        return white_lines, yellow_lines, red_lines
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert compressed image to CV2
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

            # Process the image to detect lane markings
            white_lines, yellow_lines, red_lines = self.process_image(cv_image)

            # Update the map with the detected lane markings
            if white_lines is not None:
                for line in white_lines:
                    x1, y1, x2, y2 = line[0]
                    self.update_map(x1, y1, is_lane_marking=True)
                    self.update_map(x2, y2, is_lane_marking=True)
            if yellow_lines is not None:
                for line in yellow_lines:
                    x1, y1, x2, y2 = line[0]
                    self.update_map(x1, y1, is_lane_marking=True)
                    self.update_map(x2, y2, is_lane_marking=True)
            if red_lines is not None:
                for line in red_lines:
                    x1, y1, x2, y2 = line[0]
                    self.update_map(x1, y1, is_lane_marking=True)
                    self.update_map(x2, y2, is_lane_marking=True)
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect features
            keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)
            
            if descriptors is None:
                rospy.logwarn("No features detected in current frame")
                return
                
            if self.prev_keypoints is not None and self.prev_descriptors is not None:
                try:
                    # Match features
                    matches = self.matcher.match(self.prev_descriptors, descriptors)
                    
                    if len(matches) < 10:
                        rospy.logwarn("Not enough matches found")
                        return
                        
                    # Sort matches by distance
                    matches = sorted(matches, key=lambda x: x.distance)
                    
                    # Take only good matches
                    good_matches = matches[:30]
                    
                    # Extract matched points
                    prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in good_matches])
                    curr_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches])
                    
                    # Estimate motion
                    E, mask = cv2.findEssentialMat(prev_pts, curr_pts, 
                                                 focal=1.0, pp=(cv_image.shape[1]/2, cv_image.shape[0]/2),
                                                 method=cv2.RANSAC, prob=0.999, threshold=3.0)
                    
                    if E is not None:
                        # Recover pose
                        _, R, t, _ = cv2.recoverPose(E, prev_pts, curr_pts)
                        
                        # Scale the translation (tuned for Duckiebot)
                        scale = 0.01  # 1cm per step maximum
                        delta_x = float(t[0] * scale)
                        delta_y = float(t[1] * scale)
                        
                        # Update position
                        theta = math.atan2(R[1,0], R[0,0])
                        self.position[0] += delta_x * math.cos(self.position[2]) - delta_y * math.sin(self.position[2])
                        self.position[1] += delta_x * math.sin(self.position[2]) + delta_y * math.cos(self.position[2])
                        self.position[2] += theta
                        
                        # Update map
                        self.update_map(self.position[0], self.position[1])
                    
                except cv2.error as e:
                    rospy.logwarn(f"OpenCV error: {str(e)}")
                except Exception as e:
                    rospy.logwarn(f"Error in motion estimation: {str(e)}")
            
            # Store current features for next frame
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing image: {str(e)}")

if __name__ == '__main__':
    try:
        node = DuckietownSLAM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass