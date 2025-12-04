#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class EnhancedLaneDetector:
    def __init__(self):
        self.veh = os.environ['VEHICLE_NAME']   #ee483mm10
        self.bridge = CvBridge()
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30))  # Increase buffer time
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Wait for TF tree to be available
        rospy.sleep(2.0)  # Wait for TF publishers to start
        
        try:
            # Wait for the transform to become available
            self.tf_buffer.lookup_transform(
                'map',
                f'{self.veh}/camera_optical_frame',
                rospy.Time(0),
                rospy.Duration(5.0)
            )
            rospy.loginfo("Transform from map to camera found!")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Initial transform lookup failed: {e}")
        
        # Parameters for lane detection
        self.color_configs = {
            'white': {
                'low': np.array([0, 0, 150]),
                'high': np.array([180, 50, 255]),
                'id': Segment.WHITE
            },
            'yellow': {
                'low': np.array([15, 80, 80]),
                'high': np.array([35, 255, 255]),
                'id': Segment.YELLOW
            },
            'red': {
                'low': np.array([0, 140, 100]),
                'high': np.array([15, 255, 255]),
                'id': Segment.RED
            }
        }
        
        # Processing parameters
        self.roi_top = rospy.get_param('~roi_top', 0.5)
        self.blur_kernel = (5, 5)
        self.canny_thresholds = (50, 150)
        self.hough_threshold = 20
        self.min_line_length = rospy.get_param('~min_line_length', 30)
        self.max_line_gap = rospy.get_param('~max_line_gap', 10)
        self.line_memory = 5  # Number of frames to remember lines
        
        # Line filtering
        self.valid_line_angles = [-30, 30]  # Degrees from vertical
        self.line_history = []
        
        # Publishers
        self.pub_segments = rospy.Publisher(
            f'/{self.veh}/lane_detector_node/segment_list', 
            SegmentList, 
            queue_size=1
        )
        self.pub_debug = rospy.Publisher(
            f'/{self.veh}/lane_detector_node/debug/detection/compressed', 
            CompressedImage, 
            queue_size=1
        )

        # Subscribe to camera
        self.sub_image = rospy.Subscriber(
            f'/{self.veh}/camera_node/image/compressed',
            CompressedImage,
            self.process_image,
            queue_size=1,
            buff_size=10000000
        )

    def process_image(self, msg):
        """Main image processing pipeline"""
        try:
            # Convert compressed image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            
            # Get camera transform
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    f'{self.veh}/camera_optical_frame',
                    msg.header.stamp,
                    rospy.Duration(0.1)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                rospy.logwarn_throttle(1, "Could not get camera transform")
                return
                
            # Process image
            lanes = self.detect_lanes(cv_image)
            
            # Convert to map frame and publish
            if lanes:
                self.publish_lanes(lanes, transform, msg.header.stamp)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def detect_lanes(self, image):
        """Detect lanes using color segmentation and line detection"""
        # Crop ROI
        h, w = image.shape[:2]
        roi_y = int(h * self.roi_top)
        roi = image[roi_y:, :]
        
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        all_lanes = []
        debug_img = roi.copy()
        
        # Process each color
        for color, config in self.color_configs.items():
            # Color threshold
            mask = cv2.inRange(hsv, config['low'], config['high'])
            
            # Clean up mask
            kernel = np.ones((3,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Edge detection
            edges = cv2.Canny(mask, *self.canny_thresholds)
            
            # Line detection
            lines = cv2.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi/180,
                threshold=self.hough_threshold,
                minLineLength=self.min_line_length,
                maxLineGap=self.max_line_gap
            )
            
            if lines is not None:
                filtered_lines = self.filter_lines(lines)
                for line in filtered_lines:
                    x1, y1, x2, y2 = line
                    # Adjust y coordinates for ROI
                    y1 += roi_y
                    y2 += roi_y
                    
                    lane = {
                        'points': [(x1, y1), (x2, y2)],
                        'color': config['id']
                    }
                    all_lanes.append(lane)
                    
                    # Draw on debug image
                    color_bgr = (0,255,0) if color == 'white' else (0,255,255) if color == 'yellow' else (0,0,255)
                    cv2.line(debug_img, (x1, y1-roi_y), (x2, y2-roi_y), color_bgr, 2)
        
        # Publish debug image
        if rospy.get_param('~publish_debug', True):
            msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            self.pub_debug.publish(msg)
            
        return all_lanes

    def filter_lines(self, lines):
        """Filter lines based on angle and other criteria"""
        filtered = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate line angle
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            angle = angle % 180  # Normalize angle
            
            # Check if angle is within valid range
            if self.valid_line_angles[0] <= angle <= self.valid_line_angles[1]:
                # Calculate line length
                length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                
                if length >= self.min_line_length:
                    filtered.append([x1, y1, x2, y2])
        
        return filtered

    def publish_lanes(self, lanes, transform, stamp):
        """Convert detected lanes to SegmentList and publish"""
        segment_list = SegmentList()
        segment_list.header.stamp = stamp
        segment_list.header.frame_id = 'map'
        
        # Get transform data
        trans = transform.transform.translation
        rot = transform.transform.rotation
        euler = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        
        # Process each lane
        for lane in lanes:
            segment = Segment()
            segment.color = lane['color']
            
            # Convert both endpoints
            for i, (px, py) in enumerate(lane['points']):
                # Convert to camera frame
                x_cam = (px - 320) * 0.002  # Approximate pixel to meter conversion
                y_cam = (py - 240) * 0.002
                
                # Transform to map frame
                x_map = x_cam * np.cos(euler[2]) - y_cam * np.sin(euler[2]) + trans.x
                y_map = x_cam * np.sin(euler[2]) + y_cam * np.cos(euler[2]) + trans.y
                
                # Store point
                point = Point()
                point.x = x_map
                point.y = y_map
                point.z = 0
                segment.points.append(point)
            
            segment_list.segments.append(segment)
        
        self.pub_segments.publish(segment_list)

if __name__ == '__main__':
    rospy.init_node('lane_detector_node')
    detector = EnhancedLaneDetector()
    rospy.spin()