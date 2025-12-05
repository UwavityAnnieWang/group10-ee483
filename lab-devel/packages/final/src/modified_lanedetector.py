#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv2
import os
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import Segment, SegmentList, Vector2D # IMPORT THESE
from cv_bridge import CvBridge

class LaneDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.veh = os.environ['VEHICLE_NAME'] # Get robot name dynamically
        
        # Keep your original Publishers
        self.pub_detection = rospy.Publisher(f"/{self.veh}/lane_detector_node/debug/detection", Image, queue_size=1)
        
        # ADD THIS NEW PUBLISHER (This talks to SLAM)
        self.pub_segments = rospy.Publisher(f"/{self.veh}/lane_detector_node/segment_list", SegmentList, queue_size=1)

        # Updated Subscriber to use dynamic vehicle name
        self.sub_image = rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        # 1. Convert Image
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        
        # 2. Store original dimensions for normalization
        original_height, original_width = cv_img.shape[:2]

        # 3. Your Original Cropping Logic
        crop_offset = original_height // 3
        cropped_img = cv_img[crop_offset:original_height, :]
        
        # 4. Your Original Filters (Gray, Blur, Canny, Dilate)
        gray_cropped = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray_cropped, (7,7), 0)
        canny_img = cv2.Canny(blur, 20, 80)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        canny_img = cv2.dilate(canny_img, kernel)

        # 5. Your Original HSV Filters
        hsv_image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        
        # Your specific thresholds
        white_filter = cv2.inRange(hsv_image, (0,0,180), (180,60, 255))
        yellow_filter = cv2.inRange(hsv_image, (20,66,92), (60,255,255))
        red_filter1 = cv2.inRange(hsv_image, (0, 100, 100), (20, 255, 255))
        red_filter2 = cv2.inRange(hsv_image, (225, 100, 100), (255, 255, 255))
        red_filter = cv2.bitwise_or(red_filter1, red_filter2)

        # 6. Apply Erosion & Combine
        erosion_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        white_mask = cv2.erode(white_filter, erosion_kernel)
        yellow_mask = cv2.erode(yellow_filter, erosion_kernel)
        red_mask = cv2.erode(red_filter, erosion_kernel)

        combined_white = cv2.bitwise_and(canny_img, canny_img, mask=white_mask)
        combined_yellow = cv2.bitwise_and(canny_img, canny_img, mask=yellow_mask)
        combined_red = cv2.bitwise_and(canny_img, canny_img, mask=red_mask)

        # 7. Find Lines
        lines_white = cv2.HoughLinesP(combined_white, 1, np.pi/180, 40, minLineLength=20, maxLineGap=5)
        lines_yellow = cv2.HoughLinesP(combined_yellow, 1, np.pi/180, 40, minLineLength=20, maxLineGap=5)
        lines_red = cv2.HoughLinesP(combined_red, 1, np.pi/180, 40, minLineLength=20, maxLineGap=5)

        # ==========================================================
        # NEW SECTION: CONVERT LINES TO ROS MESSAGES FOR SLAM
        # ==========================================================
        segment_list = SegmentList()
        segment_list.header = msg.header # Keep original timestamp

        # Helper function to process lines
        def add_lines_to_list(lines, color_id):
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    
                    # Create Segment Object
                    seg = Segment()
                    seg.color = color_id
                    
                    # Normalize points (0.0 to 1.0) and Fix Crop Offset
                    # Important: We must add 'crop_offset' to y because we cropped the top!
                    
                    pt1 = Vector2D()
                    pt1.x = float(x1) / original_width
                    pt1.y = float(y1 + crop_offset) / original_height
                    
                    pt2 = Vector2D()
                    pt2.x = float(x2) / original_width
                    pt2.y = float(y2 + crop_offset) / original_height
                    
                    seg.pixels_normalized = [pt1, pt2]
                    segment_list.segments.append(seg)

        # Add all colors
        add_lines_to_list(lines_white, Segment.WHITE)
        add_lines_to_list(lines_yellow, Segment.YELLOW)
        add_lines_to_list(lines_red, Segment.RED)

        # Publish the list for SLAM
        self.pub_segments.publish(segment_list)

        # ==========================================================
        # END NEW SECTION
        # ==========================================================

        # Optional: Keep your debug visualization if you want to see it
        debug_img = cropped_img.copy()
        if lines_white is not None:
            for l in lines_white:
                cv2.line(debug_img, (l[0][0], l[0][1]), (l[0][2], l[0][3]), (255, 255, 255), 2)
        if lines_yellow is not None:
            for l in lines_yellow:
                cv2.line(debug_img, (l[0][0], l[0][1]), (l[0][2], l[0][3]), (0, 255, 255), 2)
                
        self.pub_detection.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

if __name__=="__main__":
    rospy.init_node("image_detector", anonymous=True)
    img_detect = LaneDetector()
    rospy.spin()