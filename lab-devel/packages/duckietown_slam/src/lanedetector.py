#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class MultiColorLaneDetector:
    def __init__(self):
        self.veh = os.environ['VEHICLE_NAME']
        self.bridge = CvBridge()
        
        # Publishers for debug images
        self.pub_cropped = rospy.Publisher(f'/{self.veh}/lane_detector_node/debug/cropped', Image, queue_size=1)
        self.pub_white = rospy.Publisher(f'/{self.veh}/lane_detector_node/debug/white_filter', Image, queue_size=1)
        self.pub_yellow = rospy.Publisher(f'/{self.veh}/lane_detector_node/debug/yellow_filter', Image, queue_size=1)
        self.pub_red = rospy.Publisher(f'/{self.veh}/lane_detector_node/debug/red_filter', Image, queue_size=1)
        self.pub_edges = rospy.Publisher(f'/{self.veh}/lane_detector_node/debug/edges', Image, queue_size=1)
        self.pub_lines = rospy.Publisher(f'/{self.veh}/lane_detector_node/debug/lines', Image, queue_size=1)
        
        # Create kernel for morphological operations
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        # Color detection parameters in HSV
        self.white_range = {
            'low': np.array([0, 0, 140]),
            'high': np.array([180, 50, 255])
        }
        
        self.yellow_range = {
            'low': np.array([20, 100, 100]),
            'high': np.array([35, 255, 255])
        }
        
        # Red requires two ranges in HSV
        self.red_range1 = {
            'low': np.array([0, 100, 100]),
            'high': np.array([10, 255, 255])
        }
        self.red_range2 = {
            'low': np.array([160, 100, 100]),
            'high': np.array([180, 255, 255])
        }
        
        # Subscribe to compressed camera image
        self.sub_image = rospy.Subscriber(
            f'/{self.veh}/camera_node/image/compressed',
            CompressedImage,
            self.image_cb,
            queue_size=1,
            buff_size=10000000
        )

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {str(e)}")
            return
            
        # Process image
        # 1. Crop image
        cropped = self.crop_image(cv_image)
        self.pub_cropped.publish(self.bridge.cv2_to_imgmsg(cropped, "bgr8"))
        
        # 2. Filter for colors
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        white_filtered = self.filter_color(hsv, self.white_range)
        yellow_filtered = self.filter_color(hsv, self.yellow_range)
        red_filtered = self.filter_red(hsv)
        
        self.pub_white.publish(self.bridge.cv2_to_imgmsg(white_filtered, "bgr8"))
        self.pub_yellow.publish(self.bridge.cv2_to_imgmsg(yellow_filtered, "bgr8"))
        self.pub_red.publish(self.bridge.cv2_to_imgmsg(red_filtered, "bgr8"))
        
        # 3. Edge detection
        edges = self.detect_edges(cropped)
        self.pub_edges.publish(self.bridge.cv2_to_imgmsg(edges, "mono8"))
        
        # 4. Combine masks with edges
        white_mask = self.create_binary_mask(white_filtered)
        yellow_mask = self.create_binary_mask(yellow_filtered)
        red_mask = self.create_binary_mask(red_filtered)
        
        combined_color_mask = cv2.bitwise_or(cv2.bitwise_or(white_mask, yellow_mask), red_mask)
        combined = cv2.bitwise_and(edges, combined_color_mask)
        
        # 5. Detect and draw lines
        lines_img = self.detect_lines(cropped, combined)
        self.pub_lines.publish(self.bridge.cv2_to_imgmsg(lines_img, "bgr8"))

    def crop_image(self, image):
        height = image.shape[0]
        return image[height//2:, :]

    def filter_color(self, hsv_image, color_range):
        mask = cv2.inRange(hsv_image, color_range['low'], color_range['high'])
        mask = self.apply_morphology(mask)
        return cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    def filter_red(self, hsv_image):
        mask1 = cv2.inRange(hsv_image, self.red_range1['low'], self.red_range1['high'])
        mask2 = cv2.inRange(hsv_image, self.red_range2['low'], self.red_range2['high'])
        mask = cv2.bitwise_or(mask1, mask2)
        mask = self.apply_morphology(mask)
        return cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    def apply_morphology(self, mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return mask

    def create_binary_mask(self, filtered_img):
        gray = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        return mask

    def detect_edges(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.Canny(blurred, 50, 150)

    def detect_lines(self, original, filtered):
        lines = cv2.HoughLinesP(
            filtered,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=50,
            maxLineGap=10
        )
        
        result = original.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
        return result

if __name__ == '__main__':
    rospy.init_node('multi_color_lane_detector')
    rospy.sleep(2.0)
    detector = MultiColorLaneDetector()
    rospy.spin()