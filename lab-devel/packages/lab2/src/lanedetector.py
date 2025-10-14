#!/usr/bin/env python3

import sys
import rospy

import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import os
from cv_bridge import CvBridge

class LaneDetector:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        #rospy.Subscriber("image", Image, self.flipper_cb)
        rospy.Subscriber("/ee483mm10/camera_node/image/compressed", CompressedImage, self.flipper_cb, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
        self.pub_crop = rospy.Publisher("/cropped", Image, queue_size=10)
        self.pub_mask1 = rospy.Publisher("/mask1", Image, queue_size=10)
        self.pub_mask2 = rospy.Publisher("/mask2", Image, queue_size=10)
        self.pub_detection = rospy.Publisher("/detection", Image, queue_size=10)
    def flipper_cb(self, msg):
        # convert to a ROS image using the bridge
        rospy.loginfo("Successfully run")
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        #crop
        height = cv_img.shape[0]
        cropped_img = cv_img[height//3:height, :]
        ros_cropped = self.bridge.cv2_to_imgmsg(cropped_img, 'bgr8')
        self.pub_crop.publish(ros_cropped)


        mask1(ros_cropped)

        #filters
        hsv_image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        white_filter = cv2.inRange(hsv_image, (0,0,159), (127,76, 255))
        ros_white_filter =self.bridge.cv2_to_imgmsg(white_filter, "mono8")
        self.pub_mask1.publish(ros_white_filter)

        yellow_filter = cv2.inRange(hsv_image, (0,66,92), (72,156,255))
        ros_yellow_filter =self.bridge.cv2_to_imgmsg(yellow_filter, "mono8")
        self.pub_mask2.publish(ros_yellow_filter)
        
        rospy.loginfo("Successfully finish")
    # def mask1(self, msg):
    #     white_filter = cv2.inRange(hsv_image, (0,0,159), (127,76, 255))
    #     ros_white_filter =self.bridge.cv2_to_imgmsg(white_filter, "mono8")
    #     self.pub_mask1.publish(ros_white_filter)




if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_detector", anonymous=True)
    img_detect = LaneDetector()
    rospy.spin()