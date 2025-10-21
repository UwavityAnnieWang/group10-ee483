#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import os
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

class LaneDetector:
	def __init__(self):
		# Instatiate the converter class once by using a class member
		self.bridge = CvBridge()
		#rospy.Subscriber("image", Image, self.flipper_cb)
		rospy.Subscriber("/ee483mm10/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
		self.pub_crop = rospy.Publisher("/cropped", Image, queue_size=10)
		self.pub_mask1 = rospy.Publisher("/mask1", Image, queue_size=10)
		self.pub_mask2 = rospy.Publisher("/mask2", Image, queue_size=10)
		self.pub_detection = rospy.Publisher("/detection", Image, queue_size=10)
		""" self.sub_crop=Subscriber("/cropped", Image)
		self.sub_detection=Subscriber("/detection", Image)
		self.ats=ApproximateTimeSynchronizer([self.sub_crop, self.sub_detection], queue_size=5, slop=0.1)
		self.ats.registerCallback(self.callback) """
	def output_lines(self, original_image, lines, color):
		output = np.copy(original_image)
		if lines is not None:
			for i in range(len(lines)):
				l = lines[i][0]
				if color is True:

					cv2.line(output, (l[0],l[1]),(l[2],l[3]), (0,255,0), 2, cv2.LINE_AA)
				else:
					cv2.line(output, (l[0],l[1]),(l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
				cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
			
				cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
		return output
	def callback(self, msg):
		# convert to a ROS image using the bridge
		#rospy.loginfo("Successfully run")
		cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
		#crop
		height = cv_img.shape[0]
		cropped_img = cv_img[height//3:height, :]
		ros_cropped = self.bridge.cv2_to_imgmsg(cropped_img, 'bgr8')# convert to a ROS image using the bridge

		gray_cropped=cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
		blur=cv2.GaussianBlur(gray_cropped, (7,7), 0)
		canny_img=cv2.Canny(blur, 20, 80)
		kernel1=cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
		canny_img=cv2.dilate(canny_img, kernel1)

		#filters
		hsv_image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

		
		#erosion kernel 
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		
		#Original values for white 
		#(0,0,159)(127,76,255)
		white_filter = cv2.inRange(hsv_image, (0,0,180), (180,60, 255))
		yellow_filter = cv2.inRange(hsv_image, (0,66,92), (72,156,255))
		
		#applying erosion to the yellow and white masks 
		image_erode1=cv2.erode(white_filter, kernel)
		image_erode2=cv2.erode(yellow_filter, kernel)

		combined1=cv2.bitwise_and(canny_img, canny_img, mask=image_erode1)
		combined2=cv2.bitwise_and(canny_img, canny_img, mask=image_erode2)

		lines1=cv2.HoughLinesP(combined1, 1, np.pi/180, 40, minLineLength=20, maxLineGap=5)
		lines2=cv2.HoughLinesP(combined2, 1, np.pi/180, 40, minLineLength=20, maxLineGap=5)

		

		image_with_lines1=self.output_lines(cropped_img, lines1, True)
		image_with_lines2=self.output_lines(cropped_img, lines2, False)
		image_with_lines3=cv2.bitwise_or(image_with_lines1, image_with_lines2)
		combined3=self.bridge.cv2_to_imgmsg(image_with_lines3, "bgr8")
		self.pub_detection.publish(combined3)




		
		
		self.pub_crop.publish(ros_cropped)



		
		ros_yellow_filter=self.bridge.cv2_to_imgmsg(image_erode2, "mono8")
		ros_white_filter=self.bridge.cv2_to_imgmsg(image_erode1, "mono8")


		#ros_yellow_filter =self.bridge.cv2_to_imgmsg(yellow_filter, "mono8")
		#ros_white_filter =self.bridge.cv2_to_imgmsg(white_filter, "mono8")
		
		#publishing the masks 
		self.pub_mask2.publish(ros_yellow_filter)
		self.pub_mask1.publish(ros_white_filter)

		#rospy.loginfo("Successfully finish")
   
		self.pub_crop.publish(ros_cropped)



		#filters
		hsv_image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

		
		#erosion kernel 
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		
		#Original values for white 
		#(0,0,159)(127,76,255)
		white_filter = cv2.inRange(hsv_image, (0,0,180), (180,60, 255))
		yellow_filter = cv2.inRange(hsv_image, (0,66,92), (72,156,255))
		
		#applying erosion to the yellow and white masks 
		image_erode1=cv2.erode(white_filter, kernel)
		image_erode2=cv2.erode(yellow_filter, kernel)
		
		ros_yellow_filter=self.bridge.cv2_to_imgmsg(image_erode2, "mono8")
		ros_white_filter=self.bridge.cv2_to_imgmsg(image_erode1, "mono8")


		#ros_yellow_filter =self.bridge.cv2_to_imgmsg(yellow_filter, "mono8")
		#ros_white_filter =self.bridge.cv2_to_imgmsg(white_filter, "mono8")
		
		#publishing the masks 
		self.pub_mask2.publish(ros_yellow_filter)
		self.pub_mask1.publish(ros_white_filter)

		#rospy.loginfo("Successfully finish")
   
	

if __name__=="__main__":
	# initialize our node and create a publisher as normal
	rospy.init_node("image_detector", anonymous=True)
	img_detect = LaneDetector()
	rospy.spin()