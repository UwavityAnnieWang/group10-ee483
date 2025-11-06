#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose 

import os
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Filter:
    def __init__(self):
        rospy.Subscriber("/ee483mm10/lane_filter_node/lane_pose", LanePose, self.callback, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
        self.movingavg=rospy.Publisher("/moving_avg", Float32, queue_size=10)
        self.avgArray = [0,0,0]
        print("here")

    def callback(self, msg):
        i = 0
        while i < len(self.avgArray):
            if i != len(self.avgArray)-1:
                self.avgArray[i] = self.avgArray[i+1]
            elif i == len(self.avgArray)-1:
                self.avgArray[i] = msg.phi
            i += 1
        phiAverage = sum(self.avgArray)/len(self.avgArray)
        self.movingavg.publish(phiAverage)

if __name__=="__main__":
	# initialize our node and create a publisher as normal
	rospy.init_node("average_phi", anonymous=True)
	average = Filter()
	rospy.spin()



