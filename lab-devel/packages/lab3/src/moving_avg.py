#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LanePose 

import os
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
class Filter:
    def __init__(self):
        rospy.Subscriber("/ee483mm10/lane_filter_node/lane_pose", LanePose, self.callback, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
        self.movingavg=rospy.Publisher("/moving_avg", LanePose, queue_size=10)
        avgArray = [0,0,0,0,0]
    def callback(self, msg, avgArray):
        for i in avgArray:
            if i != range(avgArray):
                avgArray[i] = avgArray[i+1]
            elif i = range(avgArray):
                avgArray[i] = msg.phi
        phiAverage = sum(avgArray)/len(avgArray)




