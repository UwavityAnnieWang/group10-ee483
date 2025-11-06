#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import Twist2DStamped

import os
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Controller:
    def __init__(self):
        rospy.Subscriber("/moving_avg", Float32, self.callback, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
        self.movingavg=rospy.Publisher("ee483mm10/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

    def callback(self, msg):
        self.error = msg.data
        self.kp = 1
        self.kd = .05
        self.ki = .02
        #pid_static(error)
        #pid_dynamic(error)
    
    def pid_static(self, msg):

    def pid_dynamic(self, msg):

if __name__=="__main__":
	# initialize our node and create a publisher as normal
	rospy.init_node("pid_control", anonymous=True)
	pid = Controller()
	rospy.spin()