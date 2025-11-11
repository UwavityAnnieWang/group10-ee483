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

class Filter:
    def __init__(self):
        rospy.Subscriber("/ee483mm10/lane_filter_node/lane_pose", LanePose, self.callback, queue_size=1, buff_size=10000000) #BUFF SIZE 10MB
        self.movingavg=rospy.Publisher("/moving_avg", Float32, queue_size=10)
        self.controlinput=rospy.Publisher("/ee483mm10/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.avgArray = [0,0,0]
        self.erroray=[]

        self.kp= rospy.get_param("Kp")
        self.ki= rospy.get_param("Ki")
        self.kd= rospy.get_param("Kd")

        self.e_prev = 0.0
        self.i_term = 0.0
        self.t_prev = rospy.Time.now()

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

        error = -phiAverage
        now = rospy.Time.now()
        dt = (now - self.t_prev).to_sec()

        self.i_term += error*dt

        d_term = (error - self.e_prev)/dt

        control = self.kp * error + self.ki * self.i_term + self.kd * d_term

        '''
        timestart=rospy.get_time()
        for j in range(10000):
            self.erroray.append(phiAverage)
        timestop=rospy.get_time()
        inte=(sum(self.erroray)+self.erroray[(len(self.erroray)-1)])*(timestop-timestart)
        dx=self.erroray[(len(self.erroray)-1)]-self.erroray[(len(self.erroray)-2)]
        kp= rospy.get_param("Kp")
        ki= rospy.get_param("Ki")
        kd= rospy.get_param("Kd")
        control=(kp*self.erroray[(len(self.erroray)-1)])+(ki*inte)+(kd*dx)
        print(control)
        '''

        self.e_prev = error
        self.t_prev = now

        car_cmd = Twist2DStamped()
        car_cmd.v = 0
        car_cmd.omega = control
        self.controlinput.publish(car_cmd)


if __name__=="__main__":
	# initialize our node and create a publisher as normal
	rospy.init_node("average_phi", anonymous=True)
	average = Filter()
	rospy.spin()



