#!/usr/bin/env python3

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LaneMonitor:
    def __init__(self):
        self.veh = os.environ['VEHICLE_NAME']
        self.bridge = CvBridge()
        
        # Subscribe to lane detection output
        self.lane_sub = rospy.Subscriber(
            f'/{self.veh}/lane_detector_node/debug/lines',
            Image,
            self.lane_callback,
            queue_size=1
        )
        
        rospy.loginfo("Lane monitor initialized")
        
    def lane_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo(f"Received lane image: shape={cv_image.shape}, non-zero pixels={cv_image.any()}")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

if __name__ == '__main__':
    rospy.init_node('lane_monitor')
    monitor = LaneMonitor()
    rospy.spin()