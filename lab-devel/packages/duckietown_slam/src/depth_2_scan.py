#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import sin, cos, pi

class DepthToScan:
    def __init__(self):
        # Get namespace for proper topic naming
        self.veh = rospy.get_namespace().strip('/')
        
        # Get parameters
        self.scan_fov = rospy.get_param('~scan_fov', 0.4)  # Field of view in radians
        self.scan_resolution = rospy.get_param('~scan_resolution', 0.01)  # Angular resolution
        self.min_range = rospy.get_param('~min_range', 0.02)  # Minimum range
        self.max_range = rospy.get_param('~max_range', 1.0)  # Maximum range
        
        # Subscribe to depth topic with proper namespace
        self.depth_sub = rospy.Subscriber(
            f'front_center_tof_driver_node/range', 
            Float32, 
            self.depth_cb,
            queue_size=1
        )
        
        # Create laser scan publisher with proper namespace
        self.scan_pub = rospy.Publisher(
            'depth_to_scan_node/scan',
            LaserScan,
            queue_size=1
        )
        
        # Initialize scan message
        self.scan_msg = LaserScan()
        self.scan_msg.angle_min = -self.scan_fov/2
        self.scan_msg.angle_max = self.scan_fov/2
        self.scan_msg.angle_increment = self.scan_resolution
        self.scan_msg.time_increment = 0.0
        self.scan_msg.scan_time = 0.1
        self.scan_msg.range_min = self.min_range
        self.scan_msg.range_max = self.max_range
        self.scan_msg.ranges = []
        self.scan_msg.intensities = []
        self.scan_msg.header.frame_id = 'front_center_tof_frame'

    def depth_cb(self, msg):
        """Convert depth measurement to laser scan"""
        depth = msg.data
        
        if depth < self.min_range or depth > self.max_range:
            rospy.logwarn_throttle(1, f"Depth {depth}m out of range [{self.min_range}, {self.max_range}]m")
            return
            
        # Generate scan angles
        num_points = int((self.scan_msg.angle_max - self.scan_msg.angle_min) / self.scan_msg.angle_increment)
        ranges = [depth] * num_points  # Single depth value spread across FOV
        
        # Populate and publish scan message
        self.scan_msg.header.stamp = rospy.Time.now()
        self.scan_msg.ranges = ranges
        self.scan_pub.publish(self.scan_msg)

if __name__ == '__main__':
    rospy.init_node('depth_to_scan')
    depth_to_scan = DepthToScan()
    rospy.spin()