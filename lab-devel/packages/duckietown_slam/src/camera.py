#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from duckietown_msgs.msg import SegmentList, Segment
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
import math

class CameraToLaserBridge:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_laser_bridge')

        # Constants for laser scan conversion
        self.SCAN_ANGLE_MIN = 0  # -90 degrees
        self.SCAN_ANGLE_MAX = 0   # 90 degrees
        self.SCAN_NUM_POINTS = 1      # Number of points in scan
        self.SCAN_RANGE_MIN = 0.1       # Minimum range (meters)
        self.SCAN_RANGE_MAX = 3.5       # Maximum range (meters)

        # Publishers
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.debug_pub = rospy.Publisher('debug/segments', MarkerArray, queue_size=1)
        
        # Subscribe to the ground-projected line segments
        # self.segment_sub = rospy.Subscriber('ground_projection_node/lineseglist_out', 
        #                                   SegmentList, 
        #                                   self.segment_callback)
        
        self.range_sub = rospy.Subscriber('/casagoesdb04/front_center_tof_driver_node/range', 
                                          Range, 
                                          self.ToF_callback)
        
        rospy.loginfo("CameraToLaserBridge initialized")

    # def publish_debug_markers(self, segment_list):
    #     marker_array = MarkerArray()
        
    #     for i, segment in enumerate(segment_list.segments):
    #         marker = Marker()
    #         marker.header = segment_list.header
    #         marker.ns = "segments"
    #         marker.id = i
    #         marker.type = Marker.LINE_STRIP
    #         marker.action = Marker.ADD
    #         marker.scale.x = 0.01  # Line width
    #         marker.color.a = 1.0
            
    #         # Color based on segment color
    #         if segment.color == segment.WHITE:
    #             marker.color.r = marker.color.g = marker.color.b = 1.0
    #         elif segment.color == segment.YELLOW:
    #             marker.color.r = marker.color.g = 1.0
    #             marker.color.b = 0.0
    #         elif segment.color == segment.RED:
    #             marker.color.r = 1.0
    #             marker.color.g = marker.color.b = 0.0
                
    #         marker.points = [segment.points[0], segment.points[1]]
    #         marker_array.markers.append(marker)
            
    #     self.debug_pub.publish(marker_array)
    def publish_debug_markers(self, range_msg):
        marker_array = MarkerArray()
        
        # Create a marker for the ToF reading
        marker = Marker()
        current_time = rospy.Time.now()
        marker.header.stamp = current_time
        marker.ns = "tof_reading"
        marker.id = 0
        marker.type = Marker.ARROW  # Using arrow to show direction and distance
        marker.action = Marker.ADD
        marker.scale.x = range_msg.range  # Length of arrow = range reading
        marker.scale.y = 0.05  # Width of arrow
        marker.scale.z = 0.05  # Height of arrow
        marker.color.a = 1.0
        marker.color.r = 1.0  # Red color
        
        # # Color based on segment color
        # if segment.color == segment.WHITE:
        #     marker.color.r = marker.color.g = marker.color.b = 1.0
        # elif segment.color == segment.YELLOW:
        #     marker.color.r = marker.color.g = 1.0
        #     marker.color.b = 0.0
        # elif segment.color == segment.RED:
        #     marker.color.r = 1.0
        #     marker.color.g = marker.color.b = 0.0
            
        # marker.points = [segment.points[0], segment.points[1]]
        # marker_array.markers.append(marker)
            
        # self.debug_pub.publish(marker_array)
         # Start point at sensor
        marker.points.append(Point(0, 0, 0))
        # End point at detection
        marker.points.append(Point(range_msg.range, 0, 0))
            
        marker_array.markers.append(marker)
        self.debug_pub.publish(marker_array)

    def segment_to_range_bearing(self, point):
        """Convert a point to range and bearing"""
        range = math.sqrt(point.x**2 + point.y**2)
        bearing = math.atan2(point.y, point.x)
        return range, bearing

    # def create_scan_message(self, segment_list):
    #     """Create a LaserScan message from segments"""
    #     scan_msg = LaserScan()
    #     scan_msg.header = segment_list.header
    #     scan_msg.header.frame_id = "laser"
        
    #     scan_msg.angle_min = self.SCAN_ANGLE_MIN
    #     scan_msg.angle_max = self.SCAN_ANGLE_MAX
    #     scan_msg.angle_increment = (self.SCAN_ANGLE_MAX - self.SCAN_ANGLE_MIN) / self.SCAN_NUM_POINTS
    #     scan_msg.time_increment = 0.0
    #     scan_msg.scan_time = 0.1
    #     scan_msg.range_min = self.SCAN_RANGE_MIN
    #     scan_msg.range_max = self.SCAN_RANGE_MAX

    #     # Initialize ranges array with max range
    #     ranges = [float('inf')] * self.SCAN_NUM_POINTS

    #     # Process each line segment
    #     for segment in segment_list.segments:
    #         # Get endpoints
    #         p1, p2 = segment.points
            
    #         # Convert both endpoints to range and bearing
    #         r1, b1 = self.segment_to_range_bearing(p1)
    #         r2, b2 = self.segment_to_range_bearing(p2)
            
    #         # Skip if endpoints are out of range
    #         if (r1 < self.SCAN_RANGE_MIN or r1 > self.SCAN_RANGE_MAX or
    #             r2 < self.SCAN_RANGE_MIN or r2 > self.SCAN_RANGE_MAX):
    #             continue
                
    #         # Skip if endpoints are out of angle range
    #         if (b1 < self.SCAN_ANGLE_MIN or b1 > self.SCAN_ANGLE_MAX or
    #             b2 < self.SCAN_ANGLE_MIN or b2 > self.SCAN_ANGLE_MAX):
    #             continue
            
    #         # Interpolate points along the line segment
    #         num_points = max(int(abs(b2 - b1) / scan_msg.angle_increment), 2)
    #         for i in range(num_points):
    #             # Interpolate position
    #             t = float(i) / (num_points - 1)
    #             x = p1.x + t * (p2.x - p1.x)
    #             y = p1.y + t * (p2.y - p1.y)
                
    #             # Convert to range and bearing
    #             r = math.sqrt(x*x + y*y)
    #             b = math.atan2(y, x)
                
    #             # Convert bearing to array index
    #             idx = int((b - self.SCAN_ANGLE_MIN) / scan_msg.angle_increment)
    #             if 0 <= idx < self.SCAN_NUM_POINTS:
    #                 # Keep the closest range for this angle
    #                 ranges[idx] = min(ranges[idx], r)

    #     # scan_msg.ranges = ranges
    #     scan_msg.ranges = self.range_sub
    #     return scan_msg
    def create_scan_message(self, range_msg):
        """Create a LaserScan message from ToF reading"""
        scan_msg = LaserScan()
        scan_msg.header = range_msg.header
        scan_msg.header.frame_id = "base_link"
        
        scan_msg.angle_min = self.SCAN_ANGLE_MIN
        scan_msg.angle_max = self.SCAN_ANGLE_MAX
        # scan_msg.angle_increment = (self.SCAN_ANGLE_MAX - self.SCAN_ANGLE_MIN) / self.SCAN_NUM_POINTS
        scan_msg.angle_increment = 0
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.06
        scan_msg.range_min = range_msg.min_range
        scan_msg.range_max = range_msg.max_range

        # Initialize all ranges to infinity
        ranges = [float('inf')] * self.SCAN_NUM_POINTS
        
        # Calculate central index where ToF is pointing
        center_idx = self.SCAN_NUM_POINTS // 2
        
        # Calculate how many points to fill based on field of view
        points_in_fov = int(range_msg.field_of_view / scan_msg.angle_increment)
        start_idx = center_idx - (points_in_fov // 2)
        end_idx = center_idx + (points_in_fov // 2)
        
        # Fill in range reading for points within field of view
        for i in range(start_idx, end_idx + 1):
            if 0 <= i < self.SCAN_NUM_POINTS:  # Ensure within bounds
                ranges[i] = range_msg.range

        scan_msg.ranges = ranges
        return scan_msg

    def segment_callback(self, segment_list):
        """Process incoming segment list and convert to laser scan"""
        rospy.loginfo(f"Received segment list with {len(segment_list.segments)} segments")
        
        if len(segment_list.segments) == 0:
            rospy.logwarn("Received empty segment list")
            return

        # Publish debug visualization
        self.publish_debug_markers(segment_list)
        
        # Create and publish laser scan
        scan_msg = self.create_scan_message(segment_list)
        self.scan_pub.publish(scan_msg)
        rospy.loginfo(f"Published scan with {len(scan_msg.ranges)} points")
    
    # def ToF_callback(self, range_msg):
    #     """Process incoming segment list and convert to laser scan"""
    #     rospy.loginfo(f"Received ToF reading: {range_msg.range} meters")
        
    #     if len(range_msg) == 0:
    #         rospy.logwarn("Received empty segment list")
    #         return

    #     # Publish debug visualization
    #     self.publish_debug_markers(range_msg)
        
    #     # Create and publish laser scan
    #     scan_msg = self.create_scan_message(range_msg)
    #     self.scan_pub.publish(scan_msg)
    #     rospy.loginfo(f"Published scan with {len(scan_msg.ranges)} points")
    def ToF_callback(self, range_msg):
        """Process incoming ToF range message and convert to laser scan"""
        rospy.loginfo(f"Received ToF reading: {range_msg.range} meters")
        
        # Skip if invalid reading
        if range_msg.range < range_msg.min_range or range_msg.range > range_msg.max_range:
            rospy.logwarn("ToF reading out of range bounds")
            return

        # Publish debug visualization
        # self.publish_debug_markers(range_msg)
        
        # Create and publish laser scan
        # scan_msg = self.create_scan_message(range_msg)
        # self.scan_pub.publish(scan_msg)

        scan_msg = LaserScan()
        # scan_msg.header = range_msg.header
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "base_link"
        
        scan_msg.angle_min = 0
        scan_msg.angle_max = 0.01
        # scan_msg.angle_increment = (self.SCAN_ANGLE_MAX - self.SCAN_ANGLE_MIN) / self.SCAN_NUM_POINTS
        scan_msg.angle_increment = 0.01
        scan_msg.time_increment = 0.06
        scan_msg.scan_time = 0.06
        scan_msg.range_min = range_msg.min_range
        scan_msg.range_max = range_msg.max_range
        ranges = [range_msg.range, range_msg.range]
        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)
        rospy.loginfo("Published scan message from ToF data")

if __name__ == '__main__':
    try:
        bridge = CameraToLaserBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass