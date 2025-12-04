#!/usr/bin/env python3

import os
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from duckietown_msgs.msg import WheelEncoderStamped  # Try this message type instead

class WheelOdomToPoseConverter:
    def __init__(self):
        rospy.loginfo("Starting wheel odometry converter...")
        self.veh = os.environ['VEHICLE_NAME']
        rospy.loginfo(f"Vehicle name: {self.veh}")
        
        # Robot parameters
        self.wheel_radius = 0.0318  # meters
        self.wheel_base = 0.1  # meters
        self.ticks_per_revolution = 135
        
        # Current pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous tick counts
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.first_message_received = False
        
        # Publisher for pose
        self.pose_pub = rospy.Publisher(
            f'/{self.veh}/pose',
            Pose,
            queue_size=1
        )
        
        # Individual callbacks for better debugging
        self.left_sub = rospy.Subscriber(
            f'/{self.veh}/left_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.left_wheel_callback,
            queue_size=1
        )
        
        self.right_sub = rospy.Subscriber(
            f'/{self.veh}/right_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.right_wheel_callback,
            queue_size=1
        )
        
        rospy.loginfo("Subscribers and publisher initialized")
        
        # Store latest wheel readings
        self.latest_left = None
        self.latest_right = None

    def left_wheel_callback(self, msg):
        rospy.loginfo("Left wheel message received: %s", str(msg))
        self.latest_left = msg.data
        self.update_pose()

    def right_wheel_callback(self, msg):
        rospy.loginfo("Right wheel message received: %s", str(msg))
        self.latest_right = msg.data
        self.update_pose()

    def update_pose(self):
        if self.latest_left is None or self.latest_right is None:
            return

        # Get current tick counts
        left_ticks = self.latest_left
        right_ticks = self.latest_right
        
        # Initialize previous ticks if not set
        if not self.first_message_received:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.first_message_received = True
            rospy.loginfo(f"Initial ticks set - Left: {left_ticks}, Right: {right_ticks}")
            return
        
        # Calculate change in ticks
        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks
        
        rospy.loginfo(f"Delta ticks - Left: {delta_left_ticks}, Right: {delta_right_ticks}")
        
        # Convert ticks to distance
        meters_per_tick = (2 * np.pi * self.wheel_radius) / self.ticks_per_revolution
        delta_left_dist = delta_left_ticks * meters_per_tick
        delta_right_dist = delta_right_ticks * meters_per_tick
        
        # Calculate average distance and change in orientation
        delta_distance = (delta_left_dist + delta_right_dist) / 2.0
        delta_theta = (delta_right_dist - delta_left_dist) / self.wheel_base
        
        # Update pose
        avg_theta = self.theta + delta_theta / 2.0
        self.x += delta_distance * np.cos(avg_theta)
        self.y += delta_distance * np.sin(avg_theta)
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
        # Create and publish pose message
        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0
        
        # Convert theta to quaternion
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = np.sin(self.theta / 2.0)
        pose_msg.orientation.w = np.cos(self.theta / 2.0)
        
        self.pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published pose - x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f}")
        
        # Update previous tick counts
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

if __name__ == '__main__':
    rospy.init_node('wheel_odom_to_pose_converter')
    converter = WheelOdomToPoseConverter()
    rospy.loginfo("Wheel odometry converter node started")
    rospy.spin()