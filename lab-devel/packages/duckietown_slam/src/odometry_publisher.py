#!/usr/bin/env python3

import rospy
import tf2_ros
import math
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf_conversions import transformations
from duckietown_msgs.msg import WheelEncoderStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from duckietown_msgs.msg import WheelEncoderStamped

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')
        
        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        
        # Publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        
        # Parameters
        self.wheel_radius = 0.0318  # meters
        self.wheel_separation = 0.1  # meters
        self.ticks_per_revolution = 135  # adjust based on encoder specs
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()
        self.seq = 0
        
        # Subscribers
        # rospy.Subscriber('/casagoesdb04/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_tick_callback)
        # rospy.Subscriber('/casagoesdb04/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_tick_callback)
        sync = ApproximateTimeSynchronizer([Subscriber('/casagoesdb04/left_wheel_encoder_node/tick', WheelEncoderStamped), 
                                                            Subscriber('/casagoesdb04/right_wheel_encoder_node/tick', WheelEncoderStamped)], 
                                                            5, 0.1)
        sync.registerCallback(self.process_ticks)
        rospy.loginfo("Odometry publisher initialized")

    def left_tick_callback(self, msg):
        self.process_ticks(msg.data, self.last_right_ticks)
        self.last_left_ticks = msg.data

    def right_tick_callback(self, msg):
        self.process_ticks(self.last_left_ticks, msg.data)
        self.last_right_ticks = msg.data

    def process_ticks(self, left_ticks, right_ticks):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt == 0:
            return
        
        # Calculate wheel movements
        left_diff = (left_ticks.data - self.last_left_ticks) * 2 * math.pi * self.wheel_radius / self.ticks_per_revolution
        right_diff = (right_ticks.data - self.last_right_ticks) * 2 * math.pi * self.wheel_radius / self.ticks_per_revolution
        
        self.last_left_ticks = left_ticks.data
        self.last_right_ticks = right_ticks.data
        # Calculate robot movement
        distance = (left_diff + right_diff) / 2.0
        rotation = (right_diff - left_diff) / self.wheel_separation
        
        # Update pose
        if abs(rotation) < 0.0001:
            # Straight line approximation
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
        else:
            # Arc approximation
            radius = distance / rotation
            self.x += radius * (math.sin(self.theta + rotation) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + rotation) - math.cos(self.theta))
        self.theta += rotation
        
        # Create and publish transform
        # t = TransformStamped()
        # t.header.stamp = current_time
        # t.header.seq = self.seq
        # t.header.frame_id = "odom"
        # t.child_frame_id = "base_link"
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # q = transformations.quaternion_from_euler(0, 0, self.theta)
        # t.transform.rotation = Quaternion(*q)
        # self.br.sendTransform(t)
        
        # # Create and publish odometry
        # odom = Odometry()
        # odom.header.stamp = current_time
        # odom.header.frame_id = "odom"
        # odom.child_frame_id = "base_link"
        # odom.pose.pose.position.x = self.x
        # odom.pose.pose.position.y = self.y
        # odom.pose.pose.position.z = 0.0
        # odom.pose.pose.orientation = t.transform.rotation
        
        # Add twist information
        # odom.twist.twist.linear.x = distance / dt
        # odom.twist.twist.angular.z = rotation / dt
        
        # self.odom_pub.publish(odom)
        self.last_time = current_time
        self.sending_Tf()
    def sending_Tf(self):
        current_time = rospy.Time.now()
        # Create and publish transform
        t = TransformStamped()
        # t.header = left_ticks.header
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        # t.header.seq = self.seq
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation = Quaternion(*q)
        self.br.sendTransform(t)
        rospy.sleep(0.03)
        # self.seq += 1

if __name__ == '__main__':
    try:
        odom_publisher = OdometryPublisher()
        # while not rospy.is_shutdown():
            # odom_publisher.sending_Tf()
            # rospy.sleep(0.01)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass