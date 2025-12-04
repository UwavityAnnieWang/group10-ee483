#robot coord to world using odom

#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class Mapper:
    def __init__(self):
        rospy.init_node('simple_mapper', anonymous=True)

        # Robot State (World Frame)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # Yaw in radians

        # Storage for the Global Map (Points)
        self.map_points = [] 

        # Subscribers
        # 1. Odometry: Updates where the robot is
        rospy.Subscriber("/ee483mm10/velocity_controller_node/odom", Odometry, self.odom_callback)
        
        # 2. Metric Lines: The lines we detected relative to the robot
        rospy.Subscriber("/lane_segments_metric", MarkerArray, self.lines_callback)

        # Publisher: Show the Global Map in Rviz
        self.pub_global_map = rospy.Publisher("/global_map_markers", Marker, queue_size=10)

    def odom_callback(self, msg):
        """
        Update the robot's internal belief of its position based on wheel encoders.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract Yaw from Quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.theta = yaw

    def lines_callback(self, msg):
        """
        Take lines seen by robot -> Transform to World -> Add to Map
        """
        for marker in msg.markers:
            # Get start and end points of the line in Robot Frame
            p1_robot = marker.points[0]
            p2_robot = marker.points[1]

            # TRANSFORM: Robot Frame -> World Frame
            # Formulas:
            # x_world = x_robot * cos(theta) - y_robot * sin(theta) + x_robot_global
            # y_world = x_robot * sin(theta) + y_robot * cos(theta) + y_robot_global

            # Point 1
            x1_w = p1_robot.x * np.cos(self.theta) - p1_robot.y * np.sin(self.theta) + self.x
            y1_w = p1_robot.x * np.sin(self.theta) + p1_robot.y * np.cos(self.theta) + self.y

            # Point 2
            x2_w = p2_robot.x * np.cos(self.theta) - p2_robot.y * np.sin(self.theta) + self.x
            y2_w = p2_robot.x * np.sin(self.theta) + p2_robot.y * np.cos(self.theta) + self.y

            # Add to our "Map" (List of points)
            # In a real SLAM system, you would check if these points already exist to avoid duplicates
            self.map_points.append(Point(x1_w, y1_w, 0))
            self.map_points.append(Point(x2_w, y2_w, 0))

        # Publish the updated map
        self.publish_map()

    def publish_map(self):
        m = Marker()
        m.header.frame_id = "map" # World Frame
        m.header.stamp = rospy.Time.now()
        m.id = 0
        m.type = Marker.POINTS # Draw detection as dots
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.color.r = 1.0
        m.color.a = 1.0
        
        # Limit map size for performance (keep last 1000 points)
        m.points = self.map_points[-1000:] 
        
        self.pub_global_map.publish(m)

if __name__ == "__main__":
    try:
        Mapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass