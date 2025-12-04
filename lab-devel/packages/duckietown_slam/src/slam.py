#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from duckietown_msgs.msg import SegmentList
from geometry_msgs.msg import TransformStamped, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ImprovedSlam:
    def __init__(self):
        # Initialize the node first
        rospy.init_node('slam_node')
        
        # Get namespace
        self.veh = rospy.get_namespace().strip('/')
        rospy.loginfo(f"Starting SLAM node for {self.veh}")
        
        # Wait for TF tree to be available
        rospy.sleep(3.0)  # Initial delay
        
        # Get parameters
        self.map_resolution = rospy.get_param('~map_resolution', 0.05)  # meters/pixel
        self.map_width = int(rospy.get_param('~map_width', 50.0) / self.map_resolution)
        self.map_height = int(rospy.get_param('~map_height', 50.0) / self.map_resolution)
        self.log_odds_occ = rospy.get_param('~log_odds_occ', 0.85)
        self.log_odds_free = rospy.get_param('~log_odds_free', -0.4)
        self.log_odds_max = 100
        self.log_odds_min = -100
        
        # Initialize map as log-odds 
        self.log_odds_map = np.zeros((self.map_height, self.map_width))
        
        # Initialize map message
        self.map = OccupancyGrid()
        self.map.info.resolution = self.map_resolution
        self.map.info.width = self.map_width
        self.map.info.height = self.map_height
        self.map.info.origin.position.x = -self.map_width * self.map_resolution / 2
        self.map.info.origin.position.y = -self.map_height * self.map_resolution / 2
        
        # Robot pose tracking
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update = rospy.Time.now()
        
        # Set up TF buffer with longer duration
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Wait for critical transforms
        try:
            rospy.loginfo("Waiting for TF tree...")
            self.tf_buffer.lookup_transform(
                'map',
                f'{self.veh}/base_link',
                rospy.Time(0),
                rospy.Duration(5.0)
            )
            rospy.loginfo("TF tree is ready!")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
        
        # Publishers
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        self.metadata_pub = rospy.Publisher('map_metadata', MapMetaData, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('lane_detector_node/segment_list', SegmentList, self.segment_callback, queue_size=1)
        
        # Timer for publishing transforms
        rospy.Timer(rospy.Duration(0.1), self.publish_tf)

    def scan_callback(self, msg):
        """Process laser scan data"""
        try:
            # Update pose estimation
            scan_time = msg.header.stamp
            dt = (scan_time - self.last_update).to_sec()
            
            if dt > 0:
                # Get odometry update
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'odom', f'{self.veh}/base_link',
                        scan_time,
                        rospy.Duration(0.1)
                    )
                    
                    self.x = trans.transform.translation.x
                    self.y = trans.transform.translation.y
                    quat = [
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w
                    ]
                    self.theta = euler_from_quaternion(quat)[2]
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                    rospy.logwarn_throttle(1.0, "Could not get transform")
                    return
            
            # Process scan points
            angle = msg.angle_min
            for r in msg.ranges:
                if msg.range_min <= r <= msg.range_max:
                    # Calculate endpoint in map frame
                    x_scan = r * np.cos(angle + self.theta) + self.x
                    y_scan = r * np.sin(angle + self.theta) + self.y
                    
                    # Ray-trace and update map
                    self.update_map_ray(x_scan, y_scan)
                
                angle += msg.angle_increment
                
            self.last_update = scan_time
            self.publish_map()
            
        except Exception as e:
            rospy.logerr(f"Error processing scan: {e}")

    def segment_callback(self, msg):
        """Process lane segments"""
        try:
            for segment in msg.segments:
                # Convert endpoints to map frame
                for i, point in enumerate(segment.points):
                    point_in_map = self.tf_buffer.lookup_transform(
                        'map',
                        f'{self.veh}/camera_optical_frame',
                        msg.header.stamp,
                        rospy.Duration(0.1)
                    )
                    # Update map with transformed point
                    self.update_map_point(
                        point_in_map.transform.translation.x,
                        point_in_map.transform.translation.y
                    )
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Error processing segments: {e}")

    def update_map_ray(self, end_x, end_y):
        """Update map using ray tracing"""
        start_x, start_y = self.world_to_grid(self.x, self.y)
        end_x, end_y = self.world_to_grid(end_x, end_y)
        
        if not self.is_valid_cell(start_x, start_y) or not self.is_valid_cell(end_x, end_y):
            return
            
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        x = start_x
        y = start_y
        
        n = 1 + dx + dy
        x_inc = 1 if end_x > start_x else -1
        y_inc = 1 if end_y > start_y else -1
        error = dx - dy
        dx *= 2
        dy *= 2
        
        for i in range(n):
            if self.is_valid_cell(x, y):
                if x == end_x and y == end_y:
                    # Mark endpoint as occupied
                    self.log_odds_map[y, x] += self.log_odds_occ
                else:
                    # Mark as free space
                    self.log_odds_map[y, x] += self.log_odds_free
                    
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

    def update_map_point(self, x, y):
        """Update single point in map"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if self.is_valid_cell(grid_x, grid_y):
            self.log_odds_map[grid_y, grid_x] += self.log_odds_occ
            self.log_odds_map[grid_y, grid_x] = np.clip(
                self.log_odds_map[grid_y, grid_x],
                self.log_odds_min,
                self.log_odds_max
            )

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x - self.map.info.origin.position.x) / self.map_resolution)
        grid_y = int((y - self.map.info.origin.position.y) / self.map_resolution)
        return grid_x, grid_y

    def is_valid_cell(self, x, y):
        """Check if cell coordinates are valid"""
        return 0 <= x < self.map_width and 0 <= y < self.map_height

    def publish_map(self):
        """Publish current map"""
        # Convert log-odds to probabilities and then to occupancy values
        odds = np.exp(self.log_odds_map)
        probs = odds / (1 + odds)
        self.map.data = np.round(probs * 100).astype(np.int8).flatten().tolist()
        
        self.map.header.stamp = rospy.Time.now()
        self.map.header.frame_id = 'map'
        self.map_pub.publish(self.map)
        self.metadata_pub.publish(self.map.info)

    def publish_tf(self, event=None):
        """Publish transforms"""
        try:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'map'
            t.child_frame_id = f'{self.veh}/odom'
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0
            q = quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            rospy.logwarn(f"Error publishing transforms: {e}")

if __name__ == '__main__':
    try:
        slam = ImprovedSlam()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass