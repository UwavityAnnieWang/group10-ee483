#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import Image
from duckietown_msgs.msg import SegmentList
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import tf2_ros

class MapVisualizer:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Get namespace
        self.veh = rospy.get_namespace().strip('/')
        
        # Parameters
        self.visualization_rate = rospy.get_param('~visualization_rate', 2.0)  # Hz
        self.show_lanes = rospy.get_param('~show_lanes', True)
        self.show_trajectory = rospy.get_param('~show_trajectory', True)
        self.tf_buffer_time = rospy.get_param('~tf_buffer_time', 2.0)  # seconds
        
        # Initialize TF buffer with longer cache time
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(self.tf_buffer_time))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Store map and visualization data
        self.current_map = None
        self.current_lanes = []
        self.robot_trajectory = []
        self.last_update = rospy.Time.now()
        
        # Initialize transforms with None
        self.map_to_odom = None
        self.odom_to_base = None
        self.base_to_camera = None
        
        # Wait for transforms
        rospy.sleep(1.0)  # Wait for TF tree to be populated
        self.cache_static_transforms()
        
        # Set up publishers and subscribers
        self.setup_publishers()
        self.setup_subscribers()
        
        # Timer for visualization updates
        rospy.Timer(rospy.Duration(1.0/self.visualization_rate), self.visualization_timer_cb)

    def setup_publishers(self):
        """Set up all publishers"""
        # Main visualization publishers
        self.map_image_pub = rospy.Publisher(
            'map_visualizer_node/map_image/compressed',
            Image,
            queue_size=1
        )
        
        # Marker publishers for RVIZ
        self.lane_markers_pub = rospy.Publisher(
            'map_visualizer_node/lane_markers',
            MarkerArray,
            queue_size=1
        )
        
        self.trajectory_pub = rospy.Publisher(
            'map_visualizer_node/robot_trajectory',
            Marker,
            queue_size=1
        )

    def setup_subscribers(self):
        """Set up all subscribers"""
        # Subscribe to map and lanes
        rospy.Subscriber(
            'map',
            OccupancyGrid,
            self.map_cb,
            queue_size=1
        )
        
        if self.show_lanes:
            rospy.Subscriber(
                'lane_detector_node/segment_list',
                SegmentList,
                self.lanes_cb,
                queue_size=1
            )
            
        # Subscribe to map metadata for updates
        rospy.Subscriber(
            'map_metadata',
            MapMetaData,
            self.metadata_cb,
            queue_size=1
        )

    def cache_static_transforms(self):
        """Cache static transforms to reduce TF lookups"""
        try:
            self.base_to_camera = self.tf_buffer.lookup_transform(
                f'{self.veh}/base_link',
                f'{self.veh}/camera_optical_frame',
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not cache transforms: {e}")

    def transform_to_map(self, point, from_frame, time):
        """Transform a point to map frame with better timing handling"""
        try:
            # First transform to base_link using cached transform
            if from_frame == f'{self.veh}/camera_optical_frame':
                point_in_base = self.transform_point(point, self.base_to_camera)
                from_frame = f'{self.veh}/base_link'
            else:
                point_in_base = point

            # Then transform to map
            transform = self.tf_buffer.lookup_transform(
                'map',
                from_frame,
                time,
                rospy.Duration(0.1)
            )
            
            return self.transform_point(point_in_base, transform)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1, f"Transform failed: {e}")
            return None

    def transform_point(self, point, transform):
        """Apply transform to point"""
        p_out = Point()
        p_out.x = (point.x * transform.transform.rotation.w + 
                   transform.transform.translation.x)
        p_out.y = (point.y * transform.transform.rotation.w + 
                   transform.transform.translation.y)
        p_out.z = 0
        return p_out

    def map_cb(self, msg):
        """Process incoming occupancy grid map"""
        try:
            # Convert occupancy grid to image
            self.current_map = self.occupancy_grid_to_image(msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing map: {e}")

    def lanes_cb(self, msg):
        """Process incoming lane segments"""
        try:
            # Convert lane segments to map frame
            map_lanes = []
            for segment in msg.segments:
                p1 = self.transform_to_map(segment.pixels_normalized[0], 
                                         f'{self.veh}/camera_optical_frame',
                                         msg.header.stamp)
                p2 = self.transform_to_map(segment.pixels_normalized[1], 
                                         f'{self.veh}/camera_optical_frame',
                                         msg.header.stamp)
                if p1 is not None and p2 is not None:
                    map_lanes.append((p1, p2, segment.color))
            
            self.current_lanes = map_lanes
            
        except Exception as e:
            rospy.logerr(f"Error processing lanes: {e}")

    def metadata_cb(self, msg):
        """Handle map metadata updates"""
        # Update visualization parameters based on metadata
        self.map_resolution = msg.resolution
        self.map_origin = msg.origin

    def occupancy_grid_to_image(self, grid_msg):
        """Convert OccupancyGrid message to CV image"""
        # Convert occupancy grid data to numpy array
        grid_data = np.array(grid_msg.data, dtype=np.int8)
        grid_data = grid_data.reshape(
            (grid_msg.info.height, grid_msg.info.width)
        )
        
        # Convert to visualization image (grayscale)
        image = np.zeros_like(grid_data, dtype=np.uint8)
        image[grid_data == 0] = 255    # Free space = white
        image[grid_data == 100] = 0    # Occupied = black
        image[grid_data == -1] = 128   # Unknown = gray
        
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    def visualization_timer_cb(self, event):
        """Timer callback to update visualization"""
        if self.current_map is None:
            return
            
        try:
            # Start with current map
            vis_image = self.current_map.copy()
            
            # Add lanes if enabled
            if self.show_lanes and self.current_lanes:
                for start, end, color in self.current_lanes:
                    start_px = self.world_to_image(start.x, start.y)
                    end_px = self.world_to_image(end.x, end.y)
                    if start_px is not None and end_px is not None:
                        cv2.line(vis_image, start_px, end_px, (0, 255, 0), 2)
            
            # Publish visualization image
            msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            msg.header.stamp = rospy.Time.now()
            self.map_image_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Error in visualization: {e}")

    def world_to_image(self, x, y):
        """Convert world coordinates to image coordinates"""
        if not hasattr(self, 'map_resolution') or not hasattr(self, 'map_origin'):
            return None
            
        try:
            # Convert world coordinates to pixel coordinates
            px = int((x - self.map_origin.position.x) / self.map_resolution)
            py = int((y - self.map_origin.position.y) / self.map_resolution)
            return (px, py)
        except:
            return None

if __name__ == '__main__':
    rospy.init_node('map_visualizer')
    visualizer = MapVisualizer()
    rospy.spin()