#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point as PointMsg
from sensor_msgs.msg import CameraInfo, CompressedImage

class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_message(msg):
        return Point(msg.x, msg.y, msg.z)

class GroundProjectionGeometry:
    def __init__(self, im_width, im_height, homography):
        self.im_width = im_width
        self.im_height = im_height
        self.H = homography

    def pixel2ground(self, pixel):
        # Convert pixel to homogeneous coordinates
        u = pixel.x * self.im_width
        v = pixel.y * self.im_height
        uv_homogeneous = np.array([u, v, 1])

        # Apply homography
        ground_homogeneous = np.dot(self.H, uv_homogeneous)
        
        # Convert back from homogeneous coordinates
        x = ground_homogeneous[0] / ground_homogeneous[2]
        y = ground_homogeneous[1] / ground_homogeneous[2]
        
        return Point(x, y, 0.0)

    def vector2pixel(self, vector):
        # Normalize coordinates
        return Point(
            vector.x * self.im_width,
            vector.y * self.im_height,
            0.0
        )

class GroundProjectionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ground_projection_node')
        rospy.loginfo("Initializing ground projection node...")

        self.bridge = CvBridge()
        self.camera_info_received = False
        self.first_processing_done = False

        # Define a more realistic homography matrix for Duckiebot
        # This matrix transforms from image coordinates to ground coordinates
        # These values should work better for the typical Duckiebot camera mounting
        self.homography = np.array([
            [1.6, -0.3, 0.1],    # Increased x scaling for better spread
            [0.1, 1.5, -0.4],    # Adjusted y scaling and added offset
            [0.0, -0.007, 1.0]   # Added slight perspective effect
        ], dtype=np.float32)

        # Publishers
        self.pub_lineseglist = rospy.Publisher('~lineseglist_out', 
                                             SegmentList, 
                                             queue_size=1)
        self.pub_debug_img = rospy.Publisher('~debug/ground_projection_image/compressed',
                                           CompressedImage,
                                           queue_size=1)

        # Subscribers
        self.sub_camera_info = rospy.Subscriber("/casagoesdb04/camera_node/camera_info", 
                                              CameraInfo, 
                                              self.camera_info_callback, 
                                              queue_size=1)
        self.sub_lineseglist = rospy.Subscriber("/line_detector_node/segment_list", 
                                              SegmentList,
                                              self.lineseglist_callback, 
                                              queue_size=1)

        self.debug_image_bg = None
        
        rospy.loginfo("Ground projection node initialized. Waiting for camera info...")

    def camera_info_callback(self, msg):
        """Process camera calibration information"""
        if not self.camera_info_received:
            rospy.loginfo("Received camera info!")
            self.ground_projector = GroundProjectionGeometry(
                im_width=msg.width,
                im_height=msg.height,
                homography=self.homography
            )
            self.camera_info_received = True
            rospy.loginfo("Ground projection geometry initialized")

    def point_to_ground(self, point_msg):
        """Convert a point from image coordinates to ground plane coordinates"""
        if not self.camera_info_received:
            rospy.logwarn_throttle(1.0, "Cannot project point - waiting for camera info")
            return None

        # Convert message to Point object
        image_point = Point.from_message(point_msg)
        
        try:
            # Project point to ground plane
            ground_point = self.ground_projector.pixel2ground(image_point)
            
            # Convert to ROS message
            ground_msg = PointMsg()
            ground_msg.x = ground_point.x
            ground_msg.y = ground_point.y
            ground_msg.z = ground_point.z
            return ground_msg
            
        except Exception as e:
            rospy.logerr(f"Error projecting point to ground: {e}")
            return None

    def lineseglist_callback(self, seglist_msg):
        """Process incoming line segments and project them to the ground plane"""
        rospy.loginfo(f"Processing segment list with {len(seglist_msg.segments)} segments")
        
        if not self.camera_info_received:
            rospy.logwarn_throttle(1.0, "Waiting for camera info...")
            return

        # Create output segment list
        seglist_out = SegmentList()
        seglist_out.header = seglist_msg.header

        # Process each segment
        for received_segment in seglist_msg.segments:
            new_segment = Segment()
            new_segment.points[0] = self.pixel_msg_to_ground_msg(received_segment.pixels_normalized[0])
            new_segment.points[1] = self.pixel_msg_to_ground_msg(received_segment.pixels_normalized[1])
            
            new_segment.color = received_segment.color
                # TODO what about normal and points
            seglist_out.segments.append(new_segment)
        self.pub_lineseglist.publish(seglist_out)
        # for received_segment in seglist_msg.segments:
        #     try:
        #         new_segment = Segment()
        #         # Project start point
        #         ground_point1 = self.point_to_ground(received_segment.pixels_normalized[0])
        #         if ground_point1 is None:
        #             continue
        #         new_segment.points[0] = ground_point1
                
        #         # Project end point
        #         ground_point2 = self.point_to_ground(received_segment.pixels_normalized[1])
        #         if ground_point2 is None:
        #             continue
        #         new_segment.points[1] = ground_point2
                
        #         # Keep the color information
        #         new_segment.color = received_segment.color
                
        #         # Add to output list
        #         seglist_out.segments.append(new_segment)
                
        #     except Exception as e:
        #         rospy.logerr(f"Error processing segment: {e}")
        #         continue

        # # Publish the projected segments
        # self.pub_lineseglist.publish(seglist_out)
        
        if not self.first_processing_done:
            rospy.loginfo("First projected segments published!")
            self.first_processing_done = True

        # Publish debug visualization if there are subscribers
        if self.pub_debug_img.get_num_connections() > 0:
            try:
                debug_image = self.create_debug_image(seglist_out)
                debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_image)
                debug_msg.header = seglist_out.header
                self.pub_debug_img.publish(debug_msg)
            except Exception as e:
                rospy.logerr(f"Error creating debug image: {e}")

    def create_debug_image(self, seg_list):
        """Create a debug visualization of the projected segments"""
        if self.debug_image_bg is None:
            # Initialize the background image (400x400 pixels, 1m x 1m)
            self.debug_image_bg = np.ones((400, 400, 3), np.uint8) * 128
            
            # Draw grid lines
            for i in range(40, 361, 40):
                cv2.line(self.debug_image_bg, (i, 20), (i, 380), (255, 255, 0), 1)
                cv2.line(self.debug_image_bg, (20, i), (380, i), (255, 255, 0), 1)
            
            # Add scale markers
            cv2.putText(self.debug_image_bg, "-20cm", (95, 395),
                       cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_image_bg, "0", (200, 395),
                       cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_image_bg, "+20cm", (280, 395),
                       cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)

        debug_img = self.debug_image_bg.copy()

        # Define colors for different segment types
        colors = {
            Segment.WHITE: (255, 255, 255),
            Segment.YELLOW: (0, 255, 255),
            Segment.RED: (0, 0, 255)
        }

        # Draw segments
        for segment in seg_list.segments:
            try:
                # Convert ground coordinates to image pixels (1m = 400px, origin at center)
                x1 = int(200 + segment.points[0].y * 400)  # Swap x and y for visualization
                y1 = int(200 - segment.points[0].x * 400)
                x2 = int(200 + segment.points[1].y * 400)
                y2 = int(200 - segment.points[1].x * 400)
                
                # Draw line if points are within image bounds
                if (0 <= x1 < 400 and 0 <= y1 < 400 and 
                    0 <= x2 < 400 and 0 <= y2 < 400):
                    color = colors.get(segment.color, (0, 0, 0))
                    cv2.line(debug_img, (x1, y1), (x2, y2), color, 2)
            
            except Exception as e:
                rospy.logerr(f"Error drawing segment in debug image: {e}")
                continue

        return debug_img

if __name__ == "__main__":
    try:
        node = GroundProjectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass