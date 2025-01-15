#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSensePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('realsense_image_publisher', anonymous=True)

        # Create a publisher for the color image topic
        self.publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create RealSense pipeline and configure streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start the pipeline
        self.pipeline.start(self.config)

    def publish_image(self):
        # Wait for a frame of data
        frames = self.pipeline.wait_for_frames()

        # Get the color frame
        color_frame = frames.get_color_frame()

        if not color_frame:
            rospy.logwarn("No color frame received")
            return

        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the OpenCV image to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

        # Publish the ROS Image message
        self.publisher.publish(ros_image)
        rospy.loginfo('Published an image')

    def spin(self):
        # Publish images at 10 Hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_image()
            rate.sleep()

    def cleanup(self):
        # Stop the RealSense pipeline
        self.pipeline.stop()

if __name__ == '__main__':
    publisher = None  # Ensure publisher is defined at the module level
    try:
        # Create an instance of the RealSensePublisher
        publisher = RealSensePublisher()
        # Keep publishing images
        publisher.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down RealSense publisher node.")
    finally:
        if publisher:  # Only call cleanup if publisher was successfully created
            publisher.cleanup()

