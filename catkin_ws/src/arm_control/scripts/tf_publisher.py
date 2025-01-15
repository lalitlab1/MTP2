#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import time
import math  # Import math module for radians conversion

def publish_multiple_tfs():
    rospy.init_node('multiple_tfs_publisher', anonymous=True)
    rospy.loginfo("Starting TF publisher for multiple transforms")

    # Create the TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Define a list of transforms to be published
    transforms = [
        {
            "parent_frame": "base", 
            "child_frame": "marker_49", 
            "translation": (-0.3338, 0.239, 0.0), 
            "rotation": (0, 0.0, 90)  # roll, pitch, yaw (in degrees)
        },
        {
            "parent_frame": "world", 
            "child_frame": "base", 
            "translation": (0, 0.0, 0.0), 
            "rotation": (0.0, 0.0, 0.0)  # roll, pitch, yaw (in degrees)
        },
        # Example of additional transform (commented-out in the original code)
        # {
        #     "parent_frame": "world", 
        #     "child_frame": "target_frame_original", 
        #     "translation": (-0.490, 0.011, 0.116), 
        #     "rotation": (-91.751, 0.622, -0.319)  # roll, pitch, yaw (in degrees)
        # {
        #     "translation": [
        #         0.02040157237037521, 
        #         -0.20370858371346184, 
        #         0.07590973974634599
        #     ], 
        #     "child_frame": "target_frame", 
        #     "rotation": [
        #         -89.73189651958377, 
        #         -3.5555103202369325, 
        #         -2.249544592576453
        #     ], 
        #     "parent_frame": "marker_51"
        # },
    ]
    
    # Broadcast the transforms in a loop
    rate = rospy.Rate(10)  # 10 Hz publishing rate
    while not rospy.is_shutdown():
        for transform in transforms:
            # Create the TransformStamped message
            t = geometry_msgs.msg.TransformStamped()

            # Set the timestamp for the transform
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = transform["parent_frame"]
            t.child_frame_id = transform["child_frame"]

            # Set translation
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = transform["translation"]

            # Convert Euler angles from degrees to radians, then to quaternion for rotation
            roll, pitch, yaw = transform["rotation"]
            roll_rad = math.radians(roll)  # Convert roll from degrees to radians
            pitch_rad = math.radians(pitch)  # Convert pitch from degrees to radians
            yaw_rad = math.radians(yaw)  # Convert yaw from degrees to radians

            # Convert Euler angles (in radians) to quaternion
            quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            # Broadcast the transform
            tf_broadcaster.sendTransform(t)
            rospy.loginfo("Published transform from {} to {}".format(t.header.frame_id, t.child_frame_id))

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_multiple_tfs()
    except rospy.ROSInterruptException:
        pass
