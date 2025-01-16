#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import json
import os
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import math

CONFIG_FILES = [
    os.path.expanduser("~/catkin_ws/src/arm_control/config/tf_config_static.json"),
    os.path.expanduser("~/catkin_ws/src/arm_control/config/tf_config_dynamic.json"),
]  # Add more config file paths as needed

def load_tf_configs():
    """Loads all TF configuration files (JSON format)."""
    transforms = []
    for config_file in CONFIG_FILES:
        try:
            if os.path.exists(config_file):
                with open(config_file, "r") as file:
                    data = json.load(file)
                transforms.extend(data.get("transforms", []))
            else:
                rospy.logwarn("Config file not found: {}".format(config_file))
        except Exception as e:
            rospy.logerr("Failed to load TF config {}: {}".format(config_file, e))
    return transforms

def publish_dynamic_tfs():
    rospy.init_node("dynamic_tf_publisher", anonymous=True)
    rospy.loginfo("Starting dynamic TF publisher with multiple config files")

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz
    last_modified_times = {file: None for file in CONFIG_FILES}

    while not rospy.is_shutdown():
        try:
            updated = False
            for config_file in CONFIG_FILES:
                if os.path.exists(config_file):
                    new_modified_time = os.path.getmtime(config_file)
                    if last_modified_times[config_file] is None or new_modified_time > last_modified_times[config_file]:
                        last_modified_times[config_file] = new_modified_time
                        updated = True
            if updated:
                rospy.loginfo("TF configs updated, reloading...")
                transforms = load_tf_configs()
            else:
                transforms = load_tf_configs()
        except Exception as e:
            rospy.logwarn("Error checking config files: {}".format(e))
            transforms = []

        for transform in transforms:
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = transform["parent_frame"]
            t.child_frame_id = transform["child_frame"]

            # Set translation
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = transform["translation"]

            # Convert Euler angles (degrees) to quaternion
            roll, pitch, yaw = map(math.radians, transform["rotation"])
            quaternion = quaternion_from_euler(roll, pitch, yaw)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quaternion

            # Broadcast the transform
            tf_broadcaster.sendTransform(t)
            rospy.loginfo("Published transform: {} → {}".format(t.header.frame_id, t.child_frame_id))

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_dynamic_tfs()
    except rospy.ROSInterruptException:
        pass
