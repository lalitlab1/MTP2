#!/usr/bin/env python

import rospy
import tf2_ros
import math
import os
import json
import time


def quaternion_to_euler(x, y, z, w):
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw).
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def save_to_file(data, folder_name="target_frame", file_name="target_frame_info.json"):
    """
    Saves the transformation data to a file in the specified folder.
    """
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    file_path = os.path.join(folder_name, file_name)

    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

    rospy.loginfo("Saved target frame transformation to {}".format(file_path))


def main():
    rospy.init_node('target_frame_calculator')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    timeout = 20  # Maximum time to wait in seconds
    start_time = time.time()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Try to get the transform from marker_51 to tool0
            transform = tf_buffer.lookup_transform('marker_50', 'tool0', rospy.Time(0))

            # Extract translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert quaternion to Euler angles
            roll, pitch, yaw = quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)

            # Prepare transformation data
            target_frame_transformation = {
                "parent_frame": "marker_50",
                "child_frame": "target_frame",
                "translation": (translation.x, translation.y, translation.z),
                "rotation": (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))  # Convert to degrees
            }

            rospy.loginfo("Target Frame Transformation: {}".format(target_frame_transformation))

            # Save to file
            save_to_file(target_frame_transformation)

            # Exit after successful calculation
            break
        except tf2_ros.LookupException:
            rospy.logwarn("Transform not found. Retrying...")
        except tf2_ros.ConnectivityException:
            rospy.logwarn("Frames are not connected. Retrying...")
        except tf2_ros.ExtrapolationException:
            rospy.logwarn("Transform extrapolation error. Retrying...")

        # Check timeout
        if time.time() - start_time > timeout:
            rospy.logerr("Timeout reached: Could not find transform within {} seconds.".format(timeout))
            break

        rate.sleep()


if __name__ == "__main__":
    main()
