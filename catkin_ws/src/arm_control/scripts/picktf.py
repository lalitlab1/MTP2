#!/usr/bin/env python

import rospy
import tf
import yaml
import os
from geometry_msgs.msg import Pose

def save_pose(pose, name, filename="saved_poses.yaml"):
    """Save the given pose to a YAML file."""
    pose_data = {
        name: {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
        }
    }

    # Check if file exists and load existing data
    if os.path.exists(filename):
        with open(filename, "r") as file:
            existing_data = yaml.safe_load(file) or {}
    else:
        existing_data = {}

    # Merge the new pose data
    existing_data.update(pose_data)

    # Save updated data back to the file
    with open(filename, "w") as file:
        yaml.safe_dump(existing_data, file, default_flow_style=False)

    rospy.loginfo("Pose '{}' saved to {}.".format(name, filename))


def listen_and_save_frames():
    """Listen to the `approach`, `grasp`, and `lift` frames and save their poses."""
    rospy.init_node("save_tf_poses", anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(1)  # 1 Hz

    frames = ["approach", "grasp", "lift"]
    filename = "saved_poses.yaml"

    while not rospy.is_shutdown():
        for frame in frames:
            try:
                # Listen to the transform
                (trans, rot) = listener.lookupTransform("world", frame, rospy.Time(0))

                # Create a Pose message
                pose = Pose()
                pose.position.x = trans[0]
                pose.position.y = trans[1]
                pose.position.z = trans[2]
                pose.orientation.x = rot[0]
                pose.orientation.y = rot[1]
                pose.orientation.z = rot[2]
                pose.orientation.w = rot[3]

                # Save the pose to the YAML file
                save_pose(pose, frame, filename)
                rospy.loginfo("Saved pose for frame: {}".format(frame))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Frame '{}' not available.".format(frame))

        rate.sleep()


if __name__ == "__main__":
    try:
        listen_and_save_frames()
    except rospy.ROSInterruptException:
        pass
