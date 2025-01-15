#!/usr/bin/env python

import rospy
import tf
import yaml
import os
from geometry_msgs.msg import Pose

def load_saved_poses(filename="saved_poses.yaml"):
    """Load saved poses from the YAML file."""
    if not os.path.exists(filename):
        rospy.logerr("File '{}' not found.".format(filename))
        return {}

    with open(filename, "r") as file:
        data = yaml.safe_load(file)
        if data is None:
            rospy.logerr("No data found in '{}'.".format(filename))
            return {}
        rospy.loginfo("Loaded poses from '{}'.".format(filename))
        return data


def publish_saved_poses(saved_poses, rate_hz=10):
    """Publish saved poses as TF transforms."""
    rospy.init_node("publish_saved_poses", anonymous=True)
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        for name, pose_data in saved_poses.items():
            try:
                position = pose_data["position"]
                orientation = pose_data["orientation"]

                # Publish the transform
                broadcaster.sendTransform(
                    (position["x"], position["y"], position["z"]),
                    (orientation["x"], orientation["y"], orientation["z"], orientation["w"]),
                    rospy.Time.now(),
                    name,
                    "world"
                )
                rospy.loginfo("Published transform for frame: {}".format(name))

            except KeyError as e:
                rospy.logerr("Missing key {} in pose data for frame '{}': {}".format(e, name, pose_data))

        rate.sleep()


if __name__ == "__main__":
    try:
        # Load saved poses from file
        filename = "saved_poses.yaml"
        saved_poses = load_saved_poses(filename)

        if saved_poses:
            # Publish loaded poses as transforms
            publish_saved_poses(saved_poses)
        else:
            rospy.logerr("No poses to publish. Exiting.")

    except rospy.ROSInterruptException:
        pass

