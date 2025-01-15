#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
import time
import sys

# Compatibility imports
try:
    from tf.transformations import quaternion_from_matrix
except ImportError:
    from scipy.spatial.transform import Rotation as R

def detect_aruco_markers_from_camera(image, dictionary_type=cv2.aruco.DICT_6X6_250):
    """
    Detect ArUco markers in the provided grayscale image using the specified dictionary.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_type)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)

    if ids is not None:
        rospy.loginfo("Detected ArUco markers: IDs={}, Corners={}".format(ids.flatten().tolist(), corners))
    else:
        rospy.loginfo("No ArUco markers detected.")
    return corners, ids


def estimate_pose_single_markers(corners, marker_size, mtx, distortion):
    """
    Estimate the pose of detected ArUco markers.
    """
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
    rvecs, tvecs = [], []
    for corner in corners:
        try:
            # Ensure corner is a numpy array of the correct shape
            corner = np.array(corner, dtype=np.float32).reshape(-1, 1, 2)
            # SolvePnP to estimate the pose
            _, rvec, tvec = cv2.solvePnP(marker_points, corner, mtx, distortion)
            rvecs.append(rvec)
            tvecs.append(tvec)
        except cv2.error as e:
            rospy.logerr("Error in solvePnP: {}".format(e))
    return rvecs, tvecs


class ArucoSubscriber:
    def __init__(self):
        rospy.init_node('aruco_detection_node', anonymous=True)
        rospy.loginfo("Initializing ArucoSubscriber node...")

        # Subscriber
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        rospy.loginfo("Subscribed to /camera/color/image_raw")

        # Publisher
        self.image_pub = rospy.Publisher("/camera/aruco/image_raw", Image, queue_size=10)
        rospy.loginfo("Publisher created for /camera/aruco/image_raw")

        # CvBridge
        self.bridge = CvBridge()

        # TF2 Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Camera parameters
        self.camera_matrix = np.array([[612.1316528320312, 0, 314.5328369140625], 
                                        [0, 612.2755737304688, 245.1070098876953], 
                                        [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)

        # ArUco marker parameters
        self.dictionary_type = cv2.aruco.DICT_6X6_250
        self.marker_size = 0.05  # Marker size in meters

        self.last_image_time = time.time()

    def image_callback(self, msg):
        try:
            self.last_image_time = time.time()

            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids = detect_aruco_markers_from_camera(gray, self.dictionary_type)

            if ids is not None and len(corners) > 0:
                rvecs, tvecs = estimate_pose_single_markers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                # Draw markers
                for i, corner in enumerate(corners):
                    top_left = corner[0][0]
                    position = (int(top_left[0]), int(top_left[1]) - 10)
                    marker_id = str(ids[i][0])
                    cv2.putText(cv_image, "ID: {}".format(marker_id), position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    self.publish_marker_tf(ids[i][0], rvecs[i], tvecs[i])

            # Publish annotated image
            annotated_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(annotated_image)

        except Exception as e:
            rospy.logerr("Error in image callback: {}".format(e))

    def publish_marker_tf(self, marker_id, rvec, tvec):
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = quaternion_from_matrix(
            np.vstack([np.column_stack([rotation_matrix, [0, 0, 0]]), [0, 0, 0, 1]])
        )

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()

        if marker_id == 49:
            # Reverse the TF: Swap the frames
            t.header.frame_id = "marker_{}".format(marker_id)
            t.child_frame_id = "camera_frame"

            # Reverse the transformation: Invert the translation and rotation
            inverse_rotation_matrix = np.linalg.inv(rotation_matrix)
            inverse_translation = -np.dot(inverse_rotation_matrix, np.array([tvec[0][0], tvec[1][0], tvec[2][0]]))

            # Convert inverse rotation matrix back to quaternion
            quaternion = quaternion_from_matrix(
                np.vstack([np.column_stack([inverse_rotation_matrix, [0, 0, 0]]), [0, 0, 0, 1]])
            )

            t.transform.translation.x = inverse_translation[0]
            t.transform.translation.y = inverse_translation[1]
            t.transform.translation.z = inverse_translation[2]
        else:
            # Normal TF: camera_frame -> marker_id
            t.header.frame_id = "camera_frame"
            t.child_frame_id = "marker_{}".format(marker_id)

            t.transform.translation.x = tvec[0][0]
            t.transform.translation.y = tvec[1][0]
            t.transform.translation.z = tvec[2][0]

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)


if __name__ == "__main__":
    print("Running on Python {}.{}".format(sys.version_info[0], sys.version_info[1]))
    aruco_subscriber = ArucoSubscriber()
    rospy.spin()
