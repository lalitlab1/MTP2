import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf.transformations
from shape_msgs.msg import SolidPrimitive  # Correct import


# Define the base frame for easy modification
BASE_FRAME = "link0"

class ObjectManager:
    def __init__(self, scene, object_params):
        self.scene = scene
        self.object_params = object_params

    def add_collision_objects(self):
        for obj_name, params in self.object_params.items():
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = params["frame_id"]
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = params["position"]
            pose.pose.orientation.w = 1.0  # Assuming no rotation

            dimensions = params["dimensions"]
            self.scene.add_box(obj_name, pose, size=dimensions)


class GraspManager:
    @staticmethod
    def create_grasp(grasp_position):
        grasp = moveit_msgs.msg.Grasp()
        grasp.grasp_pose.header.frame_id = BASE_FRAME  # Use the global BASE_FRAME

        # Set grasp orientation
        orientation = tf.transformations.quaternion_from_euler(-1.57, -0.78, -1.57)
        grasp.grasp_pose.pose.orientation.x = orientation[0]
        grasp.grasp_pose.pose.orientation.y = orientation[1]
        grasp.grasp_pose.pose.orientation.z = orientation[2]
        grasp.grasp_pose.pose.orientation.w = orientation[3]

        # Set grasp position
        grasp.grasp_pose.pose.position.x, grasp.grasp_pose.pose.position.y, grasp.grasp_pose.pose.position.z = grasp_position

        # Configure grasping actions
        grasp.pre_grasp_posture = GraspManager.create_gripper_posture(0.04)
        grasp.grasp_posture = GraspManager.create_gripper_posture(0.0)

        # Pre-grasp approach
        grasp.pre_grasp_approach.direction.header.frame_id = BASE_FRAME
        grasp.pre_grasp_approach.direction.vector.x = 1.0
        grasp.pre_grasp_approach.min_distance = 0.095
        grasp.pre_grasp_approach.desired_distance = 0.115

        # Post-grasp retreat
        grasp.post_grasp_retreat.direction.header.frame_id = BASE_FRAME
        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.1
        grasp.post_grasp_retreat.desired_distance = 0.25

        return [grasp]

    @staticmethod
    def create_gripper_posture(position):
        posture = JointTrajectory()
        posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = rospy.Duration(0.5)
        posture.points = [point]
        return posture


class PlaceManager:
    @staticmethod
    def create_place_location(place_position):
        place_location = moveit_msgs.msg.PlaceLocation()
        place_location.place_pose.header.frame_id = BASE_FRAME  # Use the global BASE_FRAME

        # Set place orientation
        orientation = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        place_location.place_pose.pose.orientation.x = orientation[0]
        place_location.place_pose.pose.orientation.y = orientation[1]
        place_location.place_pose.pose.orientation.z = orientation[2]
        place_location.place_pose.pose.orientation.w = orientation[3]

        # Set place position
        place_location.place_pose.pose.position.x, place_location.place_pose.pose.position.y, place_location.place_pose.pose.position.z = place_position

        # Pre-place approach
        place_location.pre_place_approach.direction.header.frame_id = BASE_FRAME
        place_location.pre_place_approach.direction.vector.z = -1.0
        place_location.pre_place_approach.min_distance = 0.095
        place_location.pre_place_approach.desired_distance = 0.115

        # Post-place retreat
        place_location.post_place_retreat.direction.header.frame_id = BASE_FRAME
        place_location.post_place_retreat.direction.vector.y = -1.0
        place_location.post_place_retreat.min_distance = 0.1
        place_location.post_place_retreat.desired_distance = 0.25

        # Posture after placing
        place_location.post_place_posture = GraspManager.create_gripper_posture(0.04)

        return [place_location]


class PickAndPlaceController:
    def __init__(self, object_params, grasp_position, place_position):
        self.object_params = object_params
        self.grasp_position = grasp_position
        self.place_position = place_position

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_place', anonymous=True)

        # Initialize the robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "tmr_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Initialize the display trajectory publisher
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

    def add_collision_objects(self):
        for obj_name, params in self.object_params.items():
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = params["frame_id"]
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = params["position"]
            pose.pose.orientation.w = 1.0  # Assuming no rotation

            dimensions = params["dimensions"]
            self.scene.add_box(obj_name, pose, size=dimensions)

    def execute(self):
        self.add_collision_objects()
        rospy.sleep(2)  # Allow time for the collision objects to be added

        # Pick action
        self.pick()

        # Place action
        self.place()

    def pick(self):
        # Define the pick motion plan request
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = self.grasp_position
        pose_goal.orientation.w = 1.0

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if not plan:
            rospy.logwarn("Fail: ABORTED: Must specify group in motion plan request")

    def place(self):
        # Define the place motion plan request
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = self.place_position
        pose_goal.orientation.w = 1.0

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if not plan:
            rospy.logwarn("Fail: ABORTED: Must specify group in motion plan request")

def main():
    """Defines all parameters and runs the Pick and Place sequence."""
    rospy.sleep(2)  # Allow time for ROS setup

    object_params = {
        "table1": {
            "frame_id": BASE_FRAME,  # Use global BASE_FRAME
            "dimensions": (0.2, 0.4, 0.4),
            "position": (0.5, 0, 0.2),
        },
        "table2": {
            "frame_id": BASE_FRAME,  # Use global BASE_FRAME
            "dimensions": (0.4, 0.2, 0.4),
            "position": (0, 0.5, 0.2),
        },
        "object": {
            "frame_id": BASE_FRAME,  # Use global BASE_FRAME
            "dimensions": (0.02, 0.02, 0.2),
            "position": (0.5, 0, 0.5),
        }
    }

    grasp_position = (0.415, 0, 0.5)
    place_position = (0, 0.5, 0.5)

    controller = PickAndPlaceController(object_params, grasp_position, place_position)
    controller.execute()

if __name__ == "__main__":
    main()
