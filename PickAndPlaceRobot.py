import rospy
import moveit_commander
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PickAndPlaceRobot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pick_and_place_robot', anonymous=True)

        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Set up image subscriber
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Perform computer vision processing (e.g., object detection)
        # Here, we use a simple color-based object detection as an example
        lower_color = np.array([0, 0, 100])
        upper_color = np.array([100, 100, 255])
        mask = cv2.inRange(cv_image, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Assuming the first contour represents the detected object
            object_contour = contours[0]

            # Get the centroid of the object
            M = cv2.moments(object_contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Perform pick and place based on object centroid
            self.pick_and_place(cx, cy)

    def pick_and_place(self, target_x, target_y):
        # Move the robot to a pre-defined pick pose
        pick_pose = self.get_pick_pose(target_x, target_y)
        self.move_group.set_pose_target(pick_pose)
        self.move_group.go()

        # Perform pick action (e.g., close gripper)
        self.perform_pick_action()

        # Move the robot to a pre-defined place pose
        place_pose = self.get_place_pose()
        self.move_group.set_pose_target(place_pose)
        self.move_group.go()

        # Perform place action (e.g., open gripper)
        self.perform_place_action()

    def get_pick_pose(self, target_x, target_y):
        # Define the pick pose based on the detected object's centroid
        pick_pose = self.move_group.get_current_pose().pose
        pick_pose.position.x = target_x / 1000.0  # Convert pixel coordinates to meters
        pick_pose.position.y = target_y / 1000.0
        pick_pose.position.z = 0.1  # Hover above the object
        return pick_pose

    def get_place_pose(self):
        # Define the place pose
        place_pose = self.move_group.get_current_pose().pose
        place_pose.position.z = 0.1  # Hover above the placement location
        return place_pose

    def perform_pick_action(self):
        # Simulate closing the gripper for picking
        rospy.sleep(1.0)
        print("Gripper closed for pick")

    def perform_place_action(self):
        # Simulate opening the gripper for placing
        rospy.sleep(1.0)
        print("Gripper opened for place")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        pick_and_place_robot = PickAndPlaceRobot()
        pick_and_place_robot.run()
    except rospy.ROSInterruptException:
        pass
