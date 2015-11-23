__author__ = 'rhys'
# This script will be working towards the first target use case for the Baxter robot

# The aim is for Baxter to identify individual tennis balls on a table and then
# to move these into a bin/defined location, also on the table.

# Imports
import freenect
import numpy as np
import cv
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import octomap


class Initialise:
    def __init__(self):
        """
        Setup Baxter and the surrounding environment
        """

        # First initialize moveit_commander and rospy.
        print "============ Initialising Baxter"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('tennis_ball_sorter_node',
                        anonymous=True)

        # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate the MoveGroupCommander objects. These objects are an interface to one group of joints.
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")

        # Create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory)

        # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
        print "============ Waiting for RVIZ..."
        rospy.sleep(10)

    def test_movement(self):
        """
        Instruct Baxters arms to complete a simple motion task to ensure that initialisation was successful.
        """

        # Set a plan for each arm group so that moveit! can plan the movement needed.
        print "============ Generating right_arm plan"
        pose_target = create_pose_target(1.0, 0.7, -0.05, 1.1)
        self.right_arm.set_pose_target(pose_target)
        right_arm_plan = self.right_arm.plan()

        print "============ Generating left_arm plan"
        pose_target = create_pose_target(1.0, 0.7, -0.05, 1.1)
        self.left_arm.set_pose_target(pose_target)
        left_arm_plan = self.left_arm.plan()

        # Wait for Rviz to plan the movement for each group
        print "============ Waiting while plans are visualized..."
        rospy.sleep(5)

        # Execute the movement
        print "============ Executing plans"
        right_arm_plan.go()
        left_arm_plan.go()

    def kinect_setup(self):
        """
        Function to setup the kinect topic. As well as the Octomap
        :return:
        """


        depth_image, _ = freenect.sync_get_depth()
        depth_image = depth_image.astype(np.uint8)


        tree = octomap.OcTree(depth_image)
        view_free = False

        itr = tree.begin_tree()
        root_size = itr.getSize()
        op =[]
        fp = []
        so = []
        sf = []

        for i in itr:
            if i.isLeaf():
                so.append(i.getSize() / root_size)
                op.append(i.getCoordinate())
            else:
                if view_free:
                    sf.append(i.getSize() / root_size)
                    fp.append(i.getCoordinate())

        op = zip(*op)
        fp = zip(*fp)
        points3d(op[0], op[1], op[2], so, opacity=1.0, mode='cube',
                 color=(0, 0, 1), scale_mode='scalar', scale_factor=root_size)

        if view_free:
            points3d(fp[0], fp[1], fp[2], sf, opacity=0.3, mode='cube',
                     color=(0, 1, 0), scale_mode='scalar', scale_factor=root_size)
        show()



def create_pose_target(w, x, y, z):
    """
    Take in an  w, x, y and z values to create a pose target
    :return: pose_target - a target for a group of joints to move to.
    """

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = w
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    return pose_target


def main():

    # Run the Initialise class to setup Baxter and the environment
    baxter = Initialise()

    while 1:
        # Run a test move to show that the script is running correctly
        baxter.kinect_setup()

if __name__ == '__main__':
    main()
