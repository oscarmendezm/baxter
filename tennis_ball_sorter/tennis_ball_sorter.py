__author__ = 'rhys'
# This script will be working towards the first target use case for the Baxter robot

# The aim is for Baxter to identify individual tennis balls on a table and then
# to move these into a bin/defined location, also on the table.

# Imports
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
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=10)
                                                            
        ## Getting Basic Information
	  	## ^^^^^^^^^^^^^^^^^^^^^^^^^
	  	##
	  	## We can get the name of the reference frame for this robot
        print "============ Reference frame Left: %s" % self.left_arm.get_planning_frame()
        print "============ Reference frame Rght: %s" % self.right_arm.get_planning_frame()
        
        ## We can also print the name of the end-effector link for this group
        print "============ End Effector Left: %s" % self.left_arm.get_end_effector_link()
        print "============ End Effector Rght: %s" % self.right_arm.get_end_effector_link()
        
        ## We can get a list of all the groups in the robot
        print "============ Robot Groups:"
        print self.robot.get_group_names()
        
        ## We can get a list of all the groups in the robot
        print "============ Robot GroupsPoses:"
        print self.left_arm.get_current_pose()
        print self.right_arm.get_current_pose()

        ## Sometimes for debugging it is useful to print the entire state of the
        ## robot.
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print "============"                                                   
                                                            
        # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
        #print "============ Waiting for RVIZ..."
        #rospy.sleep(10)
        
        # Add a table to the environment
        table = PoseStamped()
        table.header.frame_id = self.robot.get_planning_frame()
        
        table.pose.position.x = 

    def test_movement(self):
        """
        Instruct Baxters arms to complete a simple motion task to ensure that initialisation was successful.
        """

        print "============ Generating left_arm plan"
        pose_target = create_pose_target(0.000000,		# Ww
        								 0.000000,		# Wx
        								 1.000000,		# Wy	
        								 0.000000,		# Wz
        								 0.8, 0.0, 0.0)	# X, Y, Z
        self.left_arm.set_goal_tolerance(0.01);
        self.left_arm.set_planner_id("RRTConnectkConfigDefault");
        self.left_arm.set_pose_target(pose_target)
        left_arm_plan = self.left_arm.plan()

        # Execute the movement
        print "============ Executing plans"
        self.left_arm.go()


def create_pose_target(Ww, Wx, Wy, Wz, x, y, z):
    """
    Take in an  w, x, y and z values to create a pose target
    :return: pose_target - a target for a group of joints to move to.
    """

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = Ww
    pose_target.orientation.x = Wx
    pose_target.orientation.y = Wy
    pose_target.orientation.z = Wz
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    return pose_target


def main():

    # Run the Initialise class to setup Baxter and the environment
    baxter = Initialise()
    baxter.test_movement()

   
if __name__ == '__main__':
    main()
