__author__ = 'rhys'
# This script will be working towards the first target use case for the Baxter robot

# The aim is for Baxter to identify individual tennis balls on a table and then
# to move these into a bin/defined location, also on the table.

# Imports
import numpy as np
import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class ObjectDetection:
    def __init__(self):
    
        self.rgb_img = None
        self.bridge = CvBridge()
        rgb_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.callback_rgb)
        depth_sub = rospy.Subscriber("camera/depth/image", Image, self.callback_depth)

        rospy.spin()

    def callback_rgb(self, data):
        """
        Function to save image to a variable in the class
        :param data:
        :return:
        """

        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def callback_depth(self, data):
        """
        Use the data received from the topic.
        Convert to CV image and then perform thresholding to identify objects
        :param data:
        :return:
        """
        # Convert the received image to a CV image and normalise it to grayscale
        depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        # Pass the image to to object detection function
        self.object_detection(depth_array)

    def object_detection(self, depth_array):
        """
        Function to detect objects from the depth image given
        :return:
        """
        self.detect_arm()

        # Perform thresholding on the image to remove all objects behind a plain
        ret, bin_img = cv2.threshold(depth_array, 0.3, 1, cv2.THRESH_BINARY_INV)

        # Erode the image a few times in order to separate close objects
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        err_img = cv2.erode(bin_img, element, iterations=20)

        # Create a new array of type uint8 for the findContours function
        con_img = np.array(err_img, dtype=np.uint8)

        # Find the contours of the image and then draw them on
        contours, hierarchy = cv2.findContours(con_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(con_img, contours, -1, (128, 255, 0), 3)

        for x in range(0, len(contours)):
            x, y, w, h = cv2.boundingRect(contours[x])
            cv2.rectangle(con_img, (x, y), ((x+w), (y+h)), (255, 0, 127), thickness=5, lineType=8, shift=0)

        # Show the colour images of the objects
        # self.show_colour(contours)

        # Show the Depth image and objects images
        cv2.imshow('Contours', con_img)
        cv2.imshow("Depth", bin_img)
        cv2.waitKey(3)

    def show_colour(self, cnt):
        """
        Use the objects found to show them in colour
        :return:
        """
        # Go through each rectangle and display the rgb
        length = len(cnt)

        # Create an array of size the amount of rectangles
        crop_rgb = []
        for i in range(0, length):
            crop_rgb.append(1)

        for x in range(0, length):
            x, y, w, h = cv2.boundingRect(cnt[x])

            # Try to crop the rgb image for each box
            try:
                crop_rgb[x] = self.rgb_img[y:y+h, x:x+w]
            except:
                pass

        for x in range(0, length):
            name = "Cropped " + str(x)
            cv2.imshow(name, crop_rgb[x])
        cv2.waitKey(3)

    def detect_arm(self):
        """
        Function to detect the colour disk on the arm and retun the co-ordinates for calibration
        :return:
        """
        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2HSV)

        lower = np.array([20, 100, 100], dtype=np.uint8)
        upper = np.array([30, 255, 255], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(self.rgb_img, self.rgb_img, mask=mask)

        cv2.imshow("Yellow", output)
        cv2.waitKey(3)

class Initialise:
    def __init__(self):
        """
        Setup Baxter and the surrounding environment
        """

        # First initialize moveit_commander and rospy.
        print "============ Initialising Baxter"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('can_node',
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
        # table = PoseStamped()
        # table.header.frame_id = self.robot.get_planning_frame()
        
        # table.pose.position.x = 

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
    objects = ObjectDetection()

   
if __name__ == '__main__':
    main()
