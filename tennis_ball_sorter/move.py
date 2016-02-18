__author__ = 'rhys'

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


class Robot:
    def __init__(self):
        """
        Set up all of the subscriptions and objects to control the robot
        :return: 
        """  
        
        # First initialize moveit_commander and rospy.
        print "============ Initialising Baxter"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('can_node',
                        anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=10)
        
        # We can get a list of all the groups in the robot
        print "============ Robot GroupsPoses:"
        print self.left_arm.get_current_pose()
        #print self.right_arm.get_current_pose()
        
    def move_to_calibrate(self, test):
        """
        Instruct baxters arm to move to a set point for calibration
        """
        
        if test is 1:
            print "============ Generating left_arm plan"
            pose_target = create_pose_target(0.251457698541,		# Ww
                                             0.132660864689,		# Wx
                                             -0.937502362532,		# Wy	
                                             0.200647554364,		# Wz
                                             0.453869796845, 		# X
                                             -0.0301870563505, 		# Y
                                             0.150496092849)		# Z

            
        else:
            pose_target = create_pose_target(0.251457698541,		# Ww
                                             0.132660864689,		# Wx
                                             -0.937502362532,		# Wy	
                                             0.200647554364,		# Wz
                                             0.553869796845, 		# X
                                             -0.1301870563505, 		# Y
                                             0.250496092849)		# Z
                                             
        self.left_arm.set_goal_tolerance(0.01);
        self.left_arm.set_planner_id("RRTConnectkConfigDefault");
        self.left_arm.set_pose_target(pose_target)
        left_arm_plan = self.left_arm.plan()
        
        # Execute the movement
        print "============ Executing plans"
        self.left_arm.go()

      
class Calibration:
    def __init__(self):
        """
        Setup subscribers and publishers for vision 
        """
        self.yellow_img = None
        self.eroded_yellow_img = None
        self.rgb_img = None
        self.depth_img = None
        self.contour = None
        self.center_x = 0
        self.center_y = 0
        self.bridge = CvBridge()
        
        self.rgb_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.callback_rgb)
        self.depth_sub = rospy.Subscriber("camera/depth/image", Image, self.callback_depth)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        													moveit_msgs.msg.DisplayTrajectory,
        													queue_size=5)      
        
    def callback_rgb(self, data):
        """
        Function to handle data arriving for RGB
        :param data: 
        :return: 
        """
        # Convert to usable format
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # View only Yellow to find point on gripper
        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_RGB2HSV)
        lower = np.array([20, 100, 100], dtype=np.uint8)
        upper = np.array([100, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        self.yellow_img = cv2.bitwise_and(self.rgb_img, self.rgb_img, mask=mask)
        
        # Erode the image a few times in order to separate close objects
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        self.eroded_yellow_img = cv2.erode(self.yellow_img, element, iterations=2)  	
        
        # Determine the location of the yellow circle in RGB 
        self.yellow_circle_detection()
        
    def callback_depth(self, data):
        """
        Function to handle data arriving for depth
        :param data: 
        :return: 
        """
        # Convert the received image to a CV image and normalise it to grayscale
        depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
        self.depth_img = np.array(depth, dtype=np.float32)
        cv2.normalize(self.depth_img, self.depth_img, 0, 1, cv2.NORM_MINMAX)

        # Add a bit to the x value to account for the off center camera
        self.center_x += 80
        
        # Print the center of the circle on the depth image
        cv2.rectangle(self.depth_img, (self.center_x, self.center_y), ((self.center_x+1), (self.center_y+1)), (255, 0, 127), thickness=5, lineType=8, shift=0)
        cv2.imshow("Depth", self.depth_img)
        cv2.waitKey(5)
        
        # Determine the depth of this point
        print self.depth_img[self.center_x][self.center_y]
        print self.center_x + 80
        print self.center_y

    def yellow_circle_detection(self):
        """
        Identify objects on depth
        :return: 
        """
        gray_image = cv2.cvtColor(self.eroded_yellow_img, cv2.COLOR_BGR2GRAY)
        
        # Perform thresholding on the image to remove all objects behind a plain
        ret, bin_img = cv2.threshold(gray_image, 0.3, 1, cv2.THRESH_BINARY_INV)
        
        # Create a new array of type uint8 for the findContours function
        con_img = np.array(gray_image, dtype=np.uint8)
        
        # Find the contours of the image and then draw them on
        contours, hierarchy = cv2.findContours(con_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(con_img, contours, -1, (128, 255, 0), 3)
        
        try:
		    # Find the largest rectangle and discard the others
		    for i in range(0, len(contours)):
		    	x, y, w, h = cv2.boundingRect(contours[i])
		    	area = w * h
		    	if i is 0:
		    		current_max = area
		    		max_x = i
		    	else:
		    		if area > current_max:
		    			current_max = area
		    			max_x = i
		    			
		    # Display the largest box
		    x, y, w, h = cv2.boundingRect(contours[max_x])
		    self.contour = contours[max_x]
		    cv2.rectangle(self.rgb_img, (x, y), ((x+w), (y+h)), (255, 0, 127), thickness=5, lineType=8, shift=0)
        except:
            print "No Circle found"

        # Find the Centre of the largest box
        self.center_x = int(x + (0.5 * w))
        self.center_y = int(y + (0.5 * h))
 
        cv2.imshow("Calibration", self.rgb_img)
        cv2.waitKey(5)

			
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
    
    # Initialise node
    rospy.init_node('can_node',anonymous=True)
    robot = Robot()
    robot.move_to_calibrate(2)
    calibration = Calibration()
    
    # Keep these images updating        
    rospy.spin()  
   
if __name__ == '__main__':
    main()
