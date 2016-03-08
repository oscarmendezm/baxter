__author__ = 'Rhys Bryant'

# Imports
import numpy as np
from numpy import *
from math import sqrt
import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from baxter_interface import Navigator
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time


class Control:
    def __init__(self):
        """
        Setup ROS communication between the Kinect and Baxter
        :return:
        """

        # Initialise variables
        self.rgb_img = None
        self.depth_img = None
        self.marker_box = None
        self.marker_center_x = None
        self.marker_center_y = None
        self.marker_depth = None

        self.bridge = CvBridge()

        # First initialize moveit_commander and rospy.
        print "============ Initialising Baxter"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('can_node', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.right_arm_navigator = Navigator('right')

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=10)

        # We can get a list of all the groups in the robot
        print "============ Robot GroupsPoses:"
        print self.right_arm.get_current_pose()
        print self.left_arm.get_current_pose()

        # Setup the subscribers and publishers
        self.rgb_sub = rospy.Subscriber("camera/rgb/image_rect_color", Image, self.callback_rgb)
        self.depth_sub = rospy.Subscriber("camera/depth_registered/image_raw", Image, self.callback_depth)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                           moveit_msgs.msg.DisplayTrajectory,
                                                           queue_size=5)
        self.screen_pub = rospy.Publisher('robot/xdisplay', Image, latch=True, queue_size=10)

    def callback_rgb(self, data):
        """
        Function to handle the arrival of RGB data
        :param data:
        :return:
        """

        # Convert the data to a usable format
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
            
        # Show the Kinect feed on Baxters screen
        msg = self.bridge.cv2_to_imgmsg(self.rgb_img, encoding="bgr8")
        self.screen_pub.publish(msg)

    def callback_depth(self, data):
        """
        Function to handle the arrival of depth data
        :param data:
        :return:
        """

        # Convert the data to a usable format
        depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
        self.depth_img = np.array(depth, dtype=np.float32)
        cv2.normalize(self.depth_img, self.depth_img, 0, 1, cv2.NORM_MINMAX)                

    def calibrate_main(self):
        """
        Function to complete the calibration between the Kinect and Baxter coordinate frames
        :return:
        """
        kinect_points = []
        baxter_points = []
        
        points_detected = False
        
        point = 0
        
        # Move the left arm out of the way for calibration
        self.move_to_remove()
        
        print "\n\nReady to calibrate. Move the arm to atleast 4 locations:"
        
        # Wait until arm button is pressed to use frame
        while self.right_arm_navigator.button1 is False:
            if self.right_arm_navigator.button0:
                while not points_detected:
                    x, y, z = self.return_current_pose("right")
                    baxter_points.append([x, y, z])
                    if (self.rgb_img is not None) and (self.depth_img is not None):
                        try:
                            points_detected = self.detect_calibration_marker()
                            if self.marker_depth is 0.0:
                                points_detected = False
                        except:
                            points_detected = False
                        if points_detected:
                            kinect_points.append([float(self.marker_center_x),
                            					 float(self.marker_center_y), 
                            					 float(self.marker_depth)])
                            print "Kinect: " + str(kinect_points[point])
                            print "Baxter: " + str(baxter_points[point])
                            point += 1
                            time.sleep(1)
                points_detected = False
            
        kinect_points = np.asarray(kinect_points)
        baxter_points = np.asarray(baxter_points)
        
        R, t = self.calculate_rigid_transform(kinect_points, baxter_points)
        
        print R
        print t   
        
    def calculate_rigid_transform(self, A, B):
		"""
		Calculate the rigid transform for the kinect and Baxter
		R = 3x3 Rotation Matrix
		t = 3x1 column vector
		"""
		
		assert len(A) == len(B)
		
		N = A.shape[0]; # total points
		
		centroid_A = mean(A, axis=0)
		
		centroid_B = mean(B, axis=0)
		
		# centre the points
		AA = A - tile(centroid_A, (N, 1))
		BB = B - tile(centroid_B, (N, 1))
		
		# dot is matrix multiplication for array
		H = transpose(AA) * BB
		U, S, Vt = linalg.svd(H)
		R = Vt.T * U.T
		
		# special reflection case
		if linalg.det(R) < 0:
		    print "Reflection detected"
		    Vt[2,:] *= -1
		    R = Vt.T * U.T
		    
		t = -R*centroid_A.T + centroid_B.T
		print t
		
		return R, t

		
		

    def move_to_remove(self):
        """
        Move Baxters Left arm out of the way of the Kinect for calibration
        """
        x = -0.0427936490711
    	y = 0.626374995844
    	z = 0.264948886765
    	pose_target = self.create_pose_target(0.0977083761873,		# Ww
                                              0.714003795927,		# Wx
                                              -0.00449042997044,	# Wy
                                              0.693275910921,		# Wz
                                              x, 		            # X
                                              y, 					# Y
                                              z)					# Z
                                              
        self.left_arm.set_goal_tolerance(0.01);
        self.left_arm.set_planner_id("RRTConnectkConfigDefault");
        self.left_arm.set_pose_target(pose_target)
        left_arm_plan = self.left_arm.plan()
        self.left_arm.go()
        

    def return_current_pose(self, limb):
    	"""
    	Function to return the current pose of a limb
    	"""
    	if limb is "right":
    		limb = self.right_arm
    	elif limb is "left":
    		limb = self.left_arm
    		
    	pose = limb.get_current_pose()
    	x = pose.pose.position.x
    	y = pose.pose.position.y
    	z = pose.pose.position.z
    	
    	return x, y, z

    def detect_calibration_marker(self):
        """
        Function to detect the marker on
        :return:
        """
        
        self.marker_center_x = None
        self.marker_center_y = None
        self.marker_depth = None
        
        # View only the yellow colours in the image
        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_RGB2HSV)
        lower = np.array([20, 100, 100], dtype=np.uint8)
        upper = np.array([100, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        yellow_img = cv2.bitwise_and(self.rgb_img, self.rgb_img, mask=mask)

        # Erode the image a few times in order to separate close objects
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        eroded_yellow_img = cv2.erode(yellow_img, element, iterations=2)

        gray_image = cv2.cvtColor(eroded_yellow_img, cv2.COLOR_BGR2GRAY)

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
            self.marker_box = contours[max_x]
            cv2.rectangle(self.rgb_img, (x, y), ((x+w), (y+h)), (255, 0, 127), thickness=5, lineType=8, shift=0)
        except:
            return False
           

        # Find the Centre of the largest box
        self.marker_center_x = int(x + (0.5 * w))
        self.marker_center_y = int(y + (0.5 * h))
        
        # Using the center of the marker, find the depth value
        self.marker_depth = self.depth_img[self.marker_center_x][self.marker_center_y]
        
        # Check if the function returned valid data and return the answer
        if self.marker_center_x or self.marker_center_y or self.marker_depth is not None:
            return True
        else:
            return False

    def create_pose_target(self, Ww, Wx, Wy, Wz, x, y, z):
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

		
    def manipulate_orange_can(self):
		"""
		Function to find and manipulate an orange can
		"""
		self.find_can_location()
	
    def find_can_location(self):
		"""
		Function to search the RGB image for an orange can, and then return its co-ordinates
		"""
		
		while raw_input("Enter: ") != "g":
			waiting = None
			
		points_detected = False

		while not points_detected:
			if (self.rgb_img is not None) and (self.depth_img is not None):
			    try:
			        points_detected = self.detect_calibration_marker()
			    except:
			        pass           
		self.marker_depth	                
		kinect_point = np.array([self.marker_center_x, self.marker_center_y, self.marker_depth])
		
		print kinect_point
		x, y, z = self.calculate_translated_point(kinect_point)
		print x 
		print y
		print z
		pose_target = self.create_pose_target(0.0977083761873,		# Ww
                                              0.714003795927,		# Wx
                                              -0.00449042997044,	# Wy
                                              0.693275910921,		# Wz
								 			  x, y, z)
								 
		self.right_arm.set_goal_tolerance(0.0001)
		self.right_arm.set_planner_id("RRTConnectkConfigDefault")
		self.right_arm.set_pose_target(pose_target)
		right_arm_plan = self.right_arm.plan()
		self.right_arm.go()
		

    def calculate_translated_point(self, kinect_point):
    	"""
    	Function to return a set of co-ordinates given Kinect co-ordinates and an affine transform
    	"""
    	
    	kin = kinect_point.reshape(3, -1)
    	new_row_k = [1]
    	
    	k_point = np.vstack([kin, new_row_k])
    	
    	outD = self.affine_transform.dot(k_point)
    	    	
    	return float(outD[0]), float(outD[1]), float(outD[2])

def main():
    """
    Main control for operations
    :return:
    """

    # Initialise the Control class to complete all operations on Baxter
    robot = Control()

    # Start the calibration process to link Baxter and Kinect Coordinates
    robot.calibrate_main()
    
    # Search for orange can and move to that location
    robot.manipulate_orange_can()

    # Keep the topics updating
    rospy.spin()

if __name__ == '__main__':
    main()
