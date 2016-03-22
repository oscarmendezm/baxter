__author__ = 'Rhys Bryant'

# Imports
import numpy as np
import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from baxter_interface import Navigator
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from numpy import *
from math import sqrt
import scipy
import math
import sys
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor

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
        
        # Setup the table in the scene
        scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene)
        table_id = 'table'
        scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_ground = -0.2
        table_size = [0.7, 1.4, 0.1]
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.robot.get_planning_frame()
        table_pose.pose.position.x = 0.7
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        
        self.sendColors()
        

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
        
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True

        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

    def callback_rgb(self, data):
        """
        Function to handle the arrival of RGB data
        :param data:
        :return:
        """

        # Convert the data to a usable format
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        try:
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
		    cv2.rectangle(self.rgb_img, (x, y), ((x+w), (y+h)), (255, 0, 127), thickness=5, 							lineType=8, shift=0)
        except:
        	pass
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
                    if (self.rgb_img is not None) and (self.depth_img is not None):
                        try:
                            points_detected = self.detect_calibration_marker()
                            if self.marker_depth == 0.0 or self.marker_depth > 1:
                                points_detected = False
                        except:
                            points_detected = False
                        if points_detected:
                            baxter_points.append([x, y, z])
                            kinect_points.append([float(self.marker_center_x),
                            					 float(self.marker_center_y), 
                            					 float(self.marker_depth)])
                            print "Kinect: " + str(kinect_points[point])
                            print "Baxter: " + str(baxter_points[point])
                            point += 1
                            time.sleep(1)
                points_detected = False
                
        output_log = open("output.txt", "w")
        output_log.write("Baxter Points\n")
        output_log.write(str(baxter_points))
        output_log.write("\n\n\nKinect Points\n")
        output_log.write(str(kinect_points))  
            
        kinect_points = np.asmatrix(kinect_points)
        baxter_points = np.asmatrix(baxter_points)
        
        self.R, self.t = self.calculate_transform(kinect_points, baxter_points)
        
        
    def calculate_transform(self, K, B):
        # Ransac parameters
		ransac_iterations = 5000  # Number of iterations
		n_samples = len(K)
		best_rmse = None
		best_R = None
		best_t = None

		print "Number of Baxter points: "
		print len(B)
		print "Number of Kinect points: "
		print len(K)

		for iteration in range(ransac_iterations):

		    # Pick 2 random points from each dataset
		    n = int(n_samples * 0.8)  # Test on 80% of the data

		    k_points = np.arange(K.shape[0])
		    np.random.shuffle(k_points)

		    sample = k_points[:n]

		    sample_k = K[sample, :]
		    sample_b = B[sample, :]

		    # Find the rotation and translation for these sample points
		    ret_R, ret_t = self.rigid_transform_3D(sample_k, sample_b)

		    # Test the calculated transform and rotation
		    rmse = self.test_point(ret_R, ret_t, K, B)

		    if (best_rmse is None) or (rmse < best_rmse):
		        best_rmse = rmse
		        best_R = ret_R
		        best_t = ret_t

		print "\n\nBest RMSE: "
		print best_rmse
		print "\nBest Rotation:"
		print best_R
		print "\nBest translation:"
		print best_t
		
		return best_R, best_t


    def rigid_transform_3D(self, A, B):
		assert len(A) == len(B)

		N = A.shape[0]  # total points

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
		    Vt[2, :] *= -1
		    R = Vt.T * U.T

		t = -R * centroid_A.T + centroid_B.T

		return R, t


    def test_point(self, ret_R, ret_t, K, B):
    
    	n = len(K)
    	
    	K2 = (ret_R * K.T) + tile(ret_t, (1, n))
    	K2 = K2.T
    	
    	# Find the error
    	err = K2 - B
    	
    	err = multiply(err, err)
    	err = sum(err)
    	rmse = sqrt(err / n)
    	
    	return rmse
        
        

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
		

    def calculate_translated_point(self, K):
    	"""
    	Function to return a set of co-ordinates given Kinect co-ordinates and an affine transform
    	"""
    	n = len(K)
    	
    	B = (self.R * K.T) + tile(self.t, (1, n))
    	B = K2.T
    	
    	return float(B[0]), float(B[1]), floatB([2])
    	
    def command(self):
    	while 1:
    		comm = raw_input("Enter Command: ")
    		
    		if comm is "c":
    			# Recalibrate robot
    			self.calibrate_main()
    			
    		if comm is "m":
    			# Move to object
    			self.find_can_location()

def main():
    """
    Main control for operations
    :return:
    """

    # Initialise the Control class to complete all operations on Baxter
    robot = Control()

    # Start the calibration process to link Baxter and Kinect Coordinates
    robot.calibrate_main()
    
    robot.command()
    
    # Search for orange can and move to that location
    robot.manipulate_orange_can()

    # Keep the topics updating
    rospy.spin()

if __name__ == '__main__':
    main()
