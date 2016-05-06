__author__ = 'Rhys Bryant'

# Imports
import numpy as np
import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import Image, PointCloud2, PointField, Range
import sensor_msgs.point_cloud2 as pc2
from baxter_interface import Navigator
import baxter_interface
import sys
import copy
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
import moveit_msgs.msg
import geometry_msgs.msg
import time
import sensor_msgs.point_cloud2 as pc2
import math
from numpy import *
from math import sqrt
import scipy
import math
import sys


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
       
        
        # Setup grippers
        self.right_gripper = baxter_interface.Gripper('right')
        
        # Setup the table in the scene
        scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene)
        table_id = 'table'
        scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_ground = -0.2
        table_size = [2.0, 2.6, 0.1]
        
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
        print "============ Right Pose:"
        print self.right_arm.get_current_pose()
        
        print "============ Left Pose:"
        print self.left_arm.get_current_pose()

        # Setup the subscribers and publishers
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback_rgb)
        self.points_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2,  self.callback_points) 
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                           moveit_msgs.msg.DisplayTrajectory,
                                                           queue_size=5)
        self.screen_pub = rospy.Publisher('robot/xdisplay', Image, latch=True, queue_size=10)
        
        self.right_hand_range_pub = rospy.Subscriber("/robot/range/right_hand_range/state", Range, self.callback_range, queue_size=1)
        
        self.left_cam_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.callback_left_hand)
        
        
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
        
    def callback_range(self, data):
        """
        Store the current range of the IR right hand sensor
        """
        
        self.right_hand_range = data
        
    def callback_left_hand(self,data):
        """
        Callback to handle image from left hand camera
        """
        
         # Convert the data to a usable format
        self.left_hand_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Use the left hand camera to identify colour
        cv2.imshow("left", self.left_hand_img)
        cv2.waitKey(2)

    def callback_points(self, data):
		"""
		Function to handle the arrival of pointcloud data
		"""
		
		cloud_msg = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=False)
		cloud_data = []
		for p in cloud_msg:
		    cloud_data.append([p[0],p[1],p[2]])
		cloud_data = np.array(cloud_data)
		self.cloud2 = np.reshape(cloud_data, [640, 480,3], order='F')
            		
    def callback_rgb(self, data):
        """
        Function to handle the arrival of RGB data
        :param data:
        :return:
        """

        # Convert the data to a usable format
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        try:
            self.detect_calibration_marker()
        except:
        	pass
        #Show the Kinect feed on Baxters screen
        msg = self.bridge.cv2_to_imgmsg(self.rgb_img, encoding="bgr8")
        self.screen_pub.publish(msg)

    def calibrate_main(self):
        """
        Function to complete the calibration between the Kinect and Baxter coordinate frames
        :return:
        """
        kinect_points = []
        baxter_points = []
        
        # List of valid Poses that will be run through during calibration
        pose_list = [[0.871794597756 , -0.248818595343 , -0.0703537153694 , 0.0709033656564 , -0.67287546428 , -0.612105858324 , -0.409313743372],
                     [0.641311970626 , -0.323393156065 , -0.0856656599984 , 0.252514120715 , -0.665067201807 , -0.624845952172 , -0.321698262313],
                     [0.882478506755 , -0.458429092769 , -0.0329601477171 , 0.00454131269522 , 0.66343558665 , 0.630680036453 , 0.402585755417],
                     [0.653324978053 , -0.57065169899 , -0.0636510147356 , 0.189341728715 , -0.670697410609 , -0.625073515754 , -0.351564777918],
                     [0.792536422455 , -0.461141772911 , 0.151155666279 , 0.191510585381 , -0.678319372104 , -0.640623362569 , -0.3046444361],
                     [0.864561704541 , -0.244390218641 , 0.0356764001488 , 0.127480789018 , -0.745281465442 , -0.574567498172 , -0.313331096038],
                     [0.534117905998 , -0.385219821222 , -0.0682428805818 , 0.149001376111 , -0.721035467481 , -0.614467107487 , -0.283437150649],
                     [0.595110030028 , -0.607734029339 , -0.0504057589407 , 0.172375088124 , -0.672435220131 , -0.643814641601 , -0.321901244145],
                     [0.910659902008 , -0.545778845863 , 0.11112090385 , 0.0272829781341 , 0.674165526954 , 0.601017519466 , 0.428409176683],
                     [1.03232068349 , -0.380814059473 , 0.199888991309 , 0.16329028014 , 0.595868911694 , 0.653820947707 , 0.436800518347],
                     [0.834797013481 , -0.425110243102 , 0.254223720424 , 0.127500167943 , -0.728321208282 , -0.646482200762 , -0.188023107194],
                     [0.794353486567 , -0.0649689611985 , -0.0492560083407 , 0.106013510238 , -0.684085162112 , -0.649613770496 , -0.314309681376],
                     [1.00412602536 , -0.193462891347 , -0.0572885809914 , 0.214668403096 , 0.702578440383 , 0.534933005577 , 0.417310066216],
                     [0.936189754067 , -0.393550498843 , -0.0374509236726 , 0.10338765408 , 0.66512261103 , 0.685644314329 , 0.277154793406],
                     [0.789472733702 , -0.408308856676 , -0.0505902183821 , 0.126946586331 , -0.684894506907 , -0.699722060748 , -0.158723395661],
                     [0.951903404203 , -0.520888609266 , -0.00500490075597 , 0.26446665775 , 0.57638169218 , 0.585804863938 , 0.504652547051],
                     [1.0224556916 , -0.56111551834 , 0.267322174804 , 0.327467167745 , 0.535077747118 , 0.451485108167 , 0.634522068719],
                     [0.848554648353 , -0.252851091969 , 0.264050755332 , 0.0565261344661 , 0.501515422731 , 0.639532439371 , 0.579901143194],
                     [0.691841399689 , -0.4431903645 , 0.292483605567 , 0.121986180375 , 0.613500196931 , 0.634909676699 , 0.453460673705],
                     [0.770529385415 , -0.19605390981 , 0.231314740707 , 0.118059117257 , 0.679305559193 , 0.608756331992 , 0.39245602345],
                     [0.47084994586 , -0.115791033839 , 0.0666746542984 , 0.113377564059 , -0.704658259364 , -0.662794583396 , -0.226507407603],
                     [0.841840360521 , -0.101902127197 , 0.0189318240062 , 0.0528098795793 , 0.620237169369 , 0.673702003001 , 0.398299612734],
                     [1.02927964226 , -0.118379651313 , 0.179498383243 , 0.134906262355 , 0.509740991367 , 0.56469775666 , 0.634886498298],
                     [0.74521248921 , -0.450054538215 , -0.059636337089 , 0.0133281802834 , -0.684511286212 , -0.50462816592 , -0.525943982587],
                     [0.913962770494 , -0.223234172205 , -0.0609746726569 , 0.0476244102278 , 0.699548491557 , 0.459334184752 , 0.545321859299],
                     [0.711562161709 , -0.3721505086 , -0.059806749837 , 0.0828444262343 , -0.633519107834 , -0.600801837829 , -0.480445098541],
                     [0.867791211141 , -0.621784550827 , 0.00445348406726 , 0.216985107832 , 0.614132345141 , 0.557714099648 , 0.514503555561],
                     [0.685707053452 , -0.603494295009 , -0.0288872251087 , 0.0736670286497 , 0.766931086241 , 0.429891352565 , 0.470726356641],
                     [0.906298007365 , -0.315264371868 , -0.0091149374359 , 0.0776119552237 , 0.728645586407 , 0.421536513639 , 0.53419000504],
                     [0.757551364849 , -0.452573896174 , 0.098654687927 , 0.151719910166 , 0.719467654038 , 0.604154979447 , 0.307154886762],
                     [0.916172353359 , -0.325594379937 , -0.00102886587335 , 0.0964633080124 , 0.710001469468 , 0.447359372933 , 0.535221762459]]                    
                     
                     
        
        points_detected = False
        
        point = 0
        
        print "\n\nStaring Calibration:"
        
        # Loop through list of poses and move to each
        for pose in pose_list:

            self.move_to_calibration_pose(pose)
            # Wait for rgb to catch up
            time.sleep(2)
            print "Number of points collected: ", len(kinect_points)
            while not points_detected:
                Bx, By, Bz, Ww, Wx, Wy, Wz = self.return_current_pose("right")
                if (self.rgb_img is not None):
                    try:
                        points_detected, Kx, Ky, Kz = self.get_marker()
                    except:
                        points_detected = False
                        print "Exception"
                    if points_detected:
                        #print Bx, ',', By, ',', Bz, ',', Ww, ',', Wx, ',', Wy, ',', Wz
                        baxter_points.append([Bx, By, Bz])
                        kinect_points.append([Kx, Ky, Kz])
                        print "Kinect: " + str(kinect_points[point])
                        print "Baxter: " + str(baxter_points[point])
                        point += 1
                        time.sleep(2)
            points_detected = False
            
        kinect_points = np.asmatrix(kinect_points)
        baxter_points = np.asmatrix(baxter_points)
        
        time.sleep(1)
           
        self.R, self.t = self.calculate_transform(kinect_points, baxter_points)
        
        print "\nTesting Result: \n"
        print "Original Baxter point: \n" + str(baxter_points[0])
        cal_bax_point = self.calculate_translated_point(kinect_points[0])
        print "\n\nCalculated Baxter point: \n" + str(cal_bax_point)
        
    def calculate_transform(self, K, B):
        # Ransac parameters
		ransac_iterations = 4000  # Number of iterations
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
        
    def move_to_calibration_pose(self, pose):
        """
        Move Baxters right arm to the given calibration pose
        """
        x = pose[0]
        y = pose[1]
        z = pose[2]
        Ww = pose[3]
        Wx = pose[4]
        Wy = pose[5]
        Wz = pose[6]
        
        pose_target = self.create_pose_target(Ww,           		# Ww
                                              Wx,           		# Wx
                                              Wy,               	# Wy
                                              Wz,		            # Wz
                                              x, 		            # X
                                              y, 					# Y
                                              z)					# Z
                                              
        self.right_arm.set_goal_tolerance(0.01);
        self.right_arm.set_planner_id("RRTConnectkConfigDefault");
        self.right_arm.set_pose_target(pose_target)
        right_arm_plan = self.left_arm.plan()
        self.right_arm.go()
        

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
    	Ww = pose.pose.orientation.w
    	Wx = pose.pose.orientation.x
    	Wy = pose.pose.orientation.y
    	Wz = pose.pose.orientation.z
    	
    	return x, y, z, Ww, Wx, Wy, Wz
    	
    def get_marker(self):
    
        box_center_x, box_center_y = self.detect_calibration_marker()
    
        print "Marker Centre (PX): ", box_center_x, box_center_y
        
        print "Waiting to get point..."
        time.sleep(1)
        
        # Using the center of the marker, find the depth value
        self.marker_center_x = self.cloud2[box_center_x][box_center_y][0]
        self.marker_center_y = self.cloud2[box_center_x][box_center_y][1]
        self.marker_center_z = self.cloud2[box_center_x][box_center_y][2]
     

        if math.isnan(self.marker_center_x) or math.isnan(self.marker_center_y) or math.isnan(self.marker_center_z):
            return False
        
        else:
            values_list = [self.marker_center_x, self.marker_center_y, self.marker_center_z]
            print "Got Value:"
            return True, values_list[0], values_list[1], values_list[2] 
        	
        	
    def detect_calibration_marker(self):
        """
        Function to detect the marker on
        :return:
        """
        
        self.marker_center_x = None
        self.marker_center_y = None
        
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
        box_center_x = int(x + (0.5 * w))
        box_center_y = int(y + (0.5 * h))
        return box_center_x, box_center_y
        
        

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
			if (self.rgb_img is not None):
				try:
				    points_detected, Kx, Ky, Kz = self.get_marker()
				except:
				    pass           
		            
		kinect_point = np.array([Kx, Ky, Kz])
	
		print kinect_point
		new_B = self.calculate_translated_point(kinect_point)
	
		new_B = np.asarray(new_B)

		x = new_B[0][0]
		self.y = new_B[0][1]
		z = new_B[0][2]

		pose_target = self.create_pose_target(0.705642809911,		    # Ww
	                                          -0.0229930939041,		    # Wx
	                                          0.708193955206,	        # Wy
	                                          -0.000929657640434,		# Wz
								 			  0.757849381922,            # X
								 			  self.y,                         # Y
								 			  -0.0288319119786)          # Z

								 
		self.right_arm.set_goal_tolerance(0.0001)
		self.right_arm.set_planner_id("RRTConnectkConfigDefault")
		self.right_arm.set_pose_target(pose_target)
		right_arm_plan = self.right_arm.plan()
		self.right_arm.go()
	
		

    def calculate_translated_point(self, K):
    	"""
    	Function to return a set of co-ordinates given Kinect co-ordinates and an affine transform
    	"""
    	K = np.asmatrix(K)
    	n = len(K)
    	
    	K2 = (self.R * K.T) + tile(self.t, (1, n))
    	B = K2.T
    	
    	print 'Translated Point :'
    	print B
    	return B
    	
    def grip_object(self):
        """
        Close the grippers around any sized object
        """
        new_x = 0.757849381922
        
        while float(self.right_hand_range.range) > 0.1:
        
            if float(self.right_hand_range.range) < 10:
            
                print float(self.right_hand_range.range)
                new_x += 0.01
                
                print float(self.right_hand_range.range)
            
                pose_target = self.create_pose_target(0.705642809911,		    # Ww
	                                                  -0.0229930939041,		    # Wx
	                                                  0.708193955206,	        # Wy
	                                                  -0.000929657640434,		# Wz
								         			  new_x,                    # X
								         			  self.y,                   # Y
								         			  -0.0288319119786)         # Z
								         			  
                self.right_arm.set_goal_tolerance(0.0001)
                self.right_arm.set_planner_id("RRTConnectkConfigDefault")
                self.right_arm.set_pose_target(pose_target)
                right_arm_plan = self.right_arm.plan()
                self.right_arm.go()
            
        print float(self.right_hand_range.range)
        
        # Calibrate the gripper
        print "calibrating"
        self.right_gripper.calibrate()
        
        print "gripping"
        # Close the gripper
        self.right_gripper.close()
        
    def drop_object(self):
        """
        Open the grippers
        """
      
        print "Releasing"
        # Open the gripper
        self.right_gripper.open()
        
    def identify_brand(self):
        """
        Identify the brand of the can using its colour
        """
        
        # Move the can so baxters left arm can see it
        
        pose_target = self.create_pose_target(0.459579094092,		    # Ww
	                                          -0.526618773291,		    # Wx
	                                          0.526858854386,   	    # Wy
	                                          0.483610867792,   		# Wz
								 			  0.525552632416,            # X
								 			  -0.179808145256,           # Y
								 			  0.559583765188)          # Z
								 			  
        self.right_arm.set_goal_tolerance(0.0001)
        self.right_arm.set_planner_id("RRTConnectkConfigDefault")
        self.right_arm.set_pose_target(pose_target)
        right_arm_plan = self.right_arm.plan()
        self.right_arm.go()
        
        pose_target = self.create_pose_target(0.477334952959,		    # Ww
	                                          0.529770117458,		    # Wx
	                                          0.437533990701,   	    # Wy
	                                          -0.547776388971,   		# Wz
								 			  0.545503941288,            # X
								 			  0.136721002146,           # Y
								 			  0.517426262626)          # Z
								 			  
        self.left_arm.set_goal_tolerance(0.0001)
        self.left_arm.set_planner_id("RRTConnectkConfigDefault")
        self.left_arm.set_pose_target(pose_target)
        left_arm_plan = self.left_arm.plan()
        self.left_arm.go()
        
        # wait for image to catch up.
        time.sleep(2)
        
        # Crop the image region
        cropped = self.left_hand_img[80:470, 210:400]
        
        mean = np.mean(cropped)
        print mean
        
        
    def move_can_to_bin(self):
        """
        Move the gripped object to a pre_determined location
        """
        
        pose_target = self.create_pose_target(0.595552795507,		    # Ww
	                                          -0.402736166486,		    # Wx
	                                          0.606073515679,	    # Wy
	                                          0.340287145747,		    # Wz
								 			  0.771553488123,         # X
								 			  0.318602280105,           # Y
								 			  0.263525150839)            # Z
								 			  
        self.right_arm.set_goal_tolerance(0.0001)
        self.right_arm.set_planner_id("RRTConnectkConfigDefault")
        self.right_arm.set_pose_target(pose_target)
        right_arm_plan = self.right_arm.plan()
        self.right_arm.go()
        
        self.drop_object()
        
    def move_right_to_neutral(self):
        """
        Move the right arm to it's neutral position
        """
        
        pose_target = self.create_pose_target(0.0194490701404,		    # Ww
	                                          0.0532674104646,		    # Wx
	                                          0.997887473793,	    # Wy
	                                          0.0317002570828,		    # Wz
								 			  0.557041277551,         # X
								 			  -0.723330201644,           # Y
								 			  0.262966200216)            # Z
								 			  
        self.right_arm.set_goal_tolerance(0.0001)
        self.right_arm.set_planner_id("RRTConnectkConfigDefault")
        self.right_arm.set_pose_target(pose_target)
        right_arm_plan = self.right_arm.plan()
        self.right_arm.go()
        
        
        

def main():
    """
    Main control for operations
    :return:
    """

    # Initialise the Control class to complete all operations on Baxter
    robot = Control()
    
    # Set the transform matrices
    robot.R = [[-0.97120955, -0.08497271, 0.22255705],
              [-0.23768779, 0.40844356, -0.88129358],
              [-0.01601608, -0.90881984, -0.41688126]]
    robot.t = [[0.50490536], [1.13662038], [0.74441346]]
    
    while 1:
    
        # Wait for keyboard to determine method
        waiting = True
        
        robot.move_right_to_neutral()
        robot.move_to_remove()
        
        while waiting == True:
            print "c -> Calibrate"
            print "m -> Load current transform and move to can location"
            usr_input = raw_input("Enter: ")
            if usr_input == 'c':
				
				    # Start the calibration process to link Baxter and Kinect Coordinates
				    robot.calibrate_main()
				    
				    # Move the right arm out of the way again
				    robot.move_right_to_neutral()
				    robot.move_to_remove()
            
            elif usr_input == 'm':
                waiting = False
                                
                # Move left arm out of the way before gripping begins
                robot.move_to_remove()
        
                # Search for orange can and move to that location
                robot.manipulate_orange_can()
        
        # Wait for keyboard to determine next method
        waiting = True
        
        print "g -> Grip item"
        print "Any other Key to return to menu"
        
        while waiting == True:
            usr_input = raw_input("Enter: ")
            if usr_input == 'g':
				    waiting = False  
				    robot.grip_object() 
				    
				    waiting = True
				    
				    print "m -> Move Item"
				    print "d -> Drop Item"
				    
				    while waiting == True:
				        usr_input = raw_input("Enter: ")
				        if usr_input == 'm':
				            waiting = False
				            robot.identify_brand()
				            robot.move_can_to_bin()
				            time.sleep(1)
				            robot.move_right_to_neutral()
				            
				            
				        if usr_input == 'd':
				            waiting = False
				            robot.drop_object()  
				            time.sleep(1)
				            robot.move_right_to_neutral()
				            robot.move_to_remove()
        
            else:
                # Return to prvious menu
                waiting = False
        
        

    # Keep the topics updating
    rospy.spin()

if __name__ == '__main__':
    main()
