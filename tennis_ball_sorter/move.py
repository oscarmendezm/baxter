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
        table_ground = -0.3
        table_size = [4.0, 4.6, 0.1]
        
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
        pose_list = [[0.967549571713 , -0.243194570899 , -0.0147116101372 , 0.189819515383 , 0.576035463292 , 0.565770228928 , 0.558619499005],
                     [1.00591841119 , 0.0653722802741 , 0.0390316996546 , 0.152757801693 , 0.59959204309 , 0.54160654591 , 0.569049018374],
                     [0.660820491796 , -0.177701888979 , 0.113357076542 , 0.173421623611 , -0.677841472218 , -0.63695170379 , -0.323648584184],
                     [0.905797337361 , -0.331304537267 , 0.0462269328641 , 0.0493525714893 , -0.721181615556 , -0.56377896248 , -0.39951806284],
                     [0.772376384097 , 0.0978054138748 , -0.026135473969 , 0.201319020728 , -0.601787830146 , -0.664505383498 , -0.394657642373],
                     [0.811704350811 , -0.103656896994 , 0.141110073754 , 0.143214065634 , -0.594841017502 , -0.739429664627 , -0.280887284104],
                     [0.755979226564 , -0.117592203468 , 0.150116490599 , 0.268454143496 , -0.711205271974 , -0.541262411561 , -0.359380628007],
                     [0.979268205741 , 0.159483927641 , 0.169687141748 , 0.085900093927 , -0.69086093398 , -0.497660362668 , -0.517364965177],
                     [0.696525868986 , 0.0386367320766 , 0.102296723968 , 0.174107553829 , -0.596925269821 , -0.690000302377 , -0.3704947566],
                     [0.814383547029 , -0.114150948284 , 0.117337537566 , 0.256909164836 , -0.730541492608 , -0.479022752633 , -0.41333280908],
                     [1.07873925387 , -0.0206345594802 , 0.228813657798 , 0.241138278494 , 0.520589892643 , 0.601439839116 , 0.555975371978],
                     [0.809596487214 , -0.186215963667 , 0.0406577192334 , 0.136382316321 , 0.572872574265 , 0.617914774945 , 0.520958931543],
                     [0.583496017246 , 0.0913085281482 , 0.0735311335765 , 0.178854233934 , -0.634977291594 , -0.660956918962 , -0.357702325182],
                     [0.753218240633 , -0.0994920476031 , -0.0329215926621 , 0.0724749338894 , -0.64398059595 , -0.644760658705 , -0.405364119013],
                     [0.990711394828 , -0.286979140017 , 0.0288606526987 , 0.186902642758 , 0.567888846611 , 0.639916747179 , 0.482779677187],
                     [0.971247139794 , 0.238743820558 , 0.242769320113 , 0.159909170901 , 0.384929043639 , 0.660235113347 , 0.624778587604],
                     [0.726572230958 , -0.219190090899 , 0.19089983206 , 0.160700064095 , -0.596639840593 , -0.739102334662 , -0.268186742615],
                     [0.640192523703 , -0.107968230285 , 0.230995078633 , 0.225368689666 , -0.61504059185 , -0.682986068997 , -0.323208993765],
                     [0.885666308204 , 0.179141109904 , 0.167234926829 , 0.0348897624627 , -0.519551753777 , -0.714712882343 , -0.466941297633],
                     [0.741446231388 , -0.0818345763105 , 0.0555464394075 , 0.190399201766 , -0.632430315906 , -0.71040075171 , -0.243127150807],
                     [1.01323608274 , -0.133186233845 , -0.021826983584 , 0.138802425528 , 0.655369312371 , 0.582661815133 , 0.460141456788],
                     [1.03260814353 , 0.0346221999128 , 0.132681545664 , 0.152929969979 , 0.527931656776 , 0.665691023969 , 0.504733643284],
                     [0.784673172817 , 0.230747438213 , 0.0473197447253 , 0.168223152068 , -0.503451364408 , -0.806335531932 , -0.260884466245],
                     [0.780593767564 , -0.17977555903 , 0.0408083386682 , 0.165445612739 , -0.603657068794 , -0.743499201958 , -0.235446021859],
                     [0.958648177818 , -0.363982170786 , 0.223764130815 , 0.0889160271122 , 0.693497085579 , 0.633182445796 , 0.332017654278],
                     [0.892436390819 , 0.102284104689 , 0.0413739748571 , 0.031729435966 , 0.669928169245 , 0.65825398922 , 0.341893516497],
                     [0.870188202725 , -0.0219473395379 , 0.0942454996925 , 0.195686453524 , -0.535399360374 , -0.708099812793 , -0.416712121182],
                     [0.721524918703 , -0.0685114050026 , 0.107947295647 , 0.215929700552 , -0.598127059979 , -0.729024735834 , -0.253261365159],
                     [0.754605545006 , -0.270272463086 , 0.213347148072 , 0.184106655957 , -0.582021981644 , -0.747339452982 , -0.262371671741],
                     [0.921488386934 , -0.146508175922 , -0.0681952879441 , 0.229020339028 , 0.641492396891 , 0.617573303821 , 0.393243440441]]                    
                     
                     
        
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

       
        
        # 203, 134, 107 
        
        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_RGB2HSV)
        #print hsv[350][200]
        #self.rgb_img[350][200] = [255, 255, 255]
        lower = np.array([5, 100, 50], dtype=np.uint8)
        upper = np.array([10, 130, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        yellow_img = cv2.bitwise_and(self.rgb_img, self.rgb_img, mask=mask)
        

        
        #cv2.imshow("ImgHSV", hsv)
        #cv2.imshow("MASK", mask)
        #cv2.waitKey(3)

        # Erode the image a few times in order to separate close objects
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        eroded_yellow_img = cv2.erode(yellow_img, element, iterations=1)

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
		#self.y = new_B[0][1] + 0.097
		z = new_B[0][2]

		pose_target = self.create_pose_target(0.705642809911,		    # Ww
	                                          -0.0229930939041,		    # Wx
	                                          0.708193955206,	        # Wy
	                                          -0.000929657640434,		# Wz
								 			  0.740849381922,            # X
								 			  self.y,                    # Y
								 			  -0.0458319119786)          # Z

								 
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
        #self.right_gripper.open()
        
        new_x = 0.757849381922
        
        print float(self.right_hand_range.range)
        
        while float(self.right_hand_range.range) > 0.1:
        
            if float(self.right_hand_range.range) < 10:
            
                print float(self.right_hand_range.range)
            
                new_x += 0.01
     
            
                pose_target = self.create_pose_target(0.705642809911,		    # Ww
	                                                  -0.0229930939041,		    # Wx
	                                                  0.708193955206,	        # Wy
	                                                  -0.000929657640434,		# Wz
								         			  new_x,                    # X
								         			  self.y,                   # Y
								         			  -0.050319119786)         # Z
								         			  
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
        
        # Move to neutral to avoid hitting screen
        self.move_right_to_neutral()
        
        # Move the can so baxters left arm can see it
        
        pose_target = self.create_pose_target(0.459579094092,		    # Ww
	                                          -0.526618773291,		    # Wx
	                                          0.526858854386,   	    # Wy
	                                          0.483610867792,   		# Wz
								 			  0.525552632416,            # X
								 			  -0.179808145256,           # Y
								 			  0.559583765189)          # Z
								 			  
        self.right_arm.set_goal_tolerance(0.0001)
        self.right_arm.set_planner_id("RRTConnectkConfigDefault")
        self.right_arm.set_pose_target(pose_target)
        right_arm_plan = self.right_arm.plan()
        self.right_arm.go()
        
        
        pose_target = self.create_pose_target(0.477457561703,		    # Ww
	                                          0.538641679605,		    # Wx
	                                          0.496295434842,   	    # Wy
	                                          -0.485376409728,   		# Wz
								 			  0.54700109441,            # X
								 			  0.144138140554,           # Y
								 			  0.525005607012)          # Z
								 			  
        self.left_arm.set_goal_tolerance(0.0001)
        self.left_arm.set_planner_id("RRTConnectkConfigDefault")
        self.left_arm.set_pose_target(pose_target)
        left_arm_plan = self.left_arm.plan()
        self.left_arm.go()
        
        # wait for image to catch up.
        time.sleep(2)
        
        # Crop the image region and work out mean
        cropped = self.left_hand_img[40:440, 290:430]
        cv2.imwrite("cropped.png", cropped)
        mean = np.mean(cropped)
        
        print mean
        
        time.sleep(1)
        
        self.move_to_remove()
        
        time.sleep (1)
        
        if mean > 65:
            self.move_can_to_bin(1)
            
        else:
            self.move_can_to_bin(2)
            
        
    def move_can_to_bin(self, loc):
        """
        Move the gripped object to a pre_determined location
        """
       
        
        # Move can to relevant location
        if loc is 1:
            # Gold Can
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
            
        else:
            # Black/Blue can
            pose_target = self.create_pose_target(0.575874166982,		    # Ww
	                                              0.309178325864,		    # Wx
	                                              0.683713462965,	    # Wy
	                                              -0.324520580517,		    # Wz
								     			  0.6665828906,         # X
								     			  -1.10438377753,           # Y
								     			  0.164512813294)            # Z
								     			  
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
    robot.R = [[-0.98980833,  0.07509671, 0.12099568],
                [-0.08007363,  0.40911359, -0.9089633 ],
                [-0.11776113, -0.90938801, -0.39893077]]

    robot.t = [[ 0.41756282], [ 1.25144796], [ 0.69034295]]
    
    # Set the transform matrices
    #robot.R = [[-0.96073394, 0.27636793, -0.02471969],
                #[-0.16448139, -0.63899495, -0.75141954],
                #[-0.22346402, -0.71784832,  0.65936152],]

    #robot.t = [[ 0.77018528], [ 1.57431638], [-1.21704818]]
    
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
