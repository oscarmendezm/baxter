__author__ = 'Rhys Bryant'

# Imports
import numpy as np
import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from baxter_interface import Navigator
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
        print self.robot.get_group_names()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.right_arm_navigator = Navigator('right')
        
        print self.right_arm.get_current_pose()
        
        # Setup the table in the scene
        scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene)
        table_id = 'table'
        scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_ground = -0.2
        table_size = [0.7, 1.6, 0.1]
        
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
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback_rgb)
        self.points_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2,  self.callback_points) 
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

    def callback_points(self, data):
		"""
		Function to handle the arrival of pointcloud data
		"""
		
		#cloud = list(pc2.read_points(data, skip_nans=False))
		#cloud = np.resize(cloud, [640, 480, 3])
		#self.cloud = cloud
		
		cloud_msg = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=False)
		cloud_data = []
		for p in cloud_msg:
		    cloud_data.append([p[0],p[1],p[2]])
		cloud_data = np.array(cloud_data)
		self.cloud2 = np.reshape(cloud_data, [640, 480,3], order='F')
		#print self.cloud2.shape
            		
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
        pose_list = [[0.787082536265 , 0.0464219034876 , 0.113832593889 , 0.0902413158339 , -0.699323807464 , -0.579964001839 , -0.407976070134],
                     [0.9203820355 , -0.0729821186959 , 0.109042642348 , 0.203073548259 , -0.669931735536 , -0.61986528851 , -0.354569637475],
                     [0.9176410591 , 0.236163363183 , 0.105185924381 , 0.133509561856 , -0.678203164309 , -0.55311662273 , -0.465056627168],
                     [0.76350579339 , 0.422182771222 , -0.0123084298775 , 0.0680085666316 , -0.634769281773 , -0.587676513534 , -0.497070527414],
                     [0.775824047998 , 0.384138797362 , 0.284389568413 , 0.184226847216 , -0.715274096694 , -0.588940201608 , -0.328013527607],
                     [0.600601637936 , 0.431651264303 , 0.428590173906 , 0.218921719552 , -0.645882662974 , -0.699813998697 , -0.212530547465],
                     [0.516591138013 , 0.416104256118 , 0.541125162777 , 0.262576354547 , -0.691399421877 , -0.55411854129 , -0.382064313511],
                     [0.483131948853 , 0.00118009192429 , 0.497918440567 , 0.253281043624 , -0.758986070501 , -0.547195488701 , -0.245694840952],
                     [0.549003175022 , -0.419982200359 , 0.511696354884 , 0.141002812329 , -0.654664096357 , -0.676898302167 , -0.305518929659],
                     [0.529469057705 , -0.607839161102 , 0.279883892425 , 0.173437135357 , -0.604096535132 , -0.721157192148 , -0.291409060486],
                     [0.485909750256 , -0.498242031097 , 0.00649625484521 , 0.0858643850182 , -0.658568325994 , -0.638571756475 , -0.38876879918],
                     [0.7491075757 , -0.388328948562 , -0.000115268865141 , 0.0931689739491 , -0.66551053709 , -0.646662436881 , -0.360891895229],
                     [0.946636223637 , -0.19432699517 , 0.103586046609 , 0.00330010896101 , 0.691640804844 , 0.533276278925 , 0.487071367144],
                     [0.679994195923 , -0.210525593004 , 0.248048750902 , 0.170143740573 , -0.704139830241 , -0.603049669678 , -0.334019913947],
                     [0.493920342152 , -0.075676206338 , 0.396420645852 , 0.334966178067 , -0.727200936063 , -0.576532713895 , -0.163053635189],
                     [0.652274318314 , -0.033570810864 , 0.143141847077 , 0.140690536779 , -0.719589897534 , -0.60597132622 , -0.308537362449],
                     [0.911040302856 , -0.0174263396378 , 0.00375330369347 , 0.052416693096 , -0.655659344061 , -0.67571580595 , -0.33282347336],
                     [0.727526966104 , -0.201538544521 , -0.0735051015182 , 0.219077541443 , -0.666193924813 , -0.610464866951 , -0.368134936668],
                     [0.719385936049 , 0.331423797465 , -0.0800682535388 , 0.196931104357 , -0.701994058567 , -0.570930584021 , -0.377439730425],
                     [0.908010633512 , 0.20852277378 , -0.0328940279839 , 0.00698933370487 , -0.696489059558 , -0.516879586552 , -0.497684269528],
                     [1.02756200168 , 0.0943452857437 , 0.474060022946 , 0.0958337720433 , -0.663153756165 , -0.421832353445 , -0.610819490035],
                     [0.888779964408 , -0.221033119113 , 0.10215364478 , 0.127120062117 , -0.645639690727 , -0.601487870914 , -0.452992517277],
                     [0.77456558276 , 0.520625244804 , 0.371763671161 , 0.141996073338 , -0.581230124658 , -0.457610746921 , -0.657724153159],
                     [0.589336711251 , 0.381070025288 , 0.300442784482 , 0.31933741268 , -0.711970524784 , -0.556563933495 , -0.285233547524],
                     [0.59317984995 , -0.0540883002897 , 0.310744182076 , 0.289372750853 , -0.694005777143 , -0.642692851518 , -0.146851254553],
                     [0.59317984995 , -0.0540883002897 , 0.310744182076 , 0.289372750853 , -0.694005777143 , -0.642692851518 , -0.146851254553],
                     [0.635221059044 , -0.0529884621621 , 0.0615424082709 , 0.128453076828 , -0.68436490893 , -0.677004851394 , -0.238346197091],
                     [0.736894461595 , -0.359083080285 , 0.0957591406134 , 0.230365370048 , -0.660658834617 , -0.656282894677 , -0.282408326152],
                     [0.856433148149 , 0.253602477922 , 0.0750673498 , 0.129939125028 , -0.662539923503 , -0.604385144691 , -0.422936485099],
                     [0.886592435991 , -0.222630274257 , -0.0165102187008 , 0.0705216181939 , 0.669922257118 , 0.538633527387 , 0.506067973655],
                     [0.627475557154 , -0.225710668586 , -0.0365310479227 , 0.113752502392 , -0.66051150876 , -0.665590392416 , -0.328290031098],
                     [0.980513209608 , -0.271793800941 , -0.033364794179 , 0.194009039007 , 0.661778636808 , 0.500182211874 , 0.523667149603],
                     [1.00831495143 , -0.0252682592022 , -0.0207624126643 , 0.114046144808 , 0.709570559522 , 0.416558928179 , 0.556760053585],
                     [0.590774701724 , 0.0750758571987 , -0.019888576087 , 0.250500571628 , -0.608117357827 , -0.623975011317 , -0.422016502018],
                     [0.763800611378 , -0.291507187017 , -0.0152160490523 , 0.0169800649687 , 0.605627210624 , 0.620202770361 , 0.498272899907],
                     [0.866956022804 , -0.458772059949 , -0.0235058253468 , 0.111344640732 , 0.553438276956 , 0.600254979948 , 0.56657074018],    
                     [0.703509463272 , -0.280908223969 , -0.0597751286364 , 0.119082570328 , -0.691498466215 , -0.522213383312 , -0.484708567034],
                     [0.91345803955 , -0.0493827381664 , -0.0277354797514 , 0.000272934611266 , -0.598020816835 , -0.548201530181 , -0.584676073091]]                    
                     
                     
        
        points_detected = False
        
        point = 0
        
        # Move the left arm out of the way for calibration
        self.move_to_remove()
        
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
                        baxter_points.append([Bx, By, Bz])
                        kinect_points.append([Kx, Ky, Kz])
                        print Bx, ",", By, ",", Bz, ",", Ww, ",", Wx, ",", Wy, ",", Wz
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
		
		while 1:
			while raw_input("Enter: ") != "g":
				waiting = None
			
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
			y = new_B[0][1]
			z = new_B[0][2]

			pose_target = self.create_pose_target(0.705642809911,		    # Ww
		                                          -0.0229930939041,		    # Wx
		                                          0.708193955206,	        # Wy
		                                          -0.000929657640434,		# Wz
									 			  0.627849381922,            # X
									 			  y,                         # Y
									 			  -0.0161319119786)          # Z

									 
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
