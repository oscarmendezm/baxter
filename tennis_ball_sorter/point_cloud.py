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
import moveit_msgs.msg
import geometry_msgs.msg
import time
import sensor_msgs.point_cloud2 as pc2

class Points:
	def __init__(self):
		rospy.init_node('point_viewer', anonymous=True)
		self.points_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback_points)
	
	def callback_points(self, data):
		"""
		Function to handle the arrival of pointcloud data
		"""
		
		cloud = list(pc2.read_points(data, skip_nans=True, field_names=("x", "y")))
		cloud = np.resize(cloud, [640, 480, 2])
		print cloud[100][100]
		
point_cloud = Points()
rospy.spin()
