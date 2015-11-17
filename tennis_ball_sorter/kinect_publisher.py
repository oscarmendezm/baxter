__author__ = 'rhys'
import argparse
import math
import random
import cv2
import freenect
import rospy
import numpy as np

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

def get_kinect_depth():
    """
    Get the depth image from the kinect
    """
    array, _ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

def kinect_publisher():
    """
    Publisher for the kinect data
    """
    depth_image = get_kinect_depth()
    pub = rospy.Publisher('kinect_depth', uint8, queue_size=10)
    rospy.init_node('kinect')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(depth_image)
        r.sleep()


if __name__ == '__main__':
    kinect_publisher()