import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Depth_Image:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/depth/image_raw", Image, self.callback)
        rospy.init_node('image_converter', anonymous=True)
        rospy.spin()

    def callback(self, data):
        """
        Use the data received from the topic.
        Convert to CV image and then perform thresholding to identify objects
        :param data:
        :return:
        """
        # Convert the received image to a CV image and normalise it to grey scale
        depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        depth_array = np.array(depth_image, dtype=np.float32)
        #depth_array = cv2.cvtColor(depth_array, cv2.COLOR_GRAY2BGR)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        # Perform thresholding on the image to remove all objects behind a plain
        ret, bin_img = cv2.threshold(depth_array, 0.3, 1, cv2.THRESH_BINARY)

        # Detect any objects on the plain
        #contours, hierachy = cv2.findContours(bin_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(bin_img, contours, -1, (0,255,0), 3)

        # Show the binary image
        cv2.imshow("Depth", bin_img)
        cv2.waitKey(3)


def main():
    depth = Depth_Image()

if __name__ == '__main__':
    main()
