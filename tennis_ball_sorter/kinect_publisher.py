import sys
import rospy
import cv2
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Depth_Image:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/depth_registered/image", Image, self.callback)
        rospy.init_node('image_converter', anonymous=True)
        rospy.spin()

    def callback(self, data):
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

        # Perform thresholding on the image to remove all objects behind a plain
        ret, bin_img = cv2.threshold(depth_array, 0.3, 1, cv2.THRESH_BINARY_INV)

        # Erode the image a few times in order to separate close objects
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        err_img = cv2.erode(bin_img, element, iterations=10)

        # err_img.astype(cv2.CV_32FC1)

        # im2, contours, hierarchy = cv2.findContours(err_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(err_img, contours, -1, (0, 255, 0), 3)

        # Show the images
        cv2.imshow("Erode", err_img)
        cv2.imshow("Depth", bin_img)
        cv2.waitKey(3)


def main():
    depth = Depth_Image()

if __name__ == '__main__':
    main()
