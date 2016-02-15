import sys
import rospy
import cv2
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Object_Detection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/depth/image", Image, self.callback)
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

        # Pass the image to to object detection function
        self.object_detection(depth_array)

    def object_detection(self, depth_array):
        """
        Function to detect objects from the depth image given
        :return:
        """

        # Perform thresholding on the image to remove all objects behind a plain
        ret, bin_img = cv2.threshold(depth_array, 0.3, 1, cv2.THRESH_BINARY_INV)

        # Erode the image a few times in order to separate close objects
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        err_img = cv2.erode(bin_img, element, iterations=20)

        # Create a new array of type uint8 for the findContours function
        con_img = np.array(err_img, dtype=np.uint8)

        # Find the contours of the image and then draw them on
        contours, hierarchy = cv2.findContours(con_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(con_img, contours, -1, (128, 255, 0), 3)

        # Use the contours to draw a rectangle around the object
        for x in range(0, len(contours)):
            x, y, w, h = cv2.boundingRect(contours[x])
            cv2.rectangle(con_img, (x, y), ((x+w), (y+h)), (255, 0, 127), thickness=5, lineType=8, shift=0)

        # Show the produced images
        cv2.imshow('Contours', con_img)
        cv2.imshow("Depth", bin_img)
        cv2.waitKey(3)


def main():
    objects = Object_Detection()

if __name__ == '__main__':
    main()
