import sys
import rospy
import cv2
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Object_Detection:
    def __init__(self):
        self.rgb_img = None
        self.bridge = CvBridge()
        rgb_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.callback_rgb)
        depth_sub = rospy.Subscriber("camera/depth/image", Image, self.callback_depth)

        rospy.init_node('depth_viewer', anonymous=True)
        rospy.spin()

    def callback_rgb(self, data):
        """
        Function to save image to a variable in the class
        :param data:
        :return:
        """
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def callback_depth(self, data):
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

        for x in range(0, len(contours)):
            x, y, w, h = cv2.boundingRect(contours[x])
            cv2.rectangle(con_img, (x, y), ((x+w), (y+h)), (255, 0, 127), thickness=5, lineType=8, shift=0)

        # Show the colour images of the objects
        self.show_colour(contours)

        # Show the Depth image and objects images
        cv2.imshow('Contours', con_img)
        cv2.imshow("Depth", bin_img)
        cv2.waitKey(3)

    def show_colour(self, cnt):
        """
        Use the objects found to show them in colour
        :return:
        """
        # Go through each rectangle and display the rgb
        length = len(cnt)

        # Create an array of size the amount of rectangles
        crop_rgb = []
        for i in range(0, length):
            crop_rgb.append(1)

        for x in range(0, length):
            x, y, w, h = cv2.boundingRect(cnt[x])

            # Try to crop the rgb image for each box
            try:
                crop_rgb[x] = self.rgb_img[y:y+h, x:x+w]
            except:
                pass

        for x in range(0, length):
            name = "Cropped " + str(x)
            cv2.imshow(name, crop_rgb[x])
        cv2.waitKey(3)

def main():
    objects = Object_Detection()

if __name__ == '__main__':
    main()
