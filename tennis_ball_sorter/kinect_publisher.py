import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/depth/image", Image, self.callback)

    def callback(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        ret, bin_img = cv2.threshold(depth_array, 0.5, 1, cv2.THRESH_BINARY_INV)

        cv2.imshow("Depth", bin_img)
        cv2.waitKey(3)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
