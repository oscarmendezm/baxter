__author__ = 'rhys'
import cv2
import freenect

def get_kinect_rgb():
    """
    Gets the rgb feed from the Kinect
    :return: Returns Array, the RGB image
    """
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


if __name__ == '__main__':
    while 1:

        # Display the image and loop
        image = get_kinect_rgb()
        cv2.imshow('RGB image', image)

        k = cv2.waitKey(5) & 0xFF
        if k ==27:
            break

    cv2.destroyAllWindows()
