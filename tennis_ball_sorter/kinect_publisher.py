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
        crop_img = image

        # Crop the image for the recognition box
        crop_img = image[140:300, 240:400]
        cv2.imshow('cropped image', crop_img)

        # Add a red rectangle, showing the copped region
        cv2.rectangle(image, (240, 140), (400, 300), (0, 0, 255), 2)
        cv2.imshow('RGB image', image)

        k = cv2.waitKey(5) & 0xFF
        if k ==27:
            break

    cv2.destroyAllWindows()
