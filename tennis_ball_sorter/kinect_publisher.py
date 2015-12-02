import cv2
import freenect

from PIL import Image
from numpy import *
import numpy as np


def get_kinect_rgb():
    """
    Gets the rgb feed from the Kinect
    :return: Returns Array, the RGB image
    """
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array

def pca(frame):
    # Get dimensions
    num_data, dim = frame.shape

    # Centre Data
    mean_frame = frame.mean(axis=0)
    for i in range(num_data):
        frame[i] -= mean_frame

    # Compute the covariance matrix
    cov_m = dot(frame, frame.T)

    # Get Eigenvalues and Eigenvectors
    eig_v, eig_Ve = linalg.eigh(cov_m)

    tmp = dot(frame.T, eig_Ve).T
    V = tmp[::-1]
    S = sqrt(e)[::-1]

    return V, S, mean_frame

if __name__ == '__main__':
    while 1:

        # Display the image and loop
        image = get_kinect_rgb()
        crop_img = image

        # Crop the image for the recognition box
        crop_img = crop_img[140:300, 240:400]
        cv2.imshow('cropped image', crop_img)

        # Add a red rectangle, showing the copped region
        cv2.rectangle(image, (240, 140), (400, 300), (0, 0, 255), 2)
        cv2.imshow('RGB image', image)

        # Perform PCA

        #matrix = np.array(crop_img)
        matrix = crop_img.flatten()
        matrix = np.reshape(matrix, (-1, 3))
        V, S, m_mean = pca(matrix)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()
