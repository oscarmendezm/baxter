import cv2 as cv
import Image
from matplotlib.mlab import PCA
import matplotlib.pyplot as plt
import numpy as np


def pca(image):
    """
    Function to perform PCA on a passed in image
    :param image:
    :return:
    """
    # PCA using OpenCV
    # mean, eigenvectors = cv.PCACompute(image, np.mean(image, axis=0).reshape(1, -1))

    # PCA using matplotlib
    results = PCA(image)

    return results

if __name__ == '__main__':

    # Open image as numpy array and flatten
    img_raw = cv.imread('Tennis-Ball2.jpg', 1)  # Open with 1 as flag for colour image
    img_np = img_raw.reshape((-1, 3))

    results = pca(img_np)
    img_Y = results.Y.reshape((194, 260, -1))
    img_a = results.a.reshape((194, 260, -1))


    # Show PCA result and image
    cv.imshow('Tennis_ball_raw', img_raw)
    cv.imshow('Tennis_ball_Y', img_Y)
    cv.imshow('Tennis_ball_a', img_a)
    cv.waitKey(0)
    print "Image"

    # cv.destroyAllWindows()
