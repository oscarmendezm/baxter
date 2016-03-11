from numpy import *
from math import sqrt
import numpy as np

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

A = np.array([[327.0, 173.0, 0.17753486335277557],
              [329.0, 172.0, 0.1824141889810562],
              [216.0, 186.0, 0.1738012582063675]])

A_test = np.array([235.0, 241.0, 0.12045250087976456])

B = np.array([[0.6753705484521056, 0.12884575086417685, 0.3940130838071492],
              [0.6715220912899744, 0.13081831452501663, 0.39403685951962963],
              [0.8773700275772437, 0.26311589642153776, 0.3994720214721082]])

B_test = np.array([0.8243976131849572, 0.3406685123172906, 0.3191293743979707])


def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]  # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)

    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    return R, t

# Test with random data

# number of points
n = len(A)

#A = mat(random.rand(n,3))
#B = R*A.T + tile(t, (1, n))
#B = B.T

# recover the transformation
ret_R, ret_t = rigid_transform_3D(A, B)

A2 = (ret_R*A.T) + tile(ret_t, (1, n))
A2 = A2.T

# Find the error
err = A2 - B

err = multiply(err, err)
err = sum(err)
rmse = sqrt(err/n)

print "Points A"
print A
print ""

print "Points B"
print B
print ""

print "Rotation"
print ret_R
print ""

print "Translation"
print ret_t
print ""

print "RMSE:", rmse
print "If RMSE is near zero, the function is correct!"
