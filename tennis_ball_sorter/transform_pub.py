#!/usr/bin/env python  
import roslib
import rospy
import tf
import numpy as np 

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    R = np.asmatrix([[-0.90781008, -0.34117207,  0.24389029], [-0.33676882,  0.24645706, -0.90876052], [ 0.24993522, -0.90711661, -0.33863232]])
    t =  np.asmatrix([0.47169096, 1.16026161,  0.65932058])
    C = -1*R.T*t.T

    matrix = [[-0.90781008, -0.34117207,  0.24389029, 0.47169096], [-0.33676882,  0.24645706, -0.90876052, 1.16026161], [ 0.24993522, -0.90711661, -0.33863232, 0.65932058], [0.0, 0.0, 0.0, 1.0]]
    
    print matrix
    trans= tf.transformations.translation_from_matrix(matrix)
    quat = tf.transformations.quaternion_from_matrix(matrix)
    print quat
 
    while not rospy.is_shutdown():
        br.sendTransform((C[0], C[1],C[2]),
                         quat,
                         rospy.Time.now(),
                         "base",
                         "camera_depth_optical_frame")
        rate.sleep()
