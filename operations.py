import cv2 as cv
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation as R


def build_tf(rot_matrix,tr_matrix):
    print(rot_matrix[0][0])
    tf =[[rot_matrix[0][0],rot_matrix[0][1],rot_matrix[0][2],tr_matrix[0][0]],[rot_matrix[1][0],rot_matrix[1][1],rot_matrix[1][2],tr_matrix[1][0]],[rot_matrix[2][0],rot_matrix[2][1],rot_matrix[2][2],tr_matrix[2][0]],[0,0,0,1]]
    return tf

def get_pose(tr,qt):
    rot=quaternion_rotation_matrix(qt)
    tf=build_tf(rot,np.transpose(tr))
    return tf

def multiply_mat(mat_a,mat_b):
    mat_c=np.matmul(mat_a,mat_b)
    return mat_c

def quaternion_rotation_matrix(Q):

    # Extract the values from Q
    q0 = Q[3]
    q1 = Q[0]
    q2 = Q[1]
    q3 = Q[2]
     
    r00 = 1 - 2 * (q2 * q2 + q3 * q3)
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 1 - 2 * (q1 * q1 + q3 * q3)
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 1 - 2 * (q1 * q1 + q2 * q2)
     
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    
                            
    return rot_matrix