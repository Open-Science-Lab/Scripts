import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import cv2 as cv
from cv2 import aruco
import math
import tf
from scipy.spatial.transform import Rotation as R
import serial
import time
from camer_I.py import get_tf
from robot_desc.py import MoveGroupPythonIntefaceTutorial

ser=serial.Serial("/dev/ttyACM0",9600,timeout=1)
ser.bytesize=8
ser.parity='N'
ser.stopbits=1

tutorial = MoveGroupPythonIntefaceTutorial()

def gripper(inst):
    if inst=="open":
        time.sleep(2)
        ser.write(b'a')
        time.sleep(5)

    if inst=="close":
        time.sleep(2)
        ser.write(b'c')
        time.sleep(5)

def pick(marker_id):

    move(marker_id)
    gripper("open")
    gripper("close")
    #grip the object

def move(marker_id):
    #find the marker id
    #find the Transform matrix
    final_tf,final_quat=get_tf(marker_id)
    print("Final_TF : ")
    print(final_tf)
    cartesian_plan, fraction = tutorial.plan_cartesian_path(final_tf,final_quat)
    tutorial.display_trajectory(cartesian_plan)
    tutorial.execute_plan(cartesian_plan)    
    #move the EE to the marker

def place(marker_id):
    move(marker_id)
    gripper("open")
    #release the object

def pour(marker_id,angle,time):
    move(marker_id)
    tutorial.pour(angle)
    #rotate the EE to the angle
    #wait for the time
    #return to the original position

