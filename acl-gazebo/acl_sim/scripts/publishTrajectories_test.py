#!/usr/bin/env python

#Mahathi Bhargavapuri, June 2022

#This generates trajectories using ETH's library: https://github.com/markwmuller/RapidQuadrocopterTrajectories.git

#make sure to run this node AFTER roslaunch firefly demo and global mapper

from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
import roslib
import rospy
import math
from moveit_msgs.msg import MotionPlanRequest
import numpy as np
from numpy import linalg as LA

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

from visualization_msgs.msg import Marker

import tf
from time import time
#import _thread
#import threading

##########################
###### Functions #########
##########################

def FinalGoalCB(self):
    print("Got final goal from MoveIT node")
    #print(data)

name = rospy.get_namespace() # has namespace with '/' at beginning and end of the name
#pubGoal = rospy.Publisher(name+'goal', Goal, queue_size=10)


def startNode():

    rate = rospy.Rate(5)
    #rospy.loginfo("Starting...")
    print("Starting rapid trajectories node ...")

    while not rospy.is_shutdown():
        rospy.Subscriber("move_group/motion_plan_request", MotionPlanRequest, FinalGoalCB)
        rate.sleep()

    rospy.loginfo("Bye!")

##########################
##########################

if __name__ == '__main__':

    ns = rospy.get_namespace() # with '/' before and after name
    try:
        rospy.init_node('rapidTrajectoriesTest')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is typically accomplished in a launch file.")
        else:
            print ("Starting rapid quadrotor trajectories node for: " + ns)
            startNode()
    except rospy.ROSInterruptException:
        pass
