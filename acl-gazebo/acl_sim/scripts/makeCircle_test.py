#!/usr/bin/env python

#Mahathi Bhargavapuri, Aug 2021

#This generates a circle and publishes on the topic "goal" for the perfect tracker.

#make sure to run this node AFTER roslaunch acl_sim perfect_tracker_and_sim.launch

import roslib
import rospy
import math
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
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

def circle_circumference(cx = -5,cy = -5,N = 20):
    circle_points = list()
    delta = math.pi/N
    for i in range(0, 2*N):
        px = 2*math.cos(i*delta) + cx
        py = 2*math.sin(i*delta) + cy
        circle_points.append([px, py])

    return circle_points


name = rospy.get_namespace() # has namespace with '/' at beginning and end of the name
pubGoal = rospy.Publisher(name+'goal', Goal, queue_size=10)


def startNode():
    global target_position_x
    global target_position_y
    counter = -1
    #global counter = -1
    ##call circle_circumference
    target_points = circle_circumference(0,0,20)
    target_position_x = target_points[0][0]
    target_position_y = target_points[0][1]

    rate = rospy.Rate(5)
    #rospy.loginfo("Starting...")
    print("Starting MTB's test run ...")

    desState = Goal()

    while not rospy.is_shutdown():

        if counter < len(target_points):
            counter = counter + 1
        if counter == len(target_points):
            counter = 0
        target_position_x = target_points[counter][0]
        target_position_y = target_points[counter][1]

        desState.p.x = target_position_x
        desState.p.y = target_position_y
        desState.p.z = 1.5
        desState.v.x = 0
        desState.v.y = 0
        desState.v.z = 0
        desState.a.x = 0
        desState.a.y = 0
        desState.a.z = 0
        desState.yaw = 0

        pubGoal.publish(desState)
        #print(desState)
        rate.sleep()

    rospy.loginfo("Bye!")

##########################
##########################

if __name__ == '__main__':

    ns = rospy.get_namespace() # with '/' before and after name
    try:
        rospy.init_node('makeCircleTest')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is typically accomplished in a launch file.")
        else:
            print ("Starting make circle node for: " + ns)
            startNode()
    except rospy.ROSInterruptException:
        pass
