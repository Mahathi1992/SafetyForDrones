#!/usr/bin/env python

#Mahathi Bhargavapuri, June 2022

#This generates trajectories using ETH's library: https://github.com/markwmuller/RapidQuadrocopterTrajectories.git

#make sure to run this node AFTER roslaunch firefly demo and global mapper

from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
import roslib
import rospy
import math
from geometry_msgs.msg import Pose, Twist
# import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap
import numpy as np
import pcl
import ros_numpy
from scipy import spatial
from numpy import linalg as LA

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

from visualization_msgs.msg import Marker

import tf
from time import time
#import _thread
#import threading

quad_pos = None
quad_vel = None
quad_acc = None
des_traj_moveit = None
final_goal_pos = Pose()
octomap = Octomap()
distanceGrid = None

##########################
###### Functions #########
##########################

### First get current position, velocity and acceleration

def CurrentStateCB(self):
    global quad_pos
    quad_pos = self.pose.pose.position

    global quad_vel
    quad_vel = self.twist.twist.linear

    # print(quad_pos, quad_vel)

def AccelCB(self):
    global quad_acc
    quad_acc = self.linear_acceleration
    # print(imu.linear_acceleration)

### Get final goal and desired global piece-wise linear path

def GlobalDesTrajCB(self):
    jointName = self.trajectory[0].multi_dof_joint_trajectory.joint_names[0]
    # print("RapidTrajectory node is running and JOINT name is: ", jointName)

def FinalGoalCB(self):

    x_des_final = self.goal_constraints[0].joint_constraints[0].position
    y_des_final = self.goal_constraints[0].joint_constraints[1].position
    z_des_final = self.goal_constraints[0].joint_constraints[2].position

    global final_goal_pos
    final_goal_pos.position.x = x_des_final
    final_goal_pos.position.y = y_des_final
    final_goal_pos.position.z = z_des_final

    # print(final_goal_pos)

### Get octomap grid, distance grid, raw sensor data
#
# def OctoMapCB(self):
#
#     global octomap
#
#     # print("Got octomap")
#     octomap = self.world.octomap.octomap
#     print(octomap.resolution)

def DistanceGridCB(self):
    print("Got distance grid from Global Mapper")


def PointCloudCB(data):

    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    pcl_pointcloud = pcl.PointCloud(np.array(points, dtype=np.float32))

    kdtree = pcl_pointcloud.make_kdtree_flann()

    points_2 = np.array([[quad_pos.x, quad_pos.y, quad_pos.z]], dtype=np.float32)
    pc_2 = pcl.PointCloud()
    pc_2.from_array(points_2)

    indices, sqr_distances = kdtree.nearest_k_search_for_cloud(pc_2, 5)

    # print(indices, sqr_distances)

def GenRapidTrajectories(self):

    # Define the trajectory starting state:
    pos0 = [0, 0, 0] #position
    vel0 = [0, 0, 0] #velocity
    acc0 = [0, 0, 0] #acceleration

    # Define the goal state:
    posf = [0, 0, 1]  # position
    velf = [0, 0, 0]  # velocity
    accf = [0, 0, 0]  # acceleration

    # Define the duration:
    Tf = 1

    # Define the input limits:
    fmin = 4  #[m/s**2]
    fmax = 25 #[m/s**2]
    wmax = 20 #[rad/s]
    minTimeSec = 0.02 #[s]

    # Define how gravity lies:
    gravity = [0,0,-9.81]

    traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
    traj.set_goal_position(posf)
    traj.set_goal_velocity(velf)
    traj.set_goal_acceleration(accf)

    traj.generate(Tf)

    # Test input feasibility
    inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)

    # Test whether we fly into the floor
    floorPoint  = [0,0,-0.1]  # a point on the floor #Can choose -0.1 due to landing gear of quadrotor usually.
    floorNormal = [0,0,1]  # we want to be in this direction of the point (upwards)
    positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)

    for i in range(3):
        print("Axis #" , i)
        print("\talpha = " ,traj.get_param_alpha(i), "\tbeta = "  ,traj.get_param_beta(i), "\tgamma = " ,traj.get_param_gamma(i))
    print("Total cost = " , traj.get_cost())
    print("Input feasibility result: ",    quadtraj.InputFeasibilityResult.to_string(inputsFeasible),   "(", inputsFeasible, ")")
    print("Position feasibility result: ", quadtraj.StateFeasibilityResult.to_string(positionFeasible), "(", positionFeasible, ")")

name = rospy.get_namespace() # has namespace with '/' at beginning and end of the name
#pubGoal = rospy.Publisher(name+'goal', Goal, queue_size=10)


def startNode():

    rate = rospy.Rate(5)
    #rospy.loginfo("Starting...")
    print("Starting rapid trajectories node ...")

    rospy.Subscriber("move_group/motion_plan_request", MotionPlanRequest, FinalGoalCB)
    rospy.Subscriber("odometry_sensor1/odometry", Odometry, CurrentStateCB)
    rospy.Subscriber("ground_truth/imu", Imu, AccelCB)
    rospy.Subscriber("move_group/display_planned_path", DisplayTrajectory, GlobalDesTrajCB)
    # rospy.Subscriber("move_group/monitored_planning_scene", PlanningScene, OctoMapCB)
    rospy.Subscriber("global_mapper_ros/distance_grid", PointCloud2, DistanceGridCB)
    rospy.Subscriber("vlp16/velodyne/points", PointCloud2, PointCloudCB)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     rospy.Subscriber("move_group/motion_plan_request", MotionPlanRequest, FinalGoalCB)
    #     rate.sleep()
    #
    # rospy.loginfo("Bye!")

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
