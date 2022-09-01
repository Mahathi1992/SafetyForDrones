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
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap
from struct import pack, unpack
import numpy as np
import pcl
import pcl_helper as pointcloud2pcl
import ros_numpy
from scipy import spatial
from numpy import linalg as LA
from geometry_msgs.msg import Transform, Quaternion, Point, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

from visualization_msgs.msg import Marker

import tf
from time import time
#import _thread
#import threading

quad_pos = None
quad_vel = None
quad_acc = None
posf = None
velf = [0,0,0]
accf = [0,0,0]
des_traj_moveit = None
final_goal_pos = Pose()
# octomap = Octomap()
pcl_data_global = pcl.PointCloud_PointXYZRGB()
distanceGrid = None
global output_trajectory
output_trajectory = MultiDOFJointTrajectory()
output_trajectory.header.frame_id = "world"

traj_pub = rospy.Publisher('command/trajectory', MultiDOFJointTrajectory, queue_size=1)

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
    # jointName = self.trajectory[0].multi_dof_joint_trajectory.joint_names[0]
    # print("RapidTrajectory node is running and JOINT name is: ", jointName)

    points_array = self.trajectory[0].multi_dof_joint_trajectory.points
    # print("SIZE OF TRAJ IS ", len(self.trajectory[0].multi_dof_joint_trajectory.points))
    # global posf
    global final_goal_pos
    k=0
    timer = rospy.get_time()
    timeNow = timer
    traj_size = len(self.trajectory[0].multi_dof_joint_trajectory.points)
    while k<traj_size:
        final_goal_pos.position.x = float(self.trajectory[0].multi_dof_joint_trajectory.points[k].transforms[0].translation.x)
        final_goal_pos.position.y = float(self.trajectory[0].multi_dof_joint_trajectory.points[k].transforms[0].translation.y)
        final_goal_pos.position.z = float(self.trajectory[0].multi_dof_joint_trajectory.points[k].transforms[0].translation.z)
        x_dist_squared = (quad_pos.x - final_goal_pos.position.x)*(quad_pos.x - final_goal_pos.position.x)
        y_dist_squared = (quad_pos.y - final_goal_pos.position.y)*(quad_pos.y - final_goal_pos.position.y)
        # z_dist_squared = (quad_pos.z - final_goal_pos.position.z)*(quad_pos.z - final_goal_pos.position.z)
        # print(type(x_des), type(y_des), type(z_des))
        dist_to_k_xy = math.sqrt(x_dist_squared + y_dist_squared)
        dist_to_k_z = abs(quad_pos.z - final_goal_pos.position.z)

        if (traj_size > 2):
            if (k < (traj_size - 2)):
                x_sign = (final_goal_pos.position.x - quad_pos.x)/abs(final_goal_pos.position.x - quad_pos.x)
                y_sign = (final_goal_pos.position.y - quad_pos.y)/abs(final_goal_pos.position.y - quad_pos.y)
                z_sign = (final_goal_pos.position.z - quad_pos.z)/abs(final_goal_pos.position.z - quad_pos.z)
                vmax = 5
                velf = [x_sign*vmax, y_sign*vmax, z_sign*vmax]
                accf = [0, 0, 0]
            else:
                velf = [0, 0, 0]
                accf = [0, 0, 0]
        else:
            velf = [0, 0, 0]
            accf = [0, 0, 0]

        increment = rospy.Duration.from_sec(10)
        increment_10 = rospy.Duration.from_sec(10000000)
        increment_10sec = increment_10.to_sec()
        increment_sec = increment.to_sec()
        timer += increment_sec
        if ((dist_to_k_xy < 0.7 and dist_to_k_z < 0.5) or timer > timeNow + increment_10sec):
            timer = timeNow
            # print(k, timer, timeNow)
            k += 1 #Add timeout and distance to final goal based conditions to increment

def FinalGoalCB(self):
    global x_des_final
    global y_des_final
    global z_des_final
    x_des_final = self.goal_constraints[0].joint_constraints[0].position
    y_des_final = self.goal_constraints[0].joint_constraints[1].position
    z_des_final = self.goal_constraints[0].joint_constraints[2].position

    # global final_goal_pos
    # final_goal_pos.position.x = x_des_final
    # final_goal_pos.position.y = y_des_final
    # final_goal_pos.position.z = z_des_final

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
    global pcl_data_global
    pcl_data_global = pointcloud2pcl.ros_to_pcl(self)

def OccPointCloudCB(self):
    # pc = ros_numpy.numpify(self)
    # points=np.zeros((pc.shape[0],3))
    # points[:,0]=pc['x']
    # points[:,1]=pc['y']
    # points[:,2]=pc['z']
    # pcl_pointcloud = pcl.PointCloud(np.array(points, dtype=np.float32))
    #
    # kdtree = pcl_pointcloud.make_kdtree_flann()
    # points_2 = np.array([[0, 0, 0]], dtype=np.float32)
    # pc_2 = pcl.PointCloud()
    # pc_2.from_array(points_2)
    #
    # indices, sqr_distances = kdtree.nearest_k_search_for_cloud(pc_2, 50)
    # print(indices, sqr_distances)
    pc = ros_numpy.numpify(self)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    pcl_data_occupancy = pcl.PointCloud(np.array(points, dtype=np.float32))
    # print(pcl_data_occupancy[300])

    # Define the duration:
    Tf = 1

    # Define the input limits:
    # Take max velocity as 10 m/s
    vmax = 5 #[m/s]
    fmin = 4  #[m/s/s]
    fmax = 25 #[m/s/s] default 25
    wmax = 20 #[rad/s]
    minTimeSec = 0.05 #[s]

    # Define how gravity lies:
    gravity = [0,0,-9.81]

    # Define the trajectory starting state:
    pos0 = [quad_pos.x, quad_pos.y, quad_pos.z] #position
    vel0 = [quad_vel.x, quad_vel.y, quad_vel.z] #velocity
    acc0 = [quad_acc.x, quad_acc.y, 0] #acceleration

    # distance to goal in x-y plane
    posf = [final_goal_pos.position.x, final_goal_pos.position.y, final_goal_pos.position.z]  # position
    dist_to_goal = math.sqrt((pos0[0] - posf[0]) * (pos0[0] - posf[0]) + (pos0[1] - posf[1]) * (pos0[1] - posf[1]))

    sign_x = (posf[0]-pos0[0])/abs(posf[0]-pos0[0])
    sign_y = (posf[1]-pos0[1])/abs(posf[1]-pos0[1])
    sign_z = (posf[2]-pos0[2])/abs(posf[2]-pos0[2])

    dist_Tf_x = vel0[0] * Tf + (0.5 * Tf * Tf * acc0[0])
    dist_Tf_y = vel0[1] * Tf + (0.5 * Tf * Tf * acc0[1])
    dist_Tf_z = vel0[2] * Tf + (0.5 * Tf * Tf * acc0[2])

    ## LIMIT TO x-y plane to initial testing

    dist_Tf_xy = math.sqrt(dist_Tf_x * dist_Tf_x + dist_Tf_y * dist_Tf_y)

    dist_Tf_max_xy = 1.5 #vmax * Tf + (0.5 * Tf * Tf * fmax) # see max acc max vel above

    # print("Distance to goal: ", dist_to_goal)
    iter_x = pos0[0] - dist_Tf_max_xy
    iter_y = pos0[1] - dist_Tf_max_xy
    iter_z = pos0[2] - dist_Tf_max_xy

    obst_cost_array = [0]
    # obst_cost_array_x = []
    # obst_cost_array_y = []
    cost_array = []
    # cost_array_x = []
    # cost_array_y = []
    x_array = []
    y_array = []
    dist_max_local_var = vmax * Tf + (0.5 * Tf * Tf * fmax) # compute max distance travel possible for given vmax, fmax
    res = 0.5 # distance grid resolution
    N_ctrl_pts = dist_max_local_var / res
    time_int_res = Tf / N_ctrl_pts # time interval resolution

    while iter_x <= (pos0[0] + dist_Tf_max_xy):
        while iter_y <= (pos0[1] + dist_Tf_max_xy):
            # while (iter_z <= (pos0[2] + dist_Tf_max_xy)):
            #     iter_z += 0.5

            ### Indent from here to iter_z reset if using z axis traj generation too
            # set initial position to [iter_x, iter_y, iter_z]
            final_pos = [iter_x, iter_y, final_goal_pos.position.z]
            # print(init_pos)

            ## generate multiple trajectories here
            traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
            traj.set_goal_position(final_pos)
            traj.set_goal_velocity(velf)
            traj.set_goal_acceleration(accf)

            traj.generate(Tf)

            # Test input feasibility
            inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)

            # Test whether we fly into the floor
            floorPoint  = [0,0,-0.1]  # a point on the floor #Can choose -0.1 due to landing gear of quadrotor usually.
            floorNormal = [0,0,1]  # we want to be in this direction of the point (upwards)
            positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)

            alpha = []
            beta = []
            gamma = []

            for i in range(3):
                # print("Axis #" , i)
                alpha.insert(i,traj.get_param_alpha(i))
                beta.insert(i,traj.get_param_beta(i))
                gamma.insert(i,traj.get_param_gamma(i))
            # print(alpha)
            # print(beta)
            # print(gamma)
            # print("Total cost = " , traj.get_cost())
            # print("Input feasibility result: ",    quadtraj.InputFeasibilityResult.to_string(inputsFeasible),   "(", inputsFeasible, ")")
            # print("Position feasibility result: ", quadtraj.StateFeasibilityResult.to_string(positionFeasible), "(", positionFeasible, ")")

            ## CHECK collision using distance cloud HERE!!!
            cost_goal = 0
            cost_obst = 0
            cost_obst_x = 0
            cost_obst_y = 0
            total_cost = 0

            for data in pcl_data_occupancy:
                x_grid = data[0]
                y_grid = data[1]
                z_grid = data[2]

                # a_rgb = data[3] # PCL stores32-bit color as ARGB order (according to pcl documentation)
                # int_rgb = unpack('I', pack('f', a_rgb))[0]

                # blue = (int_rgb & 0xff000000) >> 24 # NO idea which bits are blue and which are for alpha
                # red = (int_rgb & 0x0000ff00) >> 8 # however, red seems to be stored from bit 8 to 15
                # green = (int_rgb & 0x000000ff) # green seems to be stored from bit 0 to 7

                dist_to_quad_x = (x_grid - pos0[0])
                dist_to_quad_y = (y_grid - pos0[1])
                dist_to_obst = math.sqrt((dist_to_quad_x*dist_to_quad_x + dist_to_quad_y*dist_to_quad_y))
                # if red > 200 and dist_to_obst < 10:
                if dist_to_obst < 50:
                    time_t = Tf
                    while time_t <= Tf:
                        t_sq = math.pow(time_t,2)
                        t_cube = math.pow(time_t,3)
                        t_4 = math.pow(time_t,4)
                        t_5 = math.pow(time_t,5)
                        ## do the collision check for each control point
                        x_collision_check = pos0[0] + vel0[0]*time_t + acc0[0]*t_sq/2 + gamma[0]*t_cube/6 + beta[0]*t_4/24 + alpha[0]*t_5/120
                        y_collision_check = pos0[1] + vel0[1]*time_t + acc0[1]*t_sq/2 + gamma[1]*t_cube/6 + beta[1]*t_4/24 + alpha[1]*t_5/120

                        x_diff = abs(x_grid - x_collision_check)
                        y_diff = abs(y_grid - y_collision_check)
                        cost_obst += (0/dist_to_obst) + (0.05*(math.sqrt(vel0[0]*vel0[0] + vel0[1]*vel0[1]))/(x_diff*x_diff + y_diff*y_diff))
                        # cost_obst_x += 0.2*abs(vel0[0])/x_diff
                        # cost_obst_y += 0.2*abs(vel0[1])/y_diff
                        # obst_cost_array_x.append(cost_obst_x)
                        # obst_cost_array_y.append(cost_obst_y)
                        obst_cost_array.append(cost_obst)
                        time_t += time_int_res
                    # obst_cost_array.append(cost_obst)

                    # cost_obst = 10000*(math.sqrt(vel0[0]*vel0[0] + vel0[1]*vel0[1]))/dist_to_obst
                    # print("obst cost is: ", cost_obst, "x_grid: ", x_grid, "y_grid: ", y_grid)
                else:
                    obst_cost_array.append(0)
                    continue

            cost_goal = math.sqrt((iter_x - posf[0]) * (iter_x - posf[0]) + (iter_y - posf[1])*(iter_y - posf[1]))
            # total_cost_x = 1*(abs(iter_x - posf[0])) #+ 1.0*max(obst_cost_array_x)
            # total_cost_y = 1*(abs(iter_y - posf[1])) #+ 1.0*max(obst_cost_array_y)

            # total_cost = 0.050*(abs(iter_x - posf[0])) + 0.05*(abs(iter_y - posf[1])) + 1.0*max(obst_cost_array)
            total_cost = 0.75*cost_goal + 0.95*max(obst_cost_array) # set weights to 0.5, 1.5 for good obstacle avoidance
            cost_array.append(total_cost)

            # cost_array_x.append(total_cost_x)
            # cost_array_y.append(total_cost_y)
            # print("total cost : ", total_cost)
            x_array.append(iter_x)
            y_array.append(iter_y)

            cost_goal = 0
            cost_obst = 0
            cost_obst_x = 0
            cost_obst_y = 0
            iter_y += 0.5

            # if iter_z >= (pos0[2] + dist_Tf_max_xy):
            #     iter_z = pos0[2] - dist_Tf_max_xy
        if iter_y >= (pos0[1] + dist_Tf_max_xy):
            iter_y = pos0[1] - dist_Tf_max_xy
        iter_x += 0.5
    if iter_x >= (pos0[0] + dist_Tf_max_xy):
        iter_x = pos0[0] - dist_Tf_max_xy
        output_trajectory.points = []
        #find the minimum cost, corresponding alpha, beta, gamma and publish trajectory_msg HERE
        #find minimum cost
        min_cost = min(cost_array)
        index_min_cost = cost_array.index(min_cost)
        x_min_cost = x_array[index_min_cost]
        y_min_cost = y_array[index_min_cost]

        # print("******** INDEX MIN COST, MIN COST  ********", index_min_cost, min_cost)
        #generate minimum cost trajectory and publish
        traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
        # traj.set_goal_position([final_goal_pos.position.x,final_goal_pos.position.y,final_goal_pos.position.z])
        traj.set_goal_position([x_min_cost,y_min_cost,final_goal_pos.position.z])
        traj.set_goal_velocity(velf)
        traj.set_goal_acceleration(accf)

        traj.generate(Tf)

        # Test input feasibility
        # inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)

        # Test whether we fly into the floor
        floorPoint  = [0,0,-0.1]  # a point on the floor #Can choose -0.1 due to landing gear of quadrotor usually.
        floorNormal = [0,0,1]  # we want to be in this direction of the point (upwards)
        # positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)

        alpha_min = []
        beta_min = []
        gamma_min = []

        for i in range(3):
            # print("Axis #" , i)
            alpha_min.insert(i,traj.get_param_alpha(i))
            beta_min.insert(i,traj.get_param_beta(i))
            gamma_min.insert(i,traj.get_param_gamma(i))

        # dist_max_local_var = vmax * Tf + (0.5 * Tf * Tf * fmax) # compute max distance travel possible for given vmax, fmax
        # res = 0.5 # distance grid resolution
        # N_ctrl_pts = dist_max_local_var / res
        # time_int_res = Tf / N_ctrl_pts # time interval resolution

        time_t = 0
        while time_t <= Tf:
            t_sq = math.pow(time_t,2)
            t_cube = math.pow(time_t,3)
            t_4 = math.pow(time_t,4)
            t_5 = math.pow(time_t,5)

            pos_out = Point()
            vel_out = Twist()
            acc_out = Twist()
            quat_out = tf.transformations.quaternion_from_euler(0,0,0)

            x_output = pos0[0] + vel0[0]*time_t + acc0[0]*t_sq/2 + gamma_min[0]*t_cube/6 + beta_min[0]*t_4/24 + alpha_min[0]*t_5/120
            y_output = pos0[1] + vel0[1]*time_t + acc0[1]*t_sq/2 + gamma_min[1]*t_cube/6 + beta_min[1]*t_4/24 + alpha_min[1]*t_5/120
            z_output = pos0[2] + vel0[2]*time_t + acc0[2]*t_sq/2 + gamma_min[2]*t_cube/6 + beta_min[2]*t_4/24 + alpha_min[2]*t_5/120

            x_dot_output = vel0[0] + acc0[0]*time_t + gamma_min[0]*t_sq/2 + beta_min[0]*t_cube/6 + alpha_min[0]*t_4/24
            y_dot_output = vel0[1] + acc0[1]*time_t + gamma_min[1]*t_sq/2 + beta_min[1]*t_cube/6 + alpha_min[1]*t_4/24
            z_dot_output = vel0[2] + acc0[2]*time_t + gamma_min[2]*t_sq/2 + beta_min[2]*t_cube/6 + alpha_min[2]*t_4/24

            x_ddot_output = acc0[0] + gamma_min[0]*time_t + beta_min[0]*t_sq/2 + alpha_min[0]*t_cube/6
            y_ddot_output = acc0[1] + gamma_min[1]*time_t + beta_min[1]*t_sq/2 + alpha_min[1]*t_cube/6
            z_ddot_output = acc0[2] + gamma_min[2]*time_t + beta_min[2]*t_sq/2 + alpha_min[2]*t_cube/6

            # F_x = 0
            # F_y = 0
            # Fx_grad_max = 0
            # Fy_grad_max = 0
            # dist_to_obst_min = 5
            #
            # for data in pcl_data_global:
            #     x_grid = data[0]
            #     y_grid = data[1]
            #     z_grid = data[2]
            #     a_rgb = data[3] # PCL stores32-bit color as ARGB order (according to pcl documentation)
            #     int_rgb = unpack('I', pack('f', a_rgb))[0]
            #
            #     global red
            #     global green
            #     # blue = (int_rgb & 0xff000000) >> 24 # NO idea which bits are blue and which are for alpha
            #     red = (int_rgb & 0x0000ff00) >> 8 # however, red seems to be stored from bit 8 to 15
            #     green = (int_rgb & 0x000000ff) # green seems to be stored from bit 0 to 7
            #
            #     if red > 200:
            #         dist_to_quad_x = (x_grid - pos0[0])
            #         dist_to_quad_y = (y_grid - pos0[1])
            #         dist_to_obst = math.sqrt((dist_to_quad_x*dist_to_quad_x + dist_to_quad_y*dist_to_quad_y))
            #
            #         if dist_to_obst <= dist_to_obst_min:
            #             dist_to_obst_min = dist_to_obst
            #             dist_to_quad_x = (x_grid - pos0[0])
            #             dist_to_quad_y = (y_grid - pos0[1])
            #             # print(dist_to_quad_x,dist_to_quad_y,dist_to_obst_min)
            #             Fx_grad_max = (dist_to_quad_x)/(dist_to_obst_min)
            #             Fy_grad_max = (dist_to_quad_y)/(dist_to_obst_min)
            #
            #             # if dist_to_obst <= 10:
            #             # gradient_x = (dist_to_quad_x)/(dist_to_obst_min)
            #             # gradient_y = (dist_to_quad_y)/(dist_to_obst_min)
            #
            #             kr = 100
            #             gamma_repulse = 23.5
            #             F_x = kr*(1 - ((1 - math.exp(gamma_repulse*dist_to_obst_min))/(1 - math.exp(gamma_repulse*10))))*Fx_grad_max
            #             F_y = kr*(1 - ((1 - math.exp(gamma_repulse*dist_to_obst_min))/(1 - math.exp(gamma_repulse*10))))*Fy_grad_max
            #
            #             # print(F_x,F_y,dist_to_obst_min)
            #             vel_out.linear.x = x_dot_output + F_x # vel0[0] + F_x*time_t
            #             vel_out.linear.y = y_dot_output + F_y # vel0[1] + F_y*time_t
            #             vel_out.linear.z = z_dot_output
            #
            #             acc_out.linear.x = x_ddot_output #+ F_x
            #             acc_out.linear.y = y_ddot_output #+ F_y
            #             acc_out.linear.z = z_ddot_output
            #         else:
            #             F_x = 0
            #             F_y = 0
            #             vel_out.linear.x = x_dot_output + F_x
            #             vel_out.linear.y = y_dot_output + F_y
            #             vel_out.linear.z = z_dot_output
            #
            #             acc_out.linear.x = x_ddot_output
            #             acc_out.linear.y = y_ddot_output
            #             acc_out.linear.z = z_ddot_output
            #     else:
            vel_out.linear.x = x_dot_output
            vel_out.linear.y = y_dot_output
            vel_out.linear.z = z_dot_output

            acc_out.linear.x = x_ddot_output
            acc_out.linear.y = y_ddot_output
            acc_out.linear.z = z_ddot_output


            transforms = Transform()
            transforms.translation.x = x_output
            transforms.translation.y = y_output
            transforms.translation.z = z_output

            transforms.rotation.x = quat_out[0]
            transforms.rotation.y = quat_out[1]
            transforms.rotation.z = quat_out[2]
            transforms.rotation.w = quat_out[3]

            # transforms = Transform(translation=pos_out,rotation=quat_out)
            traj_point = MultiDOFJointTrajectoryPoint([transforms],[vel_out],[acc_out],rospy.Time.now() + rospy.Duration(time_t))

            # global output_trajectory

            output_trajectory.header.frame_id = "world"
            output_trajectory.header.stamp = rospy.Time.now()
            output_trajectory.points.append(traj_point)

            time_t += time_int_res
            # print(output_trajectory)
        cost_array.clear()
        x_array.clear()
        y_array.clear()
        obst_cost_array.clear()
        # obst_cost_array_x.clear()
        # obst_cost_array_y.clear()
        # cost_array_x.clear()
        # cost_array_y.clear()

name = rospy.get_namespace() # has namespace with '/' at beginning and end of the name
#pubGoal = rospy.Publisher(name+'goal', Goal, queue_size=10)


def startNode():

    rate = rospy.Rate(10)
    #rospy.loginfo("Starting...")
    # print("Starting rapid trajectories node ...")

    rospy.Subscriber("move_group/motion_plan_request", MotionPlanRequest, FinalGoalCB)
    rospy.Subscriber("odometry_sensor1/odometry", Odometry, CurrentStateCB)
    rospy.Subscriber("ground_truth/imu", Imu, AccelCB)
    rospy.Subscriber("move_group/display_planned_path", DisplayTrajectory, GlobalDesTrajCB)
    # rospy.Subscriber("move_group/monitored_planning_scene", PlanningScene, OctoMapCB)
    rospy.Subscriber("global_mapper_ros/distance_grid", PointCloud2, DistanceGridCB)
    rospy.Subscriber("global_mapper_ros/occupancy_grid", PointCloud2, OccPointCloudCB)

    # global traj_pub
    # # traj_pub = rospy.Publisher("command/trajectory", MultiDOFJointTrajectory, queue_size=1)
    # traj_pub.publish(output_trajectory)
    # # output_trajectory.points = []
    # print(output_trajectory)
    # rospy.spin()

    while not rospy.is_shutdown():
        # rospy.loginfo("In ROS whileLOOP")

        traj_pub.publish(output_trajectory)
        # print(output_trajectory)
        # output_trajectory.points = []
        rate.sleep()
    #
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
