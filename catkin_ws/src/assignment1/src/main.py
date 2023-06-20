#!/usr/bin/env python

import sys
import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import pandas as pd
import matplotlib.pyplot as plt



resolution = 121 # number of cells in width and height in the grid
size = 15/resolution # height and width of a cell in meters
initgrid = np.zeros([resolution, resolution]) # Occupancy grid
fieldgrid = np.zeros([resolution, resolution, 2]) # Field grid
cellX = 0
cellY = 0
yaw = 0
k_att = 1 # Attraction field coefficient
cellgX = 0
cellgY = 0
Qlim = 8 # Distance limit in cells for which the repulsive field apply
k_rep = 20*k_att # Repulsive field coefficient
cmd = 0
att_field_grid = np.zeros([resolution, resolution, 2]) # Attractive field grid
rep_field_grid = np.zeros([resolution, resolution, 2]) # Repulsive field grid

def odom_callback(odom):
    global initgrid
    global fieldgrid
    global yaw
    global cellX
    global cellY
    global cmd

    xpos = odom.pose.pose.position.x  # get the X measurement in m of the robot pos
    ypos = odom.pose.pose.position.y  # get the Y measurement in m of the robot pos
    yaw = quater_to_euler(odom.pose.pose.orientation)[2] # get the yaw in radian of the robot

    # get the X coordinate of the cell where the robot is
    cellX = int((resolution-1)/2) + int(xpos/size)
    # get the Y coordinate of the cell where the robot is
    cellY = int((resolution-1)/2) + int(ypos/size)

    reset_robot_pos(initgrid) # removing the previous robot pos
    initgrid[cellX, cellY] = -1 # setting the robot pos in the cell


    
    print("cell of the robot : [" + str(cellX) + "," + str(cellY) + "]")
    print("field X in the cell : " + str(fieldgrid[cellX, cellY, 0]))
    print("field Y in the cell : " + str(fieldgrid[cellX, cellY, 1]))
    obj_angle_wf = math.atan2(fieldgrid[cellX,cellY,1], fieldgrid[cellX,cellY,0]) # objective angle in the worldframe
    if obj_angle_wf < 0:
        obj_angle_wf += 2*math.pi
    if yaw < 0:
        yaw += 2*math.pi  
    print("objective angle : " + str(obj_angle_wf), "yaw : " + str(yaw))
    if yaw < obj_angle_wf - 0.1 or yaw > obj_angle_wf + 0.1: # if the yaw is too different than the objective angle
        cmd.linear.x = 0 # no linear speed
        cmd.angular.z = math.copysign(0.8, obj_angle_wf-yaw) # rotating in the orientation of the difference
        print("rotating, angular cmd :" + str(cmd.angular.z))
    else: #if the robot is in the right orientation
        cmd.angular.z = 0 # no more angular speed
        normF = math.sqrt(fieldgrid[cellX,cellY,1]**2+fieldgrid[cellX,cellY,0]**2) # norm of the field force
        if normF < 10 and normF > 0: # if we're close enough from the goal and not at the goal
            cmd.linear.x = 0.1 # constant speed
        else: # if not
            cmd.linear.x = 0.01*normF # linear speed proportional to the norm of the field force
        print("moving forward, linear cmd : " + str(cmd.linear.x))
    print("----")

def scan_callback(scan):
    global ranges
    global initgrid
    global fieldgrid
    global wait

    ranges = scan.ranges
    for i in range(len(ranges)):
        if ranges[i] < scan.range_max:
            angle_wf = yaw + i*scan.angle_increment # angle of the range in the world frame
            while(angle_wf > math.pi*2): # keeping values in the [0, 2pi]
                angle_wf -= math.pi*2
            dx_bot_to_range = ranges[i]*math.cos(angle_wf) # distance in x from the robot to the obstacle in m 
            dy_bot_to_range = ranges[i]*math.sin(angle_wf) # distance in y from the robot to the obstacle in m

            cell_X_obstacle = cellX+int(dx_bot_to_range/size) # X coordinate of the obstacle
            cell_Y_obstacle = cellY+int(dy_bot_to_range/size) # Y coordinate of the obstacle
            if cell_X_obstacle < resolution and cell_Y_obstacle < resolution: # if the obstacle is not out of the grid
                initgrid[cell_X_obstacle, cell_Y_obstacle] = 1 # setting the value for the obstacle cell
        
    fieldgrid = create_att_field() + create_rep_field(initgrid) # creating field

def reset_robot_pos(mygrid):
    global cellgX
    global cellgY
    for x in range(np.shape(mygrid)[0]):
        for y in range(np.shape(mygrid)[1]):
            # for each cell
            if mygrid[x, y] == -1: # if it's the previous robot cell
                mygrid[x, y] = 0 # it is not anymore

def create_att_field():
    global cellgX
    global cellgY
    global att_field_grid

    goalx = rospy.get_param("/goal_x")
    goaly = rospy.get_param("/goal_y")

    # Converting goal parameters from meters to cells number
    cellgX = int((resolution-1)/2) + int(goalx/size)
    cellgY = int((resolution-1)/2) + int(goaly/size)
    
    att_field_grid[cellX, cellY, 0] = k_att*(cellgX-cellX) # Attractive field on the x axis is created
    att_field_grid[cellX, cellY, 1] = k_att*(cellgY-cellY) # Attractive field on the y axis is created

    return att_field_grid

def create_rep_field(mygrid):
    global cellX
    global cellY
    global rep_field_grid

    fieldX = 0
    fieldY = 0

    for a in range(-Qlim, Qlim+1):
        for b in range(-Qlim, Qlim+1):
            # checking in the Qlim*Qlim square around the cell
            dcell = math.sqrt(a**2+b**2) # calculating the distance in terms of cells from the supposed obstacle cell
            if cellX+a >= 0 and cellY+b >= 0 and cellX+a < resolution and cellY+b < resolution:
            #if the checked cell is still in the grid
                if dcell < Qlim and dcell != 0 and mygrid[cellX+a, cellY+b] == 1:
                # eliminating non concerned-cells of the square, the field cell itself and only if an obstacle cell is detected
                    fieldX += k_rep*(((1/dcell)-(1/Qlim))/(dcell**2))*(-a/dcell) # adding the repulsive field emitted from the obstacle cell
                    fieldY += k_rep*(((1/dcell)-(1/Qlim))/(dcell**2))*(-b/dcell) # adding the repulsive field emitted from the obstacle cell

    rep_field_grid[cellX, cellY, 0] = fieldX # 0 if no obstacle, total field if there is
    rep_field_grid[cellX, cellY, 1] = fieldY # 0 if no obstacle, total field if there is

    return rep_field_grid

def quater_to_euler(quaternion):
    ori_vect = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = euler_from_quaternion(ori_vect)
    return euler


def euler_to_quater(euler):
    quaternion = quaternion_from_euler(euler.roll, euler.pitch, euler.yaw)
    return quaternion


def main():
    rospy.init_node('mpoa', anonymous=True)

    global cmd
    global initgrid
    global fieldgrid

    # Creating the publisher
    cmd = Twist()
    # Creating the suscribers
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        # We don't need these commands
        cmd.linear.z = 0
        cmd.angular.y = 0
        cmd.angular.x = 0
        cmd.linear.y = 0
        pub.publish(cmd)

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':

    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass