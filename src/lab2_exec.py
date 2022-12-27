#!/usr/bin/env python
# Tyler Smithline and Sankalp Agarwal

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])
safe = np.radians([169.97, -95.49, 107.9, -103.42, -90.25, 155.45])

# Hanoi tower location 1
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

# hanoi tower location 1 Q(column)(height)
Q11 = np.radians([152.86, -59.84, 136.04, -167.65, -89.15, 139.13])
Q12 = np.radians([152.98, -70.04, 134.9, -157.08, -88.83, 139.05])
Q13 = np.radians([152.98, -78.8, 132.43, -145.85, -88.95, 139.06])

# Hanoi tower location 2 
Q21 = np.radians([169.12, -58.71, 133.66, -166.52, -89.59, 153.39])
Q22 = np.radians([169.12, -68.91, 132.69, -155.35, -89.72, 155.38])
Q23 = np.radians([169.18, -77.07, 129.97, -144.47, -89.84, 155.43])

#hanoi tower location 3
Q31 = np.radians([185.33, -55.42, 125.92, -162.08, -90.07, 171.6])
Q32 = np.radians([185.33, -64.34, 125.17, -152.4, -90.18, 171.59])
Q33 = np.radians([185.33, -72.24, 122.91, -142.26, -90.29, 171.6])

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

#height global variable definition
h = [0,0,0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):
    
    global digital_in_0
    # global analog_in_0

    digital_in_0 = msg.DIGIN
    # analog_in_0 = msg.AIN0

############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position
    global digital_in_0

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1
    return error


# def move_arm(pub_cmd, loop_rate, dest, vel, accel):

#     global thetas
#     global SPIN_RATE

#     error = 0
#     spin_count = 0
#     at_goal = 0

#     driver_msg = command()
#     driver_msg.destination = dest
#     driver_msg.v = vel
#     driver_msg.a = accel
#     driver_msg.io_0 = current_io_0
#     pub_cmd.publish(driver_msg)

#     loop_rate.sleep()

#     while(at_goal == 0):

#         if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
#             abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
#             abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
#             abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
#             abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
#             abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

#             at_goal = 1
#             rospy.loginfo("Goal is reached!")

#         loop_rate.sleep()

#         if(spin_count >  SPIN_RATE*5):

#             pub_cmd.publish(driver_msg)
#             rospy.loginfo("Just published again driver_msg")
#             spin_count = 0

#         spin_count = spin_count + 1

#     return error

def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global h
    global digital_in_0
    global current_io_0

    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0
    move_arm(pub_cmd, loop_rate, Q[start_loc - 1][start_height - 1], 2.0, 2.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1)

    if current_io_0 == True and digital_in_0 != 1:
        gripper(pub_cmd, loop_rate, suction_off)
        print("Block not found; Quitting... ")
        sys.exit()

    move_arm(pub_cmd, loop_rate, safe, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc - 1][end_height], 2.0, 2.0)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.5)
    move_arm(pub_cmd, loop_rate, safe, 4.0, 4.0)
    h[start_loc-1] -= 1
    h[end_loc-1] += 1

    return error


############### Your Code End Here ###############


def main():

    global home
    global safe
    global Q
    global SPIN_RATE
    global h

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    start_loc = 0
    end_loc = 0

    # while(not input_done):
    #     input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")

    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    # while(not input_done):
    start_string = raw_input("Enter starting location <Either 1 2 3 or 0 to quit> ")
    print("You entered " + start_string + "\n")

    if int(start_string) < 1 or int(start_string) > 3:
        print("Quitting... ")
        sys.exit()
    else:
        start_loc = int(start_string)

    end_string = raw_input("Enter ending location <Either 1 2 3 or 0 to quit> ")
    print("You entered " + end_string + "\n")

    if int(end_string) < 1 or int(end_string) > 3:
        print("Quitting... ")
        sys.exit()
    else:
        end_loc = int(end_string)
        input_done = 1

    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # move to home at start of program
    # gripper(pub_command, loop_rate, suction_on)
    rospy.loginfo("test")
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    
    a = start_loc
    c = end_loc
    b = 6 - (start_loc + end_loc)
    h[a - 1] = 3
    # print(a, b, c)



    move_block(pub_command, loop_rate, a, h[a-1], c, h[c-1])
    move_block(pub_command, loop_rate, a, h[a-1], b, h[b-1])
    move_block(pub_command, loop_rate, c, h[c-1], b, h[b-1])
    move_block(pub_command, loop_rate, a, h[a-1], c, h[c-1])
    move_block(pub_command, loop_rate, b, h[b-1], a, h[a-1])
    move_block(pub_command, loop_rate, b, h[b-1], c, h[c-1])
    move_block(pub_command, loop_rate, a, h[a-1], c, h[c-1])

    # move_block(pub_command, loop_rate, a, 3, c, 3)


    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass

