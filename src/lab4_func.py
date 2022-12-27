#!/usr/bin/env python
from cmath import pi
import numpy as np
from scipy.linalg import expm
from scipy.linalg import norm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def make_skew(w):
	return [[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]]

def make_s(w, v):
	return [[w[0][0],w[0][1],w[0][2],v[0]],
			[w[1][0],w[1][1],w[1][2],v[1]],
			[w[2][0],w[2][1],w[2][2],v[2]],
			[0, 0, 0, 0]]

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	w1 = [0, 0, 1]
	w2 = [0, 1, 0]
	w3 = [0, 1, 0]
	w4 = [0, 1, 0]
	w5 = [1, 0, 0]
	w6 = [0, 1, 0]
	# print("w1: ", w1)
	# print("w2: ", w2)
	# print("w3: ", w3)
	# print("w4: ", w4)
	# print("w5: ", w5)
	# print("w6: ", w6)

	skew_w1 = make_skew(w1)
	skew_w2 = make_skew(w2)
	skew_w3 = make_skew(w3)
	skew_w4 = make_skew(w4)
	skew_w5 = make_skew(w5)
	skew_w6 = make_skew(w6)
    # distance
	q1 = [-150, 150, 10]
	q2 = np.array(q1) + np.array([0, 120, 152])
	q3 = np.array(q2) + np.array([244, 0, 0])
	q4 = np.array(q3) + np.array([213, -93, 0])
	q5 = np.array(q4) + np.array([0, 83, 0])
	q6 = np.array(q5) + np.array([83, 0, 0])
	# print("q1: ", q1)
	# print("q2: ", q2)
	# print("q3: ", q3)
	# print("q4: ", q4)
	# print("q5: ", q5)
	# print("q6: ", q6)

	# linear velocity
	v1 = np.cross(-1*np.array(w1),q1)
	v2 = np.cross(-1*np.array(w2),q2)
	v3 = np.cross(-1*np.array(w3),q3)
	v4 = np.cross(-1*np.array(w4),q4)
	v5 = np.cross(-1*np.array(w5),q5)
	v6 = np.cross(-1*np.array(w6),q6)
	# print("v1: ", v1)
	# print("v2: ", v2)
	# print("v3: ", v3)
	# print("v4: ", v4)
	# print("v5: ", v5)
	# print("v6: ", v6)
	
	S1 = make_s(skew_w1,v1)
	S2 = make_s(skew_w2,v2)
	S3 = make_s(skew_w3,v3)
	S4 = make_s(skew_w4,v4)
	S5 = make_s(skew_w5,v5)
	S6 = make_s(skew_w6,v6)

	S = [S1,S2,S3,S4,S5,S6]

	R = [[0, -1, 0],
		 [0, 0, -1],
		 [1, 0, 0]]
	P = [390, 401, 215.5]
	M = make_s(R,P)
	M[3][3] = 1

	# print("S: ")
	# print(S)
	# print("M: ")
	# print(M)

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, s = Get_MS()

	T = np.matmul(expm(np.array(s[0])*theta1),expm(np.array(s[1])*theta2))
	T = np.matmul(T,expm(np.array(s[2])*theta3))
	T = np.matmul(T,expm(np.array(s[3])*theta4))
	T = np.matmul(T,expm(np.array(s[4])*theta5))
	T = np.matmul(T,expm(np.array(s[5])*theta6))
	T = np.matmul(T,M)
	print("T")
	print(str(T) + "\n")
	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	#link lengths
	L1 = 152
	L2 = 120
	L3 = 244
	L4 = 93
	L5 = 213
	L6 = 83
	L7 = 83
	L8 = 82
	L9 = 53.5
	L10 = 59

	#Convert world frame coordinates to new coordinates in terms of robot origin
	xgrip = xWgrip*1000 + 150
	ygrip = yWgrip*1000 - 150
	zgrip = zWgrip*1000 - 10
	print("Gripper Point", np.array([xgrip,ygrip,zgrip]))

	#Find wrist's center point based off gripper coordinates and known geometry of link 9
	xcen = xgrip - L9*np.cos(yaw_WgripDegree*PI/180.)
	ycen = ygrip - L9*np.sin(yaw_WgripDegree*PI/180.)
	zcen = zgrip
	print("Wrist Center Point", np.array([xcen,ycen,zcen]))

	#Calculate theta1 using 
	y_s = 110 #(Zero position) Y offset between base and wrist center
	theta_s = np.arcsin(y_s/np.sqrt(xcen**2+ycen**2)) #angle between points 3end and wrist center
	theta_b = np.arctan2(ycen,xcen) #angle between x axis and wrist center point
	theta1 = theta_b-theta_s

	theta6 = theta1 - yaw_WgripDegree*PI/180 + PI/2

	theta_x = np.arctan2(83,110)
	theta_3end = theta_x - theta1
	L_3end = np.sqrt(83**2+110**2)
	x3end = xcen - L_3end*np.sin(theta_3end)
	y3end = ycen - L_3end*np.cos(theta_3end)
	z3end = zcen + 141
	p_3end = np.array((x3end,y3end,z3end))
	print("3end Point", p_3end)

	# c is sup_theta3
	# L3 is A, L5 is B
	p_L1 = np.array((0,0,L1))
	C = np.linalg.norm(p_3end - p_L1)
	theta3 = PI - np.arccos((L3**2+L5**2-C**2)/(2*L3*L5))
	# c is t2_upper, A is C from above and B is L3
	theta2_upper = np.arccos((C**2+L3**2-L5**2)/(2*C*L3))
	theta2_lower = np.arcsin((z3end-L1)/C)
	theta2 = -(theta2_lower + theta2_upper)
	theta4 = -(theta3 + theta2)
	theta5 = -PI/2

	thetas = np.transpose(np.array([theta1, theta2, theta3, theta4, theta5, theta6]))
	# thetas = thetas*180/PI
	print("Thetas: ", thetas) 
	theta1 = thetas[0]
	theta2 = thetas[1]
	theta3 = thetas[2]
	theta4 = thetas[3]
	theta5 = thetas[4]
	theta6 = thetas[5]

	# theta1 = theta1 - PI/2

	# theta1 = 0.0
	# theta2 = 0.0
	# theta3 = 0.0
	# theta4 = 0.0
	# theta5 = 0.0
	# theta6 = 0.0
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

	# Tested points
	# Point 1: -0.2 0.4 0.1 270
	# Measured: -210 402 90 270
	# Point 2: 0.1 0.1 0.15 90
	# Measured: 102 95 141
	# Point 3: 0.25 0.15 0.1 67
	# Measured: 252 151 91 67