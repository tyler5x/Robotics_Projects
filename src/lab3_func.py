#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

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
	# Fill in the correct values for S1~6, as well as the M matrix
	# omega
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

	print("S: ")
	print(S)
	print("M: ")
	print(M)

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	M, s = Get_MS()
	T = np.matmul(expm(np.array(s[0])*theta1),expm(np.array(s[1])*theta2))
	T = np.matmul(T,expm(np.array(s[2])*theta3))
	T = np.matmul(T,expm(np.array(s[3])*theta4))
	T = np.matmul(T,expm(np.array(s[4])*theta5))
	T = np.matmul(T,expm(np.array(s[5])*theta6))
	T = np.matmul(T,M)
	# ==============================================================#
	T = T.round(4)
	print("T")
	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

# ('w1: ', [0, 0, 1])
# ('w2: ', [0, 1, 0])
# ('w3: ', [0, 1, 0])
# ('w4: ', [0, 1, 0])
# ('w5: ', [1, 0, 0])
# ('w6: ', [0, 1, 0])
# ('q1: ', [-150, 150, 10])
# ('q2: ', array([-150,  270,  162]))
# ('q3: ', array([ 94, 270, 162]))
# ('q4: ', array([307, 177, 162]))
# ('q5: ', array([307, 260, 162]))
# ('q6: ', array([390, 260, 162]))
# ('v1: ', array([150, 150,   0]))
# ('v2: ', array([-162,    0, -150]))
# ('v3: ', array([-162,    0,   94]))
# ('v4: ', array([-162,    0,  307]))
# ('v5: ', array([   0,  162, -260]))
# ('v6: ', array([-162,    0,  390]))

# Test point 1: (20, -45, 105, -60, -90, 0)
# Measured x,y,z: [132,431,10]

# Test point 2: (-15, -80, 90, -50, -45, -20)
# Measured x,y,z: [235,305,362]