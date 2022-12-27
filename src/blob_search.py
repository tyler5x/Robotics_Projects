#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = -0.01
beta = 770.3/1000 #pixels/m
tx = -319.4425953101001
ty = -15.698

# Function that converts image coord to world coord
def IMG2W(col, row):
    global theta 
    global beta 
    global tx 
    global ty 
    col = col - 320
    row = row - 240

    # Xw = row / beta - tx
    # Xw = (row*np.cos(theta)) - (col*np.sin(theta)) / beta - tx
    Xw = (((row/beta)-tx)*np.cos(theta)) - (((col/beta)-ty)*np.sin(theta))
    # Yw = (row*np.sin(theta)) + (col*np.cos(theta)) / beta - ty
    Yw = (((row/beta)-tx)*np.sin(theta)) + (((col/beta)-ty)*np.cos(theta))
    Xw = Xw / 1000
    Yw = Yw / 1000
    return (Xw,Yw)
# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 1000

    # Filter by Circularity
    params.filterByCircularity = False
    # params.minCircularity = 0.8

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    lower = (110,50,50)     # blue lower
    upper = (130,255,255)   # blue upper



    if(color == "orange"):
        lower = (10,190,125)     # orange lower
        upper = (20,255,255)   # orange upper
    else:
        lower = (23,150,125)
        upper = (35,255,255)



    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
   
    
    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(mask_image, keypoints, 0)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        x = 1
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)
    # print("_______________________")
    # print(blob_image_center)
    # print(xw_yw)
    # print("_______________________")

    return xw_yw
