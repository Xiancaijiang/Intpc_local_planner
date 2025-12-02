#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import cv2
from cv2 import aruco
import csv


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters()
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out_rgb = cv2.VideoWriter('output_rgb.avi', fourcc, 30.0, (1280,720), True)
# ob = cv2.VideoWriter('ob.avi', fourcc, 30.0, (1280,720), True)
number = 3

def create_robot_pose_array(corners, markerIds):
    # Calculate the center of the ArUco marker corners
    center_x = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4
    center_y = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4
    yaw = math.atan2(corners[0][0][0][1] - corners[0][0][3][1], corners[0][0][0][0] - corners[0][0][3][0])

    state = [0]*4
    state[0] = center_x
    state[1] = center_y
    state[2] = yaw
    # state[3] = markerIds
    return state


if __name__ == '__main__': 

    rospy.init_node('detect', anonymous=True)
    pub1 = rospy.Publisher('/robot', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/obstacle', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(200)

    # filename = 'obstacle.csv'
    # file = open(filename, 'w+', encoding='utf8', newline='')

    videorgb = cv2.VideoCapture('/dev/video0')
    videorgb.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    videorgb.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
    videorgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    videorgb.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # Camera
    fx_r = 582.59118861
    fy_r = 582.65884802
    cx_r= 629.53535406
    cy_r = 348.71988126
    k1_r = 0.00239457
    k2_r = -0.03004914
    p1_r = -0.00062043
    p2_r = -0.00057221
    k3_r= 0.01083464
    cameraMatrix_r = np.array([[fx_r,0.0,cx_r], [0.0,fy_r,cy_r], [0.0,0.0,1]], dtype=np.float32)
    distCoeffs_r = np.array([k1_r, k2_r, p1_r, p2_r, k3_r], dtype=np.float32)
    dz2 = (1280,720)
    newCameraMatrix_r, _= cv2.getOptimalNewCameraMatrix(cameraMatrix_r, distCoeffs_r, dz2, 0, dz2)
    map1_r, map2_r = cv2.initUndistortRectifyMap(cameraMatrix_r, distCoeffs_r, None, newCameraMatrix_r, dz2, cv2.CV_16SC2)


    while True:
        ret, imgrgb = videorgb.read()
        img = cv2.remap(imgrgb, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

        detector = aruco.ArucoDetector(dictionary, parameters)
        frame = img
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
        # img = aruco.drawDetectedMarkers(img, markerCorners, markerIds)
        if(markerIds is not None):
            print("length of markerIds", len(markerIds))
            corners = markerCorners
            ARUIDS = markerIds
            state = create_robot_pose_array(corners, markerIds)
            data = Float64MultiArray(data=state)
            print(data)
            pub1.publish(data)

 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_red1 = np.array([[0, 120, 120]])
        h_red1 = np.array([10, 255, 255])
        l_red2 = np.array([[160, 120, 120]])
        h_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, l_red1, h_red1)
        mask2 = cv2.inRange(hsv, l_red2, h_red2)
        mask = mask1+mask2
        # res = cv2.bitwise_and(frame, frame, mask = mask)
        # cv2.imshow('mask', mask)
        x_o, y_o, r_o = [], [], []

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        print(len(contours))
        circles = []
        for contour in contours:
            (x,y), radius = cv2.minEnclosingCircle(contour)
            circles.append((x, y, radius))
        circles = sorted(circles, key=lambda x: x[2], reverse=True)[:number]
        for (x, y, r) in circles:
            x_o.append(x * 0.00228571)
            y_o.append(y * 0.00228571)
            r_o.append(r * 0.00228571)
            # cv2.circle(img, (int(x), int(y)), int(r), (0, 255, 0), 2)
            # print(x,y,r)
        # cv2.imshow('mask', mask)

        # see = cv2.resize(frame, (160,90))
        # cv2.imshow('img', see)

        obstacle = x_o + y_o + r_o
        data = Float64MultiArray(data=obstacle)
        # rospy.loginfo(data)
        pub2.publish(data)

        out_rgb.write(img)
        # ob.write(mask)

        # file = open(filename, 'a', encoding='utf8', newline='')
        # writer = csv.writer(file)  
        # writer.writerow(obstacle)
        # file.close()
        
        rate.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    out_rgb.release()
    # ob.release()
    videorgb.release()
    cv2.destroyAllWindows()