#!/usr/bin/env python3
# show the boundary and obstacle
import cv2
import numpy as np
import time
from fourier import data_point

shape = 3
x, y = data_point(shape)
x=x.flatten()
y=y.flatten()
# print(x)
# print(len(x))
point = []
for i in range(len(x)):
    point.append([x[i]/0.00228571,y[i]/0.00228571])
# print(point)

# read the image, show the obstacle
videorgb = cv2.VideoCapture('/dev/video0')
videorgb.set(cv2.CAP_PROP_BUFFERSIZE, 1)
videorgb.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
videorgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
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


while videorgb.isOpened():
    
    ret, imgrgb = videorgb.read()
    
    while ret:
        img = cv2.remap(imgrgb, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

        # plot the boundary points
        for p in point:
            # print(p)
            # print(p[0])
            pp = [int(p[0]), int(p[1])]
            cv2.circle(imgrgb, pp, 1, (255, 0, 0), 1)
        
        see = cv2.resize(imgrgb, (480,270))
        cv2.imshow('img',see)

        # time.sleep(0.1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

videorgb.release()
cv2.destroyAllWindows()

