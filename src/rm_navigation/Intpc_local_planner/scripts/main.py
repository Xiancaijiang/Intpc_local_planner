#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time
import numpy as np
import matplotlib.pyplot as plt
import csv
from fourier import fit
from vector import reference
from cbf import qp
from kinematics import Robot


l = 0.01
wheel_base = 53e-3  
wheel_diameter = 40e-3 
# 1:star, 2:dinosaur, 3:eight, 4:ring
shape = 3
Gain = {1: 10-3*0, 2: 5, 3: 5, 4: 5}
Dis = {1: 0, 2: 0, 3: 0, 4: 0}
Velocity = {1: 0.03+0.03*0, 2: 0.03, 3: 0.05, 4: 0.07-0.04}
A = {1: 1-0.5, 2: 1-0.5, 3: 1-0.5, 4: 10}
Max = {1: 0.04, 2: 0.06, 3: 0.1, 4: 0.05}
Obstacle = {1: np.array([2.3, 0.9, 0.85,  1.07, 0.91, 0.5,   0.08, 0.09, 0.09]),
            2: np.array([2.5, 1.2, 1.3,   1.47, 0.75, 0.25,  0.09, 0.07, 0.07]),
            3: np.array([2.5, 1.2, 1.77,  1.4, 0.9, 0.4,     0.09, 0.09, 0.09]),
            4: np.array([2.5, 0.4, 1.6,   0.95, 1, 0.95,     0.09, 0.08, 0.07])}
Initial = {1: [3.0, 1.3, 0.0], 2: [3, 1.5, 1], 3: [2.8, 1.4, 1], 4:[3, 1.3, 1]}   
k, distance, vd, alpha, max_speed = Gain[shape], Dis[shape], Velocity[shape], A[shape], Max[shape]
num = int(len(Obstacle[shape])/3)
x_o, y_o, r_o = Obstacle[shape][0:num], Obstacle[shape][num:2*num], Obstacle[shape][2*num:3*num]
x, y, theta = Initial[shape]
f = 100


def robot_callback(msg):
    global x, y, theta, flag
    flag = 0
    x = msg.data[0] * 0.00228571
    y = msg.data[1] * 0.00228571
    theta = msg.data[2]
    # print(x)

def obstacle_callback(msg):
    global x_o, y_o, r_o
    num = int(len(msg.data)/3)
    x_o = msg.data[0:num] 
    y_o = msg.data[num:2*num] 
    r_o = msg.data[2*num:3*num] 
    delta = [0.063]*num
    # if x>1.6:
    #     delta = [0.074]*num
    r_o = [x+y for x, y in zip(r_o, delta)]
    # print(x_o)


if __name__ == '__main__':

    x, y, theta, x_o, y_o, r_o = 0.0, 0.0, 0.0, [0.0], [0.0], [0.0]
    # x, y, theta = 861.25*0.00228571, 354.0*0.00228571, -3.079173843593836

    x_out, y_out, error = [], [], []
    vel=[0]*2
    flag = 1
    filename = 'data' + str(shape) + '.csv'
    file = open(filename, 'w+', encoding='utf8', newline='')
    px, py = fit(shape)
    print('start')

    while not rospy.is_shutdown():
        try:
            rospy.init_node('boundary', anonymous=True)
            rate = rospy.Rate(f)
            pub = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)
        
            sub_robot = rospy.Subscriber('/robot', Float64MultiArray, robot_callback, queue_size=10)
            sub_obstacle = rospy.Subscriber('/obstacle', Float64MultiArray, obstacle_callback, queue_size=10)
            # print('subscribe')
            while flag:
                continue
            
            t = time.time()

            x_bar = x + l*np.cos(theta)
            y_bar = y + l*np.sin(theta)


            x_bar = 2.3948304772883
            y_bar = 1.29463277808356
            theta = -0.965251663189926

            e, ref = reference(k, x_bar, y_bar, px, py, vd, distance, shape)
            u = qp(ref, x_bar, y_bar, theta, x_o, y_o, r_o, alpha, l, wheel_base/2, max_speed)

            print(e,ref)
            print(u)

            # omega=[u[1]]
            # index=0
            # if x>1-0.003 and x<1+0.01:
            #     omega.append(u[1])
            #     index+=1
            #     delta=omega[index]-omega[index-1]
            #     if abs(delta)>0.07:
            #         omega[index]=omega[index-1]+0.00*delta/abs(delta)
            #         u[1]=omega[index]

            # if x_bar>1.05 and x_bar <1.15 and y_bar>0.93:
            #     if u[1]>0.1:
            #         u[1]=0.1
            #     if u[1]<-0.1:
            #         u[1]=-0.1

            # if u[1]>1.2:
            #     u[1]=1.2
            # if u[1]<-1.2:
            #     u[1]=-1.2

            vel_msg = Twist()
            vel_msg.linear.x = u[0]*100
            vel_msg.angular.z = -u[1]
            # rospy.loginfo(vel_msg)
            pub.publish(vel_msg)

            data = [e, x_bar, y_bar, theta, ref[0], ref[1], u[0], u[1], t]
            file = open(filename, 'a', encoding='utf8', newline='')
            writer = csv.writer(file)  
            writer.writerow(data)
            file.close()

            error.append(e)
            x_out.append(x_bar)
            y_out.append(y_bar)

            # bot_state = Robot()
            # x, y, theta = bot_state.forward_kinematics(x, y, theta, u, 0.1, l)

            rate.sleep()

        except rospy.ROSInterruptException:
            pass


    print('stop')
    plt.figure(1)
    plt.plot(error, linewidth=1, color='red')
    plt.figure(2)
    plt.plot(x_out, y_out, linewidth=1, color='blue')
    plt.axis('equal')
    plt.show()