import numpy as np


class Robot:

    def __init__(self):
        pass

    def forward_kinematics(self, x, y, theta, u, dt, l):
        v = u[0]
        omega = u[1]
        dx = v*np.cos(theta)
        dy = v*np.sin(theta)
        dtheta = omega
        x_new = (x + dx * dt)/1
        y_new = (y + dy * dt)/1
        theta_new = theta + dtheta * dt
        return x_new, y_new, theta_new


