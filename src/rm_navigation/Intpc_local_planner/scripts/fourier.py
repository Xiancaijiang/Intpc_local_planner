import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat


def data_point(shape):
    if shape == 1:  
        theta = np.arange(0, 2 * np.pi, 0.07)
        rho = 4 * 0.05 * (1 + 1.5 ** np.sin(5 * theta))
        x = rho * np.cos(theta) + 1.2
        y = rho * np.sin(theta) + 0.7
    elif shape == 2:
        pts = loadmat('dinosaur.mat')
        x, y = pts['datax']/220+0.1, pts['datay']/230+0.1
    elif shape == 3:
        pts = loadmat('eight.mat')
        x, y = pts['datax']/300+0.1+0.1, pts['datay']/250+0.05+0.1
    elif shape == 4:
        pts = loadmat('ring.mat')
        x, y = pts['datax']/200+0.1, pts['datay']/300+0.1
    return x, y

def fitting(x, y, theta, harmonic):
    number = len(theta)
    G = np.ones((2 * number, 4 * harmonic))
    for i in range(2 * number):
        for j in range(4 * harmonic):
            if i % 2 == 0:
                if j % 4 == 0:
                    G[i, j] = np.cos((j // 4 + 1) * theta[i // 2])
                elif j % 4 == 1:
                    G[i, j] = np.sin((j // 4 + 1) * theta[i // 2])
                else:
                    G[i, j] = 0
            else:
                if j % 4 == 2:
                    G[i, j] = np.cos((j // 4 + 1) * theta[i // 2])
                elif j % 4 == 3:
                    G[i, j] = np.sin((j // 4 + 1) * theta[i // 2])
                else:
                    G[i, j] = 0
    I = np.tile(np.eye(2), (number, 1))
    H = np.hstack((G, I))
    L = np.zeros(2 * number)
    for i in range(2 * number):
        if i % 2 == 0:
            L[i] = x[i // 2]
        else:
            L[i] = y[i // 2]
    parameter = np.linalg.inv(H.T @ H) @ H.T @ L
    px = []
    py = []
    for i in range(len(parameter) - 2):
        if (i // 2) % 2 == 0:
            px.append(parameter[i])
        else:
            py.append(parameter[i])
    px.append(parameter[-2])
    py.append(parameter[-1])
    return px, py


def fit(shape):
    x, y = data_point(shape)
    if shape == 1:
        theta = np.arange(0, 2 * np.pi, 0.07)
        px, py = fitting(x, y, theta, 20)

    elif shape == 2:
        x1, y1, theta1, x2, y2, theta2 = [], [], [], [], [], []
        xc1, yc1, xc2, yc2 = 0.6, 0.5, 1.45, 0.7
        for i in range(x.shape[1]):
            if x[0,i]<1:
                x1.append(x[0,i])
                y1.append(y[0,i])
                theta1.append(np.arctan2(y[0,i]-yc1, x[0,i]-xc1))
            else:
                x2.append(x[0,i])
                y2.append(y[0,i])
                theta2.append(np.arctan2(y[0,i]-yc2, x[0,i]-xc2))
        px1, py1 = fitting(x1, y1, theta1, 20)
        px2, py2 = fitting(x2, y2, theta2, 30)
        px = {1:px1, 2:px2}
        py = {1:py1, 2:py2}

    elif shape == 3:
        x1, y1, theta1, x2, y2, theta2, x3, y3, theta3 = [], [], [], [], [], [], [], [], []
        xc1, yc1, xc2, yc2, xc3, yc3 = 0.8+0.1, 0.7+0.1, 1.25+0.1, 0.75+0.1, 2.1+0.1, 0.8+0.1
        for i in range(x.shape[1]):
            if x[0,i]<1+0.1:
                x1.append(x[0,i])
                y1.append(y[0,i])
                theta1.append(np.arctan2(y[0,i]-yc1, x[0,i]-xc1))
            elif x[0,i]>1+0.1 and x[0,i]<1.5+0.1:
                x2.append(x[0,i])
                y2.append(y[0,i])
                theta2.append(np.arctan2(y[0,i]-yc2, x[0,i]-xc2))
            else:
                x3.append(x[0,i])
                y3.append(y[0,i])
                theta3.append(np.arctan2(y[0,i]-yc3, x[0,i]-xc3))
        px1, py1 = fitting(x1, y1, theta1, 10)
        px2, py2 = fitting(x2, y2, theta2, 20)
        px3, py3 = fitting(x3, y3, theta3, 10)
        px = {1:px1, 2:px2, 3:px3}
        py = {1:py1, 2:py2, 3:py3}

    elif shape == 4:
        x1, y1, theta1, x2, y2, theta2 = [], [], [], [], [], []
        xc1, yc1, xc2, yc2 = 1.2, 0.7, 1.2, 1
        for i in range(x.shape[1]):
            if ((x[0,i]-1.2)**2/1.5+(y[0,i]-0.7)**2)<0.27 and y[0,i]>0.32:
                x1.append(x[0,i])
                y1.append(y[0,i])
                theta1.append(np.arctan2(y[0,i]-yc1, x[0,i]-xc1))
            else:
                x2.append(x[0,i])
                y2.append(y[0,i])
                theta2.append(np.arctan2(y[0,i]-yc2, x[0,i]-xc2))
        px1, py1 = fitting(x1, y1, theta1, 5)
        px2, py2 = fitting(x2, y2, theta2, 20)
        px = {1:px1, 2:px2}
        py = {1:py1, 2:py2}

    return px, py