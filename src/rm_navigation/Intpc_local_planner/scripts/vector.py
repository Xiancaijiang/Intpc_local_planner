import numpy as np


def reference(k, x, y, px, py, vd, distance, shape):
    if shape == 1:
        x_c, y_c, harmonic = 1.2, 0.7, 20
        rho = np.arctan2(y-y_c, x-x_c)
        f = np.zeros(2 * harmonic + 1)
        basis = np.zeros(2 * harmonic + 1)
        for j in range(1, 2 * harmonic + 1):
            if j % 2 == 1:
                f[j - 1] = np.cos(np.ceil(j / 2) * rho)
                basis[j - 1] = -np.ceil(j / 2) * np.sin(np.ceil(j / 2) * rho)
            else:
                f[j - 1] = np.sin(np.ceil(j / 2) * rho)
                basis[j - 1] = np.ceil(j / 2) * np.cos(np.ceil(j / 2) * rho)
        f[2 * harmonic] = 1
        basis[2 * harmonic] = 0
        tau_x = np.dot(basis, px)
        tau_y = np.dot(basis, py)
        tau = np.array([tau_x, tau_y])
        E = np.array([[0, 1], [-1, 0]])
        n = np.linalg.inv(E).dot(tau)
        cx = np.dot(f, px)
        cy = np.dot(f, py)
        e = np.sqrt((x-x_c)**2 + (y-y_c)**2) - np.sqrt((cx-x_c)**2 + (cy-y_c)**2) - distance
        vf = tau + k * e * n
        vf = vd * vf / np.linalg.norm(vf)

    elif shape == 2:
        xc1, yc1, xc2, yc2, h1, h2 = 0.6, 0.5, 1.45, 0.7, 20, 30
        if x<1:
            x_c, y_c, harmonic, px, py = xc1, yc1, h1, px[1], py[1] 
        else:
            x_c, y_c, harmonic, px, py = xc2, yc2, h2, px[2], py[2]
        
        rho = np.arctan2(y-y_c, x-x_c)
        f = np.zeros(2 * harmonic + 1)
        basis = np.zeros(2 * harmonic + 1)
        for j in range(1, 2 * harmonic + 1):
            if j % 2 == 1:
                f[j - 1] = np.cos(np.ceil(j / 2) * rho)
                basis[j - 1] = -np.ceil(j / 2) * np.sin(np.ceil(j / 2) * rho)
            else:
                f[j - 1] = np.sin(np.ceil(j / 2) * rho)
                basis[j - 1] = np.ceil(j / 2) * np.cos(np.ceil(j / 2) * rho)
        f[2 * harmonic] = 1
        basis[2 * harmonic] = 0
        tau_x = np.dot(basis, px)
        tau_y = np.dot(basis, py)
        tau = np.array([tau_x, tau_y])
        E = np.array([[0, 1], [-1, 0]])
        n = np.linalg.inv(E).dot(tau)
        cx = np.dot(f, px)
        cy = np.dot(f, py)
        e = np.sqrt((x-x_c)**2 + (y-y_c)**2) - np.sqrt((cx-x_c)**2 + (cy-y_c)**2) - distance
        vf = tau + k * e * n
        vf = vd * vf / np.linalg.norm(vf)

    elif shape == 3:
        xc1, yc1, xc2, yc2, xc3, yc3, h1, h2, h3 = 0.8+0.1, 0.7+0.1, 1.25+0.1, 0.75+0.1, 2.1+0.1, 0.8+0.1, 10, 20, 10
        if x<1+0.1:
            x_c, y_c, harmonic, px, py = xc1, yc1, h1, px[1], py[1] 
        elif x>1+0.1 and x<1.5+0.1:
            x_c, y_c, harmonic, px, py = xc2, yc2, h2, px[2], py[2]
        else:
            x_c, y_c, harmonic, px, py = xc3, yc3, h3, px[3], py[3]
        rho = np.arctan2(y-y_c, x-x_c)
        f = np.zeros(2 * harmonic + 1)
        basis = np.zeros(2 * harmonic + 1)
        for j in range(1, 2 * harmonic + 1):
            if j % 2 == 1:
                f[j - 1] = np.cos(np.ceil(j / 2) * rho)
                basis[j - 1] = -np.ceil(j / 2) * np.sin(np.ceil(j / 2) * rho)
            else:
                f[j - 1] = np.sin(np.ceil(j / 2) * rho)
                basis[j - 1] = np.ceil(j / 2) * np.cos(np.ceil(j / 2) * rho)
        f[2 * harmonic] = 1
        basis[2 * harmonic] = 0
        tau_x = np.dot(basis, px)
        tau_y = np.dot(basis, py)
        tau = np.array([tau_x, tau_y])
        E = np.array([[0, 1], [-1, 0]])
        n = np.linalg.inv(E).dot(tau)
        cx = np.dot(f, px)
        cy = np.dot(f, py)
        e = np.sqrt((x-x_c)**2 + (y-y_c)**2) - np.sqrt((cx-x_c)**2 + (cy-y_c)**2) - distance
        vf = tau + k * e * n
        vf = vd * vf / np.linalg.norm(vf)

    elif shape == 4:
        xc1, yc1, xc2, yc2, h1, h2 = 1.2, 0.7, 1.2, 1, 5, 20
        if ((x-1.2)**2/1.5+(y-0.7)**2)<0.27 and y>0.307 and x<1.21:
            x_c, y_c, harmonic, px, py = xc1, yc1, h1, px[1], py[1] 
            E = np.array([[0, -1], [1, 0]])
            direction = -1
        elif ((x-1.2)**2/1.5+(y-0.7)**2)<0.27 and y>0.33 and x>1.2:
            x_c, y_c, harmonic, px, py = xc1, yc1, h1, px[1], py[1] 
            E = np.array([[0, -1], [1, 0]])
            direction = -1
        else:
            x_c, y_c, harmonic, px, py = xc2, yc2, h2, px[2], py[2]
            E = np.array([[0, 1], [-1, 0]])
            direction = 1
        rho = np.arctan2(y-y_c, x-x_c)
        f = np.zeros(2 * harmonic + 1)
        basis = np.zeros(2 * harmonic + 1)
        for j in range(1, 2 * harmonic + 1):
            if j % 2 == 1:
                f[j - 1] = np.cos(np.ceil(j / 2) * rho)
                basis[j - 1] = -np.ceil(j / 2) * np.sin(np.ceil(j / 2) * rho)
            else:
                f[j - 1] = np.sin(np.ceil(j / 2) * rho)
                basis[j - 1] = np.ceil(j / 2) * np.cos(np.ceil(j / 2) * rho)
        f[2 * harmonic] = 1
        basis[2 * harmonic] = 0
        tau_x = np.dot(basis, px)
        tau_y = np.dot(basis, py)
        tau = direction * np.array([tau_x, tau_y])
        n = np.linalg.inv(E).dot(tau)
        cx = np.dot(f, px)
        cy = np.dot(f, py)
        e = np.sqrt((x-x_c)**2 + (y-y_c)**2) - np.sqrt((cx-x_c)**2 + (cy-y_c)**2) - distance
        vf = tau + k * e * n
        vf = vd * vf / np.linalg.norm(vf)
 
    return e, vf