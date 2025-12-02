import numpy as np
from cvxopt import matrix, solvers
# from scipy.optimize import minimize


def qp(ud, x, y, theta, x_o, y_o, r_o, alpha, l, d, max_speed):
    R_bar = np.array([[np.cos(theta), -l * np.sin(theta)],
                        [np.sin(theta), l * np.cos(theta)]])
    delta=0.5
    H = np.array([[R_bar[0, 0]**2 + R_bar[1, 0]**2 + delta, R_bar[0, 0] * R_bar[0, 1] + R_bar[1, 0] * R_bar[1, 1]],
                    [R_bar[0, 0] * R_bar[0, 1] + R_bar[1, 0] * R_bar[1, 1], R_bar[0, 1]**2 + R_bar[1, 1]**2]])
    f = -np.array([ud[0] * R_bar[0, 0] + ud[1] * R_bar[1, 0] + delta*0.02,
                    ud[0] * R_bar[0, 1] + ud[1] * R_bar[1, 1]])
    # constraints: obstacle + outward modify(deleted) + velocity
    A = np.zeros((len(x_o) + 1 + 4, 2))
    b = np.zeros(len(x_o) + 1 + 4)
    for i in range(len(x_o)):
        A[i, :] = -2 * np.array([
            R_bar[0, 0] * (x - x_o[i]) + R_bar[1, 0] * (y - y_o[i]),
            R_bar[0, 1] * (x - x_o[i]) + R_bar[1, 1] * (y - y_o[i])
        ])
        h = (x - x_o[i])**2 + (y - y_o[i])**2 - r_o[i]**2
        b[i] = alpha * h
    A[len(x_o), :] = [0, 0]
    A[len(x_o) + 1:len(x_o) + 5, :] = np.array([
        [1, d],[1, -d],[-1, -d],[-1, d]
    ])
    b[len(x_o) + 1:len(x_o) + 5] = np.ones(4) * max_speed

    P = matrix(H)
    q = matrix(f)
    G = matrix(A)
    h = matrix(b)
    solvers.options['show_progress'] = False
    solution = solvers.qp(P, q, G, h)
    u = np.array(solution['x']).flatten()
    u[0] = u[0]*1
    u[1] = u[1]*1
    return u

    # def objective(u):
    #     return 0.5 * u.T @ H @ u + f.T @ u

    # def constraint1(u):
    #     return b - A @ u
        
    # # x0 = np.array([ud[0], ud[1]])
    # x0 = np.array([0, 0])
    # constraints = [{'type': 'ineq', 'fun': constraint1}]
    # result = minimize(objective, x0, constraints=constraints, method='SLSQP')
    # return result.x