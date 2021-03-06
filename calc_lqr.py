import numpy as np
import control

A = np.array([[0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0]])


B = np.array([[0, 0, 0],
              [0, 0, 0],
              [0, 0, 0],
              [1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

#Q = np.diag([10, 10, 100, 25, 25, 100])
Q = np.diag([0.001, 0.001, 10, 0.001, 0.0001, 100])
R = np.diag([1, 1, 1])

K, S, E  = control.lqr(A, B, Q, R)

np.savetxt('K_matrix', K, delimiter=', ', newline=',\n')
