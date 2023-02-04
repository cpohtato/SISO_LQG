from cmath import pi
import os
from re import L
from tkinter.tix import DisplayStyle
import numpy as np
import matplotlib.pyplot as plt
import control
from control.matlab import *

#   Plant characteristics
J = 0.015                   #   [kg.m^2]    Plant moment of inertia 
b = 0.06                    #   [N/s]       Damping constant 
r = 0.495                   #   [m]         Distance between propeller and pivot
f_s = 250                   #   [Hz]        Sample rate
dT = 1/f_s                  #   [s]         Discrete time step
tau = 0.02                  #   [s]         Propeller time constant
F_max = 8.08                #   [N]         Maximum motor thrust
F_hold = 1.3                #   [N]         Force required from the motor to keep stationary
u_max = 180                 #   [CMD]       Maximum command
t_d = 0.04                  #   [s]         Motor time delay
t_d_taps = t_d/dT           #   [samples]   Motor time delay in taps
order = 3                   #               System order

control.config.defaults['control.default_dt'] = dT

matrix_dim = order + int(t_d_taps)

#   System matrix   
Phi = np.zeros((matrix_dim, matrix_dim))
Phi[0, 0] = 1
Phi[0, 1] = dT
Phi[1, 1] = 1 - (b/J) * dT
Phi[1, 2] = (r/J) * dT
Phi[2, 2] = 1 - (dT/tau)
Phi[2, 3] = (F_max/u_max) * (dT/tau)

for i in range(int(t_d_taps) - 1):
    Phi[3 + i, 4 + i] = 1

print(Phi)

#   Control matrix
Gamma = np.zeros((matrix_dim, 1))
Gamma[matrix_dim - 1, 0] = 1

#   Output matrix
H = np.eye(matrix_dim)

#   Feedforward matrix
J = np.zeros((matrix_dim, 1))

dsys = ss(Phi, Gamma, H, J, dT)

#   State weight matrix
Q = np.zeros((matrix_dim, matrix_dim))
Q[0, 0] = 50                               #   Penalise positional error
Q[1, 1] = 0.1                               #   Penalise velocity
Q[2, 2] = 1                                 #   Penalise force produced by motor

#   Input weight matrix
R = np.zeros((1, 1))
R[0, 0] = 0.001                             #   Penalise control action

#   LQR gain generation
K_lqr, _, _ = dlqr(dsys, Q, R)

K_old = np.matrix([[74.02857219297229, 7.530723112226729, 7.672484413842515, 
0.06859110416741712, 0.06833398486553965, 0.06812261765393542, 0.06797227609036223, 
0.06790218582258756, 0.06793651727148964, 0.06810562664735831, 0.06844760738844509, 
0.0690102296330888, 0.06985336473995314, 0.07105201612463091, 0.07270010799822392, 
0.0749152214901122, 0.07784451500792432, 0.08167212490116765]])

cont = ss([],[],[], K_lqr, dT)

N_bar = -np.linalg.pinv(H*np.linalg.pinv(Phi-Gamma*K_lqr)*Gamma).T

dsys_fbk = control.feedback(dsys, K_lqr)

#dsys_loop = control.series(K_lqr, dsys)

#dsys_fbk = control.feedback(dsys_loop, np.eye(matrix_dim))

# dsys_full = control.series(N_bar, dsys_fbk)

#   Initial condition
x0 = np.zeros((matrix_dim, 1))
x0[0] = -0.4


#   Simulation parameters
simTime = 1
simSamples = int(simTime/dT)

#   Reference signal is zero
r = np.zeros((1, simSamples))
T, yout = control.forced_response(dsys_fbk, T=None, U=r, X0 = x0)

print()
print()

print("{", end='')
K_lqr_arr = np.array(K_lqr)
for i in range(matrix_dim):
    print(K_lqr_arr[0,i], end=', ')

for i in range(simSamples):
    with open("simLog.txt","w") as f:
        f.write(str(yout.T[i, 0]) + " " + str(yout.T[i, 1]) + "\n")



print()
print()

# print(yout.T[..., 0])
# print(yout.T[..., 1])

#   Plot results
fig, axs = plt.subplots(1,2)
fig.suptitle("Pendulum LQR")

axs[0].plot(T.T, yout.T[..., 0])
axs[0].set_title("Angular Response")
axs[0].set_xlim((0, simTime))
axs[0].set_xlabel("Time [s]")
axs[0].set_ylabel("Pitch [rad]")
axs[0].grid()

# axs[1].plot(T.T, yout.T[..., matrix_dim - 1])
# axs[1].set_title("Control Action")
axs[1].plot(T.T, yout.T[..., 1])
axs[1].set_title("Velocity")
axs[1].set_xlim((0, simTime))
axs[1].set_xlabel("Time [s]")
axs[1].set_ylabel("Velocity [rad/s]")
axs[1].grid()

plt.show(block = True)
