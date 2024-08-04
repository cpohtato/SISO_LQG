# SISO_LQG
Implementation of LQG algorithm on SISO system using Python and Arduino.

Python script uses control library to simulate response of system to step input.
Pendulum arm is modelled as a discrete third-order dynamical system with time delay and control algorithm generates control gains for each state variable.

Control gains are implemented using Arduino firmware which also runs Kalman filter to fuse IMU data based on same system model.

Development log here: https://docs.google.com/document/d/13Hld7glFrMnmgPxS9Ic46O_s2rJUNu3mvF7G-0rXe5M/edit
