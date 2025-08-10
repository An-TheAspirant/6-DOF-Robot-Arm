import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.animation as anime
import numpy as np

""" This code is a simulation of a 6-DOF robotic arm using DH parameters.
It calls upon the kinematics module to generate the transformation matrices
and updates the position text file. The simulation is visualized using a 3D plot through matplotlib.

Frame of this file:
    1. Generate plot
    2. Update DH parameters
    3. Show on plot
    4. Position text file
    5. 3D plot"""
    
    
    
X = 5.0
Y = 0.0
        
# Theta2 = np.arctan2(end_effector_position[1], end_effector_position[0])  # Base rotation angle
Theta2 = np.arctan2(Y, X)
print('Theta2 = ', Theta2, 'radians')

R0_6 = [[-1.0, 0.0, 0.0], 
        [0.0, -1.0, 0.0], 
        [0.0, 0.0, 1.0]]  # Rotation matrix from base to end-effector

R0_3 = [[-np.sin(Theta2), 0.0, np.cos(Theta2)],
        [np.cos(Theta2), 0.0, np.sin(Theta2)],
        [0.0, 1.0, 0.0]]  # Rotation matrix from base to wrist

# invR0_3 = np.linalg.inv(R0_3)  # Inverse of the rotation matrix from base to wrist
invR0_3 = np.transpose(R0_3)
R3_6 = np.dot(invR0_3, R0_6)  # Rotation matrix from wrist to end-effector
# R6_3 = np.linalg.inv(R3_6)  # Inverse of the rotation matrix from end-effector to wrist
print('R3_6 = ', R3_6)

# Theta4 = np.arctan2(R3_6[2][1], R3_6[2][2])  # Wrist roll
# Theta5 = np.arctan2(-R3_6[2][0], np.sqrt(R3_6[2][1]**2 + R3_6[2][2]**2))  # Wrist pitch
# Theta6 = np.arctan2(R3_6[1][0], R3_6[0][0])  # Wrist yaw
# print('Theta4 = ', Theta4, 'radians')
# print('Theta5 = ', Theta5, 'radians')
# print('Theta6 = ', Theta6, 'radians')

Theta5 = np.arccos(R3_6[2][2])  # Wrist pitch, assuming R3_6[2][2] is cos(Theta5)
print('Theta5 = ', Theta5, 'radians')

Theta6 = np.arccos(-R3_6[2][0] / np.sin(Theta5))  # Wrist yaw, assuming R3_6[2][0] is cos(Theta5)
print('Theta6 = ', Theta6, 'radians')

Theta4 = np.arccos(R3_6[1][2] / np.sin(Theta5))  # Wrist roll, assuming R3_6[2][2] is cos(Theta5)
print('Theta4 = ', Theta4, 'radians')   

st4 = np.sin(Theta4)
st5 = np.sin(Theta5)
st6 = np.sin(Theta6)
ct4 = np.cos(Theta4)    
ct5 = np.cos(Theta5)    
ct6 = np.cos(Theta6)
R3_6_check = [[-st4*ct5*ct6 - ct4*st6, st4*ct5*ct6 - ct4*ct6, -st4*st5],
              [ct4*ct5*ct6 - st4*st6, -ct4*ct5*st6 - st4*ct6, ct4*st5],
              [-st5*ct6, st5*st6 , ct5]]
# R3_6_check = [[ct4*ct5*ct6 - st4*st6, -ct4*ct5*st6 - st4*ct6, ct4*st5],
#               [st4*ct5*ct6 + ct4*ct6, -st4*ct5*st6 + ct4*ct6, st4*st5],
#               [-st5*ct6, st5*st6, ct5]]
R3_6_check = np.array(R3_6_check)
print('R3_6_check = ', R3_6_check)
# Check if R0_6 and R0_6_check are equal