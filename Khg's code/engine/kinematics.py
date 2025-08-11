R0_6 = [[-1.0, 0.0, 0.0], 
                [0.0, -1.0, 0.0], 
                [0.0, 0.0, 1.0]]  # Rotation matrix from base to end-effector
        
R0_3 = [[-np.sin(Theta2), 0.0, np.cos(Theta2)],
        [np.cos(Theta2), 0.0, np.sin(Theta2)],
        [0.0, 1.0, 0.0]]  # Rotation matrix from base to wrist

invR0_3 = np.linalg.inv(R0_3)  # Inverse of the rotation matrix from base to wrist
R3_6 = np.dot(invR0_3, R0_6)  # Rotation matrix from wrist to end-effector
# R6_3 = np.linalg.inv(R3_6)  # Inverse of the rotation matrix from end-effector to wrist
print('R3_6 = ', R3_6)

# Theta5 = np.arctan2(np.sqrt(R3_6[0][2]**2 + R3_6[1][2]**2), R3_6[2][2])  # Wrist pitch
# Theta4 = np.arctan2(R3_6[1][2], R3_6[0][2])  # Wrist roll
# Theta6 = np.arctan2(R3_6[1][2], R3_6[0][2])  # Wrist yaw


Theta5 = np.arccos(R3_6[2][2])  # Wrist pitch, assuming R3_6[2][2] is cos(Theta5)
print('Theta5 = ', Theta5, 'radians')

Theta6 = np.arccos(-R3_6[2][2] / np.sin(Theta5))  # Wrist yaw, assuming R3_6[2][2] is cos(Theta5)
print('Theta6 = ', Theta6, 'radians')

Theta4 = np.arccos(R3_6[1][2] / np.sin(Theta5))  # Wrist roll, assuming R3_6[2][2] is cos(Theta5)
print('Theta4 = ', Theta4, 'radians')


inv0_3 = np.linalg.inv(R0_3)