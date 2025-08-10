import math
# New robot arm link lengths for different configuration
L1_new = 4
L2_new = 3
L3_new = 2

# Forward kinematics for the new configuration
def forward_kinematics_new(theta1, theta2, theta3):
    """
    Calculate the (x, y) position of the end-effector based on given joint angles.

    Parameters:
        theta1 (float): Angle of joint 1 in degrees.
        theta2 (float): Angle of joint 2 in degrees.
        theta3 (float): Angle of joint 3 in degrees.

    Returns:
        x (float): x-coordinate of the end-effector.
        y (float): y-coordinate of the end-effector.
    """
    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)
    theta3 = math.radians(theta3)
    
    # Compute the (x, y) position of the end-effector
    x = (L1_new * math.cos(theta1) +
         L2_new * math.cos(theta1 + theta2) +
         L3_new * math.cos(theta1 + theta2 + theta3))
    
    y = (L1_new * math.sin(theta1) +
         L2_new * math.sin(theta1 + theta2) +
         L3_new * math.sin(theta1 + theta2 + theta3))
    
    return x, y

# Inverse kinematics for the new configuration
def improved_inverse_kinematics_new(x_eff, y_eff):
    # Step 1: Calculate theta1
    theta1 = math.degrees(math.atan2(y_eff, x_eff))
    
    # Recalculate the distance from the base to the end-effector
    r = math.sqrt(x_eff**2 + y_eff**2)
    
    # Step 2: Calculate theta2 using the law of cosines with clamping for valid range
    c2 = (r**2 - L1_new**2 - L2_new**2) / (2 * L1_new * L2_new)
    c2 = max(-1, min(1, c2))  # Clamp the value between -1 and 1
    theta2 = math.degrees(math.acos(c2))
    
    # Adjust theta2 to be relative to the current configuration
    theta2_ref = math.degrees(math.atan2(y_eff - L1_new * math.sin(math.radians(theta1)), 
                                         x_eff - L1_new * math.cos(math.radians(theta1))))
    theta2 = theta2_ref - theta1
    
    # Step 3: Calculate theta3 based on the remaining angle needed to align with the target
    x_partial = x_eff - L1_new * math.cos(math.radians(theta1)) - L2_new * math.cos(math.radians(theta1 + theta2))
    y_partial = y_eff - L1_new * math.sin(math.radians(theta1)) - L2_new * math.sin(math.radians(theta1 + theta2))
    theta3 = math.degrees(math.atan2(y_partial, x_partial))
    
    return theta1, theta2, theta3

# Example usage with new configuration
theta1_new, theta2_new, theta3_new = 45, 30, 15  # New joint angles
x_eff_new, y_eff_new = forward_kinematics_new(theta1_new, theta2_new, theta3_new)

# Calculate inverse kinematics for the resulting (x, y) position
theta1_ik_new, theta2_ik_new, theta3_ik_new = improved_inverse_kinematics_new(x_eff_new, y_eff_new)

print(x_eff_new, y_eff_new)
print(theta1_ik_new, theta2_ik_new, theta3_ik_new)
