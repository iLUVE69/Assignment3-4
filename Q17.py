import numpy as np

def inverse_kinematics_spherical_wrist(rotation_matrix):
    # Extract rotational components from the rotation matrix
    r11, r12, r13 = rotation_matrix[0]
    r21, r22, r23 = rotation_matrix[1]
    r31, r32, r33 = rotation_matrix[2]

    # (rotation about Z-axis)
    theta1 = np.arctan2(r21, r11)

    # (rotation about Y-axis)
    theta2 = np.arctan2(-r31, np.sqrt(r11**2 + r21**2))
    
    # (rotation about X-axis)
    theta3 = np.arctan2(r32, r33)

    return theta1, theta2, theta3
