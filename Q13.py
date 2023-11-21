import numpy as np

def inverse_kinematics_SCARA(x, y, l1, l2):
    a = l1
    b = l2

    # Compute theta2 
    r = np.sqrt(x**2 + y**2)
    alpha = np.arctan2(y, x)

    cos_theta2 = (r**2 - a**2 - b**2) / (2 * a * b)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)

    theta2 = np.arctan2(sin_theta2, cos_theta2)
    # compute theta1
    beta = np.arctan2(b * np.sin(theta2), a + b * np.cos(theta2))
    theta1 = alpha - beta

    return theta1, theta2


# Given end-effector position
x_desired = 1.5  #examplevalues
y_desired = 1.0 

l1 = 2.0  # length of the first arm
l2 = 3.0  # length of the second arm

# Calculate joint angles
theta1_result, theta2_result = inverse_kinematics_SCARA(x_desired, y_desired, l1, l2)

print("Inverse Kinematics - SCARA Manipulator:")
print(f"Theta1: {np.degrees(theta1_result)} degrees")
print(f"Theta2: {np.degrees(theta2_result)} degrees")
