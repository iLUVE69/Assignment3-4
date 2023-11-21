import numpy as np

def inverse_kinematics_Stanford(x, y, z, l1, l2):
    d3 = z
    r = np.sqrt(x**2 + y**2)
    
    # Solve for theta2
    cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)
    
    # Solve for theta1
    alpha = np.arctan2(y, x)
    beta = np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    theta1 = alpha - beta
    
    return theta1, theta2, d3

# Arm lengths 
l1 = 2.0
l2 = 3.0

# Desired end-effector coordinates
x_desired = 3.0
y_desired = 4.0
z_desired = 5.0

theta1_result, theta2_result, d3_result = inverse_kinematics_Stanford(x_desired, y_desired, z_desired, l1, l2)

print("Inverse Kinematics - Stanford Manipulator:")
print(f"Theta1: {np.degrees(theta1_result)} degrees")
print(f"Theta2: {np.degrees(theta2_result)} degrees")
print(f"D3: {d3_result}")

# Calculate end-effector position using the calculated joint variables
end_effector_x = l1 * np.cos(theta1_result) + l2 * np.cos(theta1_result + theta2_result)
end_effector_y = l1 * np.sin(theta1_result) + l2 * np.sin(theta1_result + theta2_result)
end_effector_z = d3_result

# Print the calculated end-effector position
print("\nCalculated End-effector Position:")
print(f"X: {end_effector_x}, Y: {end_effector_y}, Z: {end_effector_z}")
