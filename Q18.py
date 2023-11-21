import numpy as np

# Forward Kinematics subroutine for PPP manipulator
def forward_kinematics_PPP(d_values):
    if len(d_values) != 3:
        raise ValueError("Invalid number of joint displacements for PPP manipulator. Expected 3.")

    d1, d2, d3 = d_values

    # Define transformation matrices for each prismatic joint
    T1 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d1],
        [0, 0, 0, 1]
    ])

    T2 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d2],
        [0, 0, 0, 1]
    ])

    T3 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d3],
        [0, 0, 0, 1]
    ])

    forward_transform = np.dot(np.dot(T1, T2), T3)
    end_effector_position = forward_transform[:3, 3]
    end_effector_orientation = forward_transform[:3, :3]

    return forward_transform, end_effector_position, end_effector_orientation

# Jacobian subroutine for PPP manipulator
def compute_jacobian_PPP(d_values):
    if len(d_values) != 3:
        raise ValueError("Invalid number of joint displacements for PPP manipulator. Expected 3.")

    d1, d2, d3 = d_values

    jacobian = np.zeros((6, 3))

    # Compute end-effector position for small changes in joint displacements
    epsilon = 1e-5

    for i in range(3):
        d_values_temp = d_values.copy()
        d_values_temp[i] += epsilon

        forward_transform, _, _ = forward_kinematics_PPP(d_values_temp)

        delta_pos = forward_transform[:3, 3] - forward_transform[:3, 3]
        delta_orient = np.dot(forward_transform[:3, :3], np.linalg.inv(forward_transform[:3, :3]))

        jacobian[:3, i] = delta_pos / epsilon
        jacobian[3:, i] = np.ravel(delta_orient) / epsilon

    return jacobian

dh_params_3d_printer = [
    [0, 0, 0, 'd1'],  # X-axis displacement
    [0, 0, 0, 'd2'],  # Y-axis displacement
    [0, 0, 0, 'd3']   # Z-axis displacement
]

# Representative numerical values for displacements
d_values = [5, 3, 7]  # [d1, d2, d3]

forward_transform, end_effector_pos, end_effector_orient = forward_kinematics_PPP(d_values)
jacobian = compute_jacobian_PPP(d_values)

# Print results
print("Forward Kinematics - 3D Printer:")
print("Complete Transformation Matrix:")
print(forward_transform)
print("\nEnd-effector Position (XYZ):")
print(end_effector_pos)
print("\nEnd-effector Orientation (Rotation Matrix):")
print(end_effector_orient)
print("\nManipulator Jacobian:")
print(jacobian)
