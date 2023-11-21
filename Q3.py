import numpy as np

def compute_manipulator_jacobian(dh_parameters, joint_types=None):
    num_links = len(dh_parameters)
    if joint_types is None:
        joint_types = ['revolute'] * num_links  # Default assumption: all joints are revolute

    # Initialize transformation matrices and joint velocities
    transforms = []
    joint_velocities = []
    for i in range(num_links):
        a, alpha, d, theta = dh_parameters[i]
        if joint_types[i] == 'revolute':
            theta = theta
            joint_velocities.append(1)  #taking default joint velocities as 1
        elif joint_types[i] == 'prismatic':
            d = d
            joint_velocities.append(1)  #taking default joint velocities as 1
        else:
            raise ValueError("Invalid joint type. Supported types: 'revolute' or 'prismatic'")

        transform = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        transforms.append(transform)

    # Compute manipulator Jacobian
    jacobian = np.zeros((6, num_links))
    end_effector_position = transforms[-1][:3, 3]

    #Jacobian matrix
    for i in range(num_links):
        if joint_types[i] == 'revolute':
            z_axis = transforms[i][:3, 2]
            jacobian[:3, i] = np.cross(z_axis, end_effector_position - transforms[i][:3, 3])
            jacobian[3:, i] = z_axis
        elif joint_types[i] == 'prismatic':
            jacobian[:3, i] = transforms[i][:3, 2]
        else:
            raise ValueError("Invalid joint type. Supported types: 'revolute' or 'prismatic'")

    # Compute end-effector velocity
    end_effector_velocity = np.dot(jacobian, joint_velocities)

    return jacobian, end_effector_position, end_effector_velocity
