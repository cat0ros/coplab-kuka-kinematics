"""
Optimizer Module
"""

import numpy as np
from . import forward_kinematics

def derivative(f, x, epsilon = 1e-10):
    x_ = x + epsilon
    value = (f(x_) - f(x)) / epsilon

    return value

def partial_derivative(f, x, i, epsilon = 1e-10):
    x_ = np.copy(x).astype(np.float64)
    x_[i] = x_[i] + epsilon
    value = (f(x_) - f(x)) / epsilon

    return value

def jacobian(f, x, epsilon = 1e-10):
    f_ = f(x)
    value = np.zeros((len(f_), len(x)))
    
    for i in range(len(x)):
        f_ = partial_derivative(f, x, i, epsilon)
        value[:,i] = f_

    return value

def newton_method(f, x_init = 0, epsilon = 1e-10):
    prev_value = x_init + 2 * epsilon
    value = x_init

    iterations = 0
    while abs(prev_value - value) > epsilon:
        prev_value = value

        f_dash = derivative(f, value)
        value = value - f(value) / f_dash

        iterations += 1

    print(f"Newton Method converged in {iterations} iterations")

    return value

def check_singularity(jacobian_matrix):
    return np.linalg.matrix_rank(jacobian_matrix) < 3

def newton_method_vector(f, x_init, epsilon = 1e-10, max_iterations = 1000):
    prev_value = x_init + 2 * epsilon
    value = x_init

    iterations = 0
    while np.any(np.abs(prev_value - value) > epsilon):
        prev_value = value
        j = jacobian(f, value)
        
        if (check_singularity(j)):
            print('Avoiding singularity of the robot.')
            return None
        
        value = value - np.dot(np.linalg.pinv(j), f(value))

        iterations += 1
        
        if (iterations > 1000):
            print("Can't calculate angles for this point.")
            return None

    print(f"Newton Method converged in {iterations} iterations")

    return value

def ik_sol(links_length, pos, orientation, initial_state=np.zeros(6)):
    def ik_add_func(q):
        error_pos = pos - forward_kinematics.forward_kinematics(links_length, q)[0]
        error_orientation = orientation - forward_kinematics.forward_kinematics(links_length, q)[1]
        return np.array([error_pos[0], error_pos[1], error_pos[2]])
    
    solution_ik_original = newton_method_vector(ik_add_func, initial_state)
    if (solution_ik_original is not None):
        return np.deg2rad(wrap_angles(solution_ik_original))
    else:
        return None

def wrap_angle(angle):
    angle = np.rad2deg(angle)
    return (((angle + 180) % 360) + 360) % 360 - 180

def wrap_angles(angles_list):
    new_angles_list = np.array([0, 0, 0, 0, 0, 0])
    for i in range(len(angles_list)):
        new_angles_list[i] = wrap_angle(angles_list[i])
    return new_angles_list