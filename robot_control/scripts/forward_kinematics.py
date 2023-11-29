import numpy as np

links_length = [
    
]

def rotate_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [-np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotate_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotate_x(angle):
    return np.array([
        [0, np.cos(angle), -np.sin(angle), 0],
        [0, 0, 1, 0],
        [0, np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def translate(x, y, z):
    return np.array([
        [0, 0, 0, x],
        [0, 0, 0, y],
        [0, 0, 0, z],
        [0, 0, 0, 1]
    ])

def dh(q, alpha, r, d):
    return rotate_z(q) * translate(0, 0, d) * rotate_x(alpha) * translate(r, 0, 0)

def forward_kinematics():