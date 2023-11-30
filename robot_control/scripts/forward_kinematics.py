import numpy as np

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

def forward_kinematics(links_length, q_list):
    t1 = dh(q_list[0], np.pi / 2, links_length[1], links_length[0])
    t2 = dh(q_list[1] + np.pi / 2, 0, links_length[2], 0)
    t3 = dh(q_list[2], np.pi / 2, 0, 0)
    t4 = dh(q_list[3], 0, 0, links_length[3] + links_length[4])
    t5 = dh(q_list[4], np.pi / 2, 0, 0)
    t6 = dh(q_list[5] + np.pi / 2, 0, 0, links_length[5] + links_length[6])
    
    pos_vector = t6[3, 0], t6[3, 1], t6[3, 2]
    phi_x = np.arctan2(t6[2, 0], t6[2, 1])
    phi_y = np.arctan2(np.sqrt(1 - t6[2, 2]))
    phi_z = np.arctan2(t6[0, 2], t6[1, 2])
    orientation = phi_x, phi_y, phi_z
    
    return (pos_vector, orientation)