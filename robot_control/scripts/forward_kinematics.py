import numpy as np
from scipy.spatial.transform import Rotation

def rz(a):
    return np.array([
        [np.cos(a), -np.sin(a), 0, 0],
        [np.sin(a), np.cos(a), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def ry(a):
    return np.array([
        [np.cos(a), 0, np.sin(a), 0],
        [0, 1, 0, 0],
        [-np.sin(a), 0, np.cos(a), 0],
        [0, 0, 0, 1]
    ])

def rx(a):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(a), -np.sin(a), 0],
        [0, np.sin(a), np.cos(a), 0],
        [0, 0, 0, 1]
    ])

def trs(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def dh(q, alpha, r, d):
    mul1 = rz(q)
    mul2 = trs(0, 0, d)
    mul3 = rx(alpha)
    mul4 = trs(r, 0, 0)
    return ((mul1.dot(mul2)).dot(mul3)).dot(mul4)

def forward_kinematics(links_length, q_list):
    t1 = dh(q_list[0], -np.pi / 2, links_length[1], links_length[0])
    t2 = dh(q_list[1], 0, links_length[2], 0)
    t3 = dh(q_list[2] - (np.pi / 2), np.pi / 2, 0, 0)
    t4 = dh(q_list[3], -np.pi / 2, -0.035, -links_length[3])
    t5 = dh(q_list[4], np.pi / 2, 0, 0)
    t6 = dh(q_list[5], np.pi / 2, 0, -links_length[4])
    fk_sol = ((((t1.dot(t2)).dot(t3)).dot(t4)).dot(t5)).dot(t6)
    pos_vector = fk_sol[0, 3], fk_sol[1, 3], fk_sol[2, 3]
    
    eulerY = np.arctan2(np.sqrt(fk_sol[0, 2]**2 + fk_sol[1, 2]**2), fk_sol[2, 2])
    eulerZ = np.arctan2(fk_sol[2, 1], -fk_sol[2, 0])
    eulerX = np.arctan2(fk_sol[1, 2], fk_sol[0, 2])
    
    orientation = eulerX, eulerY, eulerZ
    return np.array([pos_vector, orientation])