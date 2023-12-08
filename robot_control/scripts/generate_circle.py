from . import build_circle
from . import ik_solver

import numpy as np

kr16_lim = [
    (-185, 185),
    (-155, 35),
    (-130, 154),
    (-350, 350),
    (-130, 130),
    (-350, 350)
]

def generate_traectory(clientID, links_length, center, radius, count, robot):
    circle_points = build_circle.calculate_points(count, radius, center)
    last_generalizeds = np.zeros(6)
    list_angles = []
    for point in circle_points:
        robot.get_generalized(clientID)
        q = ik_solver.ik_sol(links_length, point, np.array([0, 0, 0]), last_generalizeds)
        
        if q is not None:
            if (check_limits(np.rad2deg(q), kr16_lim)):
                last_generalizeds = q
                list_angles.append(q)
        else:
            list_angles.append(last_generalizeds)

    
    return list_angles

def check_limits(q, limits):
    for i in range(len(limits)):
        if q[i] < limits[i][0] or q[i] > limits[i][1]:
            print(q)
            print("LIMITS ERROR for this solution.")
            return False
    
    return True