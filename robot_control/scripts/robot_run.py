from . import sim
from . import build_circle
from . import forward_kinematics
from . import ik_solver

import numpy as np
import sys
import time

class RobotControl:
    def __init__(self, clientID):
        err1, self.j1 = sim.simxGetObjectHandle(clientID, 'joint_a1', sim.simx_opmode_oneshot_wait)
        err2, self.j2 = sim.simxGetObjectHandle(clientID, 'joint_a2', sim.simx_opmode_oneshot_wait)
        err3, self.j3 = sim.simxGetObjectHandle(clientID, 'joint_a3', sim.simx_opmode_oneshot_wait)
        err4, self.j4 = sim.simxGetObjectHandle(clientID, 'joint_a4', sim.simx_opmode_oneshot_wait)
        err5, self.j5 = sim.simxGetObjectHandle(clientID, 'joint_a5', sim.simx_opmode_oneshot_wait)
        err5, self.j6 = sim.simxGetObjectHandle(clientID, 'joint_a6', sim.simx_opmode_oneshot_wait)
        self.currentGeneralized = np.array([0, 0, 0, 0, 0, 0], float)
        self.set_generalized(clientID, self.currentGeneralized)

    def get_generalized(self, clientID):
        err1, self.currentGeneralized[0] = sim.simxGetJointPosition(clientID, self.j1, sim.simx_opmode_oneshot_wait)
        err2, self.currentGeneralized[1] = sim.simxGetJointPosition(clientID, self.j2, sim.simx_opmode_oneshot_wait)
        err3, self.currentGeneralized[2] = sim.simxGetJointPosition(clientID, self.j3, sim.simx_opmode_oneshot_wait)
        err4, self.currentGeneralized[3] = sim.simxGetJointPosition(clientID, self.j4, sim.simx_opmode_oneshot_wait)
        err5, self.currentGeneralized[4] = sim.simxGetJointPosition(clientID, self.j5, sim.simx_opmode_oneshot_wait)
        err6, self.currentGeneralized[5] = sim.simxGetJointPosition(clientID, self.j6, sim.simx_opmode_oneshot_wait)
        
    def set_generalized(self, clientID, generalized):
        generalized = np.deg2rad(generalized)
        sim.simxSetJointTargetPosition(clientID, self.j1, generalized[0], sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetPosition(clientID, self.j2, generalized[1], sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetPosition(clientID, self.j3, generalized[2], sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetPosition(clientID, self.j4, generalized[3], sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetPosition(clientID, self.j5, generalized[4], sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetPosition(clientID, self.j6, generalized[5], sim.simx_opmode_oneshot_wait)
        self.currentGeneralized = generalized
    
def connect_coppelia(ip, port):
    sim.simxFinish(-1)
    clientId = sim.simxStart(ip, port, True, True, 5000, 5)
    if clientId != -1:
        print("Connected to Remote API Server")
        return clientId
    else:
        print("Connection failed")
        sys.exit('Could not reconnect')

def main():
    point1 = np.array([0.200, 100, 0.250])
    point2 = np.array([0.250, 100, 0.300])
    point3 = np.array([0.150, 0.100, 0.100])
    x, z, radius = build_circle.build(point1, point2, point3)
    

if __name__ == '__main__':
    clientID = connect_coppelia('127.0.0.1', 19990)
    links_length = [0.675, 0.260, 0.680, 0.670, 0.158]
    main()
    robot = RobotControl(clientID)
    gens = np.deg2rad(np.array([90, -90, 45, 0, 0, 0]))
    pos, rot = forward_kinematics.forward_kinematics(links_length, gens)
    robot.set_generalized(clientID, np.rad2deg(gens))
    x, y, z = pos[0], pos[1], pos[2]
    #q = ik_solver.numerical_inverse_kinematics(pos, links_length, gens)
    #pos[2] -= 0.2
    #q = ik_solver.numerical_inverse_kinematics(pos, links_length, gens)
    #print(np.rad2deg(q))
    p_desired = np.array([x, y, z])  # Desired position
    orientation_desired = np.array([0, 0, 0])
    q_solution = ik_solver.ik_sol(links_length, p_desired, orientation_desired, np.deg2rad(np.array([0, 0, 0, 0, 0, 0])))
    fk_result = forward_kinematics.forward_kinematics(links_length, q_solution)
    print(np.round(fk_result[0], 2))
    print(np.round((x, y, z), 2))
    q_new = np.array([0, 0, 0, 0, 0, 0])
    for i in range(len(q_solution)):
        q_new[i] = np.rad2deg(np.arctan2(np.sin(q_solution[i]), np.cos(q_solution[i])))

    gens = np.array([0, 0, 0, 0, 0, 0])
    robot.set_generalized(clientID, np.rad2deg(gens))
    time.sleep(3)
    robot.set_generalized(clientID, q_new)
    err, cuboid = sim.simxGetObjectHandle(clientID, 'Cuboid', sim.simx_opmode_oneshot_wait)
    err, robot_handle = sim.simxGetObjectHandle(clientID, 'base_link_respondable', sim.simx_opmode_oneshot_wait)
    err2, cub_pos = sim.simxGetObjectPosition(clientID, cuboid, robot_handle, sim.simx_opmode_oneshot_wait)
    err3, cub_or = sim.simxGetObjectPosition(clientID, cuboid, robot_handle, sim.simx_opmode_oneshot_wait)
    theta1 = np.rad2deg(np.arctan2(cub_pos[0], cub_pos[1]))
    q_solution = ik_solver.ik_sol(links_length, cub_pos, cub_or, np.deg2rad(np.array([0, 0, 0, 0, 0, 0])))
    q_new = np.array([0, 0, 0, 0, 0, 0])
    for i in range(len(q_solution)):
        angle = np.rad2deg(q_solution[i])
        q_new[i] = (((angle + 180) % 360) + 360) % 360 - 180
    robot.set_generalized(clientID, q_new)
    print('debug-------------------------------------------------------------------')
    fk_result = forward_kinematics.forward_kinematics(links_length, np.deg2rad(q_new))
    print(np.round(fk_result, 3))
    print(np.round(cub_pos, 3))
    print(np.rad2deg(fk_result[1]))