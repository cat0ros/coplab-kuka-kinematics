from . import sim
from . import build_circle
from . import forward_kinematics
from . import ik_solver
from . import generate_circle

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
        sim.simxSetJointTargetPosition(clientID, self.j1, generalized[0], sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID, self.j2, generalized[1], sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID, self.j3, generalized[2], sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID, self.j4, generalized[3], sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID, self.j5, generalized[4], sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID, self.j6, generalized[5], sim.simx_opmode_streaming)
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
    clientID = connect_coppelia('127.0.0.1', 19990)
    links_length = [0.675, 0.260, 0.680, 0.670, 0.158]
    robot = RobotControl(clientID)
    time.sleep(1)
    point1 = np.array([1.125,  0.350, 0.650])
    point2 = np.array([1.125, -0.550, 0.775])
    point3 = np.array([1.125, -0.400, +1.200])
    x, y, z, radius = build_circle.build(point1, point2, point3)
    print(x, y, z, radius)
    list_angles = generate_circle.generate_traectory(clientID, links_length, (x, y, z), radius, 50, robot)
    while True:
        for angles in list_angles:
            robot.set_generalized(clientID, np.rad2deg(angles))
            print(np.rad2deg(angles))
            time.sleep(0.1)

if __name__ == '__main__':
    main()