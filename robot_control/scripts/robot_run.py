from . import sim
from . import build_circle
from . import forward_kinematics

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
        print(err1)
        self.currentGeneralized = np.array([0, 0, 0, 0, 0, 0], float)
        self.set_generalized(clientID, self.currentGeneralized)

    def get_generalized(self, clientID):
        err1, self.currentGeneralized[0] = sim.simxGetJointPosition(clientID, self.j1, sim.simx_opmode_oneshot_wait)
        err2, self.currentGeneralized[1] = sim.simxGetJointPosition(clientID, self.j2, sim.simx_opmode_oneshot_wait)
        err3, self.currentGeneralized[2] = sim.simxGetJointPosition(clientID, self.j3, sim.simx_opmode_oneshot_wait)
        err4, self.currentGeneralized[3] = sim.simxGetJointPosition(clientID, self.j4, sim.simx_opmode_oneshot_wait)
        err5, self.currentGeneralized[4] = sim.simxGetJointPosition(clientID, self.j5, sim.simx_opmode_oneshot_wait)
        err6, self.currentGeneralized[5] = sim.simxGetJointPosition(clientID, self.j6, sim.simx_opmode_oneshot_wait)
        print(err1)
        
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
    point1 = np.array([200, 100, 250])
    point2 = np.array([250, 100, 300])
    point3 = np.array([150, 100, 100])
    x, z, radius = build_circle.build(point1, point2, point3)
    

if __name__ == '__main__':
    clientID = connect_coppelia('127.0.0.1', 19990)
    links_length = [0.675, 0.260, 0.680, 0.670, 0.158]
    main()
    robot = RobotControl(clientID)
    robot.set_generalized(clientID, np.array([90, -90, 90, 30, 30, 0], float))
    time.sleep(3)
    robot.get_generalized(clientID)
    pos, rot = forward_kinematics.forward_kinematics(links_length, robot.currentGeneralized)
    x, y, z = pos[0], pos[1], pos[2]
    err1, cuboid = sim.simxGetObjectHandle(clientID, 'Cuboid', sim.simx_opmode_oneshot_wait)
    err1, robot_handle = sim.simxGetObjectHandle(clientID, 'base_link_respondable', sim.simx_opmode_oneshot_wait)
    sim.simxSetObjectPosition(clientID, cuboid, robot_handle, [x, y, z], sim.simx_opmode_oneshot_wait)
    sim.simxSetObjectOrientation(clientID, cuboid, robot_handle, rot, sim.simx_opmode_oneshot_wait)
    err2, orient2 = sim.simxGetObjectOrientation(clientID, cuboid, robot_handle, sim.simx_opmode_oneshot_wait)
    print(err1, err2)