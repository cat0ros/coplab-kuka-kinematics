from . import sim
from . import build_circle

import numpy as np
import sys

def connect_coppelia(ip, port):
    sim.simxFinish(-1)
    clientId = sim.simxStart(ip, port, True, True, 5000, 5)
    if clientId != -1:
        print("Connected to Remote API Server")
    else:
        print("Connection failed")
        sys.exit('Could not reconnect')

def main():
    point1 = np.array([200, 100, 250])
    point2 = np.array([250, 100, 300])
    point3 = np.array([150, 100, 100])
    x, z, radius = build_circle.build(point1, point2, point3)
    

if __name__ == '__main__':
    connect_coppelia('127.0.0.1', 19990)
    main()