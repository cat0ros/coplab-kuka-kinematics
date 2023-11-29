from . import sim
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
    pass

if __name__ == '__main__':
    connect_coppelia('127.0.0.1', 19990)
    main()