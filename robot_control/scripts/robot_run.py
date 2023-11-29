from coppeliasim_zmqremoteapi_client import *

client = RemoteAPIClient()
sim = client.require('sim')

def main():
    sim.getObject('joint_a1')


main()