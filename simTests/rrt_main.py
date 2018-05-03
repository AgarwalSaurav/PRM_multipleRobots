import networkx as nx
import matplotlib.pyplot as plt
from Node import Node as nd 
from RRT import RRT
import time
import sys
sys.path.append("./klampt_simulations/simTests/")
from EnvRRT import EnvRRT as Env 
from mpl_toolkits.mplot3d import Axes3D
from klampt import vis


if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: kinematicSim.py [world_file]"
        exit()


env = Env(sys.argv[1:])
envBounds = env.getBounds()

#vis.show()
rrt = RRT(env)
qI = [-2, -2, 0] 
qG = [2, 2, 0] 
rrt.runRRT(qI, qG)

vis.setWindowTitle("Motion Planning Using Probabilistic Road Map")

vp = vis.getViewport()
vp.w,vp.h = 1800,800

vis.show()
rrt.showRRTPath()
time.sleep(5)

env.endEnv()

