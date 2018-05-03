import networkx as nx
import matplotlib.pyplot as plt
from Node import Node as nd 
from PRM import PRM
import time
import sys
sys.path.append("./klampt_simulations/simTests/")
from Env import Env 
from mpl_toolkits.mplot3d import Axes3D
from klampt import vis


if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: kinematicSim.py [world_file]"
        exit()


env = Env(sys.argv[1:])
for i in range(100):
    env.setConfig(-1 + i/float(100), -1, 2, 2, i*3.14/100)
    time.sleep(0.01)
envBounds = env.getBounds()

learnT_start = time.time()
prm = PRM(env)
prm.runPRM()
learnT_end = time.time()
G = prm.G



vp = vis.getViewport()
vp.w,vp.h = 1800,800
#prm.visRoadMap()
queryT_start = time.time()
for i in range(50):
    n1 = prm.getSample()
    n2 = prm.getSample()
    failCnt = 0
    if not prm.queryPRM(n1, n2):
        failCnt += 1

queryT_end = time.time()

print("Learning time: " + str(-learnT_start + learnT_end))
print("Query time: " + str((-queryT_start + queryT_end)/float(50)))
print("Failed queries: " + str(failCnt))
prm.visRoadMap()
vis.show()
raw_input("")

env.endEnv()

