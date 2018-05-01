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

prm = PRM(env)
newNode = prm.getSample()
prm.runPRM()
newNode.printNode()
G = prm.G
prm.visRoadMap()

vis.setWindowTitle("Motion Planning Using Probabilistic Road Map")

print "calling dialog()"
vis.dialog()
vp = vis.getViewport()
vp.w,vp.h = 1800,1000
while(True):
    n1 = prm.getSample()
    n2 = prm.getSample()
    if not prm.queryPRM(n1, n2):
        print("Path not found")
    prm.deletePath()
    
time.sleep(60)

env.endEnv()
nodeList = list(G)
edgeList = G.edges()
#print(nodeList)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(envBounds[0])
ax.set_ylim(envBounds[1])
ax.set_zlim(envBounds[2])
#for e in edgeList:
#    ax.plot([e[0].x, e[1].x], [e[0].y, e[1].y], zs=[e[0].z, e[1].z])
#

#plt.subplot(122)
#nx.draw(G, pos=nx.circular_layout(G), nodecolor='r', edge_color='b')
#plt.show()
#

