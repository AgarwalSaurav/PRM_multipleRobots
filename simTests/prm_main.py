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
prm.runPRM()
G = prm.G


vis.setWindowTitle("Motion Planning Using Probabilistic Road Map")

vp = vis.getViewport()
vp.w,vp.h = 1800,800
prm.visRoadMap()

while(True):
    inText = ''
    while not (inText == 'y' or inText =='n'):
        inText = raw_input("Continue (y/n/c) ?")
    if inText == 'n':
        break
    if inText == 'y':
        prm.deletePath()
        n1 = prm.getSample()
        n2 = prm.getSample()
        n1.printNode()
        n2.printNode()
        failCnt = 0
        while not prm.queryPRM(n1, n2):
            print("Path not found")
            n1 = prm.getSample()
            n2 = prm.getSample()
            failCnt += 1
            if failCnt > 100:
                print("Too many failed queries")
                break


env.endEnv()

