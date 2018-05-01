import networkx as nx
import matplotlib.pyplot as plt
from Node import Node as nd 
import time
import random
import sys
import math
sys.path.append("./klampt_simulations/simTests/")
from Env import Env 
from klampt.math import vectorops
from klampt import vis
from klampt.model import trajectory

class PRM():
    def __init__(self, env, maxSample = 500, maxConnect = 15, maxDist = 2.5, localSteps = 20):
        print('Initializing PRM')
        self.maxSample = maxSample
        self.maxConnect = maxConnect
        self.maxDist = maxDist
        self.G = nx.Graph()
        self.envBounds = env.getBounds()
        self.env = env
        self.localSteps = localSteps


    def getSample(self):
        x = self.envBounds[0][0] + random.random() * (self.envBounds[0][1] - self.envBounds[0][0])
        y = self.envBounds[1][0] + random.random() * (self.envBounds[1][1] - self.envBounds[1][0])
        z = self.envBounds[2][0] + random.random() * (self.envBounds[2][1] - self.envBounds[2][0])
        sc = self.envBounds[3][0] + random.random() * (self.envBounds[3][1] - self.envBounds[3][0])
        tht = random.random() * (2 * math.pi)
        newNode = nd(x, y, z, sc, tht)
        return newNode
    
    def distMetric(self, n1, n2):
        dist = self.env.n * vectorops.norm(vectorops.sub([n1.x, n1.y, n1.z], [n2.x, n2.y, n2.z]))
        dist += self.env.sumDist * math.fabs(n1.sc - n2.sc)
        dist += self.env.sumDist * math.fabs(n1.sc + n2.sc)/2 * math.fabs(n1.tht - n2.tht)
        return dist

    def localPlanner(self, n1, n2, sleepFlag = False):
        dist = self.env.n * vectorops.norm(vectorops.sub([n1.x, n1.y, n1.z], [n2.x, n2.y, n2.z]))
        vel = 0.08
        tm = dist/vel

        for i in range(int(tm)):
            delta = i/float(tm)
            x = n1.x + (n2.x - n1.x) * delta
            y = n1.y + (n2.y - n1.y) * delta
            z = n1.z + (n2.z - n1.z) * delta
            sc = n1.sc + (n2.sc - n1.sc) * delta
            tht = n1.tht + (n2.tht - n1.tht) * delta
            vis.lock()
            self.env.setConfig(x, y, z, sc, tht)
            vis.unlock()
            colFlag = False
            colFlag = self.env.checkCollision()
            if (colFlag):
                return False
            if sleepFlag:
                time.sleep(vel)
        return True

    def runPRM(self):
        n1 = self.getSample()
        n2 = self.getSample()
        self.localPlanner(n1, n2)
        for i in range(self.maxSample):
            if i%100 == 0:
                print(i)
            n1 = self.getSample()
            numNodes = self.G.number_of_nodes()
            if numNodes == 0:
                self.G.add_node(n1)
                continue
            self.G.add_node(n1)
            potentialEdges = []

            connectComps = nx.connected_components(self.G)
            nConnectedComps = nx.number_connected_components(self.G) 
            for c in connectComps:
                minNode_c = None
                minDist_c = 100000
                for nodeG in c:
                    dist = self.distMetric(nodeG, n1)
                    if dist < self.maxDist:
                        if(self.localPlanner(n1, nodeG)):
                            if (minDist_c > dist):
                                minNode_c = nodeG
                                minDist_c = dist
                if minNode_c != None:
                    potentialEdges.append((minNode_c, minDist_c))

            sorted_by_dist = sorted(potentialEdges, key=lambda tup: tup[1])
            count = 0
            for j in range(len(sorted_by_dist)):
                count = count + 1 
                if count > self.maxConnect:
                    break
                e = [(n1, sorted_by_dist[j][0], sorted_by_dist[j][1])]
                self.G.add_weighted_edges_from(e)

    def visRoadMap(self):

        edgeList = self.G.edges()
        cnt = 0
        for e in edgeList:
            n1 = e[0]
            n2 = e[1]
            tr = trajectory.Trajectory()
            tr.milestones.append([n1.x, n1.y, n1.z])
            tr.milestones.append([n2.x, n2.y, n2.z])

            fName = "a" + str(cnt)
            vis.add(fName,tr)
            vis.hideLabel(fName)
            vis.setAttribute(fName,"width",0.5)
            cnt = cnt + 1
            
    def queryPRM(self, qI, qG):
        self.pathCnt = 0
        vis.show(False)
        nodeList = list(self.G)
        minNodeI = None
        minDist = 100000
        for nodeG in nodeList:
            dist = self.distMetric(qI, nodeG)
            if dist < self.maxDist:
                if(self.localPlanner(qI, nodeG)):
                    if (minDist > dist):
                        minNodeI = nodeG
                        minDist = dist
        if minNodeI == None:
            return False

        minNodeG = None
        minDist = 100000
        for nodeG in nodeList:
            dist = self.distMetric(qG, nodeG)
            if dist < self.maxDist:
                if(self.localPlanner(qG, nodeG)):
                    if (minDist > dist):
                        minNodeG = nodeG
                        minDist = dist
        if minNodeG == None:
            return False

        try:
            shortest_path = nx.shortest_path(self.G, source=minNodeI, target=minNodeG)
        except:
            return False
        
        vis.show()
        
        vis.add("Initial configuration", [qI.x, qI.y, qI.z])
        vis.add("Goal configuration", [qG.x, qG.y, qG.z])
        cnt = 0
        tr = trajectory.Trajectory()
        tr.milestones.append([qI.x, qI.y, qI.z])
        tr.milestones.append([minNodeI.x, minNodeI.y, minNodeI.z])
        fName = "p" + str(cnt)
        vis.add(fName,tr)
        vis.hideLabel(fName)
        vis.setColor(fName,0,0,1)
        cnt = cnt + 1

        prevNode = None
        for nodeP in shortest_path:
            if prevNode == None:
                prevNode = nodeP
                continue
            tr = trajectory.Trajectory()
            tr.milestones.append([prevNode.x, prevNode.y, prevNode.z])
            tr.milestones.append([nodeP.x, nodeP.y, nodeP.z])
            prevNode = nodeP
            fName = "p" + str(cnt)
            vis.add(fName,tr)
            vis.hideLabel(fName)
            vis.setColor(fName, 0, 1, 0)
            cnt = cnt + 1

        tr = trajectory.Trajectory()
        tr.milestones.append([qG.x, qG.y, qG.z])
        tr.milestones.append([minNodeG.x, minNodeG.y, minNodeG.z])
        fName = "p" + str(cnt)
        vis.add(fName,tr)
        vis.hideLabel(fName)
        #vis.setAttribute(fName,"width",1.0)
        vis.setColor(fName, 0, 0, 1)
        cnt = cnt + 1
        time.sleep(1)

        self.localPlanner(qI, minNodeI, True)
        prevNode = None
        for nodeP in shortest_path:
            if prevNode == None:
                prevNode = nodeP
                continue
            self.localPlanner(prevNode, nodeP, True)
            prevNode = nodeP
        self.localPlanner(minNodeG, qG, True)
        self.pathCnt = cnt

    def deletePath(self):
        for i in range(self.pathCnt):
            fName = "p" + str(i)
            vis.remove(fName)









