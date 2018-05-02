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
    def __init__(self, env, maxSample = 2000, maxConnect = 15, maxDist = 10.5, localSteps = 20):
        print('Initializing PRM')
        self.maxSample = maxSample
        self.maxConnect = maxConnect
        self.maxDist = maxDist
        self.G = nx.Graph()
        self.envBounds = env.getBounds()
        self.env = env
        self.localSteps = localSteps
        self.pathCnt = 0
        self.rmCnt = 0


    def getSample(self):
        validFlag = False
        while not validFlag:
            x = self.envBounds[0][0] + random.random() * (self.envBounds[0][1] - self.envBounds[0][0])
            y = self.envBounds[1][0] + random.random() * (self.envBounds[1][1] - self.envBounds[1][0])
            z = self.envBounds[2][0] + random.random() * (self.envBounds[2][1] - self.envBounds[2][0])
            #z = 0.2
            sc = self.envBounds[3][0] + random.random() * (self.envBounds[3][1] - self.envBounds[3][0])
            #sc = 1
            tht = random.random() * (2 * math.pi)
            #tht = 0
            self.env.setConfig(x, y, z, sc, tht)
            if not self.env.checkCollision():
                newNode = nd(x, y, z, sc, tht)
                validFlag = True
        return newNode
    
    def distMetric(self, n1, n2):
        dist = self.env.n * vectorops.norm(vectorops.sub([n1.x, n1.y, n1.z], [n2.x, n2.y, n2.z]))
        dist += self.env.sumDist * math.fabs(n1.sc - n2.sc)
        dist += 1 * self.env.sumDist * math.fabs(n1.sc + n2.sc)/2 * math.fabs(n1.tht - n2.tht)
        return dist

    def localPlanner(self, n1, n2, sleepFlag = False):
        dist = self.env.n * vectorops.norm(vectorops.sub([n1.x, n1.y, n1.z], [n2.x, n2.y, n2.z]))
        vel = 0.04
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
                time.sleep(vel/2)
        self.env.setConfig(n2.x, n2.y, n2.z, n2.sc, n2.tht)
        return True

    def runPRM(self):
        n1 = self.getSample()
        n2 = self.getSample()
        self.localPlanner(n1, n2)
        for i in range(self.maxSample):
            if i%100 == 0:
                print(i)
            n1 = self.getSample()
            numNodes = nx.number_of_nodes(self.G)
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

        self.printGraphStat()
        self.expansionPRM()
        self.printGraphStat()

    def runPRM_noCC(self):
        n1 = self.getSample()
        n2 = self.getSample()
        self.localPlanner(n1, n2)
        for i in range(self.maxSample):
            if i%100 == 0:
                print(i)
            n1 = self.getSample()
            numNodes = nx.number_of_nodes(self.G)
            if numNodes == 0:
                self.G.add_node(n1)
                continue
            self.G.add_node(n1)
            nodeList = list(self.G)
            potentialEdges = []
            for nodeG in nodeList:
                dist = self.distMetric(nodeG, n1)
                if dist < self.maxDist:
                    if(self.localPlanner(n1, nodeG)):
                        potentialEdges.append((nodeG, dist))

            sorted_by_dist = sorted(potentialEdges, key=lambda tup: tup[1])
            count = 0
            for j in range(len(sorted_by_dist)):
                count = count + 1 
                if count > self.maxConnect:
                    break
                e = [(n1, sorted_by_dist[j][0], sorted_by_dist[j][1])]
                self.G.add_weighted_edges_from(e)

        self.printGraphStat()
        self.expansionPRM()
        self.printGraphStat()


    def visRoadMap(self):

        vis.show()

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
            vis.setColor(fName, 0.4940, 0.1840, 0.5560)
            cnt = cnt + 1
        self.rmCnt = cnt
            
    def queryPRM(self, qI, qG):

        vis.show()
        vis.lock()
        vis.add("Initial", [qI.x, qI.y, qI.z])
        vis.setAttribute("Initial", "size", 14)
        vis.setColor("Initial",0, 0.4470, 0.7410)
        vis.add("Goal", [qG.x, qG.y, qG.z])
        vis.setAttribute("Goal", "size", 14)
        vis.setColor("Goal", 0.8500, 0.3250, 0.0980)
        vis.unlock()
        vis.show(False)

        if(self.localPlanner(qI, qG)):
            print("Direct path found")
            cnt = 0
            tr = trajectory.Trajectory()
            tr.milestones.append([qI.x, qI.y, qI.z])
            tr.milestones.append([qG.x, qG.y, qG.z])
            fName = "p" + str(cnt)
            vis.add(fName,tr)
            vis.hideLabel(fName)
            vis.setColor(fName,0, 0.4470, 0.7410)
            cnt = cnt + 1
            self.pathCnt = cnt
            flagIn = True
            self.env.setConfig(qI.x, qI.y, qI.z, qI.sc, qI.tht)
            vis.show()
            while(flagIn):
                inText = ''
                while not (inText == 'y' or inText =='n'):
                    inText = raw_input("Run Path (y/n) ?")
                if inText == 'n':
                    flagIn = False
                    break
                if inText == 'y':
                    self.localPlanner(qI, qG, True)
            return True

        self.pathCnt = 0
        #vis.show(False)
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
            shortest_path = nx.dijkstra_path(self.G, source=minNodeI, target=minNodeG)
        except:
            return False
        
        shortest_path = self.smoothPath(shortest_path)
        vis.show()
        
        cnt = 0
        tr = trajectory.Trajectory()
        tr.milestones.append([qI.x, qI.y, qI.z])
        tr.milestones.append([minNodeI.x, minNodeI.y, minNodeI.z])
        fName = "p" + str(cnt)
        vis.add(fName,tr)
        vis.hideLabel(fName)
        vis.setColor(fName,0, 0.4470, 0.7410)
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
            vis.setColor(fName, 0.4660, 0.6740, 0.1880)
            cnt = cnt + 1

        tr = trajectory.Trajectory()
        tr.milestones.append([qG.x, qG.y, qG.z])
        tr.milestones.append([minNodeG.x, minNodeG.y, minNodeG.z])
        fName = "p" + str(cnt)
        vis.add(fName,tr)
        vis.hideLabel(fName)
        #vis.setAttribute(fName,"width",1.0)
        vis.setColor(fName, 0.8500, 0.3250, 0.0980)
        cnt = cnt + 1
        self.pathCnt = cnt

        flagIn = True
        self.env.setConfig(qI.x, qI.y, qI.z, qI.sc, qI.tht)
        vis.show()
        while(flagIn):
            inText = ''
            while not (inText == 'y' or inText =='n'):
                inText = raw_input("Run Path (y/n) ?")
            if inText == 'n':
                flagIn = False
                break
            if inText == 'y':
                self.runPath(qI, minNodeI, minNodeG, qG, shortest_path)
        return True

    def deletePath(self):
        for i in range(self.pathCnt):
            fName = "p" + str(i)
            vis.remove(fName)

    def deleteRM(self):
        for i in range(self.rmCnt):
            fName = "a" + str(i)
            vis.remove(fName)

    def smoothPath(self, shortest_path):

        for i in range(len(shortest_path)/2):
            pathLength = len(shortest_path)
            for j in range(pathLength - 2):
                if j >= pathLength - 2:
                    break
                n1 = shortest_path[j] 
                n2 = shortest_path[j + 2] 
                if (self.localPlanner(n1, n2)):
                    newPath = []
                    for j1 in range(j + 1):
                        newPath.append(shortest_path[j1])
                    for j1 in range(j + 2, pathLength):
                        newPath.append(shortest_path[j1])
                    shortest_path = newPath[:]
                    pathLength = len(shortest_path)

        pathLength = len(shortest_path)
        nTries = pathLength/20
        for i in range(nTries):
            n1Id = random.randint(0, pathLength - 2)
            n2Id = random.randint(n1Id + 1, pathLength - 1)
            n1 = shortest_path[n1Id] 
            n2 = shortest_path[n2Id] 
            if (self.localPlanner(n1, n2)):
                newPath = []
                for j in range(n1Id + 1):
                    newPath.append(shortest_path[j])
                for j in range(n2Id, pathLength):
                    newPath.append(shortest_path[j])
                shortest_path = newPath[:]
                pathLength = len(shortest_path)
            pathLength = len(shortest_path)
        return shortest_path

    def expansionPRM(self):
        nTries = nx.number_connected_components(self.G)/10
        for i in range(nTries):
            smallest_cc = min(nx.connected_components(self.G), key=len)
            n1 = random.choice(tuple(smallest_cc))

            potentialEdges = []

            connectComps = nx.connected_components(self.G)
            nConnectedComps = nx.number_connected_components(self.G) 
            for c in connectComps:
                minNode_c = None
                minDist_c = 100000
                for nodeG in c:
                    dist = self.distMetric(nodeG, n1)
                    if dist < 2*self.maxDist:
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

    def printGraphStat(self):

        print("No. of Nodes: " + str(self.G.number_of_nodes()))
        print("No. of Edges: " + str(self.G.number_of_edges() - self.G.number_of_nodes()))
        print("No. of Connected Components: " + str(nx.number_connected_components(self.G)))

        connectedComps = nx.connected_components(self.G)
        cc_count = 0
        for c in connectedComps:
            if len(c) > 1:
                cc_count += 1

        print("No. of Connected Components with more than one node: " + str(cc_count))


    def runPath(self, qI, minNodeI, minNodeG, qG, shortest_path):
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
        time.sleep(1)
    


    def smoothPath_old(self, shortest_path):
        pathLength = len(shortest_path)
        nTries = pathLength/20
        for i in range(nTries):
            n1Id = random.randint(0, pathLength - 2)
            n2Id = random.randint(n1Id + 1, pathLength - 1)
            n1 = shortest_path[n1Id] 
            n2 = shortest_path[n2Id] 
            if (self.localPlanner(n1, n2)):
                newPath = []
                for j in range(n1Id + 1):
                    newPath.append(shortest_path[j])
                for j in range(n2Id, pathLength):
                    newPath.append(shortest_path[j])
                shortest_path = newPath[:]
                pathLength = len(shortest_path)
            pathLength = len(shortest_path)
        for i in range(len(shortest_path)/2):
            pathLength = len(shortest_path)
            for j in range(pathLength - 2):
                if j >= pathLength - 2:
                    break
                n1 = shortest_path[j] 
                n2 = shortest_path[j + 2] 
                if (self.localPlanner(n1, n2)):
                    newPath = []
                    for j1 in range(j + 1):
                        newPath.append(shortest_path[j1])
                    for j1 in range(j + 2, pathLength):
                        newPath.append(shortest_path[j1])
                    shortest_path = newPath[:]
                    pathLength = len(shortest_path)
        return shortest_path
