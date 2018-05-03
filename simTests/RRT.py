import networkx as nx
import matplotlib.pyplot as plt
from NodeTB import NodeTB 
import time
import random
import sys
import math
sys.path.append("./klampt_simulations/simTests/")
from EnvRRT import EnvRRT as Env 
from klampt.math import vectorops
from klampt import vis
from klampt.model import trajectory

class RRT():
    def __init__(self, env, maxSample = 10000, maxStep = 15, maxDist = 0.5, dt = 0.05):
        print('Initializing PRM')
        self.pathCnt = 0
        self.rmCnt = 0

        self.epsGoal = 0.1

        self.maxSample = maxSample
        self.G = nx.Graph()
        self.envBounds = env.getBounds()
        self.env = env

        self.robot = env.robots[0]
        self.maxDist = maxDist
        self.maxStep = maxStep
        
        self.u1 = 0.5
        self.u2 = math.pi 
        self.dt = dt 
        
        self.wT = 0.5
        self.wR = 0.0

        u1_types = [1, 0, -1]
        u2_types = [1, 0, -1]

        self.U = []

        for i in u1_types:
            for j in u2_types:
                self.U.append([u1_types[i], u2_types[j]])

    def getSample(self):
        x = self.envBounds[0][0] + random.random() * (self.envBounds[0][1] - self.envBounds[0][0])
        y = self.envBounds[1][0] + random.random() * (self.envBounds[1][1] - self.envBounds[1][0])
        tht = random.random() * (2 * math.pi)
        newNode = NodeTB(x, y, tht, 0, 0, 0, None)
        return newNode
    
    def distMetric(self, n1, n2):
        dist = self.wT * vectorops.norm(vectorops.sub([n1.x, n1.y], [n2.x, n2.y]))
        dist += self.wR * math.fabs(n1.tht - n2.tht)
        return dist

    def localPlanner(self, qn, qs, sleepFlag = False):

        optNode = NodeTB(qn.x, qn.y, qn.tht, 0, 0, 0, None)
        minDist = self.distMetric(qn, qs)
        for uI in self.U:
            newNode = NodeTB(qn.x, qn.y, qn.tht, uI[0], uI[1], 0, None)
            vis.lock()
            self.robot.setConfig([newNode.x, newNode.y, newNode.tht])
            vis.unlock()
            stepCount = 0
            oldDist = self.distMetric(qn, qs)
            while(stepCount < self.maxStep):
                stepCount += 1
                oldConfig = self.robot.getConfig()
                vis.lock()
                self.robot.velControlKin(uI[0] * self.u1, uI[1] * self.u2, self.dt)
                vis.unlock()
                currConfig = self.robot.getConfig()
                colFlag = self.env.checkCollision()

                if colFlag:
                    break

                newNode.x = currConfig[0]
                newNode.y = currConfig[1]
                newNode.tht = currConfig[2]
                if self.distMetric(newNode, qs) >= oldDist:
                    newNode.x = oldConfig[0]
                    newNode.y = oldConfig[1]
                    newNode.tht = oldConfig[2]
                    break

                if self.distMetric(newNode, qn) > self.maxDist:
                    newNode.x = oldConfig[0]
                    newNode.y = oldConfig[1]
                    newNode.tht = oldConfig[2]
                    break
                
            newNode.T = stepCount - 1
            newDist = self.distMetric(newNode, qs)

            if (newDist < minDist):
                
                minDist = newDist
                optNode.x = newNode.x
                optNode.y = newNode.y
                optNode.tht = newNode.tht
                optNode.u1 = newNode.u1
                optNode.u2 = newNode.u2
                optNode.T = newNode.T
                optNode.parent = qn

        if not (optNode.parent == None):
            return optNode
        else:
            return None

    def checkGoal(self, qP, qG):
        dist = self.distMetric(qP, qG)
        if dist < self.epsGoal:
            print("Path found")
            return True
        else:
            return False

    def runRRT(self, qI, qG):

        self.robot.setConfig(qI)
        if(self.env.checkCollision()):
            print("Initial position in collision")
            return False

        self.robot.setConfig(qG)
        if(self.env.checkCollision()):
            print("Goal position in collision")
            return False

        self.nI = NodeTB(qI[0], qI[1], qI[2], 0, 0, 0, None)
        self.G.add_node(self.nI)
        
        self.nG = NodeTB(qG[0], qG[1], qG[2], 0, 0, 0,  None)
        #self.G.add_node(nG)


        if self.distMetric(self.nI,self.nG) < self.epsGoal:
            print("Already at goal")
            return True

        for i in range(self.maxSample):
            if i%100 == 0:
                print(i)
            qS = self.getSample()
            #qS.printNode()
            nodeList = list(self.G)
            qN = None
            minDist = 100000
            for nodeG in nodeList: 
                dist = self.distMetric(nodeG, qS) 
                if dist < minDist:
                    minDist = dist
                    qN = nodeG

            if qN == None:
                continue
            
            qP = self.localPlanner(qN, qS)
            if qP == None:
                continue
            else:
                self.G.add_node(qP)
                if self.checkGoal(qP, self.nG):
                    self.robot.setConfig(qI)
                    finalPath = [qP]
                    currNode = qP
                    while(currNode.parent != None):
                        currNode = currNode.parent
                        finalPath.append(currNode)
                        
                    self.finalPath = finalPath
                    return True
        print("Failed")
        return False
                
    def showRRTPath(self):
        vis.lock()
        vis.add("Initial", [self.nI.x, self.nI.y, 0.02])
        vis.setAttribute("Initial", "size", 14)
        vis.setColor("Initial",0, 0.4470, 0.7410)
        vis.add("Goal", [self.nG.x, self.nG.y, 0.02])
        vis.setAttribute("Goal", "size", 14)
        vis.setColor("Goal", 0.8500, 0.3250, 0.0980)
        vis.unlock()

        self.finalPath.reverse()
        pathLen = len(self.finalPath)
        for i in range(pathLen - 1):
            idx = i
            idx1 = i + 1 
            n1 = self.finalPath[idx]
            n2 = self.finalPath[idx1]
            n1.printNode()
            n2.printNode()
            stepCount = 0
            while stepCount <= n2.T: 
                vis.lock()
                self.robot.velControlKin(n2.u1 * self.u1, n2.u2 * self.u2, self.dt)
                vis.unlock()
                stepCount += 1
                time.sleep(0.1)






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
    
