#!/usr/bin/python
## The file demonstrates:
##   1. Adding rooms and walls to the environment (refer to buildWorld.py as well)
##   2. Setting up a robot
##   3. Perform collision checking
##   4. Modify the robot configurations and visualize
##   5. Adding text objects and modifying them

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide
import time
import math
import buildWorld as bW
import random
sys.path.append("./kinematics/")
from sphero6DoF import sphero6DoF
from kobuki import kobuki
from turtlebot import turtlebot
from decimal import Decimal
from klampt.math import vectorops

class Env():

    def __init__(self, fn):
        ## Creates a world and loads all the items on the command line
        self.world = WorldModel()

        for f in fn:
            print(f)
            res = self.world.readFile(f)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
        self.showVis = False

        coordinates.setWorldModel(self.world)

        bW.getDoubleRoomWindow(self.world, 8, 8, 1)

        ## Add the world to the visualizer
        vis.add("world", self.world)

        vp = vis.getViewport()
        vp.w,vp.h = 1800,800
        vis.setViewport(vp)

        self.robots = []
        self.n = self.world.numRobots();
        for i in range(self.n):
            self.robots.append(sphero6DoF(self.world.robot(i), self.world.robot(i).getName(),  vis))

        self.eps = 0.000001
        self.sj = [[0, 0, 0], [0.2, 0, 0]]
        self.xB = [-4, 4]
        self.yB = [-4, 4]
        self.zB = [0.02, 1]
        
        self.rad = 0.04

        self.currConfig = [0, 0, 1, 1, 0]

        self.scMin = 1
        self.scXMin = 1
        self.scYMin = 2
        self.sumDist = 0
        if self.n > 1:
            minSij = vectorops.norm(vectorops.sub(self.sj[0], self.sj[1]))
            minSijX = math.fabs(self.sj[0][0] - self.sj[1][0])
            minSijY = math.fabs(self.sj[0][1] - self.sj[1][1])

    
            for i in range(self.n):
                for j in range(self.n):
                    if i != j:
                        dist = vectorops.norm(vectorops.sub(self.sj[i], self.sj[j]))
                        self.sumDist += dist
                        if dist < minSij:
                            minSij = dist
                        dist =  math.fabs(self.sj[i][0] - self.sj[j][0])
                        if dist < minSijX:
                            minSijX = dist
                        dist =  math.fabs(self.sj[i][1] - self.sj[j][1])
                        if dist < minSijY:
                            minSijY = dist

            if minSij > self.eps:
                self.scMin = 2 * math.sqrt(2) * self.rad / minSij
            if minSijX > self.eps:
                self.scXMin = 2 * math.sqrt(2) * self.rad / minSijX
            if minSijY > self.eps:
                self.scYMin = 2 * math.sqrt(2) * self.rad / minSijY
        self.scMax = max(2, self.scMin)
        self.scB = [self.scMin, self.scMax]



        self.collisionChecker = collide.WorldCollider(self.world)
        if self.showVis:
            ## Display the world coordinate system
            vis.add("WCS",[so3.identity(),[0,0,0]])
            vis.setAttribute("WCS", "size", 24)

            vis.addText("textCol", "No collision")
            vis.setAttribute("textCol","size",24)
            ## On-screen text display
            vis.addText("textConfig","Robot configuration: ")
            vis.setAttribute("textConfig","size",24)
            vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

            print "Starting visualization window#..."

            ## Run the visualizer, which runs in a separate thread
            vis.setWindowTitle("Visualization for kinematic simulation")

            vis.show()
        simTime = 60
        startTime = time.time()
        oldTime = startTime
    
        self.setConfig(0, 0, 1, 1, 0)
        q = self.robots[0].getConfig()
        if self.showVis:
            q2f = [ '{0:.2f}'.format(elem) for elem in q]
            strng = "Robot configuration: " + str(q2f)
            vis.addText("textConfig", strng)
        colFlag = self.checkCollision()
        print(colFlag)

        if self.showVis:
            time.sleep(10)

    def checkCollision(self):
        vis.lock()
        ## Checking collision
        collisionFlag = False
        for iR in range(self.n):
            collRT0 = self.collisionChecker.robotTerrainCollisions(self.world.robot(iR), self.world.terrain(0))
            for i,j in collRT0:
                collisionFlag = True
                strng = "Robot collides with "+j.getName()
                if self.showVis:
                    vis.addText("textCol", strng)
                    vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
                vis.unlock()
                return collisionFlag
                break
  
        
        for iR in range(self.n):
            collRT2 = self.collisionChecker.robotObjectCollisions(self.world.robot(iR))
            for i,j in collRT2:
                collisionFlag = True
                strng = self.world.robot(iR).getName() + " collides with " + j.getName()
                if self.showVis:
                    print(strng)
                    vis.addText("textCol", strng)
                    vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
                vis.unlock()
                return collisionFlag
                break
        

        collRT3 = self.collisionChecker.robotSelfCollisions()
        for i,j in collRT3:
            collisionFlag = True
            strng = i.getName() + " collides with "+j.getName()
            if self.showVis:
                vis.addText("textCol", strng)
                vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
            vis.unlock()
            return collisionFlag
            break
        if not collisionFlag:
            if self.showVis:
                vis.addText("textCol", "No collision")
                vis.setColor("textCol", 0.4660, 0.6740, 0.1880)
            vis.unlock()
            return collisionFlag


    def getBounds(self):
        return [self.xB, self.yB, self.zB, self.scB]

    def setConfig(self, x, y, z, sc, tht):
        self.currConfig = [x, y, z, sc, tht]
        cosTht = math.cos(tht)
        sinTht = math.sin(tht)

        vis.lock()
        for iR in range(self.n):
            q = self.robots[iR].getConfig()

            scSj = vectorops.mul(self.sj[iR], sc)
            q[0] = self.currConfig[0] + cosTht * scSj[0] - sinTht * scSj[1]
            q[1] = self.currConfig[1] + sinTht * scSj[0] + cosTht * scSj[1]
            q[2] = self.currConfig[2] + scSj[2]
            self.robots[iR].setConfig(q)
        vis.unlock()
        self.checkCollision()

    def endEnv(self):
        vis.clearText()
        print "Ending klampt.vis visualization."
        vis.kill()
