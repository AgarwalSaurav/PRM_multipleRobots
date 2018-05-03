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
from klampt.math import so3
import mathUtils

class EnvRRT():

    def __init__(self, fn):
        ## Creates a world and loads all the items on the command line
        self.world = WorldModel()
        self.robotSystemName = 'O'

        for f in fn:
            print(f)
            res = self.world.readFile(f)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
        self.showVis = False

        coordinates.setWorldModel(self.world)

        vis.lock()
        bW.getDoubleRoomDoor(self.world, 8, 8, 1)
        vis.unlock()

        ## Add the world to the visualizer
        vis.add("world", self.world)

        vp = vis.getViewport()
        vp.w,vp.h = 1800,800
        vis.setViewport(vp)
        self.n = self.world.numRobots();

        self.robots = []
        self.robots.append(turtlebot(self.world.robot(0), vis))
        self.robots[0].setAltitude(0.02)

        self.xB = [-4, 4]
        self.yB = [-4, 4]
        
        vis.add("WCS",[so3.identity(),[0,0,0]])
        vis.setAttribute("WCS", "size", 32)
        vis.edit("WCS")
        self.collisionChecker = collide.WorldCollider(self.world)
        if self.showVis:
            ## Display the world coordinate system

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
    
        q = self.robots[0].getConfig()
        if self.showVis:
            q2f = [ '{0:.2f}'.format(elem) for elem in q]
            strng = "Robot configuration: " + str(q2f)
            vis.addText("textConfig", strng)
        colFlag = self.checkCollision()
        print(colFlag)

        if self.showVis:
            time.sleep(1)

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
        

        if not collisionFlag:
            if self.showVis:
                vis.addText("textCol", "No collision")
                vis.setColor("textCol", 0.4660, 0.6740, 0.1880)
            vis.unlock()
            return collisionFlag


    def getBounds(self):
        return [self.xB, self.yB]

    def endEnv(self):
        vis.clearText()
        print "Ending klampt.vis visualization."
        vis.kill()
