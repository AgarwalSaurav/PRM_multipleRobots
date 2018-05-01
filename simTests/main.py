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

if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: kinematicSim.py [world_file]"
        exit()

    ## Creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        print(fn)
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    ## Get walls
    ## Two rooms separated by a wall with a window
    #bW.getDoubleRoomWindow(world, 8, 8, 1.2)
    
    ## Two rooms separated by a wall with a door 
    #bW.getDoubleRoomDoor(world, 8, 8, 1)
    bW.getDoubleRoomWindow(world, 8, 8, 1)

    ## Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1200,800
    vis.setViewport(vp)

    
    robot = []
    for i in range(world.numRobots()):
        robot.append(sphero6DoF(world.robot(i), world.robot(i).getName(),  vis))


    ## Display the world coordinate system
    vis.add("WCS",[so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)

    vis.autoFitCamera()
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)
    ## On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    ## Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    vis.show()

    simTime = 60
    startTime = time.time()
    oldTime = startTime
    reachedGoal = False;
    while vis.shown() and (time.time() - startTime < simTime):
        if(reachedGoal):
            break
        vis.lock()
    
        ## You may modify the world here.
        ## Specifying change in configuration of the robot

        ## 6DoF spherical robot
        q = robot[0].getConfig()
        q[0] = 1
        q[1] = 2
        q[2] = 0.1
        q[3] = 0*math.pi * (math.cos(time.time()) + 1)
        q[4] = 0*math.pi * (math.sin(time.time()) + 1)
        q[5] = 0*math.pi * (math.sin(time.time() + math.pi/4.0) + 1)
        robot[0].setConfig(q)

        q = robot[0].getConfig()
        q2f = [ '{0:.2f}'.format(elem) for elem in q]
        strng = "Robot configuration: " + str(q2f)
        vis.addText("textConfig", strng)

        ## Checking collision
        collisionFlag = False
        #for i,j in collisionChecker.collisionTests():
        #    if i[1].collides(j[1]):
        #        collisionFlag = True
        #        strng = "Object "+i[0].getName()+" collides with "+j[0].getName()
        #        print(strng)
        #        vis.addText("textCol", strng)
        #        vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
        collRT0 = collisionChecker.robotTerrainCollisions(world.robot(0), world.terrain(0))
        for i,j in collRT0:
            collisionFlag = True
            strng = "Robot collides with "+j.getName()
            print(strng)
            vis.addText("textCol", strng)
            vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
            break
  
        for iR in range(world.numRobots()):
            collRT2 = collisionChecker.robotObjectCollisions(world.robot(iR))
            for i,j in collRT2:
                collisionFlag = True
                strng = world.robot(iR).getName() + " collides with " + j.getName()
                print(strng)
                vis.addText("textCol", strng)
                vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
        

        collRT3 = collisionChecker.robotSelfCollisions()
        for i,j in collRT3:
            collisionFlag = True
            strng = i.getName() + " collides with "+j.getName()
            print(strng)
            vis.addText("textCol", strng)
            vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
        if not collisionFlag:
            vis.addText("textCol", "No collision")
            vis.setColor("textCol", 0.4660, 0.6740, 0.1880)

        vis.unlock()
        #changes to the visualization must be done outside the lock
        time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
