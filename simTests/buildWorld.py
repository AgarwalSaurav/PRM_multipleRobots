#!/usr/bin/python

## The functions are for creating additional components for the environment
##  1. getWall: Get the geometry of a wall based on given dimensions
##  2. getWall_terrain: Attach a single wall to terrain
##  3. getDoubleRoomDoor: Build two rooms with a door on the separating wall
##  4. getDoubleRoomWindow: Build two rooms with a window on the separating wall
import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import time
import math

def getWall(dimX, dimY, dimZ, pos = [0, 0, 0], rotZ = 0):
        ## Get the wall geometry
	wall = Geometry3D()
	wall.loadFile("cube.off") # The wall is based upon cube primitive of unit dimension
        ## Not sure why the scaling is done through the rotation matrix, works though!
	#wall.transform([dimX, 0, 0, 0, dimY, 0, 0, 0, dimZ], pos)
        wall.scale(dimX, dimY, dimZ)
        rotMat = so3.rotation((0, 0, 1), math.radians(rotZ))
        wall.transform(rotMat, pos)
        return wall

def getWall_terrain(world, dimX, dimY, dimZ, pos = [0, 0, 0], nameWall="wall", color = [0.85, 0.85, 0.85, 1]):
        ## Attach a single wall to the world
        wall = getWall(dimX, dimY, dimZ, pos)
	world_wall = world.makeTerrain(nameWall)
	world_wall.geometry().set(wall)
        r = color[0]
        g = color[1]
        b = color[2]
        alpha = color[3]
	world_wall.appearance().setColor(r, g, b, alpha)
	return world_wall

def getDoubleRoomDoor(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    ## Build a double room with a single door in the middle of the wall
    ## The width of the door is dimX/4
    x2 = dimX/2.0
    x8 = dimX/8.0
    y2 = dimY/2.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    w5 = getWall(3*x8, wall_thickness, dimZ, [-x2, 0, 0], 0)
    w6 = getWall(3*x8, wall_thickness, dimZ, [x8, 0, 0], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4, w5, w6]):
	DRDgeom.setElement(i,elem)
    drd_setup = world.makeTerrain("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha)

def getDoubleRoomWindow(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    ## Build a double room with a single window in the middle of the wall
    ## The dimensions of the window are dimX/4, dimZ/3  
    x2 = dimX/2.0
    x8 = dimX/8.0
    y2 = dimY/2.0
    z3 = dimZ/3.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    w5 = getWall(dimX, wall_thickness*20, z3, [-x2, -wall_thickness*10, 0], 0)
    w6 = getWall(3*x8, wall_thickness*20, z3, [-x2, -wall_thickness*10, z3], 0)
    w7 = getWall(3*x8, wall_thickness*20, z3, [x8, -wall_thickness*10, z3], 0)
    w8 = getWall(dimX, wall_thickness*20, z3, [-x2, -wall_thickness*10, 2.0*z3], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4, w5, w6, w7, w8]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeRigidObject("DRDwWindow")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha)
    b1 = getWall(.4, .4, .5, [-0.75, -0.75, 0.1], 0)
    box1 = world.makeRigidObject("Box1")
    box1.geometry().set(b1)
    b2 = getWall(.3, .3, .5, [0.8, 0.8, 0.1], 0)
    box2 = world.makeRigidObject("Box2")
    box2.geometry().set(b2)
    b3 = getWall(.5, .5, .5, [1.5, 1.5, 0.1], 0)
    box3 = world.makeRigidObject("Box3")
    box3.geometry().set(b3)
    b4 = getWall(.5, .5, .5, [-1.5, -1.5, 0.1], 0)
    box4 = world.makeRigidObject("Box4")
    box4.geometry().set(b4)

def myEnviron(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    x2 = dimX/2.0
    y2 = dimY/2.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 45)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)

    b1 = getWall(.2, .2, .2, [-0.75, -0.75, 0.1], 0)
    b2 = getWall(.2, .2, .2, [0.5, 0.5, 0.1], 0)
    
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4]):
	DRDgeom.setElement(i,elem)
    drd_setup = world.makeTerrain("RoomWall")
    drd_setup.geometry().set(DRDgeom)
    box1 = world.makeRigidObject("Box1")
    box1.geometry().set(b1)
    box2 = world.makeRigidObject("Box2")
    box2.geometry().set(b2)
