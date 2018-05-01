#!/usr/bin/python
## The class contains wrapper functions to set and get configuration of 6DoF spherical robot
## The first three elements of the configuration are (x, y, z)
## The next three are zyx Euler angles
## The function getTransform is for getting the rotation and the translation of the current position of the robot

from klampt import *
from klampt import vis
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.math import so3
import mathUtils
class sphero6DoF(object):
    def __init__ (self, robot, robotName, vis=None):
        self.robot = robot
        self.vis = vis
        self.robotName = robotName
        rotMat = so3.identity()
        pt = [0, 0, 0]
        if self.vis is not None:
            self.vis.add(robotName,[rotMat, pt])
            self.vis.setAttribute(robotName, "size", 32)
            self.vis.edit(robotName)

    def getConfig(self):
        q = self.robot.getConfig()
        return [q[0], q[1], q[2], q[3], q[4], q[5]]

    def setConfig(self, qC):
        q = self.robot.getConfig()
        q[0] = qC[0]
        q[1] = qC[1]
        q[2] = qC[2]
        q[3] = qC[3]
        q[4] = qC[4]
        q[5] = qC[5]
        self.robot.setConfig(q)
        if self.vis is not None:
            trans = self.getTransform()
            rotMat = trans[0]
            pt = trans[1]
            #self.vis.add(self.robotName,[rotMat, pt], keepAppearance=True)
        
    def getTransform(self):
        q = self.robot.getConfig()
        theta = [q[3], q[4], q[5]]
        rotMat = mathUtils.euler_zyx_mat(theta)
        return [rotMat, [q[0], q[1], q[2]]]
