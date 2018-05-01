#!/usr/bin/python
## The class contains wrapper functions to set and get configuration of 3DoF holonomic robot
## The first three elements of the configuration are (x, y, z)
## The next three are zyx Euler angles
## The function getTransform is for getting the rotation and the translation of the current position of the robot
## The function velControlKin is for converting velocity control inputs (linear along local x, and angular about local z) into state vector
## The function wheelControlKin is for converting wheel velocity control inputs (angular wheel left and right) into state vector

from klampt import *
from klampt import vis
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.math import so3
import math
import mathUtils
class turtlebot(object):

    def __init__ (self, robot, vis=None):
        self.robot = robot
        self.vis = vis
        self.wheelDia = 0.076
        self.lenAxle = 0.23 ## Centre to centre wheel distance (Need to confirm the value)
        self.eps = 0.000001 ## Small value for comaparing to zero
        self.delZ = 0.3 ## dummy value for the coordinate system, else it is not visible
        rotMat = so3.identity()
        pt = [0, 0, 0]
        
        if self.vis is not None:
            self.vis.add("Robot",[rotMat, pt])
            self.vis.setAttribute("Robot", "size", 32)
            self.vis.edit("Robot")

    def velControlKin(self, vel, omega, deltaT):
        q = self.getConfig()
        if abs(omega) < self.eps:
            q[0] = q[0] + vel * deltaT * math.cos(q[2])
            q[1] = q[1] + vel * deltaT * math.sin(q[2])
        else:
            rad = vel/omega
            q[0] = q[0] - rad * math.sin(q[2]) + rad * math.sin(q[2] + omega * deltaT)
            q[1] = q[1] + rad * math.cos(q[2]) - rad * math.cos(q[2] + omega * deltaT)
            q[2] = q[2] + omega * deltaT
        self.setConfig(q)


    def wheelControlKin(self, w_l, w_r, deltaT):
        q = self.getConfig()
        v_l = w_l * self.wheelDia/2.0
        v_r = w_r * self.wheelDia/2.0
        if abs(v_r - v_l) < self.eps:
            q[0] =  q[0] + v_l * deltaT * math.cos(q[2])
            q[1] =  q[1] + v_l * deltaT * math.sin(q[2])
        else:            
            ## Distance from ICC to centre of axle
            rad = (v_l + v_r)/(2*(v_r - v_l))
            angVel = (v_r - v_l)/self.lenAxle
            icc = [q[0] - rad * math.sin(q[2]), q[1] + rad * math.cos(q[2])]
            cosOmegaDeltaT = math.cos(angVel * deltaT)
            sinOmegaDeltaT = math.sin(angVel * deltaT)
            q[0] = (q[0] - icc[0]) * cosOmegaDeltaT - (q[1] - icc[1]) * sinOmegaDeltaT + icc[0]
            q[1] = (q[0] - icc[0]) * sinOmegaDeltaT + (q[1] - icc[1]) * cosOmegaDeltaT + icc[1]
            q[2] = q[2] + angVel * deltaT
        self.setConfig(q)

    def getConfig(self):
        q = self.robot.getConfig()
        return [q[0], q[1], q[3]]

    def setConfig(self, qC):
        q = self.robot.getConfig()
        q[0] = qC[0]
        q[1] = qC[1]
        q[3] = qC[2]
        self.robot.setConfig(q)
        if self.vis is not None:
            trans = self.getTransform()
            rotMat = trans[0]
            pt = trans[1]
            pt[2] = pt[2] + self.delZ
            self.vis.add("Robot",[rotMat, pt], keepAppearance=True)
        
    def getTransform(self):
        q = self.robot.getConfig()
        theta = [q[3], q[4], q[5]]
        rotMat = mathUtils.euler_zyx_mat(theta)
        return [rotMat, [q[0], q[1], q[2]]]

    def setAltitude(self, alt):
        q = self.robot.getConfig()
        q[2] = alt
        self.robot.setConfig(q)
        if self.vis is not None:
            trans = self.getTransform()
            rotMat = trans[0]
            pt = trans[1]
            pt[2] = pt[2] + self.delZ
            self.vis.add("Robot",[rotMat, pt], keepAppearance=True)

