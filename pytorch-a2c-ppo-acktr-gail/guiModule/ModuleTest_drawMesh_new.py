from . import guiBase
from . import drawMesh as dM
#from drawMesh import drawingMesh as DM

#import guiBase
#import drawMesh as dM

import wx
import sys
import os
import pydart2 as pydart
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import cMat 
import SimbiconController as SC

from wx import glcanvas

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *


import cv2

import pyassimp
from pyassimp.postprocess import *
from pyassimp.helper import *

import logging;logger = logging.getLogger("pyassimp_opengl")
logging.basicConfig(level= logging.INFO)

import quaternion
import threading
import copy
import time

import cMat

drawLimit = 100
boxSize = 100
imageData = np.zeros((boxSize*8,boxSize*8,3))
#cameraP = ([3,3,3,0])

qrm = np.array([[1,0,0],
               [0,1,0],
               [0,0,1]])

gEye = np.array([3,3,3])
gAt = np.array([0,0,0])
gUp = np.array([0,1,0])



class dartGui(guiBase.GuiBase):
    def InitGL(self):
        self.cameraX = 0
        self.cameraY = 0
        self.cameraZ = 0
        self.xsubrad = 0
        self.ysubrad = 0
        self.Rxsubrad = 0
        self.Rysubrad = 0
        # set viewing projection
        glMatrixMode(GL_PROJECTION)
        #glFrustum(-0.5, 0.5, -0.5, 0.5, 1.0, 3.0)
        gluPerspective(60, 1, 1, 100)

        # position viewer
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.myLookAt(np.array([3,3,3]), np.array([0,0,0]), np.array([0,1,0])) 
         
        #glTranslatef(0, 0, -1.0)
        #gluLookAt(np.radians(self.cameraX),np.radians(self.cameraY),np.radians(self.cameraZ),0,0,0,0,1,0)
        #gluLookAt(3,3,3,self.cameraX, self.cameraY, self.cameraZ, 0,1,0)
        # position object
        #glRotatef(self.y, 1.0, 0.0, 0.0)
        #glRotatef(self.x, 0.0, 1.0, 0.0)
        light_pos = [100.,100,100,0.]
        
        diffuse = [1.0,0.5,0.5,1]
        ambient = [0.5,0.5,0.5,1]
        specular = [0.5,0.5,0.5,1]
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)       
        
        light_poss = [10,-2,10,0.]
        glLightfv(GL_LIGHT1, GL_POSITION, light_poss)
        #glLightfv(GL_LIGHT1, GL_AMBIENT, ambient)
        glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT1, GL_SPECULAR, specular)
        
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        #glEnable(GL_LIGHT1)
        glEnable(GL_NORMALIZE)
        #glutTimerFunc(1000, self.Timer, 0)
        self.DMlist = list() 
        """
        for i in self.sim.skeletons[1].root_bodynode().shapenodes:
            tDM = DM()
            #print(i.shape.path())
            tDM.load_Model(i.shape.path(),None)
            self.DMlist.append(tDM)
            u[M`5&#self.DM.load_Model(i.shape.path(),None)
        """
        #glTranslatef(0,0,-2)
        self.recursive_load_ModelMeshes(self.sim.skeletons[0].root_bodynode())
        self.recursive_load_ModelMeshes(self.sim.skeletons[1].root_bodynode())
        self.sidx = 0
        self.drawCount = 0
        self.Rclicked = False
        self.Lclicked = False

        self.CheckerBoardTextureData()
        glTexImage2D(GL_TEXTURE_2D, 0, 3, boxSize*8, boxSize*8, 0, GL_RGB, GL_UNSIGNED_BYTE, imageData)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glEnable(GL_TEXTURE_2D)

        self.trajectory = list()


        self.pCompoint = [0, 0, 0]

        self.cFoot_pos = [0,-1,0]
        self.cState = "0"
        self.end =[0,0,0]
        self.front = [0,0,0]
        self.befFoot = None
        self.leftFootMaximumHeight = -1
        self.rightFootMaximumHeight = -1
        self.isLeftFootMH = False
        self.isRightFootMH = False

        ##test
        self.test1Done = False
        self.test2Done = False
        self.test3Done = False
        self.test4Done = False
        self.test5Done = False
        glutInit()

    def recursive_load_ModelMeshes(self,root):
        self.load_ModelMeshes(root)
        #input("recursive_load_ModelMeshes")
        for i in root.child_bodynodes:
            self.recursive_load_ModelMeshes(i)

    def load_ModelMeshes(self,bodyNodes):
        for i in bodyNodes.shapenodes:
            if(type(i.shape) is pydart.shape.MeshShape):
                tDM = dM.drawingMesh(0)
                tDM.load_Model(i.shape.path(),None)
                self.DMlist.append(tDM)
            elif(type(i.shape) is pydart.shape.BoxShape):
                tDM = dM.drawingMesh(1,i.shape.size())
                self.DMlist.append(tDM)
            else:
                input("type Error,, add new Shape Type")
    
    def recursive_draw_ModelMeshes(self,root):        
        #if self.sidx > drawLimit:
        #   return

        glPushMatrix()
        glMultMatrixd(np.transpose(root.relative_transform()))
        #glMultMatrixd(root.transform())
        #glMultMatrixf(np.transpose(root.world_transform()))
        
        
        self.draw_ModelMeshes(root)
        
        #glPopMatrix()
        for i in root.child_bodynodes:
            self.recursive_draw_ModelMeshes(i)

        glPopMatrix()

    def draw_ModelMeshes(self,bodyNodes):
        for i in bodyNodes.shapenodes:
            #glPushMatrix()
            #glMultMatrixf(np.transpose(i.relative_transform()))
            #input(i.relative_transform())
            if(type(i.shape) is pydart.shape.MeshShape):
                if '.stl' in i.shape.path():
                    self.DMlist[self.sidx].renders(False)
            #    self.sidx += 1
            #self.DMlist[self.sidx].renders()
            #elif(type(i.shape) is pydart.shape.BoxShape):
                #print("BOX")
            self.sidx += 1

            #glPopMatrix()
    def recursive_draw_Shadow_ModelMeshes(self,root):        
        #if self.sidx > drawLimit:
        #   return
        glPushMatrix()
        #glMultMatrixd(np.transpose(root.relative_transform()))
        if self.controller.mCurrentStateMachine.returnDesiredJP() is not None:
            Djp = -1*self.controller.mCurrentStateMachine.returnDesiredJP() 
        mat = root.relative_transform()

        if self.controller.mCurrentStateMachine.mCurrentState.mName is "0":
            if root.id is 23:
                mat = self.EulerY(Djp[0])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
                
                #glMultMatrixd(mat)
            elif root.id is 24:
                mat = self.EulerY(Djp[1])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
  
                #glMultMatrixd(mat)
            elif root.id is 26:
                mat = self.EulerY(Djp[2])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]

                #glMultMatrixd(mat)
            elif root.id is 12:
                mat = self.EulerY(Djp[3])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
                #glMultMatrixd(mat)
            elif root.id is 14:
                mat = self.EulerY(Djp[4])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
        elif self.controller.mCurrentStateMachine.mCurrentState.mName is "1":
            if root.id is 23:
                #mat = self.EulerY(Djp[5])
                mat = self.EulerY(Djp[0])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 24:
                #mat = self.EulerY(Djp[6])
                mat = self.EulerY(Djp[1])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 26:
                #mat = self.EulerY(Djp[7])
                mat = self.EulerY(Djp[2])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 12:
                #mat = self.EulerY(Djp[8])
                mat = self.EulerY(Djp[3])               
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 14:
                #mat = self.EulerY(Djp[9])
                mat = self.EulerY(Djp[4])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
        elif self.controller.mCurrentStateMachine.mCurrentState.mName is "2":
            if root.id is 11:
                mat = self.EulerY(Djp[0])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 12:
                mat = self.EulerY(Djp[1])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 14:
                mat = self.EulerY(Djp[2])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 24:
                mat = self.EulerY(Djp[3])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 26:
                mat = self.EulerY(Djp[4])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            if root.id is 11:
                #Djp5
                mat = self.EulerY(Djp[0])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 12:
                #Djp6
                mat = self.EulerY(Djp[1])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 14:
                #7
                mat = self.EulerY(Djp[2])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 24:
                #8
                mat = self.EulerY(Djp[3])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]
            elif root.id is 26:
                #9
                mat = self.EulerY(Djp[4])
                mat[0:3,3] = mat[0:3,3] + root.relative_transform()[0:3,3]

        """    
            print("Kkk")
            input()
        if self.controller.mCurrentStateMachine.mCurrentState.mStanceFoot is self.sim.skeletons[1].body("l_foot"):
            print("swing is right")
            #else:
            glMultMatrixd(np.transpose(root.relative_transform()))
            #input()
        else:
            print("swing is left")
            if root.id is 11:
                #ang = Djp[0] - self.sim.skeletons[1].q[16]
                mat = self.EulerY(Djp[0])
                glMultMatrixd(mat)
                #glMultMatrixd(self.EulerY(-self.sim.skeletons[1].q[16]))
            else:
                glMultMatrixd(np.transpose(root.relative_transform()))

        if root.id is 11:
            print(Djp[0])
            print(self.sim.skeletons[1].q[16])
            print(self.sim.skeletons[1].q[28])
            print(root.relative_transform())
            #input()
            #mat = self.EulerY(np.pi*1/6)
            #glMultMatrixd(mat)
            #input()
        if root.id is 12:
            print(root.relative_transform())
            #mat = self.EulerY(-np.pi*1/6)
            #glMultMatrixd(mat)
            #input()
        else:
            print(root.name)
            print(root.id)
        """
        glMultMatrixd(np.transpose(mat))
        #glMultMatrixd(root.transform())
        #glMultMatrixf(np.transpose(root.world_transform()))
        glColor3f(0.5,0.3, 0.0) 
        #glTranslatef(0,0.3,0) 
        #self.draw_Shadow_ModelMeshes(root)
        
        #glPopMatrix()
        #for i in root.child_bodynodes:
        #    self.recursive_draw_Shadow_ModelMeshes(i)

        glPopMatrix()


    def draw_Shadow_ModelMeshes(self,bodyNodes):
        for i in bodyNodes.shapenodes:
            i.set_visual_aspect_rgba([0.5,0.5,0.5,0.5])
            #glPushMatrix()
            #glMultMatrixf(np.transpose(i.relative_transform()))
            #input(i.relative_transform())
            if(type(i.shape) is pydart.shape.MeshShape):
                if '.stl' in i.shape.path():
                    self.DMlist[self.sidx].renders(True)
            #    self.sidx += 1
            #self.DMlist[self.sidx].renders()
            #elif(type(i.shape) is pydart.shape.BoxShape):
                #print("BOX")
            self.sidx += 1

            #glPopMatrix()



    def reset_sIdx(self):
        self.sidx = 0
 
    def OnDraw(self):
        global gEye, gAt, gUp
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glClearColor(1.0,1.0,1.0,0)
        compoint = self.sim.skeletons[1].com()


        glPushMatrix()
        #print(self.cameraX)
        #print(self.cameraY)
        #print(self.cameraZ)
        self.drawGroundBox()

        self.recursive_draw_ModelMeshes(self.sim.skeletons[0].root_bodynode())
        self.recursive_draw_ModelMeshes(self.sim.skeletons[1].root_bodynode())
        self.sidx = 2
        glTranslatef(0,0,-0.9)
        self.recursive_draw_Shadow_ModelMeshes(self.sim.skeletons[1].root_bodynode())
        self.reset_sIdx()
        #print("OnDrawEnd")
        glPopMatrix()

        #glDisable(GL_LIGHTING)
        """
        compoint = self.sim.skeletons[1].com()
        tc = compoint+self.env.targetFrameXAxis
        glPushMatrix()
        glColor3f(1.0,0,0)
        glTranslatef(tc[0], tc[1],tc[2])
        glutSolidSphere(1,20,20)
        glPopMatrix()
        """
        ##직선 라인
        """
        glColor3f(1.0,0.0,0.0)  
        glBegin(GL_LINES)
        compoint = self.sim.skeletons[1].com()
        glVertex3f(compoint[0], -0.95, compoint[2])
        tc = compoint+self.env.targetFrameXAxis
        glVertex3f(tc[0], tc[1], tc[2])
        glEnd()
        """
        """
        glBegin(GL_LINES)
        glColor3f(0,1.0,0)
        glVertex3f(compoint[0], -0.95, compoint[2])
        cc = compoint + self.env.currentFrameXAxis
        glVertex3f(cc[0], -0.95, cc[2])
        glEnd()
        """


        """
        if self.env.previousState is "0":
            rcp = compoint + self.env.FXAnorm*self.env.rightFoot
            lcp = compoint - self.env.FXAnorm*self.env.leftFoot
        else:
            rcp = compoint - self.env.FXAnorm*self.env.rightFoot
            lcp = compoint + self.env.FXAnorm*self.env.leftFoot
        """

        """
        glClear(GL_DEPTH_BUFFER_BIT)

        if self.env.previousState is not self.cState:
            rcp = compoint + self.env.FXAnorm*self.env.rightFoot
            lcp = compoint + self.env.FXAnorm*self.env.leftFoot
 
            if self.env.previousState is "0":
                self.front = [rcp[0], -0.95, rcp[2]]
                self.end = [lcp[0], -0.95, lcp[2]]
            else:
                self.end = [rcp[0], -0.95, rcp[2]]
                self.front = [lcp[0], -0.95, lcp[2]]
            self.r = rcp
            self.l = lcp
        self.end = self.env.endFoot
        self.end = np.array([self.end[0], self.end[1]-1/40, self.end[2]])
        self.createForwardArrow(self.end, self.end+self.env.cStepLength*self.env.FXAnorm, 0.05)
        self.createForwardCuboid(self.end, self.end+self.env.desiredStepLength*self.env.FXAnorm, 0.1, 0.1)

        #self.createForwardArrow(self.front-self.env.cStepLength*self.env.FXAnorm, self.front, 0.05)
        #self.createForwardCuboid(self.front-self.env.desiredStepLength*self.env.FXAnorm, self.front, 0.1, 0.1)
        """

        """
        glBegin(GL_LINES)
        glColor3f(1.0,0,0)
        glVertex3f(self.r[0], -0.95, self.r[2])
        glVertex3f(self.l[0], -0.95, self.l[2])
        glEnd()
        """
        """"
        glBegin(GL_LINES)
        glColor3f(0,0,1)
        glVertex3f(compoint[0],compoint[1],compoint[2])
        if self.env.a is not None:
            aa = compoint + self.env.a
        else:
            aa = compoint + [1,1,0]
        glVertex3f(aa[0], aa[1], aa[2])
        glEnd()
        """


        """
        ##Torso 라인
        glBegin(GL_LINES)
        glColor3f(1,0.5,1)
        glVertex3f(compoint[0], compoint[1], compoint[2])
        aa = compoint + self.env.mtorso.world_transform()[0:3,2]
        glVertex3f(aa[0],aa[1],aa[2])
        glEnd()
        """

        """
        glColor3f(0.5, 0 ,1)
        pf = compoint + self.env.getCOMFrameXAxis()
        ppf = compoint + self.env.ppreviousforward
        glBegin(GL_LINES)
        glVertex3f(compoint[0], compoint[1], compoint[2])
        glVertex3f(pf[0], pf[1], pf[2])
        glEnd()
        glColor3f(0, 0.5 ,1)
        glBegin(GL_LINES)
        glVertex3f(compoint[0], compoint[1], compoint[2])
        glVertex3f(ppf[0], ppf[1], ppf[2])
        glEnd()
        """
        #glClear(GL_DEPTH_BUFFER_BIT)
        #self.cFoot_pos
        #self.cState



        """
        glBegin(GL_QUADS)
        compointt = compoint[2]
        glVertex3f(foot_pos[0],foot_pos[1],compointt)
        glVertex3f(foot_pos[0]-self.env.desiredStepLength, foot_pos[1], compointt)
        glVertex3f(foot_pos[0]-self.env.desiredStepLength, foot_pos[1] - 0.05, compointt)
        glVertex3f(foot_pos[0], foot_pos[1] - 0.05, compointt)
        glEnd()

        glBegin(GL_QUADS)
        glColor3f(0.1,0.3,0.5)
        r_foot_pos = self.env._getJointPosition(self.env.r_foot)
        glVertex3f(foot_pos[0],foot_pos[1],compointt)
        glVertex3f(foot_pos[0]-self.env.cStepLength, foot_pos[1], compointt)
        glVertex3f(foot_pos[0]-self.env.cStepLength, foot_pos[1]+0.05, compointt)
        glVertex3f(foot_pos[0], foot_pos[1] + 0.05, compointt)
        glEnd()

       
        
        glBegin(GL_QUADS)
        glColor3f(0.9,0.1,0.1)
        r_foot_pos = self.env._getJointPosition(self.env.r_foot)
        footY = foot_pos[1] - 0.1
        glVertex3f(foot_pos[0] + 0.05, footY+self.env.cMaximumSwingfootHeight, compointt)
        glVertex3f(foot_pos[0], footY+self.env.cMaximumSwingfootHeight, compointt)
        glVertex3f(foot_pos[0], footY, compointt)
        glVertex3f(foot_pos[0] + 0.05, footY, compointt)

        glEnd()
       
        glBegin(GL_QUADS)
        glColor3f(0.5,0.9,0.5)
        r_foot_pos = self.env._getJointPosition(self.env.r_foot)
        footY = foot_pos[1] - 0.1
        glVertex3f(foot_pos[0] + 0.1, footY+self.env.desiredMaximumSwingfootHeight, compointt)
        glVertex3f(foot_pos[0] + 0.05, footY+self.env.desiredMaximumSwingfootHeight, compointt)
        glVertex3f(foot_pos[0] + 0.05, footY, compointt)
        glVertex3f(foot_pos[0] + 0.1, footY, compointt)
        

        glEnd()
        """

        r_foot_pos = self.env._getJointPosition(self.env.r_foot)
        pelvisPos = self.env.pelvis.world_transform()[:3,3]
        sfoot = np.array([pelvisPos[0]-0.75,-0.93, pelvisPos[2]])
        dfoot = np.array([pelvisPos[0]-0.75,self.env.desiredMaximumSwingfootHeight-0.93,pelvisPos[2]])

        self.createUpwardCuboid(sfoot,dfoot,0,0)
        self.BOX(pelvisPos[0]-0.7,-0.93,pelvisPos[2],0.015,0.015,0.015)

        #self.BOX(pelvisPos[0],-0.93,pelvisPos[2],self.env.desiredStepLength,0.015,0.015)
    
        dStepLengthBoxPos = np.array([pelvisPos[0]-0.25, -1.05, pelvisPos[2]])
        StepLengthBoxPos = np.array([pelvisPos[0]-0.25, -1, pelvisPos[2]])
        #glClear(GL_DEPTH_BUFFER_BIT)
        self.createForwardCuboid(dStepLengthBoxPos, self.env.desiredStepLength,0.025,0.025,np.array([1,0,0]))
        self.createForwardBOX(StepLengthBoxPos, self.env.cStepLength,0.015, 0.015, np.array([1,0,0]))


        glColor3f(1,0,1,0)
        #glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ctypes.c_int(ord('a')))
        self.printInfo()

        ##카메라
        xM = self.pCompoint[0] - compoint[0]
        yM = self.pCompoint[1] - compoint[1]
        #print(xM,yM)
        zM = self.pCompoint[2] - compoint[2]
        self.pCompoint = compoint
        #print(gEye) 
        gEye = gEye - [xM, 0, zM]
        gAt = gAt - [xM, 0, zM]
        #print(gEye)
        #input()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()


        self.myLookAt(gEye,gAt,gUp)
        #time.sleep(0.001)
        self.SwapBuffers()

    def printInfo(self,):
        ###StepLength
        glColor3f(0,0,0)
        glWindowPos2f(0.0,self.size.height-15.0)
        DesiredStepLengthStr = "DesiredStepLength: " + str(np.round(self.env.desiredStepLength,3)) + " ,StepLength: " + str(np.round(self.env.cStepLength,3))
        self.printText(DesiredStepLengthStr)
        ###StepDuration
        glWindowPos2f(0.0,self.size.height-30.0)
        DesiredStepDuration = "DesiredStepDuration: " + str(np.round(self.env.desiredStepDuration,3)) + ",Step Duration: " + str(np.round(self.env.cStepDuration,3))
        self.printText(DesiredStepDuration)
        ###SwingFootHeight
        glWindowPos2f(0.0,self.size.height-45.0)
        DesiredSwingFootHeight = "DesiredSwingfootHeight: " + str(np.round(self.env.desiredMaximumSwingfootHeight,3)) + " ,SwingFootHeight: " + str(np.round(self.env.cMaximumSwingfootHeight,3))
        self.printText(DesiredSwingFootHeight)

        ###currentOffset
        glWindowPos2f(0.0,self.size.height-60.0)
        currentOffsetStr = "Current Offset: " + str(self.env.currentOffset)
        self.printText(currentOffsetStr)
        """
        glWindowPos2f(10.0,self.size.height-80.0)
        self.drawGraph(int(np.round(self.env.cStepDuration *1000)),128)
        glWindowPos2f(10.0,self.size.height-90.0)
        self.drawGraph(int(np.round(self.env.desiredStepDuration*1000)),0) 
        """


        ###StepDuration
        

        glWindowPos2f(50.0,self.size.height-100.0)
        self.drawGraphcva(10,np.array((153,0,0,1)))

        glWindowPos2f(60.0,self.size.height-100.0)
        DesiredStepLengthStr = " Desired step duration" 
        cStepLengthStr = " Current step duration"
        self.printText(DesiredStepLengthStr)

        glWindowPos2f(50.0,self.size.height-115)
        self.drawGraphcva(10,np.array((255,51,51,1)))
        glWindowPos2f(60.0,self.size.height-115)
        self.printText(cStepLengthStr)

        ##StepLength
        glWindowPos2f(50.0,self.size.height-130.0)
        self.drawGraphcva(10,np.array((0,0,102,1)))

        glWindowPos2f(60.0,self.size.height-130.0)
        DesiredStepLengthStr = " Desired step length" 
        cStepLengthStr = " Current step length"
        self.printText(DesiredStepLengthStr)

        glWindowPos2f(50.0,self.size.height-145)
        self.drawGraphcva(10,np.array((51,51,255,1)))
        glWindowPos2f(60.0,self.size.height-145)
        self.printText(cStepLengthStr)

        ##SwingFootHeight
        glWindowPos2f(50.0,self.size.height-160.0)
        self.drawGraphcva(10,np.array((0,51,0,1)))

        glWindowPos2f(60.0,self.size.height-160.0)
        DesiredStepLengthStr = " Desired maximum swingfoot height" 
        cStepLengthStr = " Current maximum swingfoot height"
        self.printText(DesiredStepLengthStr)

        glWindowPos2f(50.0,self.size.height-175)
        self.drawGraphcva(10,np.array((102,255,51,1)))
        glWindowPos2f(60.0,self.size.height-175)
        self.printText(cStepLengthStr)





        glWindowPos2f(200.0,self.size.height-500.0)
        #self.drawGraph(int(np.round(self.env.cStepDuration *500)),128)
        self.drawGraphcva(int(np.round(self.env.cStepDuration *500)),np.array((255,51,51,1)))

        glWindowPos2f(200.0,self.size.height-515.0)
        self.drawGraphcva(int(np.round(self.env.desiredStepDuration*500)),np.array((153,0,0,1)))

        ###TotalReward

    def rotateTheta(self, axis, pos, angle):
        axis = angle*axis
        rot = cv2.Rodrigues(axis)[0]
        return rot@pos

    def calNorVec(self,varr,iarr):
        ####index의 인접 평면 저장하는 함수
        tniarr = dict()
        #print(varr.size)
        for i in range(int(varr.size/3)):
            tniarr[i] = np.zeros(0)

        tniarrIndex = 0
        for i in iarr:
            tniarr[i] = np.append(tniarr[i],[int(tniarrIndex/3)])
            tniarrIndex+=1

        #print(tniarr)

        ##각 BOX Triangle의 NormalVector
        narr = np.array([])
        narr0 = np.array([0,1,0])
        narr1 = narr0
        narr2 = -narr0 
        narr3 = narr2 
        narr4 = -self.calNormal(varr[5] - varr[0], varr[1] - varr[0])
        narr5 = narr4 
        narr6 = self.calNormal(varr[2] - varr[3], varr[6] - varr[3])
        narr7 = narr6
        narr8 = -self.calNormal(varr[6] - varr[1], varr[2] - varr[1])
        narr9 = narr8
        narr10 = -narr8
        narr11 = narr10

        narr = np.concatenate([narr0,narr1,narr2,narr3,narr4,narr5,narr6,narr7,narr8,narr9,narr10,narr11])
        narr = np.append(narr,narr)

        ##planeVec의 경우
        normVec = np.array([])
        for i in range(int(narr.size/3)):
            i = i*3
            for j in range(3):
                normVec = np.append(normVec,[narr[i],narr[i+1],narr[i+2]])
            #print(i)

        #print(normVec)
        """
        nvarr = dict()
        idx = 0
        for i in tniarr:
            #print(tniarr[i])
            
            nvarr[idx] = self.calVertNorm(narr,tniarr[i])
            idx+=1

        normVec = np.array([])
        for i in iarr:
            #print(i)
            normVec = np.append(normVec,nvarr[i])
        #print(narr)
        #print(nvarr)
        """
        return normVec
        
    def calNormal(self,X,Y):
        return np.array(cMat.Matrix.normalize(np.cross(X,Y)))


    def calVertNorm(self,narr,idxarr):
        sumv = np.array([0,0,0])
        size = 0
        for i in idxarr:
            i = int(i)
            sumv = sumv + np.array([narr[i*3],narr[i*3+1],narr[i*3+2]])
            size += 1
        return np.array(cMat.Matrix.normalize(sumv/size))

    def createForwardCuboid(self, EndCenter, distance, width, height, forwardVec):
        specular = [1.0,1.0,1.0,1.0]
        shininess = [ 100.0 ]
        ambient = [1.0,1.0, 1.0, 1.0]
        diffuse = [0/255,0/255, 102/255, 1.0]
        glClearColor( 0.0, 0.0, 0.0, 0.0)
        glShadeModel(GL_SMOOTH)

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)

        #ambient = [1.0,1.0, 1.0, 0.9]
        diffuse = [0/255,51/255, 1, 0.5]
        light_poss = [0.5,-0.5,-0,1.]
        glLightfv(GL_LIGHT1, GL_POSITION, light_poss)
        #glLightfv(GL_LIGHT1, GL_AMBIENT, ambient)
        glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT1, GL_SPECULAR, specular)


        disVec = distance * forwardVec
        headVec =-0.01*np.array(cMat.Matrix.normalize(disVec))
        heightVec = np.array([0,height,0])
        widthVec = width* np.array(cMat.Matrix.normalize(np.cross(disVec,heightVec)))


        specular = [153/255,204/255, 1, 0.5]
        light_poss = EndCenter + disVec + widthVec*2
        light_poss = np.append(light_poss,[1])
        glLightfv(GL_LIGHT2, GL_POSITION, light_poss)
        #glLightfv(GL_LIGHT1, GL_AMBIENT, ambient)
        #glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT2, GL_SPECULAR, specular)
        #glEnable(GL_LIGHT2)

        specular = [255/255,204/255, 1, 0.5]
        light_poss = EndCenter + disVec + heightVec*2
        light_poss = np.append(light_poss,[1])
        glLightfv(GL_LIGHT3, GL_POSITION, light_poss)
        #glLightfv(GL_LIGHT1, GL_AMBIENT, ambient)
        #glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT3, GL_SPECULAR, specular)
        #glEnable(GL_LIGHT3)



        varr = np.array([
            EndCenter + disVec + heightVec - widthVec, #v0
            EndCenter + heightVec -widthVec, #v1
            EndCenter + heightVec + widthVec, #v2
            EndCenter + disVec + heightVec + widthVec, #v3

            EndCenter + disVec - heightVec - widthVec, #v4
            EndCenter - heightVec -widthVec, #v5
            EndCenter - heightVec + widthVec, #v6
            EndCenter + disVec - heightVec + widthVec, #v7

            EndCenter + disVec + 4*heightVec - widthVec, #v4
            EndCenter + disVec + headVec + 4*heightVec -widthVec, #v5
            EndCenter + disVec + headVec + 4*heightVec + widthVec, #v6
            EndCenter + disVec + 4*heightVec + widthVec, #v7

            EndCenter + disVec + heightVec - widthVec, #v8
            EndCenter + disVec + headVec + heightVec -widthVec, #v9
            EndCenter + disVec + headVec + heightVec + widthVec, #v10
            EndCenter + disVec + heightVec + widthVec, #v11



        ], "float32")

        iarr = np.array([
            (0,1,2),
            (0,3,2),
            (4,5,6),
            (4,6,7),
            (0,1,5),
            (0,5,4),
            (3,6,2),
            (3,7,6),
            (1,2,6),
            (1,6,5),
            (0,7,3),
            (0,4,7),
        ])





        iiarr = np.array([[j+8 for j in ar] for ar in iarr])


        iarr = np.append(iarr,iiarr)

        #print(self.calNorVec(varr,iarr))
        NormArray = self.calNorVec(varr,iarr)

        #print(NormArray)
        upp = np.array([0,1,0])
        forr = np.array([1,0,0])
        me = np.array([0,0,1])

        
        varrA = np.array([
            upp,varr[0],upp,varr[1],upp,varr[2],
            
            upp,varr[0],upp,varr[3],upp,varr[2],

            -upp,varr[4],-upp,varr[5],-upp,varr[6],

            -upp,varr[4],-upp,varr[6],-upp,varr[7],

            -me,varr[0],-me,varr[1],-me,varr[5],

            -me,varr[0],-me,varr[5],-me,varr[4],
            
            me,varr[3],me,varr[6],me,varr[2],
            
            me,varr[3],me,varr[7],me,varr[6],

            -forr,varr[1],-forr,varr[2],-forr,varr[6],

            -forr,varr[1],-forr,varr[6],-forr,varr[5],

            forr,varr[0],forr,varr[7],forr,varr[3],

            forr,varr[0],forr,varr[4],forr,varr[7],

            upp,varr[8],upp,varr[9],upp,varr[10],
            
            upp,varr[8],upp,varr[11],upp,varr[10],

            -upp,varr[12],-upp,varr[13],-upp,varr[14],

            -upp,varr[12],-upp,varr[14],-upp,varr[15],

            -me,varr[8],-me,varr[9],-me,varr[13],

            -me,varr[8],-me,varr[13],-me,varr[12],
            
            me,varr[11],me,varr[14],me,varr[10],
            
            me,varr[11],me,varr[15],me,varr[14],

            -forr,varr[9],-forr,varr[10],-forr,varr[14],

            -forr,varr[9],-forr,varr[14],-forr,varr[13],

            forr,varr[8],forr,varr[15],forr,varr[11],

            forr,varr[8],forr,varr[12],forr,varr[15]

        ], 'float32')
        


        glClear(GL_DEPTH_BUFFER_BIT)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)
        #glNormalPointer(GL_FLOAT,3*NormArray.itemsize,NormArray)
        #glVertexPointer(3, GL_FLOAT, 3*varr.itemsize, varr)
        #glDrawElements(GL_TRIANGLES, iarr.size, GL_UNSIGNED_INT, iarr)

        glNormalPointer(GL_FLOAT, 6*varrA.itemsize, varrA)
        glVertexPointer(3, GL_FLOAT, 6*varrA.itemsize, ctypes.c_void_p(varrA.ctypes.data + 3*varrA.itemsize))
        glDrawArrays(GL_TRIANGLES,0,int(varrA.size/6))


    def createForwardBOX(self, EndCenter, distance, width, height, forwardVec):
        specular = [1.0,1.0,1.0,1.0]
        shininess = [ 100.0 ]
        ambient = [1.0,1.0, 1.0, 1.0]
        diffuse = [51/255,51/255, 255/255, 1.0]
        glClearColor( 0.0, 0.0, 0.0, 0.0)
        glShadeModel(GL_SMOOTH)

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)


        disVec = distance * forwardVec
        headVec =-0.05*np.array(cMat.Matrix.normalize(disVec))
        heightVec = np.array([0,height,0])
        widthVec = width* np.array(cMat.Matrix.normalize(np.cross(disVec,heightVec)))
        varr = np.array([
            EndCenter + disVec + heightVec - widthVec, #v0
            EndCenter + heightVec -widthVec, #v1
            EndCenter + heightVec + widthVec, #v2
            EndCenter + disVec + heightVec + widthVec, #v3

            EndCenter + disVec - heightVec - widthVec, #v4
            EndCenter - heightVec -widthVec, #v5
            EndCenter - heightVec + widthVec, #v6
            EndCenter + disVec - heightVec + widthVec, #v7
        ], "float32")

        iarr = np.array([
            (0,1,2),
            (0,3,2),
            (4,5,6),
            (4,6,7),
            (0,1,5),
            (0,5,4),
            (3,6,2),
            (3,7,6),
            (1,2,6),
            (1,6,5),
            (0,7,3),
            (0,4,7),
        ])

        #print(NormArray)
        upp = np.array([0,1,0])
        forr = np.array([1,0,0])
        me = np.array([0,0,1])

        
        varrA = np.array([
            upp,varr[0],upp,varr[1],upp,varr[2],
            
            upp,varr[0],upp,varr[3],upp,varr[2],

            -upp,varr[4],-upp,varr[5],-upp,varr[6],

            -upp,varr[4],-upp,varr[6],-upp,varr[7],

            -me,varr[0],-me,varr[1],-me,varr[5],

            -me,varr[0],-me,varr[5],-me,varr[4],
            
            me,varr[3],me,varr[6],me,varr[2],
            
            me,varr[3],me,varr[7],me,varr[6],

            -forr,varr[1],-forr,varr[2],-forr,varr[6],

            -forr,varr[1],-forr,varr[6],-forr,varr[5],

            forr,varr[0],forr,varr[7],forr,varr[3],

            forr,varr[0],forr,varr[4],forr,varr[7],
        ], 'float32')
        


        glClear(GL_DEPTH_BUFFER_BIT)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)
        #glNormalPointer(GL_FLOAT,3*NormArray.itemsize,NormArray)
        #glVertexPointer(3, GL_FLOAT, 3*varr.itemsize, varr)
        #glDrawElements(GL_TRIANGLES, iarr.size, GL_UNSIGNED_INT, iarr)

        glNormalPointer(GL_FLOAT, 6*varrA.itemsize, varrA)
        glVertexPointer(3, GL_FLOAT, 6*varrA.itemsize, ctypes.c_void_p(varrA.ctypes.data + 3*varrA.itemsize))
        glDrawArrays(GL_TRIANGLES,0,int(varrA.size/6))

    def createForwardArrow(self, EndCenter, FrontCenter, radius):
        EndCenter[1] = EndCenter[1] + 1/80
        FrontCenter[1] = FrontCenter[1] + 1/80

        specular = [1.0,1.0,1.0,1.0]
        shininess = [ 100.0 ]
        light_pos = [0,1.0,100.0, 1.0]
        ambient = [1.0,1.0, 1.0, 0.9]
        diffuse = [204/255,153/255, 51/255, 0.5]
        glClearColor( 0.0, 0.0, 0.0, 0.0)
        glShadeModel(GL_SMOOTH)

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT1, GL_POSITION, light_pos)        
      

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT1)
        glEnable(GL_DEPTH_TEST)
        
        
        rotateVec = np.array([[np.cos(np.pi/2), 0],
                [0,  np.sin(np.pi/2)]])

        axis = np.array([self.env.FXAnorm[0], self.env.FXAnorm[2]])
        vec = rotateVec@axis
        vec = np.array([vec[0],0,vec[1]])
        axis = np.array([axis[0], 0, axis[1]])

        ##Arrow Head의 시작위치
        triDistVec = (self.env.cStepLength/3)*self.env.FXAnorm

        glColor4f(0.1, 0.1, 0.6, 0.5)
        ECx = EndCenter[0]
        ECy = EndCenter[1]
        ECz = EndCenter[2]
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(-1.0, 0.0, 0.0)

        glVertex3f(ECx, ECy, ECz)

        for i in range(33):
            angle = i *np.pi/16

            vv = self.rotateTheta(axis,vec,angle)
            vv = vv/np.linalg.norm(vv)/60
            #vy = ECy + radius*np.sin(angle)
            #vz = ECz + radius*np.cos(angle)
            #glVertex3f(ECx, vy,vz)
            glVertex3fv(EndCenter+vv)
            glNormal3fv(vv*40)
        glEnd()
        
         
        FCxV = FrontCenter - triDistVec
        #FCxV[1] = FCxV[1] - 1/40
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(1.0, 0.0, 0.0)

        glVertex3fv(FCxV)

        for i in range(33):
            angle = i *np.pi/16
            vv = self.rotateTheta(axis,vec,angle)
            vv = vv/np.linalg.norm(vv)/60
            glNormal3fv(vv*40)
            glVertex3fv(FCxV + vv)
        glEnd()
        
        
        glBegin(GL_TRIANGLE_STRIP)
        for i in range(33):
            angle = i * np.pi/16
            vv = self.rotateTheta(axis,vec,angle)
            vv = vv/np.linalg.norm(vv)/60
        
            #glNormal3f(np.sin(angle), 0.0, np.cos(angle))
            glNormal3fv(vv*40)
            glVertex3fv(EndCenter+vv)
            glNormal3fv(vv*40)
            glVertex3fv(FCxV + vv)
        glEnd()


       
        #### Arrow Head
        glBegin(GL_TRIANGLE_FAN)
        glNormal3fv(self.env.FXAnorm)

        glVertex3fv(FCxV)

        for i in range(33):
            angle = i *np.pi/16
            vv = self.rotateTheta(axis,vec,angle)
            vv = (vv/np.linalg.norm(vv)/40)
            glVertex3fv(FCxV + vv)
            glNormal3fv(vv*80+self.env.FXAnorm)
        glEnd()
        
        glBegin(GL_TRIANGLE_STRIP)

        for i in range(33):
            angle = i *np.pi/16
            vv = self.rotateTheta(axis,vec,angle)
            vv = (vv/np.linalg.norm(vv)/40)
            glVertex3fv(FrontCenter)
            glNormal3f(1.0, 0.0, 0.0)
            glVertex3fv(FCxV + vv)
            glNormal3fv(vv*80+self.env.FXAnorm)
        glEnd()
       

    def createUpwardCuboid(self, EC, FC, horizontal, vertical):
        glColor3f(0.0,0.0,1.0)

        specular = [1.0,1.0,1.0,1.0]
        shininess = [ 100.0 ]
        light_pos = [1.0,0.0,-10.0, 0.0]
        ambient = [1.0,1.0, 1.0, 0.9]
        diffuse = [0/255,51/255, 0/255, 1]
        glClearColor( 0.0, 0.0, 0.0, 0.0)
        glShadeModel(GL_SMOOTH)

        #glLightfv(GL_LIGHT1, GL_POSITION, light_pos)        

        #glEnable(GL_DEPTH_TEST)
        #glEnable(GL_LIGHTING)
        #glEnable(GL_LIGHT1)

        #glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
        #glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, diffuse)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, diffuse)

        #horizonVec = np.cross(self.env.FXAnorm,np.array([0,1,0]))
        #horizonVec = np.dot(self.skew(self.env.FXAnorm),np.array([0,1,0]))
        #horizonVec = 0.0125*horizonVec
        #verticalVec = 0.0125*self.env.FXAnorm

        horizonVec = [0.025,0,0]
        verticalVec = [0,0.0,0.025]
        horV = [0.1, 0,0]

        EndCenter = copy.deepcopy(EC)
        FrontCenter = copy.deepcopy(FC)

        normalY = np.array((0,1,0))
        normalX = np.array((1,0,0))
        normalZ = np.cross(normalX,normalY)

        #print(normalZ)

        varr = np.array([
            ##위
            EndCenter + horizonVec + verticalVec, #V0
            EndCenter - horizonVec + verticalVec, #v1
            EndCenter - horizonVec - verticalVec, #v2
            EndCenter + horizonVec - verticalVec, #v3

            ##아래
            FrontCenter + horizonVec + verticalVec, #v4
            FrontCenter - horizonVec + verticalVec, #v5
            FrontCenter - horizonVec - verticalVec, #v6
            FrontCenter + horizonVec - verticalVec, #v7


            FrontCenter + horV + verticalVec, #v8
            FrontCenter - horizonVec + verticalVec, #v5
            FrontCenter - horizonVec - verticalVec, #v6
            FrontCenter + horV - verticalVec, #v9




            FrontCenter + horV + verticalVec - np.array([0,0.01, 0]), #v10
            FrontCenter - horizonVec + verticalVec - np.array([0,0.01, 0]), #v11
            FrontCenter - horizonVec - verticalVec - np.array([0,0.01, 0]), #v12
            FrontCenter + horV - verticalVec - np.array([0,0.01, 0]) #v13

        ], 'float32')
        
        iarr = np.array([
            (0,1,2),
            (0,3,2),
            (4,5,6),
            (4,6,7),
            (0,1,5),
            (0,5,4),
            (3,6,2),
            (3,7,6),
            (1,2,6),
            (1,6,5),
            (0,7,3),
            (0,4,7),

            (8,5,6),
            (8,9,6),
            (10,11,12),
            (10,12,13),
            (8,5,11),
            (8,11,10),
            (9,12,6),
            (9,13,12),
            (5,6,12),
            (5,12,11),
            (8,13,9),
            (8,10,13),
        ])

        #print(NormArray)
        upp = np.array([0,1,0])
        forr = np.array([1,0,0])
        me = -np.array([0,0,1])



        varrA = np.array([
            upp,varr[0],upp,varr[1],upp,varr[2],
            
            upp,varr[0],upp,varr[3],upp,varr[2],

            -upp,varr[4],-upp,varr[5],-upp,varr[6],

            -upp,varr[4],-upp,varr[6],-upp,varr[7],

            -me,varr[0],-me,varr[1],-me,varr[5],

            -me,varr[0],-me,varr[5],-me,varr[4],
            
            me,varr[3],me,varr[6],me,varr[2],
            
            me,varr[3],me,varr[7],me,varr[6],

            -forr,varr[1],-forr,varr[2],-forr,varr[6],

            -forr,varr[1],-forr,varr[6],-forr,varr[5],

            forr,varr[0],forr,varr[7],forr,varr[3],

            forr,varr[0],forr,varr[4],forr,varr[7],

            upp,varr[8],upp,varr[9],upp,varr[10],
            
            upp,varr[8],upp,varr[11],upp,varr[10],

            -upp,varr[12],-upp,varr[13],-upp,varr[14],

            -upp,varr[12],-upp,varr[14],-upp,varr[15],

            -me,varr[8],-me,varr[9],-me,varr[13],

            -me,varr[8],-me,varr[13],-me,varr[12],
            
            me,varr[11],me,varr[14],me,varr[10],
            
            me,varr[11],me,varr[15],me,varr[14],

            -forr,varr[9],-forr,varr[10],-forr,varr[14],

            -forr,varr[9],-forr,varr[14],-forr,varr[13],

            forr,varr[8],forr,varr[15],forr,varr[11],

            forr,varr[8],forr,varr[12],forr,varr[15]

        ], 'float32')
        


        glClear(GL_DEPTH_BUFFER_BIT)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)
        #glNormalPointer(GL_FLOAT,3*NormArray.itemsize,NormArray)
        #glVertexPointer(3, GL_FLOAT, 3*varr.itemsize, varr)
        #glDrawElements(GL_TRIANGLES, iarr.size, GL_UNSIGNED_INT, iarr)

        glNormalPointer(GL_FLOAT, 6*varrA.itemsize, varrA)
        glVertexPointer(3, GL_FLOAT, 6*varrA.itemsize, ctypes.c_void_p(varrA.ctypes.data + 3*varrA.itemsize))
        glDrawArrays(GL_TRIANGLES,0,int(varrA.size/6))
        



    def BOX(self,X,Y,Z, width = 0.015, height = 0.015, length = 0.015):

        specular = [1.0,1.0,1.0,1.0]
        shininess = [ 100.0 ]
        light_pos = [1.0,0.0,-10.0, 0.0]
        ambient = [1.0,1.0, 1.0, 0.9]
        diffuse = [102/255,255/255, 51/255, 0.1]
        glClearColor( 0.0, 0.0, 0.0, 0.0)
        glShadeModel(GL_SMOOTH)

        #glLightfv(GL_LIGHT1, GL_POSITION, light_pos)        

        #glEnable(GL_DEPTH_TEST)
        #glEnable(GL_LIGHTING)
        #glEnable(GL_LIGHT1)

        #glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, diffuse)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
        #glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, diffuse)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)


        YY = Y + self.env.cMaximumSwingfootHeight

        varr = np.array([
            (X-width, Y, Z-length), #V0
            (X-width, Y, Z+length), #V1
            (X+width, Y, Z+length), #V2

            (X+width, Y, Z-length), #V3

            (X-width, YY, Z-length), #V4
            (X-width, YY, Z+length), #V5
            (X+width, YY, Z+length), #V6

            (X+width, YY, Z-length), #V7

            ], 'float32')
           
        upp = -np.array([0,1,0])
        me = np.array([1,0,0])
        forr = -np.array([0,0,1])



        varrA = np.array([
            upp,varr[0],upp,varr[1],upp,varr[2],
            
            upp,varr[0],upp,varr[3],upp,varr[2],

            -upp,varr[4],-upp,varr[5],-upp,varr[6],

            -upp,varr[4],-upp,varr[6],-upp,varr[7],

            -me,varr[0],-me,varr[1],-me,varr[5],

            -me,varr[0],-me,varr[5],-me,varr[4],
            
            me,varr[3],me,varr[6],me,varr[2],
            
            me,varr[3],me,varr[7],me,varr[6],

            -forr,varr[1],-forr,varr[2],-forr,varr[6],

            -forr,varr[1],-forr,varr[6],-forr,varr[5],

            forr,varr[0],forr,varr[7],forr,varr[3],

            forr,varr[0],forr,varr[4],forr,varr[7],



        ], 'float32')
        


        glClear(GL_DEPTH_BUFFER_BIT)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)
        #glNormalPointer(GL_FLOAT,3*NormArray.itemsize,NormArray)
        #glVertexPointer(3, GL_FLOAT, 3*varr.itemsize, varr)
        #glDrawElements(GL_TRIANGLES, iarr.size, GL_UNSIGNED_INT, iarr)

        glNormalPointer(GL_FLOAT, 6*varrA.itemsize, varrA)
        glVertexPointer(3, GL_FLOAT, 6*varrA.itemsize, ctypes.c_void_p(varrA.ctypes.data + 3*varrA.itemsize))
        glDrawArrays(GL_TRIANGLES,0,int(varrA.size/6))

    def createCylinder(self, X, Y, Z, radius = 0.015, h = 0):

        specular = [1.0,1.0,1.0,1.0]
        shininess = [ 100.0 ]
        light_pos = [1.0,1.0,-1.0, 0.0]
        ambient = [1.0,1.0, 1.0, 0.9]
        diffuse = [153/255,0/255, 102/255, 0.5]
        glClearColor( 0.0, 0.0, 0.0, 0.0)
        glShadeModel(GL_SMOOTH)

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)


        Y = Y - 0.08
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0.0, 1.0, 0.0)
        glColor3f(0.5, 0.25, 0.1)
        glVertex3f(X, -0.93+self.env.cMaximumSwingfootHeight, Z)

        
        for i in range(33):
            angle = i * np.pi/16
            vx = X + radius*np.sin(angle)
            vz = Z + radius*np.cos(angle)
            glNormal3f(0.0, 1.0, 0.0)
            glVertex3f(vx, -0.93+self.env.cMaximumSwingfootHeight, vz)
        glEnd()
        

        glBegin(GL_QUAD_STRIP)
        for i in range(33):
            angle = i * np.pi/16
            vx = X + radius*np.sin(angle)
            vz = Z + radius*np.cos(angle)
            glNormal3f(np.sin(angle), 0.0, np.cos(angle))
            glVertex3f(vx, Y, vz)
            glVertex3f(vx, -0.93, vz)
        glEnd()


    def printText(self,Text):
        for ch in Text:
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ctypes.c_int(ord(ch)))


    def drawGraph(self,size,color):
        if size < 0:
            print(size)
        pix = np.zeros(12*size*4)
        for i in range(12*size*4):
            pix[i] = color
        for i in range(12*size):
            pix[i*4+3] = 0
        glDrawPixels(size,12,GL_RGBA, GL_UNSIGNED_BYTE, pix)

    def drawGraphcva(self,size,color):
        if size < 0:
            print(size)
        pix = np.zeros(15*size*4)
        for i in range(15*size):
            pix[i*4] = color[0]
            pix[i*4+1] = color[1]
            pix[i*4+2] = color[2]
            pix[i*4+3] = color[3]
        for i in range(15*size):
            pix[i*4+3] = 0
        glDrawPixels(size,12,GL_RGBA, GL_UNSIGNED_BYTE, pix)

    def TimerFunc(self,):
        #print("TImer....")
        self.currentTimeStep = self.currentTimeStep + self.timerOffset/900
        #print(self.currentTimeStep)
        #self.controller.update()
        #self.sim.step()

        #glutPostRedisplay()
        #self.OnDraw()
                 
        self.SecondPerTest = 10
        if self.currentTimeStep < 3:
            self.env.StartMedian()
        elif self.currentTimeStep < 3.5:
            self.env.StartMinimum()
        elif not self.test1Done:
            self.env.desiredStepDuration = self.env.desiredStepDuration + (0.4/self.SecondPerTest)*self.timerOffset/900
            #desiredStep값에 맞추어  StepLength 증가
            #desitedStep값에 맞추어 MaximumSwingfOotHeight 증가 
            if self.env.desiredStepLength < 2*self.env.desiredStepDuration/3:
                self.env.desiredStepLength = 2*self.env.desiredStepDuration/3
            if self.env.desiredMaximumSwingfootHeight < self.env.desiredStepDuration/4:
                self.env.desiredMaximumSwingfootHeight = self.env.desiredStepDuration/4
            if self.env.desiredStepDuration >= 0.5:
                self.test1Done = True
                self.env.desiredStepDuration = 0.5
                print("test1Done")
        elif not self.test2Done:
            self.env.desiredStepDuration = self.env.desiredStepDuration - (0.35/self.SecondPerTest)*self.timerOffset/900
            #desiredStep값에 맞추어  StepLength 감소
            #desitedStep값에 맞추어 MaximumSwingfOotHeight 감소
            if self.env.desiredStepLength > 2*self.env.desiredStepDuration/3 + 0.2:
                self.env.desiredStepLength =  2*self.env.desiredStepDuration/3 + 0.2
            if self.env.desiredMaximumSwingfootHeight > self.env.desiredStepDuration/4 + 0.15:
                self.env.desiredMaximumSwingfootHeight = self.env.desiredStepDuration/4 + 0.15
            if self.env.desiredStepDuration <= 0.1:
                self.test2Done = True
                self.env.desiredStepDuration = 0.1
                print("test2Done")
        elif not self.test3Done:
            if self.env.desiredStepLength > 0.1:
                self.env.desiredStepLength = self.env.desiredStepLength - (0.2/(self.SecondPerTest/5))*self.timerOffset/900
            else:
                self.env.desiredMaximumSwingfootHeight = self.env.desiredMaximumSwingfootHeight - (0.15/(self.SecondPerTest/5))*self.timerOffset/900            
            if self.env.desiredMaximumSwingfootHeight <= 0.15/2:
                print("test3Done")
                self.env.desiredStepLength = 0.1
                self.env.desiredMaximumSwingfootHeight = 0.15/2
                self.test3Done = True 
        elif not self.test4Done:
            ##중간값까지 상승
            if self.env.desiredStepDuration < 0.3:
                self.env.desiredStepDuration = self.env.desiredStepDuration + (0.4/self.SecondPerTest)*self.timerOffset/900
                #desiredStep값에 맞추어  StepLength 증가
                #desitedStep값에 맞추어 MaximumSwingfOotHeight 증가 
                if self.env.desiredStepLength < 2*self.env.desiredStepDuration/3:
                    self.env.desiredStepLength = 2*self.env.desiredStepDuration/3
                if self.env.desiredMaximumSwingfootHeight < self.env.desiredStepDuration/4:
                    self.env.desiredMaximumSwingfootHeight = self.env.desiredStepDuration/4
            ##중간값까지 상승하면 다른 값 조절
            elif self.env.desiredStepLength < self.env.desiredStepDuration*(2/3)+0.2:
                self.env.desiredStepLength = self.env.desiredStepLength + (0.2/(self.SecondPerTest/2))*self.timerOffset/900
            else:
                self.env.desiredMaximumSwingfootHeight = self.env.desiredMaximumSwingfootHeight + (0.15/(self.SecondPerTest/2))*self.timerOffset/900            

            if self.env.desiredMaximumSwingfootHeight >= self.env.desiredStepDuration/4+0.15:
                self.test4Done = True
                print("test4Done")
        elif not self.test5Done:
            ##최대값까지 상승
            if self.env.desiredStepDuration < 0.5:
                self.env.desiredStepDuration = self.env.desiredStepDuration + (0.4/self.SecondPerTest)*self.timerOffset/900
                #desiredStep값에 맞추어  StepLength 증가
                #desitedStep값에 맞추어 MaximumSwingfOotHeight 증가 
                if self.env.desiredStepLength < 2*self.env.desiredStepDuration/3:
                    self.env.desiredStepLength = 2*self.env.desiredStepDuration/3
                if self.env.desiredMaximumSwingfootHeight < self.env.desiredStepDuration/4:
                    self.env.desiredMaximumSwingfootHeight = self.env.desiredStepDuration/4
            elif self.env.desiredStepLength < self.env.desiredStepDuration*(2/3)+0.2:
                self.env.desiredStepLength = self.env.desiredStepLength + (0.2/(self.SecondPerTest/2))*self.timerOffset/900
            else:
                self.env.desiredMaximumSwingfootHeight = self.env.desiredMaximumSwingfootHeight + (0.15/(self.SecondPerTest/2))*self.timerOffset/900            
            if self.env.desiredMaximumSwingfootHeight >= self.env.desiredStepDuration/4+0.15:
                self.test5Done = True
                print("test5Done")
        
        self.Refresh()
        
        return

    def OnMouseDown(self, event):
        #print(event.GetPosition())
        self.mouseDownPos = event.GetPosition()
        self.Lclicked = True 

    def OnMouseUp(self, event):
        #print(event.GetPosition())
        self.mouseUpPos = event.GetPosition()
        self.Lclicked = False
        #self.orbit()

    def orbit(self,):
        ysub = self.mouseUpPos[1] - self.mouseDownPos[1]
        ysub = (ysub/960)*45
        self.ysubrad = np.radians(ysub)

        xsub = self.mouseUpPos[0] - self.mouseDownPos[0]
        xsub = (xsub/1200)*45
        self.xsubrad = np.radians(xsub)
       
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        if np.absolute(ysub) >= np.absolute(xsub):
            self.updateEyeY()
        else:
            self.updateEyeX()
        self.myLookAt(gEye,gAt,gUp)


    def updateEyeY(self):
        global gEye, gAt, gUp
        wp = gEye - gAt
        w = wp/np.sqrt(np.dot(wp,wp))
        upt = np.cross(gUp,w)
        u = upt/np.sqrt(np.dot(upt,upt))
        v = np.cross(w,u)

        #print(u)
        #print(self.ysubrad)
    
        quat = quaternion.quaternion(np.cos(self.ysubrad),np.sin(self.ysubrad)*u[0],np.sin(self.ysubrad)*u[1],np.sin(self.ysubrad)*u[2])
        qrm = quaternion.as_rotation_matrix(quat)

        gEye = qrm @ wp + gAt
        #gEye = qrm @ gEye

    def updateEyeX(self):
        global gEye, gAt, gUp
        wp = gEye - gAt
        w = wp/np.sqrt(np.dot(wp,wp))
        upt = np.cross(gUp,w)
        u = upt/np.sqrt(np.dot(upt,upt))
        v = np.cross(w,u)

        #print("vvvvv",v)
        #print("xsubrad",self.xsubrad)
    
        quat = quaternion.quaternion(np.cos(self.xsubrad),np.sin(self.xsubrad)*gUp[0],np.sin(self.xsubrad)*gUp[1],np.sin(self.xsubrad)*gUp[2])
        qrm = quaternion.as_rotation_matrix(quat)

        gEye = qrm @ wp + gAt
        #gEye = qrm @ gEye
 


    def myLookAtRot(self,eye, at, up, Axis):     
        global qrm
        
        wp = eye - at
        w = wp/np.sqrt(np.dot(wp,wp))
        upt = np.cross(up,w)
        u = upt/np.sqrt(np.dot(upt,upt))
        v = np.cross(w,u)

        if Axis == 0:
            quat = quaternion.quaternion(np.cos(self.ysubrad),np.sin(self.ysubrad)*u[0],np.sin(self.ysubrad)*u[1],np.sin(self.ysubrad)*u[2])
            qrm = quaternion.as_rotation_matrix(quat) @ qrm
            rot = np.identity(4)
            rot[0,:3] = qrm[0]
            rot[1,:3] = qrm[1]
            rot[2,:3] = qrm[2]
        else:
            quat = quaternion.quaternion(np.cos(self.xsubrad),np.sin(self.xsubrad)*v[0],np.sin(self.xsubrad)*v[1],np.sin(self.xsubrad)*v[2])
            qrm = quaternion.as_rotation_matrix(quat) @ qrm
            rot = np.identity(4)
            rot[0,:3] = qrm[0]
            rot[1,:3] = qrm[1]
            rot[2,:3] = qrm[2]
 
        arr = np.identity(4)
        arr[0,:3] = u
        arr[1,:3] = v
        arr[2,:3] = w
        arr[0,3] = -np.dot(u,eye)
        arr[1,3] = -np.dot(v,eye)
        arr[2,3] = -np.dot(w,eye)
        #glMultMatrixf(np.transpose(arr))
                                                                    
        glMultMatrixf(np.transpose(arr))
        glMultMatrixf(rot)

    def myLookAt(self,eye, at, up):     
        wp = eye - at
        w = wp/np.sqrt(np.dot(wp,wp))
        upt = np.cross(up,w)
        u = upt/np.sqrt(np.dot(upt,upt))
        v = np.cross(w,u)

        arr = np.identity(4)
        arr[0,:3] = u
        arr[1,:3] = v
        arr[2,:3] = w
        arr[0,3] = -np.dot(u,eye)
        arr[1,3] = -np.dot(v,eye)
        arr[2,3] = -np.dot(w,eye)
                                                                   
        glMultMatrixf(np.transpose(arr))

    def OnRmouseDown(self,event):
        self.Rclicked = True
        self.rMouseDownPos = event.GetPosition()

    def OnRmouseUp(self,event):
        self.Rclicked = False
        self.rMouseUpPos = event.GetPosition()
        #self.pan()

    def pan(self,):
        self.rYsub = self.rMouseUpPos[1] - self.rMouseDownPos[1]
        self.rXsub = self.rMouseUpPos[0] - self.rMouseDownPos[0]
      
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.updateEye_At()
         
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.myLookAt(gEye,gAt,gUp)

    def updateEye_At(self):
        global gEye, gAt, gUp
        wp = gEye - gAt
        w = wp/np.sqrt(np.dot(wp,wp))
        upt = np.cross(gUp,w)
        u = upt/np.sqrt(np.dot(upt,upt))
        v = np.cross(w,u)       
        
        gEye = gEye - u*self.rXsub/100 + v*self.rYsub/100
        gAt = gAt - u*self.rXsub/100 + v*self.rYsub/100

    def OnMouseWheel(self, event):
        #print(event.GetWheelRotation())
        #print(event.GetWheelDelta())
        #input("look")
        self.zoomI = event.GetWheelRotation()/2880
        self.updateEye_zoom()
 
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.myLookAt(gEye,gAt,gUp)



    def updateEye_zoom(self):
        global gEye, gAt, gUp
        vect = gAt - gEye
        #print(gEye)
        gEye = gEye + self.zoomI * vect
        #input(gEye)

    def OnKeyDown(self, event):
        code = event.GetKeyCode()
        desiredStepDuration_MIN = 0.2
        desiredStepDuration_MAX = 0.6
        if code == 65:
            #self.cameraX = (self.cameraX + 80)%90
            self.cameraX = self.cameraX - 0.01
        elif code == 316:
            #self.env.targetAngle = np.radians(np.degrees(self.env.targetAngle) + 1)
            self.env.targetFrameXAxis = self.env.rotateYAxis(np.radians(1), self.env.targetFrameXAxis)
        elif code == 314:
            self.env.targetFrameXAxis = self.env.rotateYAxis(np.radians(-1), self.env.targetFrameXAxis)
        ##Up Arrow Key
        elif code == 315:
            self.env.desiredStepDuration += 0.01
            if self.env.desiredStepDuration > 0.5:
                self.env.desiredStepDuration = 0.5
            stepLengthMin = self.env.desiredStepDuration - self.env.desiredStepDuration/3.0
            self.env.desiredStepLength = np.clip(self.env.desiredStepLength,stepLengthMin,stepLengthMin+0.2)
            swingfootHeightMin = self.env.desiredStepDuration/4.0
            self.env.desiredMaximumSwingfootHeight = np.clip(self.env.desiredMaximumSwingfootHeight,swingfootHeightMin, swingfootHeightMin+0.15)
            """
            swingfootHeightMax = 1.0 - self.env.desiredStepDuration/2.0
            swingfootHeightGap = (1.0 + (self.env.desiredStepDuration-0.2)*10/6)/10
            self.env.desiredMaximumSwingfootHeight = np.clip(self.env.desiredMaximumSwingfootHeight,-swingfootHeightMax+swingfootHeightGap, -swingfootHeightMax)
            """
        ##Down Arrow Key
        elif code == 317:
            self.env.desiredStepDuration -= 0.01
            if self.env.desiredStepDuration < 0.1:
                self.env.desiredStepDuration = 0.1
            ##cal StepLength
            stepLengthMin = self.env.desiredStepDuration - self.env.desiredStepDuration/3.0
            self.env.desiredStepLength = np.clip(self.env.desiredStepLength,stepLengthMin,stepLengthMin+0.2)
            swingfootHeightMin = self.env.desiredStepDuration/4.0
            self.env.desiredMaximumSwingfootHeight = np.clip(self.env.desiredMaximumSwingfootHeight,swingfootHeightMin, swingfootHeightMin+0.15)
            """
            ##cal footHeight
            swingfootHeightMax = 1.0 - self.env.desiredStepDuration/2.0
            swingfootHeightGap = (1.0 + (self.env.desiredStepDuration-0.2)*10/6)/10
            self.env.desiredMaximumSwingfootHeight = np.clip(self.env.desiredMaximumSwingfootHeight,-swingfootHeightMax+swingfootHeightGap, -swingfootHeightMax)
            """
        ##Q qey(reduce the Desired StepLength)
        elif code == 81:
            self.env.desiredStepLength -= 0.01
            if self.env.desiredStepLength < (0.2)/3:
                self.env.desiredStepLength= (0.2)/3
            stepLengthMin = self.env.desiredStepDuration - self.env.desiredStepDuration/3.0
            if self.env.desiredStepLength < stepLengthMin:
                self.env.desiredStepDuration = (3/2)*self.env.desiredStepLength 
                swingfootHeightMin = self.env.desiredStepDuration/4.0
                self.env.desiredMaximumSwingfootHeight = np.clip(self.env.desiredMaximumSwingfootHeight,swingfootHeightMin, swingfootHeightMin+0.15)

        ##W key(increase the Deisried StepLength)
        elif code == 87:
            self.env.desiredStepLength += 0.01
            if self.env.desiredStepLength > 0.7 - 0.5/3:
                self.env.desiredStepLength = 0.7 - 0.5/3
            stepLengthMax = (2/3)*self.env.desiredStepDuration + 0.2
            if self.env.desiredStepLength > stepLengthMax:
                self.env.desiredStepDuration = (3/2)*(self.env.desiredStepLength - 0.2)
                swingfootHeightMin = self.env.desiredStepDuration/4.0
                self.env.desiredMaximumSwingfootHeight = np.clip(self.env.desiredMaximumSwingfootHeight,swingfootHeightMin, swingfootHeightMin+0.15)
        ##Z key(decrease the MaxmimumSwingfootHeight)
        elif code == 90:
            self.env.desiredMaximumSwingfootHeight -= 0.01
            if self.env.desiredMaximumSwingfootHeight < 0.025:
                self.env.desiredMaximumSwingfootHeight = 0.025
            swingfootHeightMin = self.env.desiredStepDuration/4.0
            if self.env.desiredMaximumSwingfootHeight < swingfootHeightMin:
                self.env.desiredStepDuration = 4.0*self.env.desiredMaximumSwingfootHeight
                stepLengthMin = self.env.desiredStepDuration - self.env.desiredStepDuration/3.0
                self.env.desiredStepLength = np.clip(self.env.desiredStepLength,stepLengthMin,stepLengthMin+0.2)

        ##X Key(increase the MaximumSwingfootHeight)
        elif code == 88:
            self.env.desiredMaximumSwingfootHeight += 0.01
            if self.env.desiredMaximumSwingfootHeight > 0.5/4 + 0.15:
                self.env.desiredMaximumSwingfootHeight = 0.5/4 + 0.15
            swingfootHeightMax = 0.15 + self.env.desiredStepDuration/4.0
            if self.env.desiredMaximumSwingfootHeight > swingfootHeightMax:
                self.env.desiredStepDuration = 4.0*(self.env.desiredMaximumSwingfootHeight - 0.15)
                stepLengthMin = self.env.desiredStepDuration - self.env.desiredStepDuration/3.0
                self.env.desiredStepLength = np.clip(self.env.desiredStepLength,stepLengthMin,stepLengthMin+0.2)

        #elif code == 315:
        #    self.env.targetForwardFrameXAxis = self.env.rotateYAxis(np.radians(1), self.env.targetForwardFrameXAxis)
        #elif code == 317:
        #    self.env.targetForwardFrameXAxis = self.env.rotateYAxis(np.radians(-1), self.env.targetForwardFrameXAxis)

        else:
            print(code)

        #glLoadIdentity()
        #glMatrixMode(GL_MODELVIEW)
        #glLoadIdentity()
        #gluLookAt(np.radians(self.cameraX),np.radians(self.cameraY),np.radians(self.cameraZ),0,0,0,0,1,0)
        #gluLookAt(3,3,3,self.cameraX,self.cameraY,self.cameraZ,0,1,0)

    def mouseMotion(self, event):
        #print(event.GetPosition())
        if self.Lclicked == True:
            self.mouseUpPos = event.GetPosition()
            self.orbit()
            self.mouseDownPos = self.mouseUpPos
        elif self.Rclicked == True:
            self.rMouseUpPos = event.GetPosition()
            self.pan()
            self.rMouseDownPos = self.rMouseUpPos

    def drawGroundBox(self,):
        glPushMatrix()
        glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glTranslatef(0.0,-1,0.0)
        glScalef(boxSize,boxSize,boxSize)
        glTranslatef(0.,-1.,0.)
        glEnable(GL_TEXTURE_2D)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
        glBegin(GL_QUADS)

        glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0,  1.0);
        glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0,  1.0);
        # Back Face
        glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0, -1.0);
        glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0, -1.0);
        # Top Face
        glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
        glTexCoord2f(0.0, 0.0); glVertex3f(-1.0,  1.0,  1.0);
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,  1.0,  1.0);
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
        # Bottom Face
        glTexCoord2f(1.0, 1.0); glVertex3f(-1.0, -1.0, -1.0);
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0, -1.0, -1.0);
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
        glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
        # Right face
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0, -1.0);
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0,  1.0);
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
        # Left Face
        glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0, -1.0);
        glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
        glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0,  1.0);
        glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
        glEnd()
        glDisable(GL_TEXTURE_2D)
        #glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glPopMatrix()

    def EulerY(self, angle):
        mat = np.array([[np.cos(angle), 0, np.sin(angle),0],
                       [0,1,0,0],
                       [-np.sin(angle), 0, np.cos(angle),0],
                       [0,0,0,1]])
        return mat

    def CheckerBoardTextureData(self):
        global imageData
        for row in range(boxSize*8):
            for col in range(boxSize*8):
                value = (((row&0x8) is 0) ^ ((col & 0x8) is 0)) * 255
                if value < 255:
                    imageData[row][col][0] = 255
                    imageData[row][col][1] = 218
                    imageData[row][col][2] = 185
                else:
                    imageData[row][col][0] = value
                    imageData[row][col][1] = value
                    imageData[row][col][2] = value
    def skew(self,x):
        return np.array([[0, -x[2], x[1]],
                        [x[2], 0, -x[0]],
                        [-x[1], x[0], 0]])
 

class wxPythonThread(threading.Thread):
    def __init__(self,app,frame):
        self.app = app
        self.frame = frame
        threading.Thread.__init__(self)
        #plt.show()
    def run(self):
        self.app.MainLoop()
    def stop(self):
        self.frame.Close()



skel_path = "/home/qfei/dart/data/sdf/atlas/"

if __name__ == '__main__':
    pydart.init()

    world = pydart.World(1/1000)


    ground = world.add_skeleton(skel_path+"ground.urdf")
    atlas = world.add_skeleton(skel_path+"atlas_v3_no_head_soft_feet.sdf");

    skel = world.skeletons[1]

    q = skel.q
    q[0] = -0.5*np.pi
    q[4] = q[4]+0.01
    #print(len(q))
    #input(len(q))
    skel.set_positions(q)

    controller = SC.Controller(skel,world)
    controller.update(None)
    world.step()

    app = wx.App(0)

    frame = wx.Frame(None, -1, size=(1200,960))
    #gui = dartGui(frame,world,controller)
    self.guiPanel = wx.Panel(frame)
    gui = dartGui(frame,world,controller)
    frame.Show(False)
    guiThread = wxPythonThread(app,frame)
    guiThread.start()
    #app.MainLoop()
    while(1):
        controller.update(None)
        world.step()
        time.sleep(0.001)
        #print("aaa")

    for i in range(len(q)):
        q[i] = q[i]+0.5
        input()
        skel.set_positions(q)
 
    print("end")


