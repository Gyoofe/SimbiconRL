import wx
import sys
import pydart2 as pydart
import numpy as np
import cMat
import SimbiconController as SC
import math
import queue
#import Cgui

from guiModule import ModuleTest_drawMesh_new
from wx import glcanvas

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import threading
import time

import matplotlib
import matplotlib.pyplot as plt

skel_path="/home/qfei/dart/data/sdf/atlas/"
STEP_PER_SEC = 900

class FooEnvBase(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        """
        pydart.init()

        self.world = pydart.World(1/1000)
        self.ground = self.world.add_skeleton(skel_path+"ground.urdf")
        self.atlas = self.world.add_skeleton(skel_path+"atlas_v3_no_head_soft_feet.sdf")

        self.skel = self.world.skeletons[1]

        q = self.skel.q
        q[0] = -0.5*np.pi
        q[4] = q[4]+0.01
        self.skel.set_positions(q)

        self.controller = SC.Controller(self.skel, self.world)

        self.app = wx.App(0)

        self.frame= wx.Frame(None, -1, size=(1200,960))
        self.gui = Cgui.dartGui(self.frame,self.world,self.controller)
        self.frame.Show(True)

        self.app.MainLoop()

        return
        """
        """
        pydart.init()

        self.frameskip = None
        self.sim = pydart.World(1/1000)
        self.ground = self.sim.add_skeleton(skel_path+"ground.urdf")
        self.model = self.sim.add_skeleton(skel_path+"atlas_v3_no_head_soft_feet.sdf")
        
        self.data = None
        self.viewer = None

        self.skel = self.sim.skeletons[1]
        q = self.skel.q
        q[0] = -0.5*np.pi
        q[4] = q[4]+0.01
        #q[3] = q[3]+1
        #x방향으로 진행
        self.initPos = q
        self.skel.set_positions(q)


        self.controller = SC.Controller(self.skel, self.sim)

        app = wx.App(0)
        frame = wx.Frame(None, -1, size=(1200,960))
        self.gui = Cgui.dartGui(frame,self.sim,self.controller)
        frame.Show(True)


        self.guiThread = wxPythonThread(app,frame)

        ##gym Setting##
        lowhigh = np.array([-300,300])
        self.action_space = spaces.Box(low = -1, high = 0, shape=(8,))


        observation_spaces = np.concatenate([self.sim.skeletons[1].q[:4],self.sim.skeletons[1].q[6:], self.sim.skeletons[1].dq, self.sim.skeletons[1].com_velocity(), self.sim.skeletons[1].com_acceleration()])
        observation_spaces = np.zeros(len(observation_spaces))
        observation_spaces[0:]=500
        self.observation_space = spaces.Box(observation_spaces, -observation_spaces)
        print(observation_spaces)
        #print(self.sim.skeletons[1].com())

        plt.ion()
        #input("init")
        self.plt_xAxis = np.array([0])
        self.plt_yAxis = np.array([0])
        self.draw_plot(self.plt_xAxis,self.plt_yAxis)
        #plt.plot(self.plt_xAxis,self.plt_yAxis)
        #plt.show()

        self.step_counter = 0
        """
        #input("init")

    def init_dart(self):
        pydart.init()

    def init_sim(self):
        self.querystep = 1
        self.frameskip = 30
        self.sim = pydart.World(1/STEP_PER_SEC)
        self.ground = self.sim.add_skeleton(skel_path+"ground.urdf")
        self.model = self.sim.add_skeleton(skel_path+"atlas_v3_no_head_soft_feet.sdf")
        
        self.data = None
        self.viewer = None

        self.skel = self.sim.skeletons[1]
        q = self.skel.q
        q[0] = -0.5*np.pi
        q[4] = q[4]+0.001
        #q[3] = q[3]+1
        #x방향으로 진행
        self.initPos = q
        self.skel.set_positions(q)


        self.controller = SC.Controller(self.skel, self.sim)

        app = wx.App(0)
        frame = wx.Frame(None, -1, size=(1200,960))
        #self.gui = Cgui.dartGui(frame,self.sim,self.controller)
        self.gui = ModuleTest_drawMesh_new.dartGui(frame,self.sim,self.controller)
        frame.Show(True)


        self.guiThread = wxPythonThread(app,frame)

        ##gym Setting##
        lowhigh = np.array([-300,300])
        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(5,))


        observation_spaces = np.concatenate([self.sim.skeletons[1].q[:4],self.sim.skeletons[1].q[6:], self.sim.skeletons[1].dq, self.sim.skeletons[1].com_velocity()[:1], self.sim.skeletons[1].com_acceleration()[:1],[int(self.controller.mCurrentStateMachine.mCurrentState.mName)]]) 
        observation_spaces = np.zeros(len(observation_spaces))
        #observation_spaces[0:]=500
        self.observation_space = spaces.Box(observation_spaces, -observation_spaces)
        #print(observation_spaces)
        #input()
        #print(self.sim.skeletons[1].com())

        #plt.ion()
        #input("init")
        #self.plt_xAxis = np.array([0])
        #self.plt_yAxis = np.array([0])
        #self.draw_plot(self.plt_xAxis,self.plt_yAxis)
        #plt.plot(self.plt_xAxis,self.plt_yAxis)
        #plt.show()

        self.step_counter = 0

        self.r_foot = self.skel.body("r_foot")
        self.l_foot = self.skel.body("l_foot")
        self.l_uleg = self.skel.body("l_uleg")
        self.r_uleg = self.skel.body("r_uleg")
        self.l_lleg = self.skel.body("l_lleg")
        self.r_lleg = self.skel.body("r_lleg")
        self.l_leg_hpy = self.skel.joint("l_leg_hpy")
        self.r_leg_hpy = self.skel.joint("r_leg_hpy")
        self.pelvisX = self.skel.joint("back_bkx")
        self.pelvisY = self.skel.joint("back_bky")
        self.pelvisZ = self.skel.joint("back_bkz")
        self.pelvis = self.skel.body("pelvis")

        self.veloQueue = CircularQueue(STEP_PER_SEC*2)
        self.actionSteps = 0
        self.episodeTotalReward = 0
        self.isrender = False
        self.desiredAction = 0.08
    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[:4],self.sim.skeletons[1].q[6:],self.sim.skeletons[1].dq, self.sim.skeletons[1].com_velocity()[:1], self.sim.skeletons[1].com_acceleration()[:1],[int(self.controller.mCurrentStateMachine.mCurrentState.mName)]])
         
    def set_env_name(self, name_v):
        self.name = name_v

    def reset_model(self):
        """reset model position
        """
        #print("reset_Model")
        #self.controller.__init__(self.skel, self.sim)
        self.skel.set_positions(self.initPos)
        self.skel.dq = np.zeros(self.skel.num_dofs())
        self.controller.mCurrentStateMachine.mCurrentState = self.controller.mCurrentStateMachine.mStates[1]
        #print(self.initPos)
        #print(self.controller.mCurrentStateMachine.mCurrentState.mTorque)
        #input("hh")
        #self.skel.dq[0:self.skel.num_dofs()] = 0

    def step(self):
        print("need implementation")

    def do_simulation(self):
        print("need implementation")

    def reset(self):
        self.step_counter = 0
        self.sim.reset()
        self.reset_model()
        self.veloQueue.reset()
        self.actionSteps = 0
        self.episodeTotalReward = 0 
        return self.get_state()

    def start_render(self, mode='human', close=False):
        self.isrender = True
        self.guiThread.start()
        return

    def stop_render(self):
        self.guiThread.stop()

    def render(self):
        #self.controller.update()
        #self.gui.Refresh()
        #time.sleep(1)
        return
    def get_viewer(self):
        return

    def draw_plot(self,x,y):
        #print(self.plt_xAxis, ",", self.plt_yAxis,x,y)
        #input("draw")
        self.plt_xAxis = np.append(self.plt_xAxis,x)
        self.plt_yAxis =np.append(self.plt_yAxis,y)
        #print(self.plt_xAxis.size, self.plt_yAxis.size)
        #input("next")
        plt.clf()
        plt.plot(self.plt_xAxis,self.plt_yAxis)
        plt.show()
        plt.pause(0.001)
        #time.sleep(1)
    def _getJointPosition(self,node):
        parentJoint = node.parent_joint
        localJointPosition = parentJoint.transform_from_child_body_node()
        localJointPosition = cMat.Matrix.getTranslation(localJointPosition)

        localJointPosition = np.append(localJointPosition,1.0)

        npdot = np.dot(node.T, localJointPosition)
        return npdot[0:3]

    def _calAngleBetweenVectors(self, v1, v2):
        theta = np.dot(v1,v2)/ (cMat.Matrix.size(v1) * cMat.Matrix.size(v2))
        theta = np.clip(theta, -1, 1)

        return math.acos(theta)


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

class CircularQueue():
    def __init__(self, maxsize):
        self.maxsize = maxsize
        self.count = 0
        self.array = np.zeros(maxsize)
        self.start = 0
        self.end = 0

    def isfull(self):
        if self.count == self.maxsize:
            return True
        else:
            return False

    def dequeue(self):
        if self.count is 0:
            return

        self.count = self.count - 1
        self.array[self.start] = 0
        self.start = (self.start + 1)%60

    def enqueue(self, item):
        if self.isfull() is True:
            self.dequeue()
        self.array[self.end] = item
        self.end = (self.end+1)%60
        self.count = self.count + 1

    def mean(self):
        if self.count is 0:
            return 0
        sum = 0
        for i in self.array:
            sum = sum + i

        return sum/self.count

    def reset(self):
        self.count = 0
        self.array = np.zeros(self.maxsize)
        self.start = 0
        self.end = 0
        
    def returnarray(self):
        return self.array
