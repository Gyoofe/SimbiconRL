import wx
import sys
import pydart2 as pydart
import numpy as np
import cMat
import SimbiconController_3d as SC
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
DESIRED_MAX_SPEED = 2
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
        self.sim.set_recording(False)
        self.ground = self.sim.add_skeleton(skel_path+"ground.urdf")
        self.model = self.sim.add_skeleton(skel_path+"atlas_v3_no_head.sdf")
        
        self.data = None
        self.viewer = None

        self.skel = self.sim.skeletons[1]
        q = self.skel.q
        q[0] = -0.5*np.pi
        q[4] = q[4]
        q[7] = q[7] + 0.2
        q[16] = q[16]+0.2
        #q[29] = q[29]+0.3
       # q[28] = q[28]-0.1
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
        #lowhigh = np.array([-300,300])
        #self.action_space = spaces.Box(low = 0, high = 1.5, shape=(5,))

        #마지막2개, desiredSpeed
        observation_spaces = np.concatenate([self.sim.skeletons[1].q[:4],self.sim.skeletons[1].q[6:],self.sim.skeletons[1].dq,[int(self.controller.mCurrentStateMachine.mCurrentState.mName),0]])
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

        self.XveloQueue = CircularQueue(self.frameskip*2)
        self.ZveloQueue = CircularQueue(self.frameskip*2)
        self.actionSteps = 0
        self.episodeTotalReward = 0
        self.isrender = False
        self.p_targetspeed = 0
        self.targetspeed = DESIRED_MAX_SPEED
        self.desiredSpeed = 0
        self.step_per_sec = STEP_PER_SEC

        self.mlinearActionRatio = 0

    def set_desiredSpeed(self):
        #print(actionSteps)
        maxtime = self.actionSteps/(self.targetspeed*self.frameskip*1.5)
        #print(maxtime)
        if maxtime <= 1:
            self.desiredSpeed = (self.targetspeed - self.p_targetspeed)*maxtime
        return

    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[:4],self.sim.skeletons[1].q[6:],self.sim.skeletons[1].dq, [int(self.controller.mCurrentStateMachine.mCurrentState.mName),self.desiredSpeed]])
         
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
        self.XveloQueue.reset()
        self.ZveloQueue.reset()
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
   
    def ForceAction10(self,action):
        action[0] = action[0]*(self.mlinearActionRatio) + 0.5*(1-self.mlinearActionRatio)
        action[1] = action[1]*(self.mlinearActionRatio) - 1.10*(1-self.mlinearActionRatio)
        action[2] = action[2]*(self.mlinearActionRatio) + 0.6*(1-self.mlinearActionRatio)
        action[3] = action[3]*(self.mlinearActionRatio) - 0.05*(1-self.mlinearActionRatio)
        action[4] = action[4]*(self.mlinearActionRatio)
        action[5] = action[5]*(self.mlinearActionRatio)
        action[6] = action[6]*(self.mlinearActionRatio)
        action[7] = action[7]*(self.mlinearActionRatio) - 0.1*(1-self.mlinearActionRatio)
        action[8] = action[8]*(self.mlinearActionRatio) - 0.05*(1-self.mlinearActionRatio)
        action[9] = action[9]*(self.mlinearActionRatio) + 0.15*(1-self.mlinearActionRatio)
        action[10] = action[10]*(self.mlinearActionRatio) - 0.1 *(1-self.mlinearActionRatio)
        action[11] = action[11]*(self.mlinearActionRatio)
        action[12] = action[12]*(self.mlinearActionRatio)
        action[13] = action[13]*(self.mlinearActionRatio)

    def clip_Normal_Actiond10(self, action):
        action = np.clip(action, -1, 1)
        action[0] = action[0]*np.pi/2
        action[1] = ((action[1] - 1)/2)*np.pi/2
        action[2] = (action[2])*np.pi/3
        action[3] = (action[3]-1)*np.pi/4
        action[4] = (action[4])*np.pi/9
        action[5] = (action[5])*math.radians(20.0)
        action[6] = (action[6])*math.radians(60.0)
        action[7] = action[7]*np.pi/2
        action[8] = (action[8]-1)*np.pi/4
        action[9] = (action[9])*np.pi/9
        action[10] = (action[10]-1)*np.pi/4
        action[11] = (action[11])*np.pi/9
        action[12] = (action[12])*math.radians(20.0)
        action[13] = (action[13])*math.radians(60.0)
        self.ForceAction10(action)

        return action

    def clip_Scaling_Actiond10(self, action):
        action = np.clip(action, -1, 1)
        action[0] = ((action[0] + 1)/2)*np.pi/3       
        action[1] = (((action[1] - 1)/4)-0.5)*np.pi/2
        action[2] = (((action[2] + 1)/2)*(2/3)+1/3)*np.pi/3
        action[3] = ((action[3]-1)/2)*np.pi/9
        action[4] = (action[4])*np.pi/9
        action[5] = (action[5])*math.radians(20.0) 
        action[6] = (action[6])*math.radians(45.0)
        action[7] = ((action[7] - 1)/2)*np.pi/6
        action[8] = ((action[8]-1)/2)*np.pi/9
        action[9] = (action[9])*np.pi/9
        action[10] = ((action[10]-1)/2)*np.pi/6
        action[11] = (action[11])*np.pi/9
        action[12] = (action[12])*math.radians(20.0)
        action[13] = (action[13])*math.radians(45.0)
        #print(action)
        self.ForceAction10(action)
        return action

    def clip_Scaling_Actiond5(self, action):
        action = np.clip(action, -1, 1)
        action[0] = action[0]*np.pi/3
        action[1] = ((action[1]-1)/2)*np.pi/2
        action[2] = (action[2])*np.pi/4
        action[3] = ((action[3]-1)/2)*np.pi/9
        action[4] = (action[4])*np.pi/9
        action[5] = (action[5])*math.radians(20.0) 
        action[6] = (action[6])*math.radians(60.0)
        return action


    def clip_Normal_Actiond5(self, action):
        action = np.clip(action, -1, 1)
        action[0] = action[0]*np.pi/2
        action[1] = (action[1]-1)*np.pi/2
        action[2] = (action[2])*np.pi/2
        action[3] = (action[3]-1)*np.pi/4
        action[4] = (action[4])*np.pi/9
        action[5] = (action[5])*math.radians(20.0)
        action[6] = (action[6])*math.radians(60.0)
        return action

    def set_linearActionRatio(self, ratio):
        self.mlinearActionRatio = ratio
        print("current llinearActionRatio: ", self.mlinearActionRatio)
  
    
    def getCOMFrameXAxis(self):
        yAxis = cMat.Matrix.UnitY()

        pelvisAxis = self.sim.skeletons[1].body("pelvis").T
        pelvisXAxis = cMat.Matrix.linear(pelvisAxis)
        pelvisXAxis = cMat.Matrix.col(pelvisXAxis,0)

        mag = np.dot(yAxis, pelvisXAxis)
        #pelvisXAxis = copy.deepcopy(pelvisXAxis - mag*yAxis)
        pelvisXAxis = pelvisXAxis - mag*yAxis

        xAxis = cMat.Matrix.normalize_2D(pelvisXAxis)
        return xAxis

class FooEnvBase_rs(FooEnvBase):
    def init_sim(self):
        super().init_sim()
        observation_spaces = np.concatenate([self.sim.skeletons[1].q[:4],self.sim.skeletons[1].q[6:], self.sim.skeletons[1].dq, self.sim.skeletons[1].com_velocity()[:1], self.sim.skeletons[1].com_acceleration()[:1],[int(self.controller.mCurrentStateMachine.mCurrentState.mName)],[self.desiredSpeed]]) 
        observation_spaces = np.zeros(len(observation_spaces))
        #observation_spaces[0:]=500
        self.observation_space = spaces.Box(observation_spaces, -observation_spaces)
        self.minSpeed = 0.01
        self.maxSpeed = 0.2
        self.speedChangeCounter = 0
    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[:4],self.sim.skeletons[1].q[6:],self.sim.skeletons[1].dq, self.sim.skeletons[1].com_velocity()[:1], self.sim.skeletons[1].com_acceleration()[:1],[int(self.controller.mCurrentStateMachine.mCurrentState.mName)],[self.desiredSpeed]])
    
    def select_TargetSpeed(self):
        self.p_targetspeed = self.targetspeed
        self.targetspeed = np.random.uniform(self.minSpeed, self.maxSpeed)

    def changeSpeed(self,spc,sec):
        if self.speedChangeCounter // (spc*sec) > 0:
            self.select_desiredSpeed()
            self.speedChangeCounter = 0
            print("change speed to" + str(self.desiredSpeed))
    def reset(self):
        super().reset()
        self.select_TargetSpeed()
        return self.get_state()

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
        self.start = (self.start + 1)%self.maxsize

    def enqueue(self, item):
        if self.isfull() is True:
            self.dequeue()

        self.array[self.end] = item
        self.end = (self.end+1)%self.maxsize
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

    def first_end_distance_square(self):
        #30hz(get action)
        #seconds = self.count/30
        #print(seconds)
        #print(self.array[self.start],self.array[self.end-1])
        return np.square(self.array[self.start] - self.array[self.end-1])
    def returnSecond(self,freq):
        return self.count/freq

    def returnarray(self):
        return self.array


if __name__ == "__main__":
    a = CircularQueue(6)
    a.enqueue(1)
    a.enqueue(2)
    a.enqueue(3)
    a.enqueue(4)
    a.enqueue(5)
    a.enqueue(6)
    print(str(a.mid_first_end()))
    print(a.isfull())
    a.enqueue(7)
    a.enqueue(8)
    print(a.mid_first_end())
    a.enqueue(9)
    a.enqueue(10)
    print(a.mid_first_end())
