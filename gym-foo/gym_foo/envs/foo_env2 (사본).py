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

class FooEnv2(gym.Env):
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
        self.sim = pydart.World(1/900)
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
        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(10,))


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

        self.veloQueue = CircularQueue(1800)
        self.actionSteps = 0
        self.episodeTotalReward = 0
        self.isrender = False 
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
    

    def step(self, action):
        #print(action)
        pos_before = self.sim.skeletons[1].com()
        #self.sim.skeletons[1].
        panelty = 0
        check = 0
        #print(action)
        #input()
        action = np.clip(action, -1, 1)
        action[1] = ((action[1] - 1)/2)*np.pi/2
        #action[1] = -1.1
        action[3] = (action[3]-1)*np.pi/4
        action[6] = (action[6]-1)*np.pi/4
        action[8] = (action[8]-1)*np.pi/4

        action[0] = action[0]*np.pi/2
        action[5] = action[5]*np.pi/2

        #if action[0] < 1e-6:
        #    action[0] = 0.01


        #print(action)
        #input()
        #if action[1] > 0:
        #    panelty = panelty + action[1]
        #    action[1] = 0
        #    check = 1
        #if action[3] > 0:
        #    panelty = panelty + action[3]
        #    action[3] = 0
        #    check = 1
        #if action[6] > 0:
        #    panelty = panelty + action[6]
        #    action[6] = 0
        #    check = 1
        #if action[8] > 0:
        #    panelty = panelty + action[8]
        #    action[8] = 0
        #    check = 1
        #if check is 0:
            #print("pass check")

        #if action[0] > 0.8:
        #    panelty = panelty + action[0]
        #    action[0] = 0.8
        #if action[5] > 0:
        #    action[5] = 0
        #    panelty = panelty + action[5]
        #assert panelty>=0, "panelty < 0"

        self.do_simulation(action,self.frameskip)

        ##다리사이 각 계산 ZY평면
        r_foot_pos = self._getJointPosition(self.r_foot) 
        l_foot_pos = self._getJointPosition(self.l_foot)

        l_uleg_jpos = self._getJointPosition(self.l_uleg)
        r_uleg_jpos = self._getJointPosition(self.r_uleg)
        l_lleg_jpos = self._getJointPosition(self.l_lleg)
        r_lleg_jpos = self._getJointPosition(self.r_lleg)

        rLeg_vector = r_lleg_jpos - self.pelvis.world_transform()[:3,3]
        lLeg_vector = l_lleg_jpos - self.pelvis.world_transform()[:3,3]

        pelvisX_axis = self.pelvisX.axis_in_world_frame()

        rLeg_vector = np.cross(pelvisX_axis, np.cross(rLeg_vector,pelvisX_axis))
        lLeg_vector = np.cross(pelvisX_axis, np.cross(lLeg_vector, pelvisX_axis))
        #print(rLeg_vector)
        #print(lLeg_vector)
        #LegZYangle = self._calAngleBetweenVectors(rLeg_vector,lLeg_vector)

        #anglesPanelty = 0
        #if LegZYangle > 0.55:
        #    anglesPanelty = anglesPanelty + LegZYangle * 0.3


        ###
        pos_after = self.sim.skeletons[1].com() 
        velocityReward = np.abs(self.veloQueue.mean() - 0.06)*100
        #print((r_foot_pos + l_foot_pos)/2)
        #print(pos_after)
        #foot_balance = (r_foot_pos[0] + l_foot_pos[0])/2 - pos_after[0] if (r_foot_pos[0] + l_foot_pos[0])/2 - pos_after[0] > 0 else 0 
        #if foot_balance < 0:
        #    print("break")
        #    input()
        #print(self.veloQueue.mean())
        #print(self.veloQueue.returnarray())
        #input()
        #lhpy_vec = self.l_leg_hpy.axis_in_world_frame()
        #rhpy_vec = self.r_leg_hpy.axis_in_world_frame()
        #npdot = np.dot(lhpy_vec, rhpy_vec)
        #sizes = cMat.Matrix.size(lhpy_vec) * cMat.Matrix.size(rhpy_vec)

        
        #angles = math.acos( np.clip(npdot / sizes, -1, 1))
        #if angles > np.pi/4:
        #    anglesPanelty = anglesPanelty + angles*0.2
        #directs = self._getJointPosition(self.l_uleg) - self._getJointPosition(self.r_uleg) 
        #if directs[0] > 0:
            #angles = -angles
        #print(angles)
        #print(directs)
        #if angles > 0:
            #input()
        #panelty to +-
        #input()
        ##직선 보행 panelty
        #y_lane = np.abs(pos_after[2])
        #if y_lane < 0:
        #    input("errrrrr!!!" 
        #alive_bonus = (self.step_counter/1000) * (self.step_counter/1000)
        #linear_val_cost = 1000000 * (pos_after[0] - pos_before[0])
        alive_bonus = 11
        #linear_val_cost = pos_after[0] - pos_before[0]
        #reward = alive_bonus + linear_val_cost - 0.5*y_lane
        reward = alive_bonus - velocityReward*0.7 -np.abs(pos_after[2])*0.1 - (np.abs(self.skel.q[0] + np.pi*0.5) + np.abs(self.skel.q[1]) + np.abs(self.skel.q[2]))*0.5
        #reward = alive_bonus - speed[0]*0.7 -np.abs(pos_after[2])*0.1 - (anglesPanelty + np.abs(self.skel.q[0] + np.pi*0.5) + np.abs(self.skel.q[1]) + np.abs(self.skel.q[2]))*0.5 
        #print(reward)
        #reward = alive_bonus - panelty * 0.2 - anglesPanelty
        #print("reward",reward, pos_after[0], pos_before[0])
        #print(self.skel.q[0], self.skel.q[1], self.skel.q[2])
        #print(self.pelvis.relative_transform())
        #print(self.pelvis.transform())
        #print(self.pelvis.world_transform())

        if pos_after[1] < 0.025 or pos_after[1] > 0.5:
            #print("pos elimination")
            #reward = reward + 0.3*self.step_counter
            #if self.step_counter > 3000:
            #    reward = reward + 1000
            reward = 0
            done = True
        elif r_foot_pos[1] > pos_after[1]:
            reward =0
            done = True
            print(action[0])
            print(action[1])
            print(action[5])
            print(action[6])
            print("current State Current" + self.controller.mCurrentStateMachine.mCurrentState.mName)
        elif l_foot_pos[1] > pos_after[1]:
            reward = 0
            done = True
            print(action[0])
            print(action[1])
            print(action[5])
            print(action[6])
            print("current State Current" + self.controller.mCurrentStateMachine.mCurrentState.mName)
        elif self.step_counter > 10000:
            #print("step_counter over")
            done = True
            #reward = reward + 1000 + 0.3*self.step_counter
        elif pos_after[0] < -2:
            print("back ward")
            done = True
            #reward = reward - 1000
        #elif pos_after[0] < 0:
        #    done = 1
        else:
            done = False
        self.step_counter += self.frameskip
        #print(done)
        #if done is 1:
            #print(pos_after[1])
            #input("done")
        #etc...
        #compute reward

        #qpos
        #check elimination
        thisState = self.get_state()
        #print(thisState)
        #input()
        thispos = pos_after[0]

        #if done is 1:
            #print("vel",self.sim.skeletons[1].com_velocity()[:1],"__acc", self.sim.skeletons[1].com_acceleration()[:1])
        if reward < 0:
            print("minus Reward")
            #reward = 0
        
        self.actionSteps += 1
        self.episodeTotalReward += reward

        if done is True:
            print("episodeDone... mean Reward: " + str(self.episodeTotalReward/self.actionSteps))
            print("velocityReward: " + str(velocityReward) + "__" + str(self.veloQueue.mean()))
            self.reset()
        return thisState, reward, done, pos_after

        #self.do_simulation(action, 60)



    def do_simulation(self, action, n_frames):
        """set action..
            for _ in range(n_frames):
                self.simulator step
        """
        #print(action)
        #input()
        #self.controller.update(action)
        for _ in range(n_frames):
            #self.controller.update(None)
            self.controller.update(action)
            self.sim.step()
            if(self.isrender):
                time.sleep(0.001)
            speed = self.sim.skeletons[1].com_velocity()
            self.veloQueue.enqueue(speed[0])

        #time.sleep(1)
        #input()
        return
        #speed = self.sim.skeletons[1].com_velocity()
        #self.veloQueue.enqueue(speed[0])
 

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
