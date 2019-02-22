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

from . import env_base

skel_path="/home/qfei/dart/data/sdf/atlas/"
STEP_PER_SEC = 900

class FooEnv(env_base.FooEnvBase):
    def init_sim(self):
        super().init_sim()
        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(10,))


    def step(self, action):
        pos_before = self.sim.skeletons[1].com()
        panelty = 0
        check = 0
        action = np.clip(action, -1, 1)
        action[1] = ((action[1] - 1)/2)*np.pi/3
        action[3] = ((action[3]-1)/2)*np.pi/9
        action[6] = ((action[6]-1)/2)*np.pi/9
        action[8] = ((action[8]-1)/2)*np.pi/9
        action[0] = ((action[0] + 1)/2)*np.pi/3
        action[5] = ((action[5] - 1)/2)*np.pi/6
        self.do_simulation(action,self.frameskip)
       
        
        ##다리사이 각 계산 ZY평면
        r_foot_pos = self._getJointPosition(self.r_foot) 
        l_foot_pos = self._getJointPosition(self.l_foot)
        """
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
        """

        ###
        pos_after = self.sim.skeletons[1].com() 
        velocityReward = np.abs(self.veloQueue.mean() - self.desiredSpeed)*100
        ###foot balance
        foot_balance = (r_foot_pos[0] + l_foot_pos[0])/2 - pos_after[0] if (r_foot_pos[0] + l_foot_pos[0])/2 - pos_after[0] > 0 else 0 
       

        ##직선 보행 panelty
        y_lane = np.abs(pos_after[2])
        alive_bonus = 11
        reward = alive_bonus - velocityReward*0.7 -y_lane*0.3 - (np.abs(self.skel.q[0] + np.pi*0.5) + np.abs(self.skel.q[1]) + np.abs(self.skel.q[2]))*0.5 - foot_balance
        #reward = alive_bonus - speed[0]*0.7 -np.abs(pos_after[2])*0.1 - (anglesPanelty + np.abs(self.skel.q[0] + np.pi*0.5) + np.abs(self.skel.q[1]) + np.abs(self.skel.q[2]))*0.5 
        #print(reward)
        #reward = alive_bonus - panelty * 0.2 - anglesPanelty
        #print("reward",reward, pos_after[0], pos_before[0])
        #print(self.skel.q[0], self.skel.q[1], self.skel.q[2])
        #print(self.pelvis.relative_transform())
        #print(self.pelvis.transform())
        #print(self.pelvis.world_transform())

        if pos_after[1] < 0.025 or pos_after[1] > 0.5:
            done = True
        elif r_foot_pos[1] > pos_after[1]:
            done = True
        elif l_foot_pos[1] > pos_after[1]:
            done = True
        elif self.step_counter > STEP_PER_SEC * 200:
            #print("step_counter over")
            done = True
            #reward = reward + 1000 + 0.3*self.step_counter
        elif pos_after[0] < -2:
            print("back ward")
            done = True
        else:
            done = False
        self.step_counter += self.frameskip
        thisState = self.get_state()
        thispos = pos_after[0]
       
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
            self.controller.update(action,0)
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
 


