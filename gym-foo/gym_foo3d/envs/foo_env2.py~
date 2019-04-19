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

from . import env_base

import threading
import time

import matplotlib
import matplotlib.pyplot as plt

skel_path="/home/qfei/dart/data/sdf/atlas/"

class FooEnv2(env_base.FooEnvBase):

    def init_sim(self):
        super().init_sim()
        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(7,))


    def step(self, action):
        #print(action)
        pos_before = self.sim.skeletons[1].com()
        #self.sim.skeletons[1].
        panelty = 0
        check = 0
        #print(action)
        #input()
        action = self.clip_Scaling_Actiond5(action)

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
        self.XveloQueue.enqueue(pos_after[0])
        self.ZveloQueue.enqueue(pos_after[2])
        velocity_2s = np.sqrt(self.XveloQueue.first_end_distance_square() + self.ZveloQueue.first_end_distance_square())/self.XveloQueue.returnSecond(30)
        velocityReward = np.abs(velocity_2s - self.desiredSpeed)
        #print("self.desiredSpeed: ",self.desiredSpeed, "velo_mean: ", self.veloQueue.mean())
        #print("velocityReward", velocityReward)

        #print((r_foot_pos + l_foot_pos)/2)
        #print(pos_after)
        #foot_balance = (r_foot_pos[0] + l_foot_pos[0])/2 - pos_after[0] if (r_foot_pos[0] + l_foot_pos[0])/2 - pos_after[0] > 0 else 0 
        ##직선 보행 panelty
        y_lane = np.abs(pos_after[2])
        alive_bonus = 3
        reward = alive_bonus - velocityReward - y_lane * 0.3
        #reward = alive_bonus - velocityReward*0.9 -y_lane*0.2 - (np.abs(self.skel.q[0] + np.pi*0.5) + np.abs(self.skel.q[1]) + np.abs(self.skel.q[2]))*0.2 - foot_balance

        if pos_after[1] < 0.025 or pos_after[1] > 0.5:
            done = True
        elif np.abs(pos_after[2]) > 1:
            done = True
        elif r_foot_pos[1] > pos_after[1]:
            done = True
        elif l_foot_pos[1] > pos_after[1]:
            done = True
        elif self.step_counter > self.step_per_sec * 60:
            done = True
        elif pos_after[0] < -1:
            done = True
        else:
            done = False
        self.step_counter += self.frameskip
        thisState = self.get_state()
        thispos = pos_after[0]
        self.actionSteps += 1
        self.episodeTotalReward += reward
        self.set_desiredSpeed()
        #print(self.step_counter)
        if done is True:
            print("episodeDone... mean Reward: " + str(self.episodeTotalReward/self.actionSteps))
            print("velocityReward: " + str(velocityReward) + "__" + str(velocity_2s))
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
            self.controller.update(action,2)
            self.sim.step()
            if(self.isrender):
                time.sleep(0.001)
        #time.sleep(1)
        #input()
        return
        #speed = self.sim.skeletons[1].com_velocity()
        #self.veloQueue.enqueue(speed[0])
 


