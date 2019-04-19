import sys
import pydart2 as pydart
import numpy as np
import cMat
import SimbiconController_3d as SC
import math
import queue
#import Cgui

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
class FooEnv7(env_base.FooEnvBase):
    def init_sim(self,cDirection,render):
        super().init_sim(cDirection,render)
        observation_spaces = np.concatenate([self.sim.skeletons[1].q[1:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[1:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[int(self.controller.mCurrentStateMachine.mCurrentState.mName),0,0]])
        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(15,))
        observation_spaces = np.zeros(len(observation_spaces))
        self.observation_space =spaces.Box(observation_spaces, -observation_spaces)
        print(self.targetAngle)

    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[1:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[1:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[int(self.controller.mCurrentStateMachine.mCurrentState.mName),self.desiredSpeed,self.leftAngle]])
 

    def step(self, action):
        pos_before = self.sim.skeletons[1].com()
        panelty = 0
        check = 0
        action = self.clip_Scaling_Actiond10(action)

        self.do_simulation(action,self.frameskip)
        
        #발의 위치
        r_foot_pos = self._getJointPosition(self.r_foot) 
        l_foot_pos = self._getJointPosition(self.l_foot)

        pos_after = self.sim.skeletons[1].com()
        self.XveloQueue.enqueue(pos_after[0])
        self.ZveloQueue.enqueue(pos_after[2])
        
        #2초간의 속도 계산
        #velocity_2s = np.sqrt(np.square(self.XveloQueue.first_end_distance())+np.square(self.ZveloQueue.first_end_distance()))
        velocity_2s = np.sqrt(self.XveloQueue.first_end_distance_square() + self.ZveloQueue.first_end_distance_square())/self.XveloQueue.returnSecond(30)
        velocityReward = np.abs(velocity_2s - self.desiredSpeed)
        
        alive_bonus = 5


        #방향 맞춤
        self.currentFrameXAxis = self.getCOMFrameXAxis()
        self.leftAngle = self._calAngleBetweenVectors(self.currentFrameXAxis, self.targetFrameXAxis)
        if np.cross(self.currentFrameXAxis, self.targetFrameXAxis)[1] < 0:
            self.leftAngle = -self.leftAngle


        #walkPenalty(직선보행 페널티)
        if self.XveloQueue.f_e_d() == 0 and self.ZveloQueue.f_e_d() == 0:
            a = [1,0,0]
        else:
            a = [self.XveloQueue.f_e_d(), 0, self.ZveloQueue.f_e_d()]
        a = cMat.Matrix.normalize(a)
        walkPenalty = self._calAngleBetweenVectors(self.currentFrameXAxis, a)

        reward = alive_bonus - 1.5*velocityReward - 1.5*self.leftAngle - 0.5*(walkPenalty)




        #reward = alive_bonus - velocityReward*0.9 -y_lane*0.2 - (np.abs(self.skel.q[0] + np.pi*0.5) + np.abs(self.skel.q[1]) + np.abs(self.skel.q[2]))*0.2 - foot_balance
        #reward = alive_bonus - speed[0]*0.7 -np.abs(pos_after[2])*0.1 - (anglesPanelty + np.abs(self.skel.q[0] + np.pi*0.5) + np.abs(self.skel.q[1]) + np.abs(self.skel.q[2]))*0.5 
        #reward = alive_bonus - panelty * 0.2 - anglesPanelty

        if pos_after[1] < 0.025 or pos_after[1] > 0.5:
            done = True
        #정면으로 걷지않을경우 빠르게 종료
        #elif np.abs(pos_after[2]) > 2:
        #    done = True
        elif r_foot_pos[1] > pos_after[1]:
            done = True
        elif l_foot_pos[1] > pos_after[1]:
            done = True
        elif self.step_counter > self.step_per_sec * 60:
            #print("step_counter over")
            done = True
            #reward = reward + 1000 + 0.3*self.step_counter
        elif pos_after[0] < -1:
            print("back ward")
            done = True
        else:
            done = False
        self.step_counter += self.frameskip
        thisState = self.get_state()
        thispos = pos_after[0]
       
        self.actionSteps += 1
        self.episodeTotalReward += reward
        self.set_desiredSpeed()


        #수정
        if self.step_counter % (self.step_per_sec * 20) == self.step_per_sec*5 and self.cDirection and self.step_counter is not 0:
            self.changeDirection()
        #if self.step_counter == self.step_per_sec * 30 and self.cDirection:
        #    self.changeDirection()


        if done is True:
            print("episodeDone... mean Reward: " + str(self.episodeTotalReward/self.actionSteps))
            print("velocityReward: " + str(velocityReward) + "__" + str(velocity_2s)+ "__" + str(self.desiredSpeed))
            #self.reset()
        return thisState, reward, done, pos_after

        #self.do_simulation(action, 60)



    def do_simulation(self, action, n_frames):
        self.controller.mCurrentStateMachine.mCurrentAction = action
        self.controller.mCurrentStateMachine.setTrainedDesiredAction(action, 0)
        for _ in range(n_frames):
            self.controller.update()
            self.sim.step()
            if(self.isrender):
                time.sleep(0.001)
        return

