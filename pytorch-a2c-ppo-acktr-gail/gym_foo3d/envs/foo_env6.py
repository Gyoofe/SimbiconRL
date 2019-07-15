import sys
import pydart2 as pydart
import numpy as np
import cMat
import SimbiconController_3dContact as SC
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
class FooEnv6(env_base.FooEnvBase):
    def init_sim(self,cDirection,render):
        super().init_sim(cDirection,render)
        self.controller = SC.Controller(self.skel, self.sim)
        observation_spaces = np.concatenate([self.sim.skeletons[1].q[1:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[1:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[int(self.controller.mCurrentStateMachine.mCurrentState.mName),0,0]])
        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(16,))
        observation_spaces = np.zeros(len(observation_spaces))
        self.observation_space =spaces.Box(observation_spaces, -observation_spaces)

        #Queue 크기 설정
        #self.XveloQueue = env_base.CircularQueue(8)
        #self.ZveloQueue = env_base.CircularQueue(8)
        
        self.step_counter = 0
        self.the_last = 0
        self.a = [1,0,0]
        self.reward_counter = 0
        self.state_reward = 0
        print(self.targetAngle)

    def reset(self):
        super().reset()
        self.step_counter = 0
        self.the_last = 0
        self.a = [1,0,0]
        self.reward_counter = 0
        self.state_reward = 0

        return self.get_state()

    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[1:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[1:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[int(self.controller.mCurrentStateMachine.mCurrentState.mName),self.desiredSpeed,self.leftAngle]])
 

    def step(self, action):
        pos_before = self.sim.skeletons[1].com()
        panelty = 0
        check = 0
        action = self.clip_Scaling_Actiond10(action)

        self.previousState = self.controller.mCurrentStateMachine.mCurrentState.mName
        #done은 에피소드가 끝났는지..
        done,n_frames = self.do_simulation(action)
        
        #발의 위치
        r_foot_pos = self._getJointPosition(self.r_foot) 
        l_foot_pos = self._getJointPosition(self.l_foot)
        """
        pos_after = self.sim.skeletons[1].com()
        self.XveloQueue.enqueue(pos_after[0])
        self.ZveloQueue.enqueue(pos_after[2])
        """
        #1초간의 속도 계산

        ###수정 예정(env_base에서도 수정해야됨)###
        #velocity_s = self.distance()
        #velocityReward = np.abs(velocity_s - self.desiredSpeed)
        #print("vs",velocity_s)
        #print("dS", self.desiredSpeed)

        """
        alive_bonus = 10

        #방향 맞춤
        self.currentFrameXAxis = self.getCOMFrameXAxis()
        self.leftAngle = self._calAngleBetweenVectors(self.currentFrameXAxis, self.targetFrameXAxis)
        if np.cross(self.currentFrameXAxis, self.targetFrameXAxis)[1] < 0:
            self.leftAngle = -self.leftAngle


        #walkPenalty(직선보행 페널티)
        ###Queue 수정해야됨!!!!!!!!!!!!! ###
        if self.XveloQueue.f_e_d() == 0 and self.ZveloQueue.f_e_d() == 0:
            self.a = [1,0,0]
        else:
            self.a = [self.XveloQueue.f_e_d(), 0, self.ZveloQueue.f_e_d()]
        self.a = cMat.Matrix.normalize(self.a)
        walkPenalty = self._calAngleBetweenVectors(self.currentFrameXAxis, self.a)
        """
        #print(self.skel.tau)
        #tausums = 0
        #for i in self.skel.tau:
        #    tausums += np.abs(i)

        #print(tausums)
        #input()
 

        #reward 수정 예정,, torque 총합 페널티..
        #reward = alive_bonus - np.exp(2*(np.abs(self.leftAngle)) + 1.5*walkPenalty + 2*velocityReward)

        ##초반 walkpenalty 상쇄?
        #reward = alive_bonus - walkPenalty - np.abs(self.leftAngle) - self.tausums/1000
        #reward = alive_bonus - walkPenalty - np.abs(self.leftAngle)

        #reward = self.state_reward/(self.reward_counter)
        reward = self.state_reward

        self.step_counter += n_frames
        thisState = self.get_state()
       
        self.actionSteps += 1
        self.episodeTotalReward += reward
        self.set_desiredSpeed()


        #수정
        #if self.actionSteps % (self.step_per_walk * 20) == self.step_per_walk*5 and self.cDirection and self.step_counter is not 0:
            #print(self.step_counter)
            #input()
            #self.changeDirection()
            ###MAXtime수정할것!!!!!!!!!!!!!!!!!!1
            #self.change_targetspeed()
        #if self.step_counter == self.step_per_sec * 30 and self.cDirection:
        #    self.changeDirection()

        
        if done is True:
            print("episodeDone... mean Reward: " + str(self.episodeTotalReward/self.actionSteps))
            #print("velocityReward: " + str(velocityReward) + "__" + str(velocity_s)+ "__" + str(self.desiredSpeed))
            print("action Step", self.actionSteps,self.step_counter)
            #self.reset()
         
        info = {
                'env_step':self.step_counter,
                'n_frames':n_frames
        }

        #print(reward)
        #print(done)
        #print(self.previousState)
        #print(self.step_counter)
        #print(n_frames)
        #input()

        if done is True:
            return thisState, 0, done, info
        return thisState, reward, done, info 

        #self.do_simulation(action, 60)



    def do_simulation(self, action):
        self.controller.mCurrentStateMachine.mCurrentAction = action
        self.controller.mCurrentStateMachine.setTrainedDesiredAction(action, 0)
        done = False
        n_frames = 0
        self.reward_counter = 0
        self.state_reward = 0

        self.tausums = 0
        CFSM = self.controller.mCurrentStateMachine
        isContact = False
        #while(int(self.previousState) == int(self.controller.mCurrentStateMachine.mCurrentState.mName)):
        while(int(self.previousState) == int(self.controller.mCurrentStateMachine.mCurrentState.mName)):
            self.controller.update()
            self.sim.step()
            n_frames += 1
            
            """
            if self.tausums is 0:
                for i in self.skel.tau:
                    self.tausums += np.abs(i)
            """
            if(self.isrender):
                time.sleep(0.001)

            if CFSM.mCurrentState is CFSM.mStates[1]:
                if self.controller.RContact.isSatisfied() is True:
                    isContact = True
            elif CFSM.mCurrentState is CFSM.mStates[3]:
                if self.controller.LContact.isSatisfied() is True:
                    isContact = True


            #if ((n_frames+self.the_last)%30 is 0) or (int(self.previousState) != int(self.controller.mCurrentStateMachine.mCurrentState.mName)) or (done is True):
            #if (n_frames%30 == 0) or (int(self.previousState) != int(self.controller.mCurrentStateMachine.mCurrentState.mName)) or (done is True):
            if (n_frames%30 == 0):
                pos_after = self.sim.skeletons[1].com()
                r_foot_pos = self._getJointPosition(self.r_foot) 
                l_foot_pos = self._getJointPosition(self.l_foot)
                
                #episode가 종료되었는지 먼저 판단
                if pos_after[1] < 0.020 or pos_after[1] > 0.5:
                    done = True
                #정면으로 걷지않을경우 빠르게 종료
                #elif np.abs(pos_after[2]) > 2:
                #    done = true
                elif r_foot_pos[1] > pos_after[1]:
                    done = True
                elif l_foot_pos[1] > pos_after[1]:
                    done = True
                elif self.actionSteps > self.step_per_walk * 100:
                    done = True

                #Contact하고 시간지났을시 State 변화
                if isContact is True:
                    CFSM.transiteTo(CFSM.mCurrentState.getNextState(), CFSM.mBeginTime + CFSM.mElapsedTime)
                

                self.XveloQueue.enqueue(pos_after[0])
                self.ZveloQueue.enqueue(pos_after[2])
                alive_bonus = 10

                #방향 맞춤
                self.currentFrameXAxis = self.getCOMFrameXAxis()
                self.leftAngle = self._calAngleBetweenVectors(self.currentFrameXAxis, self.targetFrameXAxis)
                if np.cross(self.currentFrameXAxis, self.targetFrameXAxis)[1] < 0:
                    self.leftAngle = -self.leftAngle


                #walkPenalty(직선보행 페널티)
                ###Queue 수정해야됨!!!!!!!!!!!!! ###
                if self.XveloQueue.f_e_d() == 0 and self.ZveloQueue.f_e_d() == 0:
                    self.a = [1,0,0]
                else:
                    self.a = [self.XveloQueue.f_e_d(), 0, self.ZveloQueue.f_e_d()]
                self.a = cMat.Matrix.normalize(self.a)
                walkPenalty = self._calAngleBetweenVectors(self.currentFrameXAxis, self.a)
                reward = alive_bonus - walkPenalty - np.abs(self.leftAngle)
               
                #done일시 reward 0
                if done:
                    reward = 0

                self.state_reward += reward
                self.reward_counter += 1

            #Episode 종료시 Break
            if done is True:
                #BREAK 전에 n_frame +1 해줘야함
                #n_frames += 1
                break

        """
        if (n_frames == 0):
            print("n_frames Zero")
            print(int(self.previousState))
            print(int(self.controller.mCurrentStateMachine.mCurrentState.mName))
            input()
        """
        if self.reward_counter is 0:
            print("done", done)
            print(n_frames)
            print("Contact", isContact)
            print(self.the_last)
            print(int(self.previousState),int(self.controller.mCurrentStateMachine.mCurrentState.mName))
        self.the_last = n_frames%30
        return done,n_frames
   
    def render(self):
        return

