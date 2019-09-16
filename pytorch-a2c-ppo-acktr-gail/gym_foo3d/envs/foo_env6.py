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
SIMULATION_STEP_PER_SEC=900
class FooEnv6(env_base.FooEnvBase):
    def init_sim(self,cDirection,render):
        super().init_sim(cDirection,render)
        #observation_spaces = np.concatenate([self.sim.skeletons[1].q[1:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[0:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[int(self.controller.mCurrentStateMachine.mCurrentState.mName),0,0]])
        self.sim.disable_recording()
        observation_spaces = np.concatenate([self.sim.skeletons[1].q[0:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[0:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[1,0,0,0],[0,0]])



        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(11,))
        observation_spaces = np.zeros(len(observation_spaces))
        self.observation_space =spaces.Box(observation_spaces, -observation_spaces)
        #self.Rcontact_time_before = 0
        #self.Rcontact_time_before_2step = 0
        #self.Rcontact_time_current = 0
        self.Rcontact_first = False
        self.Rcontact_mean_step = 0
        #self.Lcontact_time_before = 0
        #self.Lcontact_time_before_2step = 0
        #self.Lcontact_time_current = 0
        self.contact_time_before = 0
        self.contact_time_before_2step = 0
        self.contact_time_current = 0
        self.Lcontact_first = False
        self.Lcontact_mean_step = 0
        self.to_contact_counter_R = 0
        self.to_contact_counter_L = 0

        #이전 정면방향
        self.previousforward = [1,0,0]
        self.ppreviousforward = [1,0,0]

        self.XveloQueue = env_base.CircularQueue(16)
        self.ZveloQueue = env_base.CircularQueue(16)

        #속도 관련
        self.StepCounterQueue = env_base.CircularQueue(16)
        self.VelocityQueue = env_base.CircularQueue(16)
        self.VelocityQueueY = env_base.CircularQueue(16)
        self.VelocityQueueZ = env_base.CircularQueue(16)

        self.targetspeed = 0.2
        self.targetMaxspeed = 2

        ##Curriculum 관련
        self.curValue = 0
        
        ##footstep 관련
        self.prevFootstep = 0

        ##change condition
        self.change_step = 0

        #남은 회전 방향
        self.currentLeftAngle = 0

        ##current State
        self.currentState = [1,0,0,0]
        print(self.targetAngle)

    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[0:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[0:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],self.currentState,[self.desiredSpeed,self.currentLeftAngle]])

    #curriculum Pd value
    def setvalue(self,value):
        self.curValue += 1
        #if self.controller.mCurrentStateMachine.mCurrentState.mRootKp < 5000:            
        #    for states in self.controller.mCurrentStateMachine.mStates:
        #        states.mRootKp += 50
        #        states.mRootKd = states.rootmKp/500
        #        print(states.mRootKp)

    def reset(self):
        super().reset()
        self.contact_time_before = 0
        self.contact_time_before_2step = 0
        self.contact_time_current = 0
        self.Lcontact_first = False
        self.Rcontact_first = False
        self.Rcontact_mean_step = 0
        self.Lcontact_mean_step = 0
        self.to_contact_counter_R = 0
        self.to_contact_counter_L = 0

        #이전 정면방향
        self.previousforward = [1,0,0]
        self.ppreviousforward = [1,0,0]

        #속도 초기화
        self.targetspeed = 0.2
        #self.XveloQueue = env_base.CircularQueue(16)
        #self.ZveloQueue = env_base.CircularQueue(16)
        self.XveloQueue.reset()
        self.ZveloQueue.reset()



        #속도 관련
        self.StepCounterQueue = env_base.CircularQueue(16)
        self.VelocityQueue = env_base.CircularQueue(16)
        self.VelocityQueueY = env_base.CircularQueue(16)
        self.VelocityQueueZ = env_base.CircularQueue(16)


        ##footstep 관련
        self.prevFootstep = 0

        ##change condition
        self.change_step = 0

        ##current State
        self.currentState = [1,0,0,0]

        #남은 회전 방향
        self.currentLeftAngle = 0

        return self.get_state()
        #self.Rcontact_time_before = 0
        #self.Rcontact_time_before_2step = 0
        #self.Rcontact_time_current = 0
        #self.Lcontact_time_before = 0
        #self.Lcontact_time_before_2step = 0
        #self.Lcontact_time_current = 0

    def increaseSpeed(self):
        if self.targetspeed < self.targetMaxspeed:
            self.targetspeed += 0.025


    def clip_Scaling_Actiond10(self, action, stateName):
        action = np.clip(action, -1, 1)
        #드는거 
        if stateName is "0" or stateName is "2":
            #swh
            action[0] = ((action[0] + 1)/2)*np.pi/3       
            #swk
            action[1] = (((action[1] - 1)/4)-0.5)*np.pi/2
            #swa
            action[2] = (((action[2] + 1)/2)*(2/3)+1/3)*np.pi/3
        #내리는거
        else:
            #swh
            action[0] = ((action[0] - 1)/2)*np.pi/6
            #swk
            action[1] = ((action[1]-1)/2)*np.pi/9
            #swa
            action[2] = (action[2])*np.pi/9
        
        #stk
        action[3] = ((action[3]-1)/2)*np.pi/9
        #sta
        action[4] = (action[4])*np.pi/9
        #swhx
        action[5] = (action[5])*math.radians(10.0) 
        #swing hpz
        action[6] = (action[6])*math.radians(30.0)
        #stance hpx,hpy,hpz
        action[7] = (action[7])*math.radians(10.0)
        action[8] = ((action[8]-1)/2)*math.radians(30.0)
        action[9] = (action[9])*math.radians(30.0) 
        ##contact offset
        action[10] = action[10]*150

        ##root
        #action[14] = (action[14])*np.pi/4

        ##timer offset 
        #action[15] = action[15]*150
        #self.ForceAction10(action)

        return action



    def step(self, action):
        pos_before = self.sim.skeletons[1].com()
        panelty = 0
        check = 0
        #Curriculum Value 곱하기
        #if self.curValue <= 20:
        #    action[14] = (self.curValue/20)*action[14]
        self.previousState = self.controller.mCurrentStateMachine.mCurrentState.mName
        #done은 에피소드가 끝났는지..
        action = self.clip_Scaling_Actiond10(action, self.previousState)
        done,n_frames = self.do_simulation(action)
        
        #발의 위치
        r_foot_pos = self._getJointPosition(self.r_foot) 
        l_foot_pos = self._getJointPosition(self.l_foot)

        pos_after = self.sim.skeletons[1].com()
        self.XveloQueue.enqueue(pos_after[0])
        self.ZveloQueue.enqueue(pos_after[2])
       
        #속도 계산(단순하게)
        xDis = self.XveloQueue.f_e_d()
        zDis = self.ZveloQueue.f_e_d()
        #DisV = ((np.sqrt(np.square(xDis) + np.square(zDis)))*(16/self.XveloQueue.count))/2
        DisV = ((np.sqrt(np.square(xDis) + np.square(zDis)))/self.StepCounterQueue.sum_all())*900

        #1초간의 속도 계산

        ###수정 예정(env_base에서도 수정해야됨)###
        #velocity_s = self.distance()
        #velocityReward = np.abs(velocity_s - self.desiredSpeed)
        #print("vs",velocity_s)
        #print("dS", self.desiredSpeed)

        #두 걸음간의 속도 (X,Y,Z축 방향으로)

        #self.VelocityQueue.enqueue(pos_after[0] - pos_before[0])
        #self.VelocityQueueY.enqueue(pos_after[1] - pos_before[1])
        #self.VelocityQueueZ.enqueue(pos_after[2] - pos_before[2])
        
        #sim_time = self.StepCounterQueue.sum_all()/900
        #velocity_2step = [self.VelocityQueue.sum_all()/sim_time,self.VelocityQueueY.sum_all()/sim_time,self.VelocityQueueZ.sum_all()/sim_time]

        ##speed reward(penalty)
        #if velocity_2step[0] < self.targetspeed:
        #    speed_penalty = self.targetspeed - velocity_2step[0]
        #else:
        #    speed_penalty = 0


        alive_bonus = 10

        #방향 맞춤
        self.currentFrameXAxis = self.getCOMFrameXAxis()
        for i in range(3):
            self.currentFrameXAxis[i] = (self.currentFrameXAxis[i] + self.ppreviousforward[i])
        self.leftAngle = self._calAngleBetweenVectors(self.currentFrameXAxis, self.targetFrameXAxis)
        if np.cross(self.currentFrameXAxis, self.targetFrameXAxis)[1] < 0:
            self.leftAngle = -self.leftAngle
        self.ppreviousforward = self.previousforward
        self.previousforward = self.getCOMFrameXAxis()


        #발의 위치로 early Termination (비활성)
        #보폭을 비슷하게
        currentFrameXAxisN = np.linalg.norm(self.currentFrameXAxis)
        rightFoot = np.dot(r_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
        leftFoot = np.dot(l_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN

        """
        if self.previousState is "0":
            if rightFoot -leftFoot < 0.0001:
                done = True
        elif self.previousState is "2":
            if leftFoot - rightFoot  < 0.0001:
                done = True
        """
        FootstepDiff = 0
        if self.previousState is "1":
            FootstepDiff = np.abs((rightFoot - leftFoot) - self.prevFootstep)
            #if rightFoot -leftFoot < 0.0001:
            #    done = True
            self.prevFootstep = rightFoot - leftFoot
        elif self.previousState is "3":
            FootstepDiff = np.abs((leftFoot - rightFoot) - self.prevFootstep)
            #if leftFoot - rightFoot  < 0.0001:
            #    done = True
            self.prevFootstep = leftFoot - rightFoot

        
        ##torso 균형
        torsoMSE = 0
        for i in self.sim.skeletons[1].q[6:9]:
            torsoMSE += np.abs(i)
        #torsoMSE = torsoMSE/3

        ##root 균형(pelvis의 y축 요소중 z축 방향으로의 요소가 0이 되어야 한다.)
        ##pelvis가 양옆으로 무너지는 일이 없어야 한다는것
        rootPenalty = 0
        pelvisYAxis = cMat.Matrix.col(self.sim.skeletons[1].body("pelvis").T,2)
        rootPenalty = np.abs(pelvisYAxis[2])


        #walkPenalty(직선보행 페널티)
        ###Queue 수정해야됨!!!!!!!!!!!!! ###
        #self.a = [1,0,0]
        if self.XveloQueue.f_e_d() == 0 and self.ZveloQueue.f_e_d() == 0:
            self.a = [1,0,0]
        else:
            self.a = [self.XveloQueue.f_e_d(), 0, self.ZveloQueue.f_e_d()]
        self.a = cMat.Matrix.normalize(self.a)
        walkPenalty = self._calAngleBetweenVectors(self.currentFrameXAxis, self.a)


        #print(self.skel.tau)
        #tausums = 0
        #for i in self.skel.tau:
        #    tausums += np.abs(i)

        #print(tausums)
        #input()
 

        #reward 수정 예정,, torque 총합 페널티..
        #reward = alive_bonus - np.exp(2*(np.abs(self.leftAngle)) + 1.5*walkPenalty + 2*velocityReward)

        ##초반 walkpenalty 상쇄?
        #reward = alive_bonus - self.tausums/10000 - 3*walkPenalty - np.abs(self.leftAngle) - 5*speed_penalty
        #reward = alive_bonus - self.tausums/8000 - 3*walkPenalty - 2*np.abs(self.leftAngle) - np.abs(DisV - 1)
        
        ##다리 질질끌고 통통 튀면서 걷고 한쪽 다리 거의 못들어올리고 방향전환은 가능하나 결과 별로 좋지않다.
        ##reward = alive_bonus - self.tausums/8000 - 2*walkPenalty - 2*np.abs(self.leftAngle) - 1.4*np.abs(DisV - 1) - 3*torsoMSE - 4*FootstepDiff
        
        #reward = (alive_bonus - self.tausums/8000 - 5*walkPenalty - 5*np.abs(self.leftAngle) - 1.4*np.abs(DisV - 0.7) - 3*torsoMSE - 2*FootstepDiff)*(n_frames/SIMULATION_STEP_PER_SEC)
        #reward = (alive_bonus - self.tausums/8000 - 5*walkPenalty - 5*np.abs(self.leftAngle) - 1.4*np.abs(DisV - 1) - 3*torsoMSE - 2*FootstepDiff)
        #reward = (alive_bonus - self.tausums/8000 - 5*walkPenalty - 5*np.abs(self.leftAngle) - 4*np.abs(DisV - 1) - 3*torsoMSE - 2*FootstepDiff)
        reward = (alive_bonus - self.tausums/8000 - 5*walkPenalty - 15*np.abs(self.currentLeftAngle) - 4*np.abs(DisV - 1) - 3*torsoMSE - 2*FootstepDiff
                - 5*rootPenalty)


        self.step_counter += n_frames
        self.change_step += n_frames
        thispos = pos_after[0]
       
        self.actionSteps += 1
        self.episodeTotalReward += reward
        #self.set_desiredSpeed()

        
        #수정
        #if self.actionSteps % (self.step_per_walk * 20) == self.step_per_walk*5 and self.cDirection and self.step_counter is not 0 and self.curValue > 0:
        if self.actionSteps % (self.step_per_walk * 10) == self.step_per_walk*5 and self.cDirection and self.step_counter is not 0:
        #if self.change_step % (SIMULATION_STEP_PER_SEC*10) > SIMULATION_STEP_PER_SEC*5 and self.change_step > 0 and self.cDirection:
            #self.change_step = self.change_step - SIMULATION_STEP_PER_SEC*10

            #print(self.step_counter)
            #input()
            ##회전 각 제한을 60도 정도로
            self.changeDirection()
            ###MAXtime수정할것!!!!!!!!!!!!!!!!!!1
            #self.change_targetspeed()
        #if self.step_counter == self.step_per_sec * 30 and self.cDirection:
        #  self.changeDirection()
        
                
        if done is True:
            print("episodeDone... mean Reward: " + str(self.episodeTotalReward/self.actionSteps))
            #print("velocityReward: " + str(velocityReward) + "__" + str(velocity_s)+ "__" + str(self.desiredSpeed))
            print("action Step", self.actionSteps,self.step_counter)
            print("curValue", self.curValue)
            if self.controller.mCurrentStateMachine.mCurrentState.mRootKp:
              print(self.controller.mCurrentStateMachine.mCurrentState.mRootKp)
            #self.reset()
            #input()
        
        info = {
                'pos':pos_after[2]
        }
        
        #input()
        #print(reward)
        #print(done)
        #print(self.previousState)

        ##one hot incording으로 State 정보 넣어주기
        self.currentState = [0,0,0,0]
        self.currentState[int(self.controller.mCurrentStateMachine.mCurrentState.mName)] = 1

        ##남은 각도 10도로 제한하는 코드
        rad10deg = np.deg2rad(10)
        if np.abs(self.leftAngle) > rad10deg:
            ##남은 각도 10도
            self.currentLeftAngle = -rad10deg if self.leftAngle< 0 else rad10deg
        else:
            self.currentLeftAngle = self.leftAngle

        thisState = self.get_state()

        if done is True:
            return thisState, 0, done, info
        return thisState, reward, done, info 

        #self.do_simulation(action, 60)



    def do_simulation(self, action):
        self.controller.mCurrentStateMachine.mCurrentAction = action
        self.controller.mCurrentStateMachine.setTrainedDesiredAction(action, 0)
        done = False
        n_frames = 0 
        self.tausums = 0
        state_step = 0
        state_step_after_contact = 0
        offset = np.round(action[10])

        #offset = np.round((np.random.rand()-0.5)*20)
        #offset = 0
        CFSM = self.controller.mCurrentStateMachine

        #스윙힙이 최고 높이에 도달했을때

        #print((CFSM.mCurrentState.mName))
        if int(CFSM.mCurrentState.mName) == 1: 
            self.to_contact_counter_R = 0
        elif int(CFSM.mCurrentState.mName) == 3:
            self.to_contact_counter_L = 0

        ##속도 관련 counter
        #step_counter_queue_value = 0
        ## self.previousState는 step 들어가기 전 현재 State
        ## previousState가 1,3일때는 자동 transite가 일어나지 않기 때문에 이걸 조건문으로 이용해도 된다.?
        while(self.previousState is self.controller.mCurrentStateMachine.mCurrentState.mName):
            self.controller.update()
            self.sim.step()

            ##속도 관련 n_frames로 대체
            #step_counter_queue_value += 1

            #이 State에서의 step_counter
            state_step += 1
            self.to_contact_counter_R += 1
            self.to_contact_counter_L += 1
            n_frames += 1


            #컨택일어난 이후의 step counter
            if CFSM.mCurrentState is CFSM.mStates[1] and self.Rcontact_first is True:
                state_step_after_contact += 1
            elif CFSM.mCurrentState is CFSM.mStates[3] and self.Lcontact_first is True:
                state_step_after_contact += 1
            #print(self.previousState, "p") 
            if self.tausums is 0:
                for i in self.skel.tau:
                    self.tausums += np.abs(i)

            #if CFSM.mCurrentState is CFSM.mStates[2] or CFSM.mCurrentState is CFSM.mStates[1]:
            if self.previousState is "1" or self.previousState is "2":
                if self.controller.RContact.isSatisfied():
                    if self.Rcontact_first is False:
                        #반대쪽발 컨택트 초기화
                        self.Lcontact_first = False
                        #이쪽발 컨택트 만료
                        self.Rcontact_first = True
                        #컨택트 타임 저장
                        #self.Rcontact_time_before_2step = self.Rcontact_time_before
                        #self.Rcontact_time_before = self.Rcontact_time_current
                        #self.Rcontact_time_current = self.to_contact_counter_R
                        self.contact_time_before_2step = self.contact_time_before
                        self.contact_time_before = self.contact_time_current
                        self.contact_time_current = self.to_contact_counter_R
                        #self.Rcontact_mean_step = np.round((self.Rcontact_time_current + self.Rcontact_time_before + self.Rcontact_time_before_2step)/3)
                        self.Rcontact_mean_step = np.round((self.contact_time_current + self.contact_time_before + self.contact_time_before_2step)/3)

            #elif CFSM.mCurrentState is CFSM.mStates[3] or CFSM.mCurrentState is CFSM.mStates[0]:
            elif self.previousState is "3" or self.previousState is "0":    
                if self.controller.LContact.isSatisfied() is True and self.Lcontact_first is False:
                    #반대쪽 컨택트 초기화
                    self.Rcontact_first = False
                    #이쪽 발 컨택트 만료
                    self.Lcontact_first = True
                    #컨택트 타임 저장
                    #self.Lcontact_time_before_2step = self.Lcontact_time_before
                    #self.Lcontact_time_before = self.Lcontact_time_current
                    #self.Lcontact_time_current = self.to_contact_counter_L
                    #self.Lcontact_mean_step = np.round((self.Lcontact_time_current + self.Lcontact_time_before + self.Lcontact_time_before_2step)/3)

                    self.contact_time_before_2step = self.contact_time_before
                    self.contact_time_before = self.contact_time_current
                    self.contact_time_current = self.to_contact_counter_L
                    self.Lcontact_mean_step = np.round((self.contact_time_current + self.contact_time_before + self.contact_time_before_2step)/3)


            if offset > 0 and state_step_after_contact == offset:
                CFSM.transiteTo(CFSM.mCurrentState.getNextState(), CFSM.mBeginTime + CFSM.mElapsedTime)
            elif offset <= 0 and ((int(self.previousState) is 1) or (int(self.previousState) is 3)):
                if CFSM.mCurrentState is CFSM.mStates[1]:
                    #offset이 -이니 더해준다.
                    if self.Rcontact_mean_step + offset <= 0:
                        CFSM.transiteTo(CFSM.mCurrentState.getNextState(), CFSM.mBeginTime + CFSM.mElapsedTime)
                    elif self.Rcontact_mean_step + offset <= state_step:
                        CFSM.transiteTo(CFSM.mCurrentState.getNextState(), CFSM.mBeginTime + CFSM.mElapsedTime)
                elif CFSM.mCurrentState is CFSM.mStates[3]:
                    #offset이 -이니 더해준다.
                    if self.Lcontact_mean_step + offset <= 0:
                        CFSM.transiteTo(CFSM.mCurrentState.getNextState(), CFSM.mBeginTime + CFSM.mElapsedTime)
                    elif self.Lcontact_mean_step + offset <= state_step:
                        CFSM.transiteTo(CFSM.mCurrentState.getNextState(), CFSM.mBeginTime + CFSM.mElapsedTime)
            #elif offset is 0 and state_step_after_contact > 0:
                #print("contact transite")
                #print(state_step_after_contact, "ss", offset, " off")
                #CFSM.transiteTo(CFSM.mCurrentState.getNextState(), CFSM.mBeginTime + CFSM.mElapsedTime)
                #offset 0이면 컨택됐을 때환



            """
            print(self.controller.mCurrentStateMachine.mCurrentState.mName)
            if self.controller.mCurrentStateMachine.mCurrentState is self.controller.mCurrentStateMachine.mStates[1]:
                print("11111")
                print(self.controller.RContact.isSatisfied())
                if self.controller.RContact.isSatisfied(): 
                    print("RContact")
                    print(self.controller.mCurrentStateMachine.mElapsedTime)
                    #self.controller.mCurrentStateMachine.transiteTo(self.controller.mCurrentStateMachine.mCurrentState.getNextState(), self.controller.mCurrentStateMachine.mBeginTime + self.controller.mCurrentStateMachine.mElapsedTime)
                #input()
            if self.controller.LContact.isSatisfied() and self.controller.mCurrentStateMachine.mCurrentState is self.controller.mCurrentStateMachine.mStates[3]:
                print("LContact")
                print(self.controller.mCurrentStateMachine.mElapsedTime)
                #self.controller.mCurrentStateMachine.transiteTo(self.controller.mCurrentStateMachine.mCurrentState.getNextState(), self.controller.mCurrentStateMachine.mBeginTime + self.controller.mCurrentStateMachine.mElapsedTime)
                #input()
            """
            #print(self.controller.mCurrentStateMachine.mCurrentState.mName,"after")

            pos_after = self.sim.skeletons[1].com()
            r_foot_pos = self._getJointPosition(self.r_foot) 
            l_foot_pos = self._getJointPosition(self.l_foot)

            if(self.isrender):
                time.sleep(0.001)
            if pos_after[1] < -0.030 or pos_after[1] > 0.5:
                done = True
            #정면으로 걷지않을경우 빠르게 종료
            #elif np.abs(pos_after[2]) > 2:
            #    done = True
            elif r_foot_pos[1] > pos_after[1]:
                done = True
            elif l_foot_pos[1] > pos_after[1]:
                done = True
            elif self.actionSteps > self.step_per_walk * 200:
            #elif self.step_counter > SIMULATION_STEP_PER_SEC*40:
                done = True
            if done is True:
                break
        self.StepCounterQueue.enqueue(n_frames)
        return done,n_frames
   
    def render(self):
        return

