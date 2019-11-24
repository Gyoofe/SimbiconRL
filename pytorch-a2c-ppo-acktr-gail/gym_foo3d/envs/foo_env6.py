import sys
import pydart2 as pydart
import numpy as np
import cMat
import SimbiconController_3d as SC
import math
import queue
import random
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
GROUND_Y = -0.85
ALIVE_BONUS = 10
class FooEnv6(env_base.FooEnvBase):
    def init_sim(self,cDirection,render):
        super().init_sim(cDirection,render)
        #observation_spaces = np.concatenate([self.sim.skeletons[1].q[1:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[0:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[int(self.controller.mCurrentStateMachine.mCurrentState.mName),0,0]])
        self.sim.disable_recording()
        #self.Rcontact_time_before = 0
        #self.Rcontact_time_before_2step = 0
        #self.Rcontact_time_current = 0
        

        ##Contact Time 관련
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
        self.contactTimeQueue = env_base.CircularQueue(3)
        self.contactTimeQueue.enqueue(50)

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
        self.currentState = [0]

        ##Stride 관련 term
        self.last_Rcontact_r_foot_pos = None
        self.last_Rcontact_l_foot_pos = None
        self.last_Lcontact_r_foot_pos = None
        self.last_Lcontact_l_foot_pos = None
        self.desiredStepLength = 0

        ##Step Duration 관련
        self.stepDuration = 0 
        self.desiredStepDuration = 0
        self.currentOffset = 0
        ##Parameter값 Change 관련
        self.advancedActionstepPrevParameter = 0


        self.cStepDuration = 0
        self.cStepLength = 0
        self.cMaximumSwingfootHeight = 0 
        #SwingFoot Height 관련
        self.desiredMaximumSwingfootHeight = 0
        self.footPosWhenS0S2End = None
        self.ChangeRandom()

        #observation_spaces = np.concatenate([self.sim.skeletons[1].q[0:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[0:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],[1,0,0,0],[0,0]])
        
        #정보업데이트
        self.updateEndEffectorLocalPosition()
        observation_spaces = self.get_state()
        self.action_space = spaces.Box(low = 0, high = 1.5, shape=(23,))
        observation_spaces = np.zeros(len(observation_spaces))
        self.observation_space =spaces.Box(observation_spaces, -observation_spaces)
        #self.observation_space = self.get_state()

        #UI사용
        self.foot_pos = [0,0,0]
        self.rightFoot = 0
        self.leftFoot = 0
        self.FXAnorm = [1,0,0]
        print(self.targetAngle)
    """
    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[0:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[0:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],self.currentState,
            [self.desiredStepDuration,self.desiredStepLength,self.desiredMaximumSwingfootHeight,
            self.currentOffset,np.round(self.contactTimeQueue.sum_all()/self.contactTimeQueue.count),self.controller.LContact.isSatisfied(),self.controller.RContact.isSatisfied()],
           self.l_hand_relative_pos,self.r_hand_relative_pos,self.utorso_relative_pos,self.l_foot_relative_pos,self.r_foot_relative_pos])
    """

    def get_state(self):
        return np.concatenate([self.sim.skeletons[1].q[0:3],self.sim.skeletons[1].q[6:9],self.sim.skeletons[1].q[14:20],self.sim.skeletons[1].q[26:32],self.sim.skeletons[1].dq[0:3],self.sim.skeletons[1].dq[6:9],self.sim.skeletons[1].dq[14:20],self.sim.skeletons[1].dq[26:32],self.currentState,
            [self.desiredStepDuration,self.desiredStepLength,self.desiredMaximumSwingfootHeight],
           self.l_hand_relative_pos,self.r_hand_relative_pos,self.utorso_relative_pos,self.l_foot_relative_pos,self.r_foot_relative_pos])


    def updateEndEffectorLocalPosition(self):
        pelMinv = np.linalg.inv(self.pelvis.world_transform())
        l_hand_pos = cMat.Matrix.col(self.l_hand.world_transform(),3)
        r_hand_pos = cMat.Matrix.col(self.r_hand.world_transform(),3)
        utorso = cMat.Matrix.col(self.utorso.world_transform(),3)
        l_foot_pos = cMat.Matrix.col(self.l_foot.world_transform(),3)
        r_foot_pos = cMat.Matrix.col(self.r_foot.world_transform(),3)

        self.l_hand_relative_pos = (pelMinv@l_hand_pos)[0:3]
        self.r_hand_relative_pos = (pelMinv@r_hand_pos)[0:3]
        self.utorso_relative_pos = (pelMinv@utorso)[0:3]
        self.l_foot_relative_pos = (pelMinv@l_foot_pos)[0:3]
        self.r_foot_relative_pos = (pelMinv@r_foot_pos)[0:3]


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

        ##Contact Time 관련
        self.contact_time_before = 0
        self.contact_time_before_2step = 0
        self.contact_time_current = 0
        self.Lcontact_first = False
        self.Rcontact_first = False
        self.Rcontact_mean_step = 0
        self.Lcontact_mean_step = 0
        self.to_contact_counter_R = 0
        self.to_contact_counter_L = 0
        self.contactTimeQueue.reset()
        self.contactTimeQueue.enqueue(75)

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
        self.currentState = [0]

        #남은 회전 방향
        self.currentLeftAngle = 0

        ##Stride 관련 term
        self.last_Rcontact_r_foot_pos = None
        self.last_Rcontact_l_foot_pos = None
        self.last_Lcontact_r_foot_pos = None
        self.last_Lcontact_l_foot_pos = None
        self.desiredStepLength = 0

        ##Step Duration 관련
        self.stepDuration = 0 
        self.desiredStepDuration = 0 
        #SwingFoot Height 관련
        self.desiredMaximumSwingfootHeight = 0
        self.footPosWhenS0S2End = -0.8
        self.currentOffset = 0
        self.ChangeRandom()
        #정보업데이트
        self.updateEndEffectorLocalPosition()
        ##State초기화
        self.controller.mCurrentStateMachine.resetWalkingMachine()
        ##Parameter값 Change 관련
        self.advancedActionstepPrevParameter = 0

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
        action = np.clip(action, -200, 200)/100
        #드는거 
        #swh02
        action[0] = 0.5979495+(action[0])*1.1227705
        #swk02
        #action[1] = (((action[1] - 1)/4)-0.5)*np.pi/2
        action[1] = ((action[1] - 1)/2)*2.38569
        #swa02
        action[2] = action[2]*0.85 + 0.15
        #내리는거
        #swh13
        action[3] = 0.5979495+(action[3])*1.1227705
        #swk13
        action[4] = ((action[4] - 1)/2)*2.38569
        #swa13
        action[5] = action[5]*0.85 + 0.15

        #stk02
        action[6] = ((action[6] - 1)/2)*2.38569
        #sta02
        action[7] = action[7]*0.85 + 0.15

        #stk13
        action[8] = ((action[8] - 1)/2)*2.38569
        #sta13
        action[9] = action[9]*0.85 + 0.15

        #swhx02
        action[10] = (action[10])*math.radians(30.0)
        #swhx13
        action[11] = (action[11])*math.radians(30.0) 
        #swing hpz02
        action[12] = (action[12] * 0.6981315) - 0.5235985
        #swing hpz13
        action[13] = (action[13] * 0.6981315) - 0.5235985
        #stance hpx,hpy,hpz02
        action[14] = (action[14])*math.radians(30.0)
        action[15] = 0.5979495+(action[15])*1.1227705
        action[16] = (action[16] * 0.6981315) - 0.5235985
    
        #stance hpx,hpy,hpz13
        action[17] = (action[17])*math.radians(30.0)
        action[18] = 0.5979495+(action[18])*1.1227705
        action[19] = (action[19] * 0.6981315) - 0.5235985

        ##Duration
        action[20] = ((action[20]+1)/2)*0.45 + 0.1
        ##Torso02
        action[21] = (action[21])*0.524559-0.086132
        ##Torso13
        action[22] = (action[22])*0.524559-0.086132
        return action




    """
    def clip_Scaling_Actiond10(self, action, stateName):
        action = np.clip(action, -1, 1)
        #다리 드는거 
        #if stateName is "0" or stateName is "2":
            #swh(98 deg)
        #    action[0] = ((action[0] + 1)/2)*1.72072
            #action[0] = (action[0]+1/2)*(-self.sim.skeletons[1].dof("l_leg_hpy").position_lower_limits())
        #다리 내리는거
        #else:
            #swh(deg 30)
        #    action[0] = ((action[0] - 1)/2)*0.524821
            #action[0] = ((action[0] -1)/2)*(self.sim.skeletons[1].dof("l_leg_hpy").position_lower_limits())
        ##SwingHip
        action[0] = 0.5979495+(action[0])*1.1227705
        #swa(-40 ~ 57)
        action[2] = action[2]*0.85 + 0.15
        #swk(-136 deg)
        action[1] = ((action[1] - 1)/2)*2.38569
        #action[1] = ((action[1] - 1)/2)*(self.sim.skeletons[1].dof("l_leg_hpy").position_lower_limits())
        #stk(-136 deg)
        action[3] = ((action[3]-1)/2)*2.38569
        #sta(-40deg ~ 57 deg)
        action[4] = (action[4])*0.85 + 0.15
        #swhx(deg 30)
        action[5] = (action[5])*0.523599
        #swing hpz(deg10 ~ -70)
        action[6] = (action[6] * 0.6981315) - 0.5235985
        #stance hpx(deg30),hpy(deg30),hpz(deg 10 ~ - 70)
        action[7] = (action[7])*0.523599
        #action[8] = ((action[8]-1)/2)*0.524821
        #action[8] = (action[8])*0.524821
        action[0] = 0.5979495+(action[0])*1.1227705
        action[9] = (action[9] * 0.6981315) - 0.5235985

        ##Duration
        action[10] = ((action[10]+1)/2)*0.4 + 0.1

        ##Torso(-22도 ~ 34도)
        action[11] = (action[11])*0.524559-0.086132

        return action
    """

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

        #방향 맞춤
        self.currentFrameXAxis = self.getCOMFrameXAxis()
        for i in range(3):
            self.currentFrameXAxis[i] = (self.currentFrameXAxis[i] + self.ppreviousforward[i])
        self.leftAngle = self._calAngleBetweenVectors(self.currentFrameXAxis, self.targetFrameXAxis)
        if np.cross(self.currentFrameXAxis, self.targetFrameXAxis)[1] < 0:
            self.leftAngle = -self.leftAngle
        self.ppreviousforward = self.previousforward
        self.previousforward = self.getCOMFrameXAxis()

        
        ##torso 균형
        torsoYVec = self.mtorso.world_transform()[0:3,2]
        torsoUprightPenalty = 1 - np.dot(torsoYVec, [0,1,0])
        
        ##root 균형(pelvis의 y축 요소중 z축 방향으로의 요소가 0이 되어야 한다.)
        ##pelvis가 양옆으로 무너지는 일이 없어야 한다는것
        rootPenalty = 0
        #pelvisYAxis = cMat.Matrix.col(self.sim.skeletons[1].body("pelvis").T,2)
        pelvisForward = cMat.Matrix.col(self.pelvis.world_transform(),0)[1]
        #rootPenalty = np.abs(pelvisYAxis[2])
        rootPenalty = pelvisForward if pelvisForward > 0 else 0        

        #walkPenalty(직선보행 페널티)
        ###Queue 수정해야됨!!!!!!!!!!!!! ###
        #self.a = [1,0,0]
        if self.XveloQueue.f_e_d() == 0 and self.ZveloQueue.f_e_d() == 0:
            self.a = [1,0,0]
        else:
            self.a = [self.XveloQueue.f_e_d(), 0, self.ZveloQueue.f_e_d()]
        self.a = cMat.Matrix.normalize(self.a)
        walkPenalty = self._calAngleBetweenVectors(self.currentFrameXAxis, self.a)



        ##Stride Reward
        #보폭을 비슷하게
        currentFrameXAxisN = np.linalg.norm(self.currentFrameXAxis)
        self.FXAnorm = self.currentFrameXAxis/currentFrameXAxisN
        ##Step이전 State에 따라서 정해짐
        ##rFoot 올릴때(rFoot Contact가 일어났다가 떨어짐)
        if self.previousState is "0":
            #rightFoot = np.dot(self.last_Rcontact_r_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            #leftFoot = np.dot(self.last_Rcontact_l_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            rightFoot = np.dot(r_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            leftFoot = np.dot(l_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            StepLengthPenalty = np.abs(rightFoot - leftFoot - self.desiredStepLength)
            self.cStepLength = rightFoot - leftFoot
            self.endFoot = pos_after + leftFoot*self.FXAnorm
            self.endFoot[1] = -0.95
        ##lFoot을 올렸을때 (lFoot Contact가 일어났다가 떨어짐)
        elif self.previousState is "2":
            #rightFoot = np.dot(self.last_Lcontact_r_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            #leftFoot = np.dot(self.last_Lcontact_l_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            rightFoot = np.dot(r_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            leftFoot = np.dot(l_foot_pos - pos_after, self.currentFrameXAxis)/currentFrameXAxisN
            StepLengthPenalty = np.abs(leftFoot - rightFoot - self.desiredStepLength)
            self.cStepLength = leftFoot - rightFoot
            self.endFoot = pos_after + rightFoot*self.FXAnorm
            self.endFoot[1] = -0.95
        else:
            StepLengthPenalty = 0


        ##Step Duration Reward
        if self.previousState is "0" or self.previousState is "2":
            stepDurationPenalty = np.abs((self.stepDuration/900.0) - self.desiredStepDuration)
            self.cStepDuration = self.stepDuration/900.0
            #stepDurationPenalty = np.round(np.abs(self.stepDuration - np.round(self.desiredStepDuration*900)))
            #if math.isclose(stepDurationPenalty,0):
            #    stepDurationPenalty = 1
            #else:
            #    stepDurationPenalty = 1.0/stepDurationPenalty

        else:
            stepDurationPenalty = 0

        ##Desired Maximum swing foot Height
        FootHeightPenalty = np.abs(self.footPosWhenS0S2End - self.desiredMaximumSwingfootHeight)
        self.cMaximumSwingfootHeight = self.footPosWhenS0S2End
        #print(self.footPosWhenS0S2End)
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
        #reward = (alive_bonus - self.tausums/8000 - 5*walkPenalty - 5*np.abs(self.currentLeftAngle) - 3*torsoMSE - 10*StepLengthPenalty - 15*stepDurationPenalty - 20*FootHeightPenalty)
        #reward = (alive_bonus - self.tausums/8000 - 5*walkPenalty - 5*np.abs(self.currentLeftAngle) - 3*torsoMSE - 10*StepLengthPenalty - 10*FootHeightPenalty)*stepDurationPenalty
        """
        reward = (np.exp(-np.square(self.tausums/8000))*
                np.exp(-np.square(walkPenalty))*
                np.exp(-np.square(self.leftAngle))*
                np.exp(-np.square(torsoMSE))*
                np.exp(-4*np.square(StepLengthPenalty))*
                np.exp(-9*np.square(2*stepDurationPenalty))*
                np.exp(-9*np.square(FootHeightPenalty)))
        """

        """
        reward = (np.exp(-np.square(self.tausums/8000))*
                np.exp(-np.square(walkPenalty))*
                np.exp(-np.square(self.leftAngle))*
                np.exp(-4*np.square(rootPenalty))*
                np.exp(-4*np.square(StepLengthPenalty))*
                np.exp(-9*np.square(2*stepDurationPenalty))*
                np.exp(-9*np.square(FootHeightPenalty)))
        """

        #reward = (alive_bonus - self.tausums/8000 - 5*walkPenalty - 5*np.abs(self.currentLeftAngle) - 3*rootPenalty - 8*StepLengthPenalty - 8*FootHeightPenalty - 8*stepDurationPenalty - 10*torsoUprightPenalty)

        alive_bonus = ALIVE_BONUS

        reward = (alive_bonus - self.tausums/32000 -2*rootPenalty - np.abs(pos_after[2]) - 10*StepLengthPenalty - 15*FootHeightPenalty - 12*stepDurationPenalty - 6*torsoUprightPenalty)/int(3/self.desiredStepDuration)

        self.step_counter += n_frames
        self.change_step += n_frames
        thispos = pos_after[0]
       
        self.episodeTotalReward += reward
        #self.set_desiredSpeed()

        #정보 업데이트
        self.updateEndEffectorLocalPosition()
     
        #수정
        #if self.actionSteps % (self.step_per_walk * 20) == self.step_per_walk*5 and self.cDirection and self.step_counter is not 0 and self.curValue > 0:/
        #if self.actionSteps % (self.step_per_walk * 10) == self.step_per_walk*5 and self.cDirection and self.step_counter is not 0:
        if self.actionSteps - self.advancedActionstepPrevParameter > int(3/self.desiredStepDuration):
            self.ChangeRandom()
            self.advancedActionstepPrevParameter = self.actionSteps

        ##one hot incording으로 State 정보 넣어주기
        if self.controller.mCurrentStateMachine.mCurrentState.mName is "0" or self.controller.mCurrentStateMachine.mCurrentState.mName is "1":
            self.currentState = [0]
        else:
            self.currentState = [1]

        """
        if done is True:
            print("episodeDone... mean Reward: " + str(self.episodeTotalReward/self.actionSteps))
            #print("velocityReward: " + str(velocityReward) + "__" + str(velocity_s)+ "__" + str(self.desiredSpeed))
            print("action Step", self.actionSteps,self.step_counter)
            print("curValue", self.curValue)
            if self.controller.mCurrentStateMachine.mCurrentState.mRootKp:
              print(self.controller.mCurrentStateMachine.mCurrentState.mRootKp)
            #self.reset()
            #input()
        """
        info = {
                'pos':pos_after[2]
        }
        
        #input()
        #print(reward)
        #print(done)
        #print(self.previousState)

        thisState = self.get_state()

        if done is True:
            return thisState, 0, done, info
        return thisState, reward, done, info 

        #self.do_simulation(action, 60)

    """
    def ChangeRandom(self):
        self.desiredStepLength = random.uniform(0.1,0.6)
        #self.desiredStepLength = np.clip(np.random.normal(0.4,0.1),0.1,0.6)
        #self.desiredStepLength = 0.3
        self.desiredStepDuration = random.uniform(0.1,0.5)
        #self.desiredStepDuration = np.clip(np.random.normal(0.3,0.1),0.1,0.5)
        #self.desiredStepDuration = 0.3
        self.desiredMaximumSwingfootHeight = -random.uniform(0.4, 0.8)
        #self.desiredMaximumSwingfootHeight = -np.clip(np.random.normal(0.6,0.1),0.4,0.8)
        #self.desiredMaximumSwingfootHeight = -0.7
        self.currentOffset = np.round(random.uniform(-100,100))
        #self.currentOffset = np.round(np.clip(np.random.normal(0,33),-100,100))
        #self.currentOffset = 0
        return 
    """
    def ChangeRandom(self):
        #self.desiredStepLength = random.uniform(0.1,0.6)
        #self.desiredStepDuration = random.uniform(0.1,0.5)
        #self.desiredMaximumSwingfootHeight = -random.uniform(0.4, 0.8)

        self.desiredStepDuration = random.uniform(0.1,0.5)
        #self.desiredStepDuration = np.clip(np.random.normal(0.3,0.06),0.1,0.5)
        stepLengthMin = self.desiredStepDuration - self.desiredStepDuration/3.0
        self.desiredStepLength = random.uniform(stepLengthMin,stepLengthMin+0.2)
        #self.desiredStepLength = np.clip(np.random.normal(0.4,0.06),0.1,0.6)
        swingfootHeightMin = self.desiredStepDuration/4.0
        self.desiredMaximumSwingfootHeight = random.uniform(swingfootHeightMin, swingfootHeightMin+0.15)
        #self.desiredMaximumSwingfootHeight = -np.clip(np.random.normal(0.6,0.06),0.4,0.8)
        #self.currentOffset = np.round(random.uniform(-100,100))
        self.currentOffset = 0
        #self.currentOffset = np.round(np.clip(np.random.normal(0,33),-100,100))

        return


    def do_simulation(self, action):
        self.controller.mCurrentStateMachine.mCurrentAction = action
        self.controller.mCurrentStateMachine.setTrainedDesiredAction(action, 0)
        done = False
        n_frames = 0 
        self.tausums = 0
        state_step = 0
        state_step_after_contact = -2
        self.actionSteps += 1

        offset = self.currentOffset 
        #offset = np.round((np.random.rand()-0.5)*20)
        #offset = 0
        CFSM = self.controller.mCurrentStateMachine

        ##발을 들어올리기 시작할 때  Contact Boolean 초기화 
        if int(CFSM.mCurrentState.mName) == 0: 
            self.Rcontact_first = False
            self.stepDuration = 0
            self.to_contact_counter_R = 0

        elif int(CFSM.mCurrentState.mName) == 2:
            self.Lcontact_first = False
            self.stepDuration = 0
            self.to_contact_counter_L = 0
 

        #print(CFSM.mCurrentState.mName)

        ##속도 관련 counter
        #step_counter_queue_value = 0
        ## self.previousState는 step 들어가기 전 현재 State
        ## previousState가 1,3일때는 자동 transite가 일어나지 않기 때문에 이걸 조건문으로 이용해도 된다.?

        ##값 초기화
        self.footPosWhenS0S2End = 0
        while(self.previousState is self.controller.mCurrentStateMachine.mCurrentState.mName
                or (int(self.previousState)+1)%4 is int(self.controller.mCurrentStateMachine.mCurrentState.mName)):
            self.controller.update()
            self.sim.step()

            pos_after = self.sim.skeletons[1].com()
            r_foot_pos = self._getJointPosition(self.r_foot) 
            l_foot_pos = self._getJointPosition(self.l_foot)


            if self.previousState is "0":
                self.foot_pos = self.r_foot.world_transform()[0:3,3]
            else:
                self.foot_pos = self.l_foot.world_transform()[0:3,3]

            ##속도 관련 n_frames로 대체
            #step_counter_queue_value += 1

            #이 State에서의 step_counter
            state_step += 1
            n_frames += 1
            self.stepDuration += 1

            ##전체 몸에 가해지는 Torque 합
            if self.tausums is 0:
                for i in self.skel.tau:
                    self.tausums += np.abs(i)

            if self.controller.mCurrentStateMachine.mCurrentState.mName is "0":
                if self.footPosWhenS0S2End < r_foot_pos[1] - GROUND_Y:
                    self.footPosWhenS0S2End = r_foot_pos[1] - GROUND_Y
                self.to_contact_counter_L += 1
            elif self.controller.mCurrentStateMachine.mCurrentState.mName is "1":
                self.to_contact_counter_R += 1
            elif self.controller.mCurrentStateMachine.mCurrentState.mName is "2":
                self.to_contact_counter_R += 1
                if self.footPosWhenS0S2End < l_foot_pos[1] - GROUND_Y:
                    self.footPosWhenS0S2End = l_foot_pos[1] - GROUND_Y
            elif self.controller.mCurrentStateMachine.mCurrentState.mName is "3":
                self.to_contact_counter_L += 1



            """
            ###컨택이 처음 일어난다면 그 떄의 foot위치 저장
            if self.Rcontact_first is False and self.controller.RContact.isSatisfied() is True:
                self.Rcontact_first = True
                ##반대쪽발 Contact를 초기화
                self.Lcontact_first = False
                ##발의 위치 저장
                self.last_Rcontact_r_foot_pos = self._getJointPosition(self.r_foot) 
                self.last_Rcontact_l_foot_pos = self._getJointPosition(self.l_foot)
            elif self.Lcontact_first is False and self.controller.LContact.isSatisfied() is True:
                self.Lcontact_first = True
                ##반대쪽발 Contact는 초기화
                self.Rcontact_first = False
                ##발의 위치 저장
                self.last_Lcontact_r_foot_pos = self._getJointPosition(self.r_foot) 
                self.last_Lcontact_l_foot_pos = self._getJointPosition(self.l_foot)
            """
            pelvisYpos = self.pelvis.world_transform()[1][3]   
            if(self.isrender):
                time.sleep(0.001)
            #if pos_after[1] < -0.030 or pos_after[1] > 0.5:
            if pelvisYpos < -0.2 or pos_after[1] > 0.5:
                done = True
            #정면으로 걷지않을경우 빠르게 종료
            #elif np.abs(pos_after[2]) > 2:
            #    done = True
            elif r_foot_pos[1] > pos_after[1]:
                done = True
            elif l_foot_pos[1] > pos_after[1]:
                done = True
            #elif self.actionSteps > self.step_per_walk * 30:
            elif self.step_counter > SIMULATION_STEP_PER_SEC*(0.3)*15:
                done = True
            if done is True:
                break
        """
        if self.previousState is "0":
            if CFSM.mCurrentState.mName is "1" and done is False:
                print("gg")
        if self.previousState is "2":
            if CFSM.mCurrentState.mName is "1" and done is False:
                print("gg")
        if self.previousState is "0":
            if CFSM.mCurrentState.mName is "1" and done is False:
                print("gg")
        if self.previousState is "2":
            if CFSM.mCurrentState.mName is "1" and done is False:
                print("gg")
        """
        #if done is False:
        #    print(CFSM.mCurrentState.mName, self.previousState)
        ##다음 다리가 다 올라갔는데도 Stance Hip이 Contact이 안났을 경우
        ##이전 State가 0.. 즉 현재 State가 1이고, 아직 L이 Contact가 안됐을 경우
        """
        if self.previousState is "0" and not self.controller.LContact.isSatisfied():
            print("termination cause not contact")
            done = True
        elif self.previousState is "2" and not self.controller.RContact.isSatisfied():
            print("termination cause not contact")
            done = True
        """
        if self.previousState is "0" and self.Lcontact_first is False:
            ##Contact이 일어난것처럼 처리
            self.Rcontact_first=False
            self.Lcontact_first=True

            self.contactTimeQueue.enqueue(self.to_contact_counter_L)
            self.Lcontact_mean_step = np.round(self.contactTimeQueue.sum_all()/self.contactTimeQueue.count)
            assert self.Lcontact_mean_step > 0, "contact Time is under zero"

        ##이전 State가 2.. 즉 현재 State가 3이고, 아직 R이 Contact가 안됐을 경우
        elif self.previousState is "2" and self.Rcontact_first is False:
            ##Contact이 일어난거처럼 처리
            ##반대쪽발 컨택트 초기화
            self.Lcontact_first=False
            ##이쪽 발 Contact True로(첫 Contact가 일어났다는 뜻)
            self.Rcontact_first=True
            ##큐에 다리를 내릴때부터 Contact까지의 시간 저장
            self.contactTimeQueue.enqueue(self.to_contact_counter_R)
            ##평균값
            self.Rcontact_mean_step = np.round(self.contactTimeQueue.sum_all()/self.contactTimeQueue.count)
            assert self.Rcontact_mean_step > 0, "contact Time is under zero"



        self.StepCounterQueue.enqueue(n_frames)
        return done,n_frames
   
    def render(self):
        return

