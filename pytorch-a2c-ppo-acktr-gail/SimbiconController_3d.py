import pydart2 as pydart
import numpy as np
import math
import cMat
import copy
import State as st

class StateMachine():
    def __init__(self, name):
        self.mName=name
        self.mStates=[]
        self.mCurrentState = None
        self.mBeginTime = 0
        self.mFrame = 0
        self.mElapsedTime = 0.0
        self.mCurrentAction = None

    def resetWalkingMachine(self):
        self.mCurrentState = self.mStates[0] 
        self.mBeginTime = 0
        self.mFrame = 0
        self.mElapsedTime = 0.0

    def addState(self, state):
        if state is not None:
            self.mStates.append(state)
        else:
            print("inValid State")
            quit()

    def setInitialState(self, state):
        if state is not None:
            self.mCurrentState = state
        else:
            print("invalid State")
            quit()
    def end(self,currentTime):
        self.mEndTime = currentTime 

    def begin(self,currentTime):
        self.mBeginTime = currentTime
        self.mFrame = 0
        self.mElapsedTime = 0.0
    
    def computeControlForce_state(self,dt):
        if self.mCurrentState is not None:
            type(self.mCurrentState)
            #print(self.mCurrentState)
            #print("....")
            #quit()
            #self.mCurrentState.isTerminalConditionSatisfied()
            self.mCurrentState.computeControlForce(dt)

            self.mElapsedTime = self.mElapsedTime+dt

            self.mFrame = self.mFrame+1
        
            booleanS = self.mCurrentState.isTerminalConditionSatisfied()

            if booleanS is True:
                #quit()
                #print(self.mCurrentState.mName)
                #input()
                self.transiteTo(self.mCurrentState.getNextState(), self.mBeginTime+self.mElapsedTime)
        else:
            print("Error.. inVaild current State.")

    def setTrainedDesiredAction(self,action,env):
        #print(self)
        #print(self.mStates)
        if self.mStates[0] is not None:
            #print("mStates")
            wState0 = self.mStates[0]
            wState1 = self.mStates[1]
            wState2 = self.mStates[2]
            wState3 = self.mStates[3]
            if int(self.mCurrentState.mName) is 0:
                tCond0 = st.TimerCondition(wState0, action[20])
                wState0.setTerminalCondition(tCond0)
                #tCond1 = st.TimerCondition(wState1, action[10]*(1-action[11]))
                #wState1.setTerminalCondition(tCond1)
            if int(self.mCurrentState.mName) is 2:
                tCond2 = st.TimerCondition(wState2, action[20])
                wState2.setTerminalCondition(tCond2)
                #tCond3 = st.TimerCondition(wState3, action[10]*(1-action[11]))
                #wState3.setTerminalCondition(tCond3)

            swh02 = action[0] #양수 드는거
            swk02 = action[1] #음수 반대로 꺾임
            swa02 = action[2] #양수 발목 위로
            swh13 = action[3]
            swk13 = action[4]
            swa13 = action[5]

            stk02=  action[6]
            sta02 = action[7]
            stk13=  action[8]
            sta13 = action[9]

            swhx02 = action[10]
            swhx13 = action[11]

            swhz02=action[12]
            swhz13=action[13]
            sthx02=action[14]
            sthy02=action[15]
            sthz02=action[16]
            sthx13=action[17]
            sthy13=action[18]
            sthz13=action[19]

            pelvis02 = action[21]
            pelvis13 = action[22]

            #양수가 앞으로 숙인다. 하지만 이건 제ㅚ하기로했다
            wState0.setDesiredJointPosition("j_abdomen_2", pelvis02)
            wState1.setDesiredJointPosition("j_abdomen_2", pelvis13)
            wState2.setDesiredJointPosition("j_abdomen_2", pelvis02)
            wState3.setDesiredJointPosition("j_abdomen_2", pelvis13)


            #00
            wState0.setDesiredJointPosition("j_thigh_right_z", swh02) # +가 다리 올리는거
            wState0.setDesiredJointPosition("j_shin_right", swk02) # +면 반대로 꺽임 
            wState0.setDesiredJointPosition("j_heel_right_1", swa02) # +면 발목위로

            wState0.setDesiredJointPosition("j_thigh_right_x", swhx02) # +면 다리 안쪽으로 모음 
            wState0.setDesiredJointPosition("j_thigh_right_y", swhz02) # 다리 돌리는거겠지
            #Stance Hip control
            wState0.setDesiredJointPosition("j_thigh_left_x", sthx02)
            wState0.setDesiredJointPosition("j_thigh_left_z", sthy02)
            #print(sthy02)
            wState0.setDesiredJointPosition("j_thigh_left_y", sthz02)
            wState0.setDesiredJointPosition("j_shin_left", stk02)
            wState0.setDesiredJointPosition("j_heel_left_1",sta02)
            ##########################################################

            wState1.setDesiredJointPosition("j_thigh_right_z", swh13)
            wState1.setDesiredJointPosition("j_shin_right", swk13)
            wState1.setDesiredJointPosition("j_heel_right_1", swa13)

            wState1.setDesiredJointPosition("j_thigh_right_x", swhx13)
            wState1.setDesiredJointPosition("j_thigh_right_y", swhz13)

            #Stance Hip control
            wState1.setDesiredJointPosition("j_thigh_left_x", sthx13)
            wState1.setDesiredJointPosition("j_thigh_left_z", sthy13)
            wState1.setDesiredJointPosition("j_thigh_left_y", sthz13)
            wState1.setDesiredJointPosition("j_shin_left", stk13)
            wState1.setDesiredJointPosition("j_heel_left_1", sta13)
            ############################################################
            ##22
            wState2.setDesiredJointPosition("j_thigh_left_z", swh02)
            wState2.setDesiredJointPosition("j_shin_left", swk02)
            wState2.setDesiredJointPosition("j_heel_left_1", swa02)

            wState2.setDesiredJointPosition("j_thigh_left_x", swhx02)
            wState2.setDesiredJointPosition("j_thigh_left_y", swhz02)

            #Stance Hip control
            wState2.setDesiredJointPosition("j_thigh_right_x", sthx02)
            wState2.setDesiredJointPosition("j_thigh_right_z", sthy02)
            wState2.setDesiredJointPosition("j_thigh_right_y", sthz02)
            wState2.setDesiredJointPosition("j_shin_right", stk02)
            wState2.setDesiredJointPosition("j_heel_right_1",sta02)
            ############################################################
            wState3.setDesiredJointPosition("j_thigh_left_z", swh13)
            wState3.setDesiredJointPosition("j_shin_left", swk13)
            wState3.setDesiredJointPosition("j_heel_left_1", swa13)

            wState3.setDesiredJointPosition("j_thigh_left_x", swhx13)
            wState3.setDesiredJointPosition("j_thigh_left_y", swhz13)

            #Stance Hip control
            wState3.setDesiredJointPosition("j_thigh_right_x", sthx13)
            wState3.setDesiredJointPosition("j_thigh_right_z", sthy13)
            wState3.setDesiredJointPosition("j_thigh_right_y", sthz13)
            wState3.setDesiredJointPosition("j_shin_right", stk13)
            wState3.setDesiredJointPosition("j_heel_right_1", sta13)



    def transiteTo(self, _state, _currentTime):
            self.mCurrentState.end(_currentTime)
            self.mCurrentState = _state

            self.mCurrentState.begin(_currentTime)

            #print("state is Changed to", _state.mName)
            #input()

    def getName(self):
        return self.mName

    def returnDesiredJP(self):
        return self.mCurrentAction 

class Controller():
    def __init__(self, skel,world, collisionSolver=None): 
        self.mStateMachines = []
        self.mSkel=skel
        self.mCurrentStateMachine = None
        self.mWorld=world

        self.mCoronalLeftHip = self.mSkel.dof("j_thigh_left_x").index_in_skeleton()
        self.mSagitalLeftHip = self.mSkel.dof("j_thigh_left_z").index_in_skeleton()

        self.mCoronalRightHip = self.mSkel.dof("j_thigh_right_x").index_in_skeleton()
        self.mSagitalRightHip = self.mSkel.dof("j_thigh_right_z").index_in_skeleton()

        self._buildStateMachine()
        self._setJointDamping()
        
    def _buildStateMachine(self):
        #walking Controller
        Njoints = self.mSkel.num_joints()
        for i in range(0,Njoints):
            print(self.mSkel.joints[i].name)

        dofs = self.mSkel.num_dofs()
        for i in range(0, dofs):
            print(self.mSkel.dofs[i].name)


        self.mStateMachines.append(self._createWalkingStateMachine())

        #set Initial Controller
        self.mCurrentStateMachine = self.mStateMachines[0]
        #begin default Controller
        self.mCurrentStateMachine.begin(0.0)


    def changeStateMachine(self,StateMachine, _currentTime):
        if self.mCurrentStateMachine is StateMachine:
            return

        pN = self.mCurrentStateMachine.getName()
        nN = StateMachine.getName()

        self.mCurrentStateMachine.end(_currentTime)

        self.mCurrentStateMachine = StateMachine
        self.mCurrentStateMachine.begin(_currentTime)
        
        input()

    def _createWalkingStateMachine(self):
        cd = -0.5
        cv = -0.2

        pelvis = math.radians(0.0)

        swh02 = 0.5
        swk02 = -1.10
        swa02 = 0.6
        stk02= -0.05
        sta02 = 0.0

        swh13 = -0.2
        swk13 = -0.05
        swa13 = 0.15
        stk13 = -0.1
        sta13 = 0
        self.walkingMachine = StateMachine("walking")
        self.walkingMachine.mCurrentAction = np.array([0.5,-1.10,0.6, -0.05, 0, -0.1, -0.05, 0.15, -0.1, 0.0])

        wState0 = st.State(self.mSkel, "0")
        wState1 = st.State(self.mSkel,"1")
        wState2 = st.State(self.mSkel, "2")
        wState3 = st.State(self.mSkel, "3")
        
        #기본값 0.1
        tCond0 = st.TimerCondition(wState0, 0.3)
        tCond1 = st.CollisionCondition(wState1, self.mWorld,self._getRightFoot())
        self.RContact = tCond1
        #tCond1 = st.TimerCondition(wState1, 1/30)
        tCond2 = st.TimerCondition(wState2, 0.3)
        tCond3 = st.CollisionCondition(wState3,self.mWorld,self._getLeftFoot())
        self.LContact = tCond3
        #tCond3 = st.TimerCondition(wState3, 1/30)
        #tCond5 = st.TimerCondition(wState1, 1)
        #tCond6 = st.TimerCondition(wState3, 1)

        wState0.setTerminalCondition(tCond0)
        wState1.setTerminalCondition(tCond1)
        wState2.setTerminalCondition(tCond2)
        wState3.setTerminalCondition(tCond3)

        wState0.setNextState(wState1)
        wState1.setNextState(wState2)
        wState2.setNextState(wState3)
        wState3.setNextState(wState0)

        wState0.setStanceFootToLeft()
        wState1.setStanceFootToLeft()
        wState2.setStanceFootToRight()
        wState3.setStanceFootToRight()

        wState0.setDesiredPelvisGlobalAngleOnSagital(math.radians(0.0))
        wState1.setDesiredPelvisGlobalAngleOnSagital(math.radians(0.0))
        wState2.setDesiredPelvisGlobalAngleOnSagital(math.radians(0.0))
        wState3.setDesiredPelvisGlobalAngleOnSagital(math.radians(0.0))


        wState0.setDesiredPelvisGlobalAngleOnCoronal(math.radians(0.0))
        wState1.setDesiredPelvisGlobalAngleOnCoronal(math.radians(0.0))
        wState2.setDesiredPelvisGlobalAngleOnCoronal(math.radians(0.0))
        wState3.setDesiredPelvisGlobalAngleOnCoronal(math.radians(0.0))


        # wState0.setDesiredJointPosition("j_abdomen_1", -pelvis)

        # wState0.setDesiredJointPosition("j_thigh_right_y", -swh02)
        # wState0.setDesiredJointPosition("j_shin_right", -swk02)
        # wState0.setDesiredJointPosition("j_heel_right_1", -swa02)

        # wState0.setDesiredJointPosition("j_shin_left", -stk02)
        # wState0.setDesiredJointPosition("j_heel_left_1",-sta02)

        # wState0.setDesiredJointPosition("j_bicep_left_x", math.radians(-20.0))
        # wState0.setDesiredJointPosition("j_bicep_right_x", math.radians(10.0)) 
        # wState0.setDesiredJointPosition("j_bicep_left_x", math.radians(-80.00))   
        # wState0.setDesiredJointPosition("j_bicep_right_x",math.radians(80.00))
        # #3D
        # wState0.setDesiredJointPosition("j_thigh_right_x", math.radians(0.0))

        wState0.setDesiredJointPosition("j_abdomen_2", pelvis)

        #wState0.setDesiredJointPosition("j_thigh_right_y", -swh02)
        wState0.setDesiredJointPosition("j_thigh_right_z", swh02)
        wState0.setDesiredJointPosition("j_shin_right", swk02)
        wState0.setDesiredJointPosition("j_heel_right_1", swa02)

        wState0.setDesiredJointPosition("j_shin_left", stk02)
        wState0.setDesiredJointPosition("j_heel_left_1",sta02)

        # wState0.setDesiredJointPosition("j_bicep_left_z", math.radians(-20.0))
        # wState0.setDesiredJointPosition("j_bicep_right_z", math.radians(10.0)) 
        # wState0.setDesiredJointPosition("j_bicep_left_y", math.radians(80.00))
        # wState0.setDesiredJointPosition("j_bicep_right_y",-math.radians(80.00))
        #3D
        wState0.setDesiredJointPosition("j_thigh_right_x", math.radians(0.0))

        
        wState0.setFeedBackCoronalCOMDistance(self.mCoronalLeftHip,-cd)
        wState0.setFeedBackCoronalCOMVelocity(self.mCoronalLeftHip,-cv)
        wState0.setFeedBackCoronalCOMDistance(self.mCoronalRightHip,-cd)
        wState0.setFeedBackCoronalCOMVelocity(self.mCoronalRightHip,-cv)
        wState0.setFeedBackSagitalCOMDistance(self.mSagitalLeftHip,-cd)
        wState0.setFeedBackSagitalCOMVelocity(self.mSagitalLeftHip,-cv)
        wState0.setFeedBackSagitalCOMDistance(self.mSagitalRightHip,-cd)
        wState0.setFeedBackCoronalCOMVelocity(self.mSagitalRightHip,-cv)

        #state1

        wState1.setDesiredJointPosition("j_abdomen_2", pelvis)

        wState1.setDesiredJointPosition("j_thigh_left_z", -swh13)
        wState1.setDesiredJointPosition("j_shin_left", swk13)
        wState1.setDesiredJointPosition("j_heel_left_1", swa13)

        wState1.setDesiredJointPosition("j_shin_right", stk13)
        wState1.setDesiredJointPosition("j_heel_right_1", sta13)

        # wState1.setDesiredJointPosition("j_bicep_right_z", math.radians(20.0))
        # wState1.setDesiredJointPosition("j_bicep_left_z", math.radians(-10.0)) 
        # wState1.setDesiredJointPosition("j_bicep_right_y", math.radians(80.00))
        # wState1.setDesiredJointPosition("j_bicep_left_y",-math.radians(80.00))

        #3D
        wState1.setDesiredJointPosition("j_thigh_left_x", math.radians(0.0))

        wState1.setFeedBackCoronalCOMDistance(self.mCoronalLeftHip,-cd)
        wState1.setFeedBackCoronalCOMVelocity(self.mCoronalLeftHip,-cv)
        wState1.setFeedBackCoronalCOMDistance(self.mCoronalRightHip,-cd)
        wState1.setFeedBackCoronalCOMVelocity(self.mCoronalRightHip,-cv)
        wState1.setFeedBackSagitalCOMDistance(self.mSagitalLeftHip,-cd)
        wState1.setFeedBackSagitalCOMVelocity(self.mSagitalLeftHip,-cv)
        wState1.setFeedBackSagitalCOMDistance(self.mSagitalRightHip,-cd)
        wState1.setFeedBackCoronalCOMVelocity(self.mSagitalRightHip,-cv)

      #State2
        wState2.setDesiredJointPosition("j_abdomen_2", pelvis)

        wState2.setDesiredJointPosition("j_thigh_left_z", swh02)
        wState2.setDesiredJointPosition("j_shin_left", swk02)
        wState2.setDesiredJointPosition("j_heel_left_1", swa02)

        wState2.setDesiredJointPosition("j_shin_right", stk02)
        wState2.setDesiredJointPosition("j_heel_right_1",sta02)

        # wState2.setDesiredJointPosition("j_bicep_right_z", math.radians(20.0))
        # wState2.setDesiredJointPosition("j_bicep_left_z", math.radians(-10.0)) 
        # wState2.setDesiredJointPosition("j_bicep_left_y", math.radians(80.00))
        # wState2.setDesiredJointPosition("j_bicep_right_y",-math.radians(80.00))

        #3D
        wState2.setDesiredJointPosition("j_thigh_left_x", math.radians(0.0))

        wState2.setFeedBackCoronalCOMDistance(self.mCoronalLeftHip,-cd)
        wState2.setFeedBackCoronalCOMVelocity(self.mCoronalLeftHip,-cv)
        wState2.setFeedBackCoronalCOMDistance(self.mCoronalRightHip,-cd)
        wState2.setFeedBackCoronalCOMVelocity(self.mCoronalRightHip,-cv)
        wState2.setFeedBackSagitalCOMDistance(self.mSagitalLeftHip,-cd)
        wState2.setFeedBackSagitalCOMVelocity(self.mSagitalLeftHip,-cv)
        wState2.setFeedBackSagitalCOMDistance(self.mSagitalRightHip,-cd)
        wState2.setFeedBackCoronalCOMVelocity(self.mSagitalRightHip,-cv)

        #State3

        wState3.setDesiredJointPosition("j_abdomen_2", pelvis)

        wState3.setDesiredJointPosition("j_thigh_right_z", swh13)
        wState3.setDesiredJointPosition("j_shin_right", swk13)
        wState3.setDesiredJointPosition("j_heel_right_1", swa13)

        wState3.setDesiredJointPosition("j_shin_left", stk13)
        wState3.setDesiredJointPosition("j_heel_left_1",sta13)

        # wState3.setDesiredJointPosition("j_bicep_left_z", math.radians(20.0))
        # wState3.setDesiredJointPosition("j_bicep_right_z", math.radians(-10.0)) 
        # wState3.setDesiredJointPosition("j_bicep_left_y", math.radians(80.00))
        # wState3.setDesiredJointPosition("j_bicep_right_y",-math.radians(80.00))

        #3D
        wState3.setDesiredJointPosition("j_thigh_right_x", math.radians(0.0))

        wState3.setFeedBackCoronalCOMDistance(self.mCoronalLeftHip,-cd)
        wState3.setFeedBackCoronalCOMVelocity(self.mCoronalLeftHip,-cv)
        wState3.setFeedBackCoronalCOMDistance(self.mCoronalRightHip,-cd)
        wState3.setFeedBackCoronalCOMVelocity(self.mCoronalRightHip,-cv)
        wState3.setFeedBackSagitalCOMDistance(self.mSagitalLeftHip,-cd)
        wState3.setFeedBackSagitalCOMVelocity(self.mSagitalLeftHip,-cv)
        wState3.setFeedBackSagitalCOMDistance(self.mSagitalRightHip,-cd)
        wState3.setFeedBackCoronalCOMVelocity(self.mSagitalRightHip,-cv)

        self.walkingMachine.addState(wState0)
        self.walkingMachine.addState(wState1)
        self.walkingMachine.addState(wState2)
        self.walkingMachine.addState(wState3)

        self.walkingMachine.setInitialState(wState0)

        return self.walkingMachine

    def _setJointDamping(self):
        for i in range(1,self.mSkel.num_bodynodes()):
            joint = self.mSkel.joint(i)
            for j in range(0,joint.num_dofs()):
                joint.set_damping_coefficient(j, 120.0)
     
    def update(self):
        #check After
        self.mCurrentStateMachine.computeControlForce_state(1/900)
    
    """
    def update(self,action,env):
        if action is not None:
            self.mCurrentStateMachine.mCurrentAction = action
            self.mCurrentStateMachine.setTrainedDesiredAction(action,env)
        self.mCurrentStateMachine.computeControlForce_state(1/900)
    """
    def _getLeftFoot(self):
        return [self.mSkel.body("h_toe_left"), self.mSkel.body("h_heel_left")]

    def _getRightFoot(self):
        return [self.mSkel.body("h_toe_right"), self.mSkel.body("h_heel_right")]
