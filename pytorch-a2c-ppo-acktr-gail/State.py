import math
import numpy as np
import SimbiconController_3d
import cMat
import copy
import pydart2 as pydart
import cv2

class State():
    def __init__(self,skel,name):
        mSkel = skel
        self.mName=name
        self.mSkel=skel
        self.mNextState = self
        self.mBeginTime = 0.0
        self.mEndTime = 0.0
        self.mFrame = 0
        self.mElapsedTime = 0
        self.mDesiredGlobalSwingLegAngleOnSagital=0.0
        self.mDesiredGlobalSwingLegAngleOnCoronal=0.0
        self.mDesiredGlobalPelvisAngleOnSagital=0.0
        self.mDesiredGlobalPelvisAngleOnCoronal=0.0
        self.mDesiredGlobalPelvisAngleOnTransverse=0.0

        self.numDof = self.mSkel.num_dofs()
        self.mDesiredJointPosition = np.zeros(self.numDof)
        self.mSagitalCd = np.zeros(self.numDof)
        self.mSagitalCv = np.zeros(self.numDof)
        self.mCoronalCd = np.zeros(self.numDof)
        self.mCoronalCv = np.zeros(self.numDof) 

        #for atlas
        self.mCoronalLeftHipDOFIndex = self.mSkel.dof("j_thigh_left_x").index_in_skeleton()
        self.mCoronalRightHipDOFIndex = self.mSkel.dof("j_thigh_right_x").index_in_skeleton()
        self.mSagitalLeftHipDOFIndex = self.mSkel.dof("j_thigh_left_z").index_in_skeleton()
        self.mSagitalRightHipDOFIndex = self.mSkel.dof("j_thigh_right_z").index_in_skeleton()
        self.mTransverseLeftHipDOFIndex = self.mSkel.dof("j_thigh_left_y").index_in_skeleton()
        self.mTransverseRightHipDOFIndex = self.mSkel.dof("j_thigh_right_y").index_in_skeleton()
        self.mlay = self.mSkel.dof("j_bicep_left_y").index_in_skeleton()
        self.mray = self.mSkel.dof("j_bicep_right_y").index_in_skeleton()
        self.mlax = self.mSkel.dof("j_bicep_left_x").index_in_skeleton()
        self.mrax = self.mSkel.dof("j_bicep_right_x").index_in_skeleton()
        #self.mPelvis = self.mSkel.dof("pelvis").index_in_skeleton()
        self.mTorque = np.zeros(self.numDof)
        
        self.mKp = np.zeros(self.numDof)
        self.mKd = np.zeros(self.numDof)    

        for i in range(0,self.numDof):
            self.mKp[i] = 650
            self.mKd[i] = 1


        self.mDesiredJointPositionBalance = np.zeros(self.numDof)


        self.mLeftFoot = self.mSkel.body("h_heel_left")
        self.mRightFoot = self.mSkel.body("h_heel_right")

        self.mStanceFoot = self.mLeftFoot

        self.mRootKp = 0
        self.mRootKd = 0

        self.test = 0

    def setFeedBackCoronalCOMDistance(self,index,val):
        self.mCoronalCd[index] = val

    def setFeedBackCoronalCOMVelocity(self,index,val):
        self.mCoronalCv[index] = val

    def setFeedBackSagitalCOMDistance(self,index,val):
        self.mSagitalCd[index] = val
    
    def setFeedBackSagitalCOMVelocity(self,index,val):
        self.mSagitalCv[index] = val

    def setDesiredPelvisGlobalAngleOnSagital(self,val):
        self.mDesiredGlobalPelvisAngleOnSagital = val

    def setDesiredPelvisGlobalAngleOnCoronal(self,val):
        self.mDesiredGlobalPelvisAngleOnCoronal = val

    def setDesiredPelvisGlobalAngleOnTransverse(self,val):
        self.mDesiredGlobalPelvisAngleOnTransverse = val

    def setStanceFootToLeft(self):
        self.mStanceFoot = self.mLeftFoot

    def setStanceFootToRight(self):
        self.mStanceFoot = self.mRightFoot

    def getNextState(self):
        return self.mNextState
    def end(self,_currentTime):
        self.mEndTime = _currentTime

    def begin(self, _currentTime):
        self.mBeginTime = _currentTime
        self.mFrame = 0
        self.mElapsedTime = 0.0

    def setTerminalCondition(self,terminalCondition):
        self.mTerminalCondition = terminalCondition 

    def setNextState(self,nextState):
        self.mNextState=nextState

    def setDesiredJointPosition(self,jointName, value):
        index=self.mSkel.dof_index(jointName)
        self.mDesiredJointPosition[index] = value
    
    def computeControlForce(self, timestep):
        #dofs = self.mSkel.num_dofs()
        q = self.mSkel.q
        dq = self.mSkel.dq
        getCOMFrameLinear = cMat.Matrix.linear(self.getCOMFrame())
        self.getStanceAnklePosition()
        
        self.mDesiredJointPositionBalance = self.mDesiredJointPosition + self.getSagitalCOMDistance(getCOMFrameLinear) * self.mSagitalCd + self.getSagitalCOMVelocity(getCOMFrameLinear) * self.mSagitalCv + self.getCoronalCOMDistance(getCOMFrameLinear) * self.mCoronalCd + self.getCoronalCOMVelocity(getCOMFrameLinear) * self.mCoronalCv
        #self.mDesiredJointPositionBalance = self.mDesiredJointPosition

        self.mTorque[0:6] = 0
        for i in range(6,self.numDof):
            self.mTorque[i] = -self.mKp[i] * (q[i] - self.mDesiredJointPositionBalance[i]) - self.mKd[i]* dq[i]
            #self.mTorque[i] = 0.5*self.mTorque[i]
            #self.mTorque[i] = -self.mTorque[i]
            #print(self.mTorque[i])
        #------------------------------------
        
        #print("qq",q[self.mlax], q[self.mlay])
        #print("dd",self.mDesiredJointPosition[self.mlax],self.mDesiredJointPosition[self.mlay])
        #print("md",self.mDesiredJointPositionBalance[self.mlax])
        #print("torq", self.mTorque[self.mlax],self.mTorque[self.mlay])
        #if self.test is not 0:
            #g = self.mSkel.tau
            #print("tau",self.mSkel.forces()[self.mlax])
        #else:
        #   self.test = 1
        #---------------------------------------------------
        #input()
        #print("mTorqueprev", self.mTorque)

        
        
        ##비활성 
        #self._updateTorqueForStanceLeg(getCOMFrameLinear)

        
        
        
        #print("mTorque",self.mTorque)
        #print("dofs",dofs)
        #input("torq")
        #self.mTorque = self.mTorque * 0.1

        self.mSkel.set_forces(self.mTorque)

        self.mElapsedTime = self.mElapsedTime + timestep
        self.mFrame = self.mFrame + 1
        #name = input() 
        #print("Torque",self.mTorque)

    ##Case: Action == feedback gain //
    """
    def computeControlForce(self, timestep,action):
        dofs = self.mSkel.num_dofs()
        q = self.mSkel.q
        dq = self.mSkel.dq

        ##RLCODE
        ##action = action.cpu().numpy()
        self.mSagitalCd[26] = action[0]
        self.mSagitalCd[16] = action[0]
        self.mSagitalCv[26] = action[1]
        self.mSagitalCv[16] = action[1]
        self.mCoronalCd[26] = action[2]
        self.mCoronalCd[16] = action[2]
        self.mCoronalCv[16] = action[3]
        self.mCoronalCv[26] = action[3]
        
        #self.mSagitalCd[26] = -0.3
        #self.mSagitalCd[16] = -0.3
        #print(self.mSagitalCd[26],self.mSagitalCd[16])
        #print(self.mSagitalCv[26],self.mSagitalCv[16])
        #print(self.mCoronalCd[26],self.mCoronalCd[16])
        #print(self.mCoronalCv[26],self.mCoronalCv[16])
        #print("_____________________________________")


        self.mDesiredJointPositionBalance = self.mDesiredJointPosition + self.getSagitalCOMDistance() * self.mSagitalCd + self.getSagitalCOMVelocity() * self.mSagitalCv + self.getCoronalCOMDistance() * self.mCoronalCd + self.getCoronalCOMVelocity() * self.mCoronalCv 
        #print(self.mCoronalCv,"ggg",self.mSagitalCd.size)
        #input("sagital")
        
        self.mTorque[0:6] = 0
        #if action is None:
        for i in range(6,dofs):
            self.mTorque[i] = -self.mKp[i] * (q[i] - self.mDesiredJointPositionBalance[i]) - self.mKd[i]* dq[i]
        self._updateTorqueForStanceLeg(action)
        self.mSkel.set_forces(self.mTorque)

        self.mElapsedTime = self.mElapsedTime + timestep
        self.mFrame = self.mFrame + 1
    """

    def _updateTorqueForStanceLeg(self,getCOMFrameLinear):
        ##RLCODE
        #action = action.cpu().numpy()
        #print(action)
        ##
        #미리 계산해두기
        comY = cMat.Matrix.col(getCOMFrameLinear,1)
        #pelvisY = cMat.Matrix.col(cMat.Matrix.linear(self.mSkel.body("pelvis").T),2)
        
        q = self.mSkel.q
        dq = self.mSkel.dq

        action = self.mDesiredGlobalPelvisAngleOnTransverse
        #action = 0
        #print(action)

        defaultRot = np.zeros(3)
        defaultRot[0] = -0.5*np.pi
        defaultRot[1] = 0
        defaultRot[2] = 0
        rm_qRc = cv2.Rodrigues(q[0:3])[0]
        rm_qRd1 = cv2.Rodrigues(defaultRot)[0]
        rm_qRd2 = cv2.Rodrigues(action*comY)[0]
        pos_d = -cv2.Rodrigues(rm_qRc.T@rm_qRd2@rm_qRd1)[0]
        #pos_d = self.mSkel.position_differences(q, desiredq)

        if self.mStanceFoot is self.mLeftFoot:
            #input()
            #pelvisSagitalAngle = self.getSagitalPelvisAngle(getCOMFrameLinear,comY,pelvisZ)
            #tauTorsoSagital = -5000.0 * (pelvisSagitalAngle + self.mDesiredGlobalPelvisAngleOnSagital)
            tauTorsoSagital = -5000.0*pos_d[1]

            #coronal left 10
            #coronal right 11
            #sagital left 13
            #sagital right 14

            ##RLCODE
            #self.mTorque[self.mSagitalRightHipDOFIndex] = action[0] 
            ##
            self.mTorque[self.mSagitalLeftHipDOFIndex] = tauTorsoSagital - self.mTorque[self.mSagitalRightHipDOFIndex]

            #print("sagitalTorque",self.mTorque[self.mSagitalLeftHipDOFIndex])


            #pelvisCoronalAngle = self.getCoronalPelvisAngle(getCOMFrameLinear,comY,pelvisZ)
            #tauTorsoCoronal = -5000.0 * (pelvisCoronalAngle - self.mDesiredGlobalPelvisAngleOnCoronal)
            tauTorsoCoronal = -5000.0*pos_d[0] 
            ##RLCODE
            #self.mTorque[self.mCoronalRightHipDOFIndex] = action[1]
            ##
            self.mTorque[self.mCoronalLeftHipDOFIndex] = -tauTorsoCoronal - self.mTorque[self.mCoronalRightHipDOFIndex]


            #tauTorsoTransverse = 1000.0*pos_d[2] - 10*dq[2]
            #if self.mRootKp > 0:
            #    tauTorsoTransverse = self.mRootKp*pos_d[2] - self.mRootKd*dq[2] 
            #    self.mTorque[self.mTransverseLeftHipDOFIndex] = (self.mRootKp/5000)*(tauTorsoTransverse - self.mTorque[self.mTransverseRightHipDOFIndex])

            # tauTorsoTransverse = 250*pos_d[2] - 25*dq[2] 
            # self.mTorque[self.mTransverseLeftHipDOFIndex] = (tauTorsoTransverse - self.mTorque[self.mTransverseRightHipDOFIndex])


            #print("coronalTorque",self.mTorque[self.mCoronalLeftHipDOFIndex])

            #print("mTLDOF",self.mTorque[self.mTransverseLeftHipDOFIndex])


        elif self.mStanceFoot is self.mRightFoot :
            #pelvisSagitalAngle = self.getSagitalPelvisAngle(getCOMFrameLinear,comY,pelvisZ)
            #tauTorsoSagital =-5000.0 * (pelvisSagitalAngle + self.mDesiredGlobalPelvisAngleOnSagital)
            tauTorsoSagital = -5000.0*pos_d[1]

            ##RLCODE
            #self.mTorque[self.mSagitalLeftHipDOFIndex] = action[0]
            ## 
            self.mTorque[self.mSagitalRightHipDOFIndex] = tauTorsoSagital - self.mTorque[self.mSagitalLeftHipDOFIndex]
            
            #print("srh", self.mTorque[self.mSagitalRightHipDOFIndex], "tts", tauTorsoSagital, "msl", self.mTorque[self.mSagitalLeftHipDOFIndex])

            #pelvisCoronalAngle = self.getCoronalPelvisAngle(getCOMFrameLinear,comY,pelvisZ)
            #tauTorsoCoronal = -5000.0 * (pelvisCoronalAngle - self.mDesiredGlobalPelvisAngleOnCoronal)
            tauTorsoCoronal = -5000.0*pos_d[0]
            ##RLCODE
            #self.mTorque[self.mCoronalLeftHipDOFIndex] = action[1]
            ##
            self.mTorque[self.mCoronalRightHipDOFIndex] = -tauTorsoCoronal - self.mTorque[self.mCoronalLeftHipDOFIndex]

            #print("crh",self.mTorque[self.mCoronalRightHipDOFIndex], "ttc" , tauTorsoCoronal, "mcL", self.mTorque[self.mCoronalLeftHipDOFIndex])
            #input()
            #if self.mRootKp > 0:
            #    tauTorsoTransverse = self.mRootKp*pos_d[2] - self.mRootKd*dq[2]
            #    self.mTorque[self.mTransverseRightHipDOFIndex] = (self.mRootKp/5000)*(tauTorsoTransverse - self.mTorque[self.mTransverseLeftHipDOFIndex])
           
            # tauTorsoTransverse = 250*pos_d[2] - 25*dq[2]
            # self.mTorque[self.mTransverseRightHipDOFIndex] = (tauTorsoTransverse - self.mTorque[self.mTransverseLeftHipDOFIndex])

            #print("mTRDOF",self.mTorque[self.mTransverseRightHipDOFIndex])

    
    def getSagitalPelvisAngle(self,getCOMFrameLinear,comY,pelvisZ):
        #comR = cMat.Matrix.linear(self.getCOMFrame())
        comR = getCOMFrameLinear
        #comY = cMat.Matrix.col(comR,1)

        #pelvisZ =cMat.Matrix.col(cMat.Matrix.linear(self.mSkel.body("pelvis").T),2)
        projPelvisZ =np.dot(np.transpose(comR),pelvisZ)
        projPelvisZ[2] = 0.0
        
        projPelvisZ = cMat.Matrix.normalize(projPelvisZ)

        angle = self._getAnglesBetweenTwoVectors(projPelvisZ, comY)

        #cross = np.cross(comY, projPelvisZ)
        cross = np.dot(self.skew(comY), projPelvisZ)
        if cross[2] > 0 :
            return angle
        else :
            return -angle

    def _getAnglesBetweenTwoVectors(self, v1, v2):
            sizee = cMat.Matrix.size(v1) * cMat.Matrix.size(v2)
            if sizee < 1e-6:
                print("sizee")
                print(v1)
                print(v2)
                input()
                theta = np.dot(v1,v2)
            else:
                #theta = np.dot(v1,v2)/ (cMat.Matrix.size(v1) * cMat.Matrix.size(v2))
                theta = np.dot(v1,v2) / sizee
                #theta = np.dot(v1,v2) / (np.dot(cMat.Matrix.normalize(v1),cMat.Matrix.normalize(v2)))
            theta = np.clip(theta, -1, 1)

            return math.acos(theta)
            #return math.acos(np.dot(v1,v2)/(np.dot(cMat.Matrix.normalize(v1),cMat.Matrix.normalize(v2))))
            #return math.acos(np.dot(v1,v2)/((cMat.Matrix.normalize(v1)) * (cMat.Matrix.normalize(v2))))
            #return math.acos( (np.dot(v1,v2)) / (cMat.Matrix.size(v1) * cMat.Matrix.size(v2)) )
            #check this


    def getCoronalPelvisAngle(self,getCOMFrameLinear,comY,pelvisZ):
        #input()
        #comR = cMat.Matrix.linear(self.getCOMFrame())
        comR = getCOMFrameLinear
        #comY = cMat.Matrix.col(comR,1)
        #pelvisZ = cMat.Matrix.col(cMat.Matrix.linear(self.mSkel.body("pelvis").T),2)

        projPelvisZ = np.dot(np.transpose(comR),pelvisZ)

        projPelvisZ[0] = 0.0
        projPelvisZ = cMat.Matrix.normalize(projPelvisZ)

        angle = self._getAnglesBetweenTwoVectors(projPelvisZ, comY)

        #print(projPelvisZ)
        #print(comY)

        #cross = np.cross(comY,projPelvisZ)
        cross = np.dot(self.skew(comY), projPelvisZ)

        #print(cross)
        #input()

        if cross[0] > 0 :
            return angle
        else :
            return -angle


    def skew(self,x):
        return np.array([[0, -x[2], x[1]],
                        [x[2], 0, -x[0]],
                        [-x[1], x[0], 0]])


    def getSagitalCOMDistance(self,getCOMFrameLinear):
        #ll = cMat.Matrix.linear(self.getCOMFrame())
        ll = getCOMFrameLinear
        xAxis = cMat.Matrix.col(ll,0)
       
        #print("getCOM",self.getCOM())
        #print("gsAP",self.getStanceAnklePosition())
        #d = self.getCOM() - self.getStanceAnklePosition()
        d = self.getCOM() - self.returnStanceAnklePositionValue()

        return np.dot(d,xAxis)

    def getSagitalCOMVelocity(self,getCOMFrameLinear): 
        xAxisl = getCOMFrameLinear
        #xAxisl = cMat.Matrix.linear(self.getCOMFrame())
        xAxis = cMat.Matrix.col(xAxisl,0)

        v = self.getCOMVelocity()

        return np.dot(v,xAxis)

    def getCoronalCOMDistance(self,getCOMFrameLinear): 
        YAxis = getCOMFrameLinear
        #YAxis = cMat.Matrix.linear(self.getCOMFrame())
        YAxis = cMat.Matrix.col(YAxis,2)

        #d = self.getCOM() - self.getStanceAnklePosition()
        d = self.getCOM() - self.returnStanceAnklePositionValue()
        return np.dot(d,YAxis)

    def getCoronalCOMVelocity(self,getCOMFrameLinear):
        YAxis = getCOMFrameLinear
        #YAxis = cMat.Matrix.linear(self.getCOMFrame())
        YAxis = cMat.Matrix.col(YAxis,2)

        v = self.getCOMVelocity()

        return np.dot(v,YAxis)
        
    def getCOMFrame(self):
        T = cMat.Matrix.identityMat4x4()

        yAxis = cMat.Matrix.UnitY()

        pelvisAxis = self.mSkel.body("h_pelvis").T

        pelvisXAxis = cMat.Matrix.linear(pelvisAxis)
        pelvisXAxis = cMat.Matrix.col(pelvisXAxis,0)

        mag = np.dot(yAxis, pelvisXAxis)
        #pelvisXAxis = copy.deepcopy(pelvisXAxis - mag*yAxis)
        pelvisXAxis = pelvisXAxis - mag*yAxis

        xAxis = cMat.Matrix.normalize_2D(pelvisXAxis)
        #zAxis = np.cross(xAxis,yAxis[0]);
        zAxis = np.dot(self.skew(xAxis), yAxis[0])

        trans = self.mSkel.com()
        cMat.Matrix.setTranslation(T,trans)
        cMat.Matrix.setlinearCol(T,0,xAxis)
        cMat.Matrix.setlinearCol(T,1,yAxis)
        cMat.Matrix.setlinearCol(T,2,zAxis)
        return T

    def getCOM(self):
        return self.mSkel.com()

    def getCOMVelocity(self):
        return self.mSkel.com_velocity()
    
    def returnStanceAnklePositionValue(self):
        return self.mStanceAnklePosition

    def getStanceAnklePosition(self):
        if self.mStanceFoot is None:
            self.mStanceAnklePosition = self.getCOM()
        else:
            self.mStanceAnklePosition = self._getJointPosition(self.mStanceFoot) 

    def _getJointPosition(self,node):
        parentJoint = node.parent_joint
        localJointPosiion = parentJoint.transform_from_child_body_node()
        localJointPosiion = cMat.Matrix.getTranslation(localJointPosiion)

        #checkthis..
        localJointPosiion = np.append(localJointPosiion,1.0)
        #print("ljp",localJointPosiion)

        npdot = np.dot(node.T,localJointPosiion)
        return npdot[0:3]

    def getElapsedTime(self):
        return self.mElapsedTime

    def isTerminalConditionSatisfied(self,):
        if self.mTerminalCondition is not None:
            return self.mTerminalCondition.isSatisfied()

    def setValues(self, abdoman, hip_x, hip_y, hip_z, knee, ankle, ):
        return


class TerminalCondition():
    def __init__(self,state):
        self.mState=state
        self.mDuration=0
    
class TimerCondition(TerminalCondition):
    def __init__(self,state,duration):
        super().__init__(state)
        self.mDuration=duration
        #print(self.mDuration)
    def isSatisfied(self):
        #prie*_*, ..., sep, end, file, flush) ?!u
        #print(self.mState.getElapsedTime())
        #print(self.mDuration)
        
        if self.mState.getElapsedTime() > self.mDuration:
            #print("TimeCOME")
            #input()
            return True
        else:
            return False

class CollisionCondition(TerminalCondition):
    def __init__(self,state,world,bodynodes):
        super().__init__(state)
        self.collisionChecker = pydart.collision_result.CollisionResult(world)
        self.mDuration = 0.3
        self.mbodynodes = bodynodes
        self.checkOut = 0
        #print(bodynode)
        #input()
         
    def isSatisfied(self):
        self.collisionChecker.update()
        #print("collisionChecker", self.collisionChecker.num_contacted_bodies())
        #print(self.collisionChecker.contacted_bodies)
        #print("elapsedTime",self.mState.getElapsedTime())
        #input()
        #if self.collisionChecker.num_contacted_bodies() > 2:
        #if self.mState.getElapsedTime() > self.mDuration:
        #    print("eliminated by duration")
        #    return True
        """
        if self.mbodynode in self.collisionChecker.contacted_bodies:
            if len(self.collisionChecker.contacted_bodies) < 2:
                return False
            else:
                return True

        """
        for bodynode in self.mbodynodes:
            #print(self.collisionChecker.contacted_bodies, "checkOut:", self.checkOut, "bodyNode:", bodynode)
            if bodynode in self.collisionChecker.contacted_bodies:
            #print("return True")
            #input()
                if self.checkOut is not 0:
                    if len(self.collisionChecker.contacted_bodies) < 2:
                        continue
                    else:
                        return True
                else:
                    self.checkOut = 0
                    return False
        #elif self.mState.getElapsedTime() > self.mDuration:
            #return True
            else:
                self.checkOut = 1
                continue
        
        return False

