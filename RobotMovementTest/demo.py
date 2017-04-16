
# To download PyUSB with conda:
# conda install -c trentonoliphant pyusb=1.0.0b2
# conda install -c m-labs libusb=1.0.20
from __future__ import print_function,division
import math, copy, Queue, threading, random, cv2, time, sys
import numpy as np
import usb.core
import usb.util
from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
import matplotlib.pyplot as plt
from MMM import MMM
from findLightSaber import LightSaberTracker
'''
IMPORTANT: Kinect depth frame contents are full depth values in mm.
They range up to 4000 mm. NO NEED FOR BITWISE SHIFT.

TO GET DIMENSIONS OF DEPTH FRAME, USE self.kinectDepthStream.depth_frame_desc.Width, ''.Height

The coordinate system used in the fighting function is the same as the skeleton
space coordinate system. X,Y,Z from the depth stream are in mm.

ALL DATA ACQUIRED FROM BODY DATA STREAM IS IN METERS.
'''
#TODO: 1st arg of mmm.setWheelVelocity is for right motor
#TODO: Robot rotates around point 6-7 cm in front of robot
def rgbString(red, green, blue):
    #http://www.cs.cmu.edu/~112/notes/notes-graphics.html#customColors
    return "#%02x%02x%02x" % (red, green, blue)

def checkIfKinectConnected():
    # Actually checks if a microsoft device is connected
    # Microsoft vendorID = 0x045e
    return True
    '''
    device = usb.core.find(idVendor = 0x045e)
    if device != None:
        return True
    else:
        return False
    '''
##############################################################################
#Description:
#Currently displays depth image from kinect that is repeated
##############################################################################

def convertTo2DGrid(oneDArray,cols):
    grid=[]
    while(len(oneDArray)>0):
        grid.append(list(reversed(oneDArray[:cols])))
        del oneDArray[:cols]
    return np.array(grid)

def makeArrayRectangular(targetArray):
    #Makes sure last row of 2D array matches row 1
    targetCols=len(targetArray[0])
    missingCols=targetCols-len(targetArray[-1])
    newArray=targetArray.tolist()
    newArray[-1].extend([0]*missingCols)
    return newArray


def convertColorFrameIntoRGBArray(frame,width,height):
    RGBframe=np.delete(frame,slice(3,None,4))
    RGBframe=np.reshape(RGBframe,(height,width,3))
    return RGBframe

def distance(x1,x2,y1,y2):
    return (((x1-x2)**2)+((y1-y2)**2))**0.5
class humanObject(object):
    def __init__(self,hipPosition,rightWrist,rightElbow,rightShoulder):
        self.hip=hipPosition
        self.wrist=rightWrist
        self.elbow=rightElbow
        self.shoulder=rightShoulder

class KinectHandler(object):
    def __init__(self):
        self.kinectBodyStream=PyKinectRuntime.PyKinectRuntime(
                              PyKinectV2.FrameSourceTypes_Body)
        self.kinectDepthStream=PyKinectRuntime.PyKinectRuntime(
                               PyKinectV2.FrameSourceTypes_Depth)
        self.kinectColorStream=PyKinectRuntime.PyKinectRuntime(
                               PyKinectV2.FrameSourceTypes_Color)
    def getBodyPositions(self):
        bodies=None
        if self.kinectBodyStream.has_new_body_frame():
            bodies=self.kinectBodyStream.get_last_body_frame()
        if bodies!=None:
            bodyList=[]
            for body in bodies.bodies:
                if body.is_tracked:
                    joints = body.joints
                    hip = joints[PyKinectV2.JointType_SpineBase]
                    rightWrist=joints[PyKinectV2.JointType_WristRight]
                    rightElbow=joints[PyKinectV2.JointType_ElbowRight]
                    rightShoulder=joints[PyKinectV2.JointType_ShoulderRight]
                    newBody=humanObject(hip.Position,rightWrist.Position,
                                        rightElbow.Position,rightShoulder.Position)
                    bodyList.append(newBody)
            return bodyList
    def getPerson(self):
        bodies=None
        if self.kinectBodyStream.has_new_body_frame():
            bodies=self.kinectBodyStream.get_last_body_frame()
        if bodies!=None:
            for body in bodies.bodies:
                if body.is_tracked:
                    joints=body.joints
                    hip = joints[PyKinectV2.JointType_SpineBase]
                    rightWrist=joints[PyKinectV2.JointType_WristRight]
                    rightElbow=joints[PyKinectV2.JointType_ElbowRight]
                    rightShoulder=joints[PyKinectV2.JointType_ShoulderRight]
                    newBody=humanObject(hip.Position,rightWrist.Position,
                                        rightElbow.Position,rightShoulder.Position)
                    return newBody
    def getPersonPosition(self): #Use after initial approach
        bodies=None
        if self.kinectBodyStream.has_new_body_frame():
            bodies=self.kinectBodyStream.get_last_body_frame()
        if bodies!=None:
            for body in bodies.bodies:
                if body.is_tracked:
                    joints = body.joints
                    hip = joints[PyKinectV2.JointType_SpineBase]
                    hipX = hip.Position.x
                    hipZ = hip.Position.z
                    theta =  (hipX/np.abs(hipX))*math.atan(np.abs(hipX)/hipZ)*180/math.pi
                    return theta

    def getNewDepthData(self):
        #Returns 1D numpy array of 2 byte objects
        frame=None
        if self.kinectDepthStream.has_new_depth_frame():
            frame=self.kinectDepthStream.get_last_depth_frame()
            return (frame, self.kinectDepthStream.depth_frame_desc.Width,
                    self.kinectDepthStream.depth_frame_desc.Height)

    def getNewColorData(self):
        frame=None
        if self.kinectColorStream.has_new_color_frame():
            frame=self.kinectColorStream.get_last_color_frame()
            return (frame, self.kinectColorStream.color_frame_desc.Width,
                    self.kinectColorStream.color_frame_desc.Height)
    def end(self):
        self.kinectBodyStream.close()
        self.kinectDepthStream.close()
        self.kinectColorStream.close()
class Robot(object):
    def __init__(self):
        self.kinect=KinectHandler()
        self.isRunning=True
        self.colorWidth,self.colorHeight=None,None
        self.depthWidth,self.depthHeight=None,None
        self.depthFrame=None
        self.colorFrame=None
        self.verticalDepthPixelRatio=424/60 #pixels per degree vertically
        self.horizontalDepthPixelRatio=512/70.6 #pixels per degree horizontally
        self.verticalColorPixelRatio=1080/53.8 #pixels per degree vertically
        self.horizontalColorPixelRatio=1920/84.1 #pixels per degree horizontally
        self.saberTracker=LightSaberTracker()
        self.kinectX=0.1 #m between kinect and anatomical right side of robot
        self.kinectY=1 #m from kinect to ground
        self.mmm=MMM('COM4')
        time.sleep(4)
        self.mainThread=threading.Thread(target=self.updateRobot,args=())
        self.mainThread.daemon=True
        self.mainThread.start()
    def updateRobot(self):
        while True:
            self.mmm.clampAll()
            self.mmm.update()
            time.sleep(0.1)
            self.mmm.parseData()
    def getDepthFrame(self):
        newDepthFrame=self.kinect.getNewDepthData()
        while newDepthFrame==None:
            newDepthFrame=self.kinect.getNewDepthData()
        (depthFrame,depthWidth,depthHeight)=newDepthFrame
        print("New depth frame acquired!")
        if self.depthWidth==None or self.depthHeight==None:
            self.depthWidth,self.depthHeight=depthWidth,depthHeight
        depthContainer=list(copy.deepcopy(depthFrame))
        depthGrid=convertTo2DGrid(depthContainer,depthWidth)
        depthGrid=makeArrayRectangular(depthGrid)
        self.depthFrame=copy.deepcopy(depthGrid)
    def getColorFrame(self):
        newColorFrame=self.kinect.getNewColorData()
        while newColorFrame==None:
            newColorFrame=self.kinect.getNewColorData()
        (colorFrame,colorWidth,colorHeight)=newColorFrame
        if self.colorWidth==None or self.colorHeight==None:
            self.colorWidth,self.colorHeight=colorWidth,colorHeight
        colorRGBFrame=convertColorFrameIntoRGBArray(colorFrame,colorWidth,colorHeight)
        self.colorFrame=copy.deepcopy(colorRGBFrame)#cv2.cvtColor(colorRGBFrame,cv2.COLOR_RGB2BGR)
    def skeletonSpaceToColor(self,x,y,z,cameraOffset): #x,y,z in meters
        cameraPos=(x+cameraOffset,y,z)
        x=(x+cameraOffset)
        if x==0:
            colX=(self.colorWidth/2)
        else:
            colX=((self.colorWidth/2)-(x/np.abs(x))*
                   self.horizontalColorPixelRatio*(math.atan(np.abs(x)/z))*(180/math.pi))
        colX=self.colorWidth-int(colX)
        if y==0:
            rowY=(self.colorHeight/2)
        else:
            rowY=((self.colorHeight/2)-(y/np.abs(y))*
                   self.verticalColorPixelRatio*(math.atan(np.abs(y)/z))*(180/math.pi))
        rowY=int(rowY)
        return (rowY,colX) #Index in color frame
    def depthSpaceToSkeleton(self,row,col,z):
        z=z/1000
        if col==int(self.depthWidth/2):
            x=0
        else:
            x=((((self.depthWidth/2)-col)/np.abs((self.depthWidth/2)-col))*z
                  *math.tan((np.abs((self.depthWidth/2)-col)/
                  self.horizontalDepthPixelRatio)*math.pi/180))
        if row==int(self.depthHeight/2):
            y=0
        else:
            y=((((self.depthHeight/2)-row)/np.abs((self.depthHeight/2)-row))*z
                  *math.tan((np.abs((self.depthHeight/2)-row)/
                  self.verticalDepthPixelRatio)*math.pi/180))
        return (x,y,z) #in meters
    def getTargetPosition(self):
        allHumans=self.kinect.getBodyPositions()
        cameraOffset=9.5/100
        for i in range(len(allHumans)):
            self.getDepthFrame()
            self.getColorFrame()
            currentHuman=allHumans[i]
            (rowY,colX)=self.skeletonSpaceToColor(currentHuman.wrist.x,
                                                  currentHuman.wrist.y,
                                                  currentHuman.wrist.z,
                                                  cameraOffset)
            lightSaberCoords=self.saberTracker.track(self.colorFrame,(rowY,colX))
            if lightSaberCoords==None: continue
            (hiltY,hiltX)=coords[0]
            pixelDiff=distance(colX,hiltX,rowY,hiltY)
            if pixelDiff<400:
                return currentHuman
        return False #target not found

    def turnTowardsOnlyPerson(self):
        angleToPerson=self.kinect.getPersonPosition()
        while np.abs(angleToPerson)>20:
            if angleToPerson>0:
                self.mmm.setWheelVelocity(0.03,-0.03)
            else:
                self.mmm.setWheelVelocity(-0.03,0.03)
            time.sleep(0.1)
            self.mmm.setWheelVelocity(0,0)
            angleToPerson=self.kinect.getPersonPosition()
        self.mmm.setWheelVelocity(0,0)
    def goToTarget(self):
        target=self.getTargetPosition()
        while target==None or target==False: target=self.getTargetPosition()
        angleToTarget=math.atan(np.abs(target.hip.x)/target.hip.z)*180/math.pi
        timePerDegree=0
        if target.hip.x>0:
            self.mmm.setWheelVelocity(0.05,-0.05)
            timePerDegree=24.2/360
        else:
            self.mmm.setWheelVelocity(-0.05,0.05)
            timePerDegree=22.5/360
        timeRequired=angleToTarget*(timePerDegree) #time required for robot to turn 1 degree
        time.sleep(timeRequired)
        distanceToTarget=(target.hip.x**2+target.hip.z**2)**0.5 # meters
        if distanceToTarget>1: #m
            self.mmm.setWheelVelocity(0.15,0.15)
            timeForForward=(distanceToTarget-1)/0.15
            time.sleep(timeForForward)
        self.mmm.setWheelVelocity(0,0)
        self.turnTowardsOnlyPerson()
    def calculateLightsaberPosition(bodyObject):
        #Assume elbow is origin point for wrist and shoulder vectors
        #ASSUME LIGHTSABER IS HELD AT 90 DEGREE ANGLE TO ARM UPWARDS
        wristVector=(np.array([bodyObject.wrist.x,bodyObject.wrist.y,
                     bodyObject.wrist.z])-
                     np.array([bodyObject.elbow.x,bodyObject.elbow.y,bodyObject.elbow.z]))
        shoulderVector=(np.array([bodyObject.shoulder.x,bodyObject.shoulder.y,
                     bodyObject.shoulder.z])-
                     np.array([bodyObject.elbow.x,bodyObject.elbow.y,bodyObject.elbow.z]))
        perpenVector=np.cross(wristVector,shoulderVector)
        lightsaberVector=np.cross(perpenVector,wristVector)
        lightsaberVector=lightsaberVector/np.linalg.norm(lightsaberVector)
        lightsaberVector*=0.595
        lightsaberTipVector=lightsaverVector+wristVector
        lightsaberPosition=lightsaberTipVector+np.array([bodyObject.elbow.x,bodyObject.elbow.y,bodyObject.elbow.z])
        return lightsaberPosition
    def inHitBox(position):
        return position[0]>=-0.3 and position[0]<=0.3 and position[1]>=-1.1 and position[1]<0.11
    def fightPerson(self):
        #Assume first person detected is the lightsaber person
        currentBody=self.kinect.getPerson()
        lastSampleTime=time.time()
        prevSampleTime=None
        prevBody=None
        lastAttackTime=time.time()
        while distance(0,0,currentBody.hip.x,currentBody.hip.z)<2: #Person is fighting if within 2 meters
            if prevBody==None:
                prevBody=copy.deepcopy(currentBody)
                currentBody=self.kinect.getPerson()
                prevSampleTime=lastSampleTime
                lastSampleTime=time.time()
            currentPos=calculateLightsaberPosition(currentBody)
            lastPos=calculateLightsaberPosition(prevBody)
            tipDisplacement=currentPos-lastPos
            bottomDisplacement=np.array([currentBody.wrist.x-prevBody.wrist.x,currentBody.wrist.y-prevBody.wrist.y,currentBody.wrist.z-prevBody.wrist.z])
            tipVelocity=copy.deepcopy(tipDisplacement)/(lastSampleTime-prevSampleTime)
            bottomVelocity=copy.deepcopy(bottomDisplacement)/(lastSampleTime-prevSampleTime)
            tipSpeed=np.linalg.norm(tipVelocity)
            bottomSpeed=np.linalg.norm(bottomVelocity)
            tTip=-currentPos[2]/tipVelocity[2]
            bottomT=-currentBody.wrist.z/bottomVelocity[2]
            if tTip>0 and bottomT>0:
                tipProjectedPosition=currentPos+tTip*tipVelocity
                bottomProjectedPosition=np.array([currentBody.wrist.x,currentBody.wrist.y,currentBody.wrist.z])+bottomT*bottomVelocity
                if (inHitBox(tipProjectedPosition) or inHitBox(bottomProjectedPosition)) and (tipSpeed>0.5):
                    #Calculate hit line and block
                    lastAttackTime=time.time()
                    xArm=(0.2+bottomProjectedPosition[0])
                    shoulderTheta=math.acos((abs((0.2+bottomProjectedPosition[0])-0.263)/0.452))*180/math.pi
                    shoulderMotorValue=160-shoulderTheta
                    if shoulderMotorValue<0:
                        shoulderMotorValue=0
                    elif shoulderMotorValue>120:
                        shoulderMotorValue=120
                    theta=160-shoulderMotorValue
                    currentArmZ=0.452*math.sin(theta)
                    tipTime=(currentArmZ-currentPos[2])/tipVelocity[2]
                    bottomTime=(currentArmZ-currentBody.wrist.z)/bottomVelocity[2]
                    newTipProjection=currentPos+tipTime*tipVelocity
                    newBottomProjection=np.array([currentBody.wrist.x,currentBody.wrist.y,currentBody.wrist.z])+bottomTime*bottomVelocity
                    defenseSaberTipY=0.2+newBottomProjection[1]+((newTipProjection[1]-newBottomProjection[1])/(newTipProjection[0]-newBottomProjection[0]))*(0.6*math.cos(60*math.pi/180)+xArm)-newBottomProjection[0]*((newTipProjection[1]-newBottomProjection[1])/(newTipProjection[0]-newBottomProjection[0]))
                    dY=defenseSaberTipY-0.6*math.sin(60*math.pi/180)
                    elbowTheta=(dY/np.abs(dY))*(math.asin(np.abs(dY)/0.385))*180/math.pi
                    if elbowTheta>60:
                        elbowTheta=60
                    elif elbowTheta<-60:
                        elbowTheta=-60
                    self.mmm.rotateShoulders(shoulderMotorValue,0)
                    self.mmm.rotateElbows(elbowTheta,-60)
                    time.sleep(1)
            elif time.time()-lastAttackTime>10:
                #Attack!
                self.turnTowardsOnlyPerson()
                jedi=self.kinect.getPerson()
                distanceToJedi=distance(0,jedi.hip.x,0,jedi.hip.z)
                timeToMoveForwardAndBack=(distanceToJedi-0.3)/0.1
                self.mmm.setWheelVelocity(0.1,0.1)
                time.sleep(timeToMoveForwardAndBack)
                self.mmm.setWheelVelocity(0,0)
                self.mmm.rotateShoulders(90,120)
                self.mmm.rotateElbows(60,-60)
                time.sleep(1)
                self.mmm.rotateElbows(-30,-60)
                time.sleep(0.5)
                self.mmm.rotateElbows(60,-60)
                time.sleep(1)
                self.mmm.setWheelVelocity(-0.1,-0.1)
                time.sleep(timeToMoveForwardAndBack)
                self.mmm.setWheelVelocity(0,0)


            #UPDATE DATA
            prevBody=copy.deepcopy(currentBody)
            currentBody=self.kinect.getPerson()
            prevSampleTime=lastSampleTime
            lastSampleTime=time.time()
    def run(self):
        self.mmm.setWheelVelocity(0,0)
        self.mmm.rotateShoulders(70,80)
        self.mmm.rotateElbows(-60,-60)
        time.sleep(1)
        while self.isRunning:
            self.goToTarget()
            self.fightPerson()
            if not checkIfKinectConnected():
                print("Bye!")
                self.isRunning=False
        self.kinect.end()
        self.mmm.ser.close()
        quit()


if __name__=='__main__':
    robot=Robot()
    robot.run()
