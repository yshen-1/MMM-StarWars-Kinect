
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
from scipy.signal import hilbert
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

def condenseDepthFrame(depthFrame):
    condensedRow=depthFrame[0]
    for k in range(len(condensedRow)):
        if condensedRow[k]>4000: condensedRow[k]=4000
    for i in range(1,len(depthFrame)):
        for j in range(len(depthFrame[i])):
            depthValue=depthFrame[i][j]
            if depthValue<4000 and depthValue>800:
                condensedRow[j]=(depthValue if depthValue<condensedRow[j]
                                            else condensedRow[j])
    return condensedRow

def convertColorFrameIntoRGBArray(frame,width,height):
    RGBframe=np.delete(frame,slice(3,None,4))
    RGBframe=np.reshape(RGBframe,(height,width,3))
    return RGBframe
class KinectHandler(object):
    def __init__(self):
        self.kinectBodyStream=PyKinectRuntime.PyKinectRuntime(
                              PyKinectV2.FrameSourceTypes_Body)
        self.kinectDepthStream=PyKinectRuntime.PyKinectRuntime(
                               PyKinectV2.FrameSourceTypes_Depth)
        self.kinectColorStream=PyKinectRuntime.PyKinectRuntime(
                               PyKinectV2.FrameSourceTypes_Color)

    def getPeoplePosition(self):
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
                    distanceToUser = (hipX**2+hipZ**2)**0.5
                    theta =  math.atan(hipX/hipZ)
                    playerIndex = bodies.bodies.index(body)
                    return (distanceToUser, theta, playerIndex)
    def getRightWristPosition(self):
        bodies=None
        if self.kinectBodyStream.has_new_body_frame():
            bodies=self.kinectBodyStream.get_last_body_frame()
        if bodies!=None:
            for i in range(0,kinect.max_body_count):
                body=bodies.bodies[i]
                if body.is_tracked:
                    joints=body.joints
                    rightWristX=joints[PyKinectV2.JointType_WristRight].Position.x
                    rightWristY=joints[PyKinectV2.JointType_WristRight].Position.y
                    rightWristZ=joints[PyKinectV2.JointType_WristRight].Position.z
                    return rightWristX,rightWristY,rightWristZ #in meters
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


class humanFighter(object):
    def __init__(self):
        self.kinect=KinectHandler()
        self.isRunning=True
        self.depthLine=None
        self.depthFrame=None
        self.colorFrame=None
        self.colorWidth,self.colorHeight=None,None
        self.depthWidth,self.depthHeight=None,None
        self.verticalPixelRatio=424/60 #pixels per degree vertically
        self.horizontalPixelRatio=512/70.6 #pixels per degree horizontally
        self.saberTracker=LightSaberTracker()
        self.mmm=MMM('COM4')
        time.sleep(4)
        self.mainThread=threading.Thread(target=self.updateRobot, args=())
        self.mainThread.daemon=True
        self.mainThread.start()
	def updateRobot(self):
		while True:
			self.mmm.clampAll();
			self.mmm.update();
			time.sleep(0.1);
			self.mmm.parseData();
    def getDepthLineAndFrame(self):
        newDepthFrame=self.kinect.getNewDepthData()
        while newDepthFrame==None:
            newDepthFrame=self.kinect.getNewDepthData()
        (depthFrame,depthWidth,depthHeight)=newDepthFrame
        self.depthWidth,self.depthHeight=depthWidth,depthHeight
        depthContainer=list(copy.deepcopy(depthFrame))
        depthGrid=convertTo2DGrid(depthContainer,depthWidth)
        depthGrid=makeArrayRectangular(depthGrid)
        self.depthFrame=copy.deepcopy(depthGrid)
        self.depthLine=condenseDepthFrame(depthGrid)
    def getColorFrame(self):
        newColorFrame=self.kinect.getNewColorData()
        while newColorFrame==None:
            newColorFrame=self.kinect.getNewColorData()
        (colorFrame,colorWidth,colorHeight)=newColorFrame
        self.colorWidth,self.colorHeight=colorWidth,colorHeight
        colorRGBFrame=convertColorFrameIntoRGBArray(colorFrame,colorWidth,colorHeight)
        self.colorFrame=cv2.cvtColor(colorRGBFrame,cv2.COLOR_RGB2BGR)
    def getPersonLocation(self):
        posData=self.kinect.getPeoplePosition()
        while posData==None:
            posData=self.kinect.getPeoplePosition()
        return posData

    def goToPerson(self):
        self.mmm.setWheelVelocity(0,0)
        self.getColorFrame()
        self.getDepthLineAndFrame()
        wristPos=self.kinect.getRightWristPosition()
        while wristPos==None:
            wristPos=self.kinect.getRightWristPosition()
        (wristX,wristY,wristZ)=wristPos
        colX=((self.depthWidth/2)-(wristX/np.abs(wristX))*
               self.horizontalPixelRatio*(math.atan(np.abs(wristX)/wristZ))*(180/math.pi))
        colX=int(colX)
        rowY=((self.depthHeight/2)+(wristY/np.abs(wristY))*
               self.verticalPixelRatio*(math.atan(np.abs(wristY)/wristZ))*(180/math.pi))
        rowY=int(rowY)
        rowY=self.depthHeight-rowY
        rowY=int(rowY*self.colorHeight/self.depthHeight)
        colX=int(colX*self.colorWidth/self.depthWidth)
        coords=self.saberTracker.track(self.colorFrame,(rowY,colX))
        while coords==None:
            coords=self.saberTracker.track(self.colorFrame,(rowY,colX))
        (colorX,colorY)=coords
        (depthX,depthY)=(int(colorX*self.depthWidth/self.colorWidth),
                         int(colorY*self.depthHeight/self.colorHeight))
        distanceToPerson=self.depthFrame[depthY][depthX]
        while distanceToPerson>900 and distanceToPerson<4000:
            dist=depthX-self.depthWidth//2
            while abs(dist)>50: #accuracy threshold
                if dist>0: self.mmm.setWheelVelocity(0.03,-0.03)
                else: self.mmm.setWheelVelocity(-0.03,0.03)
                time.sleep(0.1)
                self.mmm.setWheelVelocity(0,0)
                self.getColorFrame()
                self.getDepthLineAndFrame()
                wristPos=self.kinect.getRightWristPosition()
                if wristPos==None: continue
                (wristX,wristY,wristZ)=wristPos
                colX=((self.depthWidth/2)-(wristX/np.abs(wristX))*
                       self.horizontalPixelRatio*(math.atan(np.abs(wristX)/wristZ))*(180/math.pi))
                colX=int(colX)
                rowY=((self.depthHeight/2)+(wristY/np.abs(wristY))*
                       self.verticalPixelRatio*(math.atan(np.abs(wristY)/wristZ))*(180/math.pi))
                rowY=int(rowY)
                rowY=self.depthHeight-rowY
                rowY=int(rowY*self.colorHeight/self.depthHeight)
                colX=int(colX*self.colorWidth/self.depthWidth)
                coords=self.saberTracker.track(self.colorFrame,(rowY,colX))
                if coords==None: continue
                (colorX,colorY)=coords
                (depthX,depthY)=(int(colorX*self.depthWidth/self.colorWidth),
                                 int(colorY*self.depthHeight/self.colorHeight))
                dist=depthX-self.depthWidth//2
            self.mmm.setWheelVelocity(0.1,0.1)
            time.sleep(1)
            self.mmm.setWheelVelocity(0,0)
        self.mmm.setWheelVelocity(0,0)
    def respondToMove(self,tipInitPos,tipVector,handInitPos,handVector,prevTime,sampleTime):
        samplingTime=sampleTime-prevTime
        #If the tip velocity vector is greater than the velocity handVector, the
        #person is swinging the lightsaber
        tipVelocity=tipVector/samplingTime
        tipSpeed=np.linalg.norm(tipVelocity)
        handVelocity=handVector/samplingTime
        handSpeed=np.linalg.norm(handVelocity)
        #If the person is not swinging the lightsaber and the vectors are not
        #directed at the robot, attack.
        #The robot's arm is at minimum 20 cm from the main body

    def fightPerson(self):
        #@TODO Use time module to assign timestamps to sensor readings
        (personDist,angle,index)=self.getPersonLocation()
        (prevWristX,prevWristY,prevWristZ)=(None,None,None)
        (prevTipX,prevTipY,prevTipZ)=(None,None,None)
        prevTime=None
        while personDist<1.5: #If the person is in range (1.5 m)
        '''
            while personDist<0.900: #If person is too close, back off!
                self.mmm.setWheelVelocity(-0.1,-0.1)
                time.sleep(0.1)
                (personDist,angle,index)=self.getPersonLocation()
        '''
            self.mmm.setWheelVelocity(0,0)
            #ROBOT DATA GATHERING SECTION HERE
            self.getColorFrame()
            self.getDepthLineAndFrame()
            sampleTime=time.time()
            wristPos=self.kinect.getRightWristPosition()
            if wristPos==None: continue
            (wristX,wristY,wristZ)=wristPos
            colX=((self.depthWidth/2)-(wristX/np.abs(wristX))*
                   self.horizontalPixelRatio*(math.atan(np.abs(wristX)/wristZ))*(180/math.pi))
            colX=int(colX)
            rowY=((self.depthHeight/2)+(wristY/np.abs(wristY))*
                   self.verticalPixelRatio*(math.atan(np.abs(wristY)/wristZ))*(180/math.pi))
            rowY=int(rowY)
            rowY=self.depthHeight-rowY
            rowY=int(rowY*self.colorHeight/self.depthHeight)
            colX=int(colX*self.colorWidth/self.depthWidth)
            tipCoords=self.saberTracker.track(self.colorFrame,(rowY,colX))
            if tipCoords==None: continue
            (tipColorX,tipColorY)=tipCoords
            (tipDepthX,tipDepthY)=(int(tipColorX*self.depthWidth/self.colorWidth),
                                   int(tipColorY*self.depthHeight/self.colorHeight))
            tipDistanceFromCameraPlane=self.depthFrame[tipDepthY][tipDepthX]
            tipDepthY=self.depthHeight-tipDepthY
            #xTipPos,xDistance>0 if person to the left of Kinect, else <0
            xTipPos=(self.depthWidth/2-tipDepthX)/self.horizontalPixelRatio #Degrees from center plane
            xDistance=(xTipPos/abs(xTipPos))*(tipDistanceFromCameraPlane*math.tan(abs(xTipPos)*math.pi/180))
            yTipPos=(tipDepthY-self.depthHeight/2)/self.verticalPixelRatio #Degrees up from camera plane
            yDistance=(yTipPos/abs(yTipPos))*(tipDistanceFromCameraPlane*math.tan(abs(yTipPos)*math.pi/180))
            zDistance=tipDistanceFromCameraPlane
            (tipX,tipY,tipZ)=(xDistance/1000,yDistance/1000,zDistance/1000)
            if (prevWristX==None or prevWristY==None or prevWristZ==None
                or prevTipX==None or prevTipY==None or prevTipZ==None or prevTime==None):
                continue
            #ROBOT FIGHTING SECTION BELOW HERE
            #ALL UNITS IN METERS
            tipDisplace=(tipX-prevTipX,tipY-prevTipY,tipZ-prevTipZ) #displacement vector for tip
            handDisplace=(wristX-prevWristX,wristY-prevWristY,wristZ-prevWristZ) #Displacement vector for hand
            tipDisplace=np.array(tipDisplace)
            handDisplace=np.array(handDisplace)
            self.respondToMove(np.array([tipX,tipY,tipZ]),tipDisplace,
                               np.array([wristX,wristY,wristZ]),handDisplace,prevTime,sampleTime)
            #time.sleep(0.1)
            (prevWristX,prevWristY,prevWristZ)=(wristX,wristY,wristZ)
            (prevTipX,prevTipY,prevTipZ)=(tipX,tipY,tipZ)
            prevTime=sampleTime
            (personDist,angle,index)=self.getPersonLocation()

    def run(self):
        while self.isRunning:
            self.goToPerson()
            self.fightPerson()
            if not checkIfKinectConnected():
                print("Bye!")
                self.isRunning=False
        self.kinect.end()
        self.mmm.ser.close()
        quit()

if __name__=='__main__':
    tracker=humanFighter()
    tracker.run()
