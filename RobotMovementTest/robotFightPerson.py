
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
        coords=self.saberTracker.track(self.colorFrame)
        if coords==None: continue
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
                coords=self.saberTracker.track(self.colorFrame)
                if coords==None: continue
                (colorX,colorY)=coords
                (depthX,depthY)=(int(colorX*self.depthWidth/self.colorWidth),
                                 int(colorY*self.depthHeight/self.colorHeight))
                dist=depthX-self.depthWidth//2
            self.mmm.setWheelVelocity(0.1,0.1)
            time.sleep(1)
            self.mmm.setWheelVelocity(0,0)
        self.mmm.setWheelVelocity(0,0)
    def fightPerson(self):
        (personDist,angle,index)=self.getPersonLocation()
        while personDist<1500: #If the person is in range (1.5 m)

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
