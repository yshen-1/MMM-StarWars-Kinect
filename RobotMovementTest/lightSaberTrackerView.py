
# To download PyUSB with conda:
# conda install -c trentonoliphant pyusb=1.0.0b2
# conda install -c m-labs libusb=1.0.20
from __future__ import print_function,division
import math, copy
import numpy as np
import usb.core
import usb.util
import pygame
import Queue
import threading
import random
import cv2
from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
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

def convertTo2DGrid(oneDArray,cols=640):
    grid=[]
    while(len(oneDArray)>0):
        grid.append(oneDArray[:cols])
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
            for i in range(0,self.kinectBodyStream.max_body_count):
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

    def convertColorToDepth(self,depthFrame,refToLargeArray):
        self.kinectDepthStream.color_frame_to_camera_space(depthFrame,refToLargeArray)

    def end(self):
        self.kinectBodyStream.close()
        self.kinectDepthStream.close()
        self.kinectColorStream.close()

class humanTracker():
    def __init__(self):
        self.kinect=KinectHandler()
        self.isRunning=True
        self.playerIndexBits=0
        self.depthWidth,self.depthHeight=None,None
        self.depthFrame=None
        self.originalDepthFrame=None
        self.colorWidth,self.colorHeight=None, None
        self.colorFrame=None
        self.verticalDepthPixelRatio=424/60 #pixels per degree vertically
        self.horizontalDepthPixelRatio=512/70.6 #pixels per degree horizontally
        self.verticalColorPixelRatio=1080/53.8 #pixels per degree vertically
        self.horizontalColorPixelRatio=1920/84.1 #pixels per degree horizontally
        self.saberTracker=LightSaberTracker()
    def stop(self):
        self.isRunning=False
    def getDepthLineAndFrame(self):
        newDepthFrame=self.kinect.getNewDepthData()
        while newDepthFrame==None:
            newDepthFrame=self.kinect.getNewDepthData()
        (depthFrame,depthWidth,depthHeight)=newDepthFrame
        print("New depth frame acquired!")
        self.depthWidth,self.depthHeight=depthWidth,depthHeight
        self.originalDepthFrame=depthFrame
        depthContainer=list(copy.deepcopy(depthFrame))
        depthGrid=convertTo2DGrid(depthContainer,depthWidth)
        depthGrid=makeArrayRectangular(depthGrid)
        self.depthFrame=copy.deepcopy(depthGrid)
    def getColorFrame(self):
        newColorFrame=self.kinect.getNewColorData()
        while newColorFrame==None:
            newColorFrame=self.kinect.getNewColorData()
        (colorFrame,colorWidth,colorHeight)=newColorFrame
        self.colorWidth,self.colorHeight=colorWidth,colorHeight
        colorRGBFrame=convertColorFrameIntoRGBArray(colorFrame,colorWidth,colorHeight)
        self.colorFrame=copy.deepcopy(colorRGBFrame)#cv2.cvtColor(colorRGBFrame,cv2.COLOR_RGB2BGR)

    def getTipAndHandData(self):
        wristPos=self.kinect.getRightWristPosition()
        while wristPos==None:
            wristPos=self.kinect.getRightWristPosition()
        (wristX,wristY,wristZ)=wristPos
        print("Wrist relative to kinect center: ",wristPos)
        self.getColorFrame()
        self.getDepthLineAndFrame()
        kinectCameraOffset=9.5/100
        wristCameraPos=(wristX+kinectCameraOffset,wristY,wristZ)
        colX=((self.colorWidth/2)-(wristX/np.abs(wristX))*
               self.horizontalColorPixelRatio*(math.atan(np.abs(wristX)/wristZ))*(180/math.pi))
        colX=self.colorWidth-int(colX)
        rowY=((self.colorHeight/2)-(wristY/np.abs(wristY))*
               self.verticalColorPixelRatio*(math.atan(np.abs(wristY)/wristZ))*(180/math.pi))
        rowY=int(rowY)
        cv2.imwrite("ColorImageTaken.PNG",self.colorFrame)
        coords=self.saberTracker.track(self.colorFrame,(rowY,colX))
        self.getDepthLineAndFrame()
        if coords==None: return None
        (tipColorY,tipColorX)=coords[1]
        (startingColorY,startingColorX)=coords[0]
        transformedColorFrame=[[0]*1920 for i in range(1080)]
        print("Saber start: ",startingColorY,startingColorX)
        print("Saber tip: ",tipColorY,tipColorX)


        #b=raw_input("Press enter to continue.")
        '''
        tipXDiff=self.colorWidth/2-tipColorX
        colorXAngle=np.abs(tipXDiff)/self.horizontalColorPixelRatio #Degrees

        distanceToPerson=self.depthFrame[endDepthY][endDepthX]
        return (distanceToPerson, depthX, depthY, endDepthX, endDepthY, endColorY,endColorX, wristX,wristY, wristZ, rowY,colX)
        '''

    def run(self):
        while self.isRunning:
            #DO STUFF
            #(distanceToPerson, depthX, depthY, endDepthX, endDepthY, endColorY,endColorX, wristX,wristY, wristZ,wristRow,wristCol)=self.getTipAndHandData()
            #colorContainer=list(copy.deepcopy(frame))
            #depthGrid=convertTo2DGrid(depthContainer,width)
            #print("Distance:",distanceToPerson)
            self.getTipAndHandData()
            if not checkIfKinectConnected():
                print("Bye!")
                self.isRunning=False
        self.kinect.end()


if __name__=='__main__':
    debugger=humanTracker()
    debugger.run()
