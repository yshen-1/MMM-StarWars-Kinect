
# To download PyUSB with conda:
# conda install -c trentonoliphant pyusb=1.0.0b2
# conda install -c m-labs libusb=1.0.20
from __future__ import print_function,division
import math, copy
import numpy as np
import usb.core
import usb.util
import Queue
import threading
import random
#import cv2
from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *

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
    condensedRow=depthFrame.pop(0)
    for remainingRows in depthFrame:
        for i in range(len(remainingRows)):
            depthValue=remainingRows[i]
            if depthValue<4000 and depthValue>800:
                condensedRow[i]=(depthValue if depthValue<condensedRow[i]
                                            else condensedRow[i])
    return condensedRow

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


class humanTracker(object):
    def __init__(self):
        self.kinect=KinectHandler()
        self.isRunning=True

    def run(self):
        while self.isRunning:
            #DO STUFF
            newDepthFrame=self.kinect.getNewDepthData()
            if newDepthFrame==None: continue
            (frame,width,height)=newDepthFrame
            depthContainer=list(copy.deepcopy(frame))
            depthGrid=convertTo2DGrid(depthContainer,width)
            depthGrid=makeArrayRectangular(depthGrid)
            depthLine=condenseDepthFrame(depthGrid)

            if not checkIfKinectConnected():
                print("Bye!")
                self.isRunning=False
        self.kinect.end()
