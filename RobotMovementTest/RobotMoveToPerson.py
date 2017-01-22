
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

def rgbString(red, green, blue):
    #http://www.cs.cmu.edu/~112/notes/notes-graphics.html#customColors
    return "#%02x%02x%02x" % (red, green, blue)

def checkIfKinectConnected():
    # Actually checks if a microsoft device is connected
    # Microsoft vendorID = 0x045e
    device = usb.core.find(idVendor = 0x045e)
    if device != None:
        return True
    else:
        return False

##############################################################################
#Description:
#Currently prints out 2D array of Kinect depth data
#Kinect raw depth data is 1D numpy array of uint16 numbers
##############################################################################

def convertTo2DGrid(oneDArray,cols=640):
    grid=[]
    while(len(oneDArray)>0):
        grid.append(oneDArray[:cols])
        del oneDArray[:cols]
    return np.array(grid)


class KinectHandler(object):
    def __init__(self):
        self.kinectBodyStream=PyKinectRuntime.PyKinectRuntime(
                              PyKinectV2.FrameSourceTypes_Body)
        self.kinectDepthStream=PyKinectRuntime.PyKinectRuntime(
                               PyKinectV2.FrameSourceTypes_Depth)
        self.kinectColorStream=PyKinectRuntime.PyKinectRuntime(
                               PyKinectV2.FrameSourceTypes_Color)

    def getPersonPosition(self):
        bodies=None
        if self.kinectBodyStream.has_new_body_frame():
            bodies=self.kinectBodyStream.get_last_body_frame()
        if bodies!=None:
            body=bodies.bodies[-1]
            if body.is_tracked:
                joints = body.joints
                hip = joints[PyKinectV2.JointType_SpineBase]
                hipX = hip.Position.x
                hipZ = hip.Position.z
                distanceToUser = (hipX**2+hipZ**2)**0.5
                theta =  math.atan(hipX/hipZ)
                return (distanceToUser, theta)

    def getNewDepthData(self):
        #Returns 1D numpy array of 2 byte objects
        frame=None
        if self.kinectDepthStream.has_new_depth_frame():
            frame=self.kinectDepthStream.get_last_depth_frame()
        return frame

    def end(self):
        self.kinectBodyStream.close()
        self.kinectDepthStream.close()
        self.kinectColorStream.close()


class humanTracker(threading.Thread):
    def __init__(self,queue):
        threading.Thread.__init__(self)
        self.kinect=KinectHandler()
        self.isRunning=True
        self.playerIndexBits=3
        self.queue=queue
    def stop(self):
        self.isRunning=False
    def run(self):
        while self.isRunning:
            #DO STUFF
            newDepthFrame=self.kinect.getNewDepthData()
            if newDepthFrame==None: continue
            #Assume 3 least significant bits represent player index
            depthFrameToGrid=list(copy.copy(newDepthFrame >>
                                            self.playerIndexBits))
            '''
            #Scale to 8 bits
            depthFrameToGrid=list(depthFrameToGrid*
                                  ((2**8)/(2**(16-self.playerIndexBits))))
            '''
            #@TODO Look up actual resolution of kinect
            depthGrid=convertTo2DGrid(depthFrameToGrid)
            self.queue.put(depthGrid)
            if not checkIfKinectConnected():
                self.isRunning=False
        self.kinect.end()



##############################################################################
#Graphical Debugger
#Plots 2D array graphically
##############################################################################
def makeArrayRectangular(targetArray):
    #Makes sure last row of 2D array matches row 1
    targetCols=len(targetArray[0])
    missingCols=targetCols-len(targetArray[-1])
    newArray=targetArray.tolist()
    newArray[-1].extend([0]*missingCols)
    return np.array(newArray)




def convert2dArrayToImage(newNumpyArray):
    #Converts 2D grid to redscale image array
    #Scale numpy array to 255 max (8 bits)
    maximum=np.max(newNumpyArray)
    scalingFactor=(255/maximum)
    numpyArray=copy.deepcopy(newNumpyArray)
    numpyArray=np.array(numpyArray)*scalingFactor
    Rarray=numpyArray.astype(np.uint8)
    Garray=np.zeros(numpyArray.shape,np.uint8)
    Barray=np.zeros(numpyArray.shape,np.uint8)
    RGBarray=np.stack((Rarray,Garray,Barray),axis=2)
    return RGBarray
    #cv2.imwrite('test.jpg',BGRarray)

class graphicalDebugger():
    def trackerSetup(self):
        self.dataStorage=Queue.Queue()
        self.tracking=humanTracker(self.dataStorage)
        self.tracking.setDaemon(True)
        self.tracking.start()
    def __init__(self):
        pygame.init()
        self.trackerSetup()
        '''
        ##############################################
        #Test code
        ##############################################
        for i in range(100):
            testArray=[[random.random()*100 for j in range(640)]
                       for k in range(480)]
            testArray=np.array(testArray)
            self.dataStorage.put(testArray)
            print("Generating test array: ", i)
        #################################################
        '''
        while self.dataStorage.empty():
            #Wait until data starts arriving
            pass
        self.dataArray=self.dataStorage.get()
        self.dataArray=makeArrayRectangular(self.dataArray)
        self.image=convert2dArrayToImage(self.dataArray)
        self.height=len(self.image)
        self.width=len(self.image[0])
        self.screen=pygame.display.set_mode((self.width,self.height))
        self.image=np.rot90(self.image,axes=(1,0))
        self.isRunning=True
    def timerFired(self):
        if not self.dataStorage.empty():
            self.dataArray=self.dataStorage.get()
            self.dataArray=makeArrayRectangular(self.dataArray)
            self.image=convert2dArrayToImage(self.dataArray)
            self.image=np.rot90(self.image,axes=(1,0))
    def drawAll(self):
        #rotate image array 90 degrees clockwise
        pygame.surfarray.blit_array(self.screen,self.image)
    def run(self):
        while self.isRunning:
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    self.isRunning=False
                elif event.type==pygame.KEYDOWN and event.key==pygame.K_ESCAPE:
                    self.isRunning=False

            self.drawAll()
            self.timerFired()
            pygame.display.update()
        pygame.quit()


debugger=graphicalDebugger()
debugger.run()
