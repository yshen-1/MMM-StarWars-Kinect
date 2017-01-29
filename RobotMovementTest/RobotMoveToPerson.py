
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

'''
IMPORTANT: Kinect depth frame contents are full depth values in mm.
They range up to 4000 mm. NO NEED FOR BITWISE SHIFT.
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
        self.playerIndexBits=0
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
                print("Bye!")
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
    maximum=4000
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
        while self.dataStorage.empty():
            #Wait until data starts arriving
            pass
        self.dataArray=self.dataStorage.get()
        self.dataArray=makeArrayRectangular(self.dataArray)
        self.image=convert2dArrayToImage(self.dataArray)
        self.height=len(self.image)
        self.width=len(self.image[0])
        self.screen=pygame.display.set_mode((self.width,self.height))
        self.overlay=pygame.Surface((self.width,self.height))
        self.overlay.fill((0,0,0))
        self.display=pygame.Surface((self.width,self.height))
        self.display.fill((0,0,0))
        self.display=self.display.convert()
        self.overlay=self.overlay.convert()
        self.image=np.rot90(self.image,axes=(1,0))
        self.isRunning=True
    def timerFired(self):
        if not self.dataStorage.empty():
            self.dataArray=self.dataStorage.get()
            print("Update!")
            self.dataArray=makeArrayRectangular(self.dataArray)
            self.image=convert2dArrayToImage(self.dataArray)
            self.image=np.rot90(self.image)
    def drawAll(self):
        #rotate image array 90 degrees clockwise
        #Rotation required by pygame (surface array is [x][y] while image is
        #[y][x])
        self.screen.blit(self.overlay,(0,0))
        pygame.surfarray.blit_array(self.display,self.image)
        self.display=self.display.convert()
        self.screen.blit(self.display,(0,0))
    def run(self):
        while self.isRunning:
            (mouseX,mouseY)=pygame.mouse.get_pos()
            screenArray=pygame.surfarray.array3d(self.screen)
            print(screenArray[mouseX][mouseY][0]*(4000/255))
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    self.isRunning=False
                elif event.type==pygame.KEYDOWN and event.key==pygame.K_ESCAPE:
                    self.isRunning=False
            self.drawAll()
            self.timerFired()
            pygame.display.update()
        pygame.quit()
        self.tracking.stop()
#118 cm test
debugger=graphicalDebugger()
debugger.run()
