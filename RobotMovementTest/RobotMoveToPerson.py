import math, copy
import numpy as np
from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
from tkinter import *
from queue import Queue
import threading
##############################################################################
#Description:
#Currently prints out 2D array of Kinect depth data
#Kinect raw depth data is 1D numpy array of uint16 numbers
##############################################################################
def checkIfKinectConnected():
    return True
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
# events-example0.py
# Barebones timer, mouse, and keyboard events
class graphicalDebugger():
    def __init__(self,width,height):
        self.dataStorage=Queue()
        self.tracking=humanTracker(self.dataStorage)
        self.tracking.setDaemon(True)
        self.tracking.start()
        self.width=width
        self.height=height
    def mousePressed(self,event):
        pass
    def keyPressed(self,event):
        pass
    def timerFired(self)
        pass
    def redrawAll(self,canvas):
        pass
    def run(self):
        def redrawAllWrapper(canvas):
            canvas.delete(ALL)
            canvas.create_rectangle(0, 0, self.width, self.height,
                                    fill='white', width=0)
            self.redrawAll(canvas)
            canvas.update()

        def mousePressedWrapper(event, canvas):
            self.mousePressed(event)
            redrawAllWrapper(canvas)

        def keyPressedWrapper(event, canvas):
            self.keyPressed(event)
            redrawAllWrapper(canvas)

        def timerFiredWrapper(canvas):
            timerFired(data)
            redrawAllWrapper(canvas, data)
            # pause, then call timerFired again
            canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
        # Set up data and call init
        class Struct(object): pass
        data = Struct()
        data.width = width
        data.height = height
        data.timerDelay = 100 # milliseconds
        init(data)
        # create the root and the canvas
        root = Tk()
        canvas = Canvas(root, width=data.width, height=data.height)
        canvas.pack()
        # set up events
        root.bind("<Button-1>", lambda event:
                                mousePressedWrapper(event, canvas, data))
        root.bind("<Key>", lambda event:
                                keyPressedWrapper(event, canvas, data))
        timerFiredWrapper(canvas, data)
        # and launch the app
        root.mainloop()  # blocks until window is closed
        print("bye!")

debugger=graphicalDebugger(300,300)
debugger.run()


'''
class arrayInterface(threading.Thread):
    def __init__(self,queue,outQueue):
        threading.Thread.__init__(self)
        self.queue=queue
        self.running=True
    def run(self):
        while self.running:
            arrayToGraph=self.queue.get()
            self.queue.task_done()
    def stop(self):
        self.running=False
'''
