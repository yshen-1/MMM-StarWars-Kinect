# To download PyUSB with conda:
# conda install -c trentonoliphant pyusb=1.0.0b2
# conda install -c m-labs libusb=1.0.20
import math
import numpy as np
import usb.core
import usb.util
from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *


def checkIfKinectConnected():
    # Actually checks if a microsoft device is connected
    # Microsoft vendorID = 0x045e
    device = usb.core.find(idVendor = 0x045e)
    if device != None:
        return True
    else:
        return False

class KinectHandler(object):
    def __init__(self):
        self.kinectBodyStream=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
        self.kinectDepthStream=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
        self.kinectColorStream=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color)

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
        #Frame gets returned as numpy array
        frame=None
        if self.kinectDepthStream.has_new_depth_frame():
            frame=self.kinectDepthStream.get_last_depth_frame()
        return frame

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
            if not checkIfKinectConnected():
                self.isRunning=False

        self.kinect.end()

Kinect = KinectHandler()
