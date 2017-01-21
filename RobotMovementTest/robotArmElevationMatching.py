from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
import math, threading, time, sys
from MMM import MMM
class kinectHandler(object):
	def __init__(self):
		self.kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
	def getRightArmElevation(self):
		bodies=None
		if self.kinect.has_new_body_frame():
			bodies=self.kinect.get_last_body_frame()
		if bodies!=None:
			for i in range(0,self.kinect.max_body_count):
				body=bodies.bodies[i]
				if body.is_tracked:
					joints=body.joints
					rightHandY=joints[PyKinectV2.JointType_HandRight].Position.y
					rightHandZ=joints[PyKinectV2.JointType_HandRight].Position.z
					rightHandAngle=math.atan(rightHandY/rightHandZ)
					return rightHandAngle

class robotTracker(object):
	def __init__(self):
		self.handler=kinectHandler()
		self.mmm=MMM('COM4')
		time.sleep(4)
		self.thread = threading.Thread(target=self.updateRobot, args=())
		self.thread.daemon = True            
		self.thread.start()
	def updateRobot(self):
		while True:
			self.mmm.clampAll();  
			self.mmm.update();
			time.sleep(0.1);
			self.mmm.parseData();
	def run(self):
		while True:
			angle=self.handler.getRightArmElevation()
			if angle!=None:
				angle=angle*180/math.pi
				if angle>60: angle=60
				elif angle<-60: angle=-60
				self.mmm.rotateElbows(0,angle)
				time.sleep(2)
		self.mmm.ser.close()
		quit()   

robotHandler=robotTracker()
robotHandler.run()