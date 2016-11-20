from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
import math

class kinectHandler(object):
	def __init__(self):
		self.kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
	def getRightArmElevation():
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



