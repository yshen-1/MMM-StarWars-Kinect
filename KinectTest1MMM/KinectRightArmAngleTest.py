from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
import math
kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
def getRightArmElevation():
	bodies=None
	if kinect.has_new_body_frame():
		bodies=kinect.get_last_body_frame()
	if bodies!=None:
		for i in range(0,kinect.max_body_count):
			body=bodies.bodies[i]
			if body.is_tracked:
				joints=body.joints
				rightHandY=joints[PyKinectV2.JointType_HandRight].Position.y
				rightHandZ=joints[PyKinectV2.JointType_HandRight].Position.z
				rightHandAngle=math.atan(rightHandY/rightHandZ)
				return rightHandAngle



