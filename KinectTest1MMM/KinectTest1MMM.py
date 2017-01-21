from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
#Prints the position of the right hand of a person
def printHandPosition():
	kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
	bodies=None
	while True:
		if kinect.has_new_body_frame():
			bodies=kinect.get_last_body_frame()
		if bodies!=None:
			for i in range(0,kinect.max_body_count):
				body=bodies.bodies[i]
				if not body.is_tracked:
					continue
				joints=body.joints
				rightHandPosition=joints[PyKinectV2.JointType_HandRight].Position
				print rightHandPosition.x,rightHandPosition.y,rightHandPosition.z
	kinect.close()
printHandPosition()
