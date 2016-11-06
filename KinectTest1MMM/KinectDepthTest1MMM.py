from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *
import pygame
import ctypes
import sys
import _ctypes

#Draws video feed of depth data from Kinect
def draw_frame(kinect,frame,target_surface):
        target_surface.lock()
        address=kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address,frame.ctypes.data,frame.size)
        del address
        target_surface.unlock()
def getDepthData():
	kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
	pygame.init()
	screen=pygame.display.set_mode((960,540),pygame.HWSURFACE|pygame.DOUBLEBUF,32)
	frame_surface=pygame.Surface((kinect.depth_frame_desc.Width,kinect.depth_frame_desc.Height),0,32)
	print(frame_surface)
	clock=pygame.time.Clock()
	while True:
		if kinect.has_new_depth_frame():
			frame=kinect.get_last_depth_frame()
			draw_frame(kinect,frame,frame_surface)
		aspect_ratio=float(frame_surface.get_height())/float(frame_surface.get_width())
		target_height=int(aspect_ratio*screen.get_width())
		surfaceToDraw=pygame.transform.scale(frame_surface,(screen.get_width(),target_height))
		screen.blit(surfaceToDraw,(0,0))
		pygame.display.update()
		clock.tick()
getDepthData()