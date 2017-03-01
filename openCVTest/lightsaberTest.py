'''
Lightsaber Example
==================
Uses OpenCV to track a colored object and sends the velocity
to a Pure Data app that produces corresponding sounds / hums. Use a brightly
colored object for best results (highlighter-colored straws, highlighters)
Use the knobs that pop up to select hue, hueWidth, and minimumBlobSize.
Requirements:
-Pure Data (get the full version)
-PyOSC installed
-Numpy/Scipy. Might need to install Python(X,Y) in order to get it now though.
-OpenCV (copy the cv2.pyd file from C:\opencv\build\python\2.7 to C:\Python27\Lib\site-packages)
Notes:
-Other Python libraries for audio seemed too laggy/difficult (PyAudio, Pyo)
'''

#Source: https://github.com/gauss1181/PongOpenCV/blob/master/lightsaber.py

import numpy as np
import cv2
import time
from OSC import OSCClient, OSCMessage
client = OSCClient()
client.connect( ("localhost", 9001) )


########## CONFIG ############
#Webcam width / height
width = 320
height = 240

#Minimum brightness of the detected object out of 255
minBrightness = 80

#Will extract this color +/- 5 in Hue space
## These are adjustable parameters in the knobs that pop up ##
#They are initialized here so you don't have to change it all the time
global hue, hueWidth, minContourArea
hue = 25     #Red = 5, Orange = 10, Yellow = 25, Green =
hueWidth = 4 #How much on either side of the hue do we allow?
#Minimum size of the contour to select for tracking
minContourArea = height*width/(15*15)
##############################

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH,width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,height)

cv2.namedWindow('Original')
cv2.namedWindow('Processed')
cv2.namedWindow('Extracted Hue')
cv2.namedWindow('knobs')

velocity_old = 0
center_x_old = 0
center_y_old = 0

element = cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))

def update(dummy=None):
    global hue
    hue = cv2.getTrackbarPos('hue','knobs')
    minContourArea = cv2.getTrackbarPos('minContourArea','knobs')

    #                           var, max, function
cv2.createTrackbar('hue','knobs',hue,179,update)
cv2.createTrackbar('hueWidth','knobs',hueWidth,80,update)
cv2.createTrackbar('minContourArea','knobs',minContourArea,height*width/4,update)
update()



clash_wait = 0
while (True):
    ret, img = cam.read()
    if (img == None):
        print "No frame read from camera. Exiting"
        break

    cv2.imshow('Original',img)


    #Convert image to Hue, Saturation, and Value(Brightness) space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #We want to select a specific color, or hue.
    #OpenCV's range is 0-180 for hue, and 0-255 for S&V
    thresholded = cv2.inRange(hsv, np.array((hue-hueWidth/2.0, float(minBrightness),float(minBrightness))), np.array((hue+hueWidth/2.0, 255.,255.)))

    #Might help improve speed of finding contours, but commenting out for now
    #cv2.erode(thresholded,element)

    cv2.imshow('Extracted Hue', thresholded)

    #Find the blobs ("contours") in the image
    blobs_draw = np.zeros((height,width,3),np.uint8)
    i=0
    _,contour,hier = cv2.findContours(thresholded,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contour:
        #Contours must be somewhat big
        if (cv2.contourArea(cnt) < minContourArea):
            np.delete(contour,i,0)
        else:
            cv2.drawContours(blobs_draw,[cnt],0,255-i*20,-1)
            i = i+1

            #Find center of contour using boundingRect (slightly faster?)
            x,y,w,h = cv2.boundingRect(cnt)

            center_x = x + w/2
            center_y = y + h/2
            cv2.circle(blobs_draw, (center_x, center_y), 3, (0, 0, 255), -1)

            #Find velocity of point (assuming just one light for now)
            velocity = np.sqrt(np.square(np.abs(center_x - center_x_old)) + np.square(np.abs(center_y - center_y_old)))

            center_x_old = center_x
            center_y_old = center_y

            ## Humming Sound ##
            #Change amplitude of hum based on velocity of moving object (all the time)


            ## Clash Detection ##
            #Wait for a fast motion, and trigger a clash.
            #Then wait a pre-defined period of time before clashing again
            clash_wait += 1
            if (velocity > 30.0 and clash_wait > 10):
                print("L")
                clash_wait = 0
            #If we've returned to a slow speed, that's fine too
            if (velocity < 10.0):
                clash_wait = 10

    cv2.imshow('Processed',blobs_draw)

    ch = 0xFF & cv2.waitKey(1)
    if (ch == 27): #ESC key
        cv2.destroyAllWindows()
        cam.release()
        break
