import cv2
import numpy as np

image = cv2.imread('c:/users/arthur/desktop/lightsaber.jpg',1)


class LightSaberTracker(object):
    def __init__(self):
        self.saberHSVRange = (np.array([5, 100, 100]),np.array([25, 255, 255]))

    def track(self, frame):
        self.frame = frame
        cv2.imshow('frame', frame)
        isolated = self.isolateSaberColor(frame)
        cv2.imshow("isolated", isolated)
        #self.findSaber(isolated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()   
    def getDistance(point1, point2):
        (x1, y1) = point1
        (x2, y2) = point2
        return ((x1-x2)**2 +(y1-y2)**2)**0.5

    def isolateSaberColor(self, frame):
        #smooth out the image and convert it to hsv
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        #remove everything but objects in the saber color
        mask = cv2.inRange(hsv,self.saberHSVRange[0], self.saberHSVRange[1])
        masked = cv2.bitwise_and(frame,frame, mask= mask)
        #remove small blobs or errors left on the frame
        #masked = cv2.erode(mask, None, iterations=2)
        #masked = cv2.dilate(mask, None, iterations=2)
        return masked

    def findSaber(self, im):
        imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,127,255,0)
        contours = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]
        print(contours)
        cv2.drawContours(self.frame,contours,-1, (0,255,0), 3 )
        """for contour in contours:
            rect = cv2.minAreaRect(contour)
            (length, width) = rect[1]
            if lenght/width > 10:
                cv2.drawContours(im, [contour], 0, (0,255,0), 3)
            #box = cv2.boxPoints(rect)
            #print(length, width)"""


tracker = LightSaberTracker()
tracker.track(image)




