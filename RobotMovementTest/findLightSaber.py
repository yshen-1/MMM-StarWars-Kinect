from __future__ import print_function,division
import cv2
import numpy as np
import math
import time


class LightSaberTracker(object):
    def __init__(self, saberColor = 'blue'):
        self.saberColor = saberColor
        self.HSVRanges = {'blue': (np.array([110, 100, 100]),np.array([130, 255, 255])),
                            'red': (np.array([160, 100, 100]),np.array([179, 255, 255]))}
        self.saberHSVRange = self.HSVRanges[saberColor]
        self.debugInit()

    def track(self, frame, handPosition):
        self.frame = frame
        self.handPosition = handPosition
        isolated = self.isolateSaberColor(frame)
        trackedPoints = self.findLightSaber(isolated, handPosition)
        if trackedPoints == None:
            print('EndPoints not found')
            return None
        else:
            startPoint, endPoint = trackedPoints
        self.debugShow(isolated, startPoint, endPoint)
        if self.check(handPosition, startPoint, endPoint):
            return startPoint, endPoint
        else:
            print('saber check failed')
            return None

    def isolateSaberColor(self, frame):
        #smooth out the image and convert it to hsv
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        #remove everything but objects in the saber color
        mask = cv2.inRange(hsv,self.saberHSVRange[0], self.saberHSVRange[1])
        masked = cv2.bitwise_and(frame,frame, mask= mask)
        #remove small blobs or errors left on the frame
        masked = cv2.erode(masked, None, iterations=2)
        masked = cv2.dilate(masked, None, iterations=2)
        return masked

    def findLightSaber(self, image, handPosition):
        grid = self.getGrid(image)
        startCellPos = self.startCellPos(handPosition, grid)
        if startCellPos == None:
            return None
        startPoint = self.getEndPoint(startCellPos, grid)
        results = []
        self.getSaberCells(grid, startCellPos, results)
        endCellPos = self.getEndCellPos(results, startCellPos)
        if endCellPos == None:
            return None
        endPoint = self.getEndPoint(endCellPos, grid)
        return startPoint, endPoint

    def getGrid(self, image):
        grid = []
        for i in range(54):
            row = []
            for j in range(96):
                cell = image[i*20: (i+1)*20, j*20: (j+1)*20, 0]
                visited = False
                row.append([cell, visited])
            grid.append(row)
        return np.array(grid)

    ###############
    # Start Point Finder:
    ###############

    def startCellPos(self, handPos, grid):
        handCellPos = ( handPos[0] // 20, handPos[1] // 20)
        searchFront = [handCellPos]
        startCellPos = self.getStartCell(grid, searchFront)
        return startCellPos


    def getStartCell(self, grid, searchFront):
        directions = [[-1, -1], [-1, 0], [-1, 1],
                      [0, -1],            [0, 1],
                      [1, -1],  [1, 0],  [1, 1]]
        for cellPos in searchFront:
            for direction in directions:
                found, pos = self.getStartCellDirected(direction,cellPos, grid,
                                                                     searchFront)
                if found:
                    return pos

    def getStartCellDirected(self, direction, cellPos, grid, searchFront):
        (drow, dcol) = direction
        (row, col) = cellPos
        if row+drow >= 54 or row+drow < 0 or col+dcol >= 96 or col+dcol < 0:
            return False, None
        newCell, visited = grid[row+drow][col+dcol]
        if visited:
            return False, None
        if np.average(newCell) > 15:
            return True, (row+drow, col+dcol)
        else:
            grid[row+drow][col+dcol][1] = True
            searchFront.append((row+drow, col+dcol))
            return False, None


    ###############
    # End Point Finder:
    ###############

    def getSaberCells(self, grid, cellPos, results):
        directions = [[-1, -1], [-1, 0], [-1, 1],
                      [0, -1],           [0, 1],
                      [1, -1],  [1, 0],  [1, 1]]
        for direction in directions:
            self.getSaberCellsDirected(grid,cellPos,direction, results)
        return False

    def getSaberCellsDirected(self, grid, cellPos, direction, results):
        (row, col) = cellPos
        (drow, dcol) = direction
        if row+drow >= 54 or row+drow < 0 or col+dcol >= 96 or col+dcol < 0:
            return False
        newCell, visited = grid[row+drow][col+dcol]
        colorAvg = np.average(newCell)
        if colorAvg < 5 or visited:
            return False
        elif colorAvg > 5 and not visited:
            grid[row+drow][col+dcol][1] = True
            results.append((row+drow, col+dcol))
            return self.getSaberCells(grid, (row+drow, col+dcol), results)
        else:
            return False

    def getEndCellPos(self, results, startCellPos):
        greatestDistance = 0
        endCell = None
        for cell in results:
            distance = self.getDistance(cell, startCellPos)
            if distance > greatestDistance:
                greatestDistance = distance
                endCell = cell
        return endCell

    def getEndPoint(self, endCellPos, grid):
        (row, col) = endCellPos
        endCell = grid[row][col][0]
        topLeft = endCell[:10, :10]
        botLeft = endCell[10: , :10]
        topRight = endCell[:10, 10:]
        botRight = endCell[10:, 10:]
        subCells = ((topLeft,(0, 0)),(topRight, (0, 10)),(botRight, (10, 10)),
                        (botLeft, (10, 0)))
        maxAvg = 0
        for (cell, dpos) in subCells:
            avg = np.average(cell)
            if avg > maxAvg:
                maxAvg = avg
                endSubCell = cell
                endSubCelldpos = dpos
        found = False
        for row1 in range(10):
            for col1 in range(10):
                pixel = endSubCell[row1][col1]
                if pixel != 0:
                    pixeldPos = (row1, col1)
                    found = True
                    break
            if found:
                break
        endRow = row*20 + endSubCelldpos[0] + pixeldPos[0]
        endCol = col*20 + endSubCelldpos[1] + pixeldPos[1]
        endPoint = (endRow, endCol)
        return endPoint

    def getDistance(self, point1, point2):
        (x1, y1) = point1
        (x2, y2) = point2
        return ((x1-x2)**2 +(y1-y2)**2)**0.5

    ###############
    # Debug:
    ###############
    def debugInit(self):
        self.debugOn = False
        self.imageDebugDir = r'C:/Users/Arthur/Desktop/RoboticsClub/MMM-StarWars-Kinect/openCVTest'

    def debugShow(self, isolated, startPoint, endPoint):
        if self.debugOn:
            self.showPoint(startPoint, isolated)
            self.showPoint(endPoint, isolated)
            self.showPoint(self.handPosition, isolated, color = 'G')
            self.showPoint(startPoint, self.frame)
            self.showPoint(endPoint, self.frame)
            self.showPoint(self.handPosition, self.frame, color = 'G')
            cv2.imwrite(self.imageDebugDir + r"/isolatedTracked.png", isolated)
            cv2.imwrite(self.imageDebugDir + r'/frameTracked.png', self.frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def showPoint(self, point, image, color = 'R'):
        colorValues = {'B':0, 'G':1, 'R':2}
        colorValue = colorValues[color]
        (row, col) = point
        for i in range(-10, 10, 1):
            for j in range(-10, 10, 1):
                image.itemset((row+i, col+j, colorValue), 255)

    def check(self, hP, sP, eP):
        for point in (hP, sP, eP):
            if type(point) != tuple or len(point) != 2:
                return False
            for coord in point:
                if type(coord) not in (float, int):
                    return False
        (h2s, s2e) = (self.getDistance(hP, sP), self.getDistance(sP, eP))
        if h2s == 0 or s2e/h2s > 10:
            return False
        h2e = self.getDistance(hP, eP)
        h2eSlope = (eP[1]-hP[1]) / (eP[0]-hP[0])
        h2eIntercept = hP[1]-(hP[0]*h2eSlope)
        sPDeviation = sP[1]-(sP[0]*h2eSlope + h2eIntercept)
        if h2e == 0 or (sPDeviation / h2e > 0.1):
            return False
        return True


    def trackWithRuntime(self, frame, handPosition):
        startTime = time.time()
        trackValues = self.track(frame, handPosition)
        endTime = time.time()
        delay = (endTime - startTime)
        print('delay:',delay)
        return trackValues
