from __future__ import print_function,division
import cv2
import numpy as np
import math



image = cv2.imread('c:/users/arthur/desktop/blueLightSaberRoom.jpg',1)


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
        startPoint, endPoint = self.findLightSaber(isolated, handPosition)
        self.debugShow(isolated, startPoint, endPoint)
        return startPoint, endPoint

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
        startPoint = self.getEndPoint(startCellPos, grid)
        results = []
        self.getSaberCells(grid, startCellPos, results)
        endCellPos = self.getEndCellPos(results, startCellPos)
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
        subCells = ((topLeft,(2, 2)),(topRight, (2, 7)), (botRight, (7, 7)),
                        (botLeft, (7, 2)))
        maxAvg = 0
        (drow, dcol) = (5, 5)
        for (cell, dpos) in subCells:
            avg = np.average(cell) 
            if avg > maxAvg:
                maxAvg = avg
                (drow, dcol) = dpos
        endPoint = (row*20 + drow , col*20 + dcol)
        return endPoint

    def getDistance(self, point1, point2):
        (x1, y1) = point1
        (x2, y2) = point2
        return ((x1-x2)**2 +(y1-y2)**2)**0.5

    ###############
    # Debug:
    ###############
    def debugInit(self):
        self.debugOn = True
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



   

tracker = LightSaberTracker('blue')
tracker.track(image, (770, 990))




