import cv2
import numpy as np
import math

image = cv2.imread('c:/users/arthur/desktop/blueLightSaberRoom.jpg',1)


class LightSaberTracker(object):
    def __init__(self):
        # red range : [5, 100, 100], [25, 255, 255]
        # blue range  : [230, 100, 100] [250, 255, 255]
        self.saberHSVRange = (np.array([110, 100, 100]),np.array([130, 255, 255]))

    def track(self, frame, handPosition):
        self.frame = frame
        isolated = self.isolateSaberColor(frame)
        endpoint = self.findLightSaber(isolated, handPosition)
        self.debugShow(isolated)
        return endpoint

    def getDistance(self, point1, point2):
        (x1, y1) = point1
        (x2, y2) = point2
        return ((x1-x2)**2 +(y1-y2)**2)**0.5

    def debugShow(self, isolated):
        cv2.imshow("isolatedtracked", isolated)
        cv2.imshow('frame', self.frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows() 

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
        results = []
        self.getSaberCells(grid, startCellPos, results)
        endCellPos = self.getEndCell(results, startCellPos)
        endPoint = self.getEndPoint(endCellPos, grid)
        return endpoint

    def getGrid(self, image):
        grid = []
        for i in range(54):
            row = []
            for j in range(96):
                cell = image[i: i+20,j: j+20, 0]
                visited = False
                row.append([cell, visited])
            grid.append(row)
        return grid

    def startCellPos(self, handPos, grid):
        handCell = ( handPos[0] % 20, handPos[1] % 20)
        found, pos = self.getStartCell(handCell, grid)
        return pos


    def getStartCell(self, cell, grid):
        directions = [[-1, -1], [-1, 0], [-1, 1],
                      [0, -1],          , [0, 1],
                      [1, -1],  [1, 0],  [1, 1]]
        for direction in directions:
            found, pos = self.getStartCellDirected(direction,cell, grid)
            if found:
                return found, pos

    def getStartCellDirected(self, direction, cell, grid):
        (drow, dcol) = direction
        (row, col) = cell
        newCell = grid[row+drow][col+dcol][0]
        if np.average(newCell) > 15:
            return True, (row+drow, col+dcol)
        else:
            return self.getStartCell((row+drow, col+dcol), grid)


    def getSaberCells(self, grid, cellPos, results):
        directions = [[-1, -1], [-1, 0], [-1, 1],
                      [0, -1],          , [0, 1],
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
        else:
            grid[row+drow][col+dcol][1] = True
            results.append((row+drow, col+dcol))
            return self.getSaberCells(grid, (row+drow, col+dcol), results)

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
        endCell = grid[row][col]
        topLeft = endCell[:10, :10]
        botLeft = endCell[10: , :10]
        topRight = endCell[:10, 10:]
        botRight = endCell[10:, 10:]
        subCells = ((topLeft,(2, 2)),(topRight, (2, 7)), (botRight, (7, 7)),
                        (botLeft, (7, 2)))
        endCell = None
        maxAvg = 0
        for cell in subCells:
            avg = np.average(cell[0]) 
            if avg > maxAvg:
                maxAvg = avg
                endCell = cell
        (drow, dcol) = endCell[1]
        endpoint = (row*20 + drow , col*20 + dcol)

    def showEndPoints(self, point1, point2, image):
        (row1, col1) = point1
        (row2, col2) = point2
        for i in range(-10, 10, 1):
            for j in range(-10, 10, 1):
                image.itemset((row1+i, col1+j, 2), 255)
                image.itemset((row2+j, col2+j, 2), 255)


   

tracker = LightSaberTracker()
tracker.track(image)




