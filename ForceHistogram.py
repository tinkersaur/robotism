from math import *

class ForceHistogram:

    def __init__(self):
        self.setHistParam(72)
        self.goal_dir=0

    # Set the number of elemts in the histogram
    def setHistParam(self, nAngles):
        self.nAngles=nAngles
        self.hist=[0.0 for a in range(self.nAngles) ]


    def setGridParam(self, resoultion, x0, y0):
        self.resolution=resolution
        self.x0=x0
        self.y0=y0

    def setRobotPos(xPos, yPos):
        self.xPos=xPos
        self.yPos=yPos

    def setGrid(self, grid):
        self.grid=grid

