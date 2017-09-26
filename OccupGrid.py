#!/usr/bin/env python
""" Simple occupancy-grid-based mapping. 

Author: David Gutow
Version: 9/2017
"""

###############################################################################
# Class Grid
###############################################################################

class Grid(object):
    """ 
    The Grid class stores an occupancy grid as a two dimensional array.
    Each cell in the grid can be thought of as a bin holding a position 
    of an obstacle as detected by the sensor.  Each cell only holds one 
    position; if another obstacle is detected in the same cell (in 
    same vicinity) it is assumed to be the same obstacle.  The newer
    position replaces the older one, on the assumption that the vehicle
    is closer to the obstacle and thus the measurement is more accurate.
    
    (Possible future enhancement - look at the 8 surrounding cells to 
    see if there is an entry within a set distance say 'resolution' and
    if so, assume that is the original for the current point)
    
    The origin (0,0) of the grid is considered to be the lower left
    corner with 'x' increasing to the right (increasing column index)
    and 'y' increasing going up (incrasing row index).
    
    The grid is dynamic in that it is referenced to the vehicle.  As the
    vehicle moves the grid moves with it.  The distance and angle variables
    are the vehicle cumulative distance and angle which correspond to the
    current grid. Angle is assumed to be an absolute angle in world 
    reference frame, something like we might obtain from a compass. 
    
    Public instance variables:
        nCols      --  Number of columns in the occupancy grid.
        nRows      --  Number of rows in the occupancy grid.
        resolution --  Width & height of each grid square in cm. 
        distance   --  The distance (position) this grid is referenced from
                       (The cumulative distance the car has travelled)
        angle      --  The angle this grid is referenced from
        grid       --  integer array with nRows rows and nCols columns.

    """
    
    ###########################################################################
    # __init__   
    
    def __init__(self, resolution=10, nCols=50, nRows=50, distance=0, angle=0):
        """ 
        Construct an empty occupancy grid.              
        """
        self.resolution = resolution
        self.nCols      = nCols 
        self.nRows      = nRows 
        self.distance   = distance
        self.angle      = angle        
        
        self.grid = [[ [0.0, 0.0] for x in range(nCols)] for y in range(nRows)]
    # end

    ###########################################################################
    # enterRange - enters a point into the map.  The vehicle only knows about
    # it's cumulative, and it's currAngle.  The scanner knows it's scan angle 
    # and range distance.
    # Parameters:
    #   carCumDist  - cumulative distance the car has travelled
    #   carCurrAngle- current angle of the car relative to left/right wall (deg).
    #                 positive values are angled to the right
    #                 negative values are angled to the left
    #   scanDist    - range of the object (cm)
    #   scanAngle   - angle of scanner relative to the heading of the car (deg)
    #                 positive values are to the right
    #                 negative values are to the left    
    
    def enterRange (self,  carCumDist, carCurrAngle, scanDist, scanAngle):
        # The last time the map was translated/rotated the map was set so 
        # that it's angle was perpindiclar to the car and the car's position
        # was at point [X = (gridWidth * gridResolution / 2), Y = 0.0]
        # Figure out where the car is relative to this position
        distTravelled = carCumDist   - self.distance
        angleDiff     = carCurrAngle - self.angle
        carXpos       = sin(rad(angleDiff)) * distTravelled
        carYpos       = cos(rad(angleDiff)) * distTravelled
        
        # Now figure out where the range point is relative to the car's 
        # current position
        scanAngle     = angleDiff + scanAngle
        rangeXpos     = sin(rad(scanAngle)) * scanDist
        rangeYpos     = cos(rad(scanAngle)) * scanDist         
        
        Xpos          = rangeXpos + carXpos
        Ypos          = rangeYpos + carYpos
        self.enterRange(Xpos, Ypos)
    # end    
    
    ###########################################################################
    # enterPoint - enters an objects position into the grid  
    
    def enterPoint(self, x, y):
        xIndex = int (x / self.resolution)
        yIndex = int (y / self.resolution)
        
        # Throw the point out if out of range
        if (xIndex < 0 or xIndex >= self.nCols):
            return
        if (yIndex < 0 or yIndex >= self.nRows):
            return
        
        # Replace any current value on the assumption that this value is more
        # accurate
        self.grid[xIndex][yIndex] = [x, y]
    # end

    ###########################################################################
    # recenterGrid - translates and rotates the map to the new distance 
    # and angle.  NOTE - rather than creating a second grid and transferring
    # all the rotated/translated points to it, we do it within the same grid.
    # CAUTION though, this method only works if we are going forward, e.g.
    # moving the data in the grid generally downward.
    def recenterGrid(self, dist, angle):
        # Calculate the delta X,Y that all points will be moved by
        deltaAngle = angle - self.angle
        deltaDist  = dist  - self.distance
        deltaY     = deltaDist * sin(radians(90 - deltaAngle))
        deltaX     = deltaDist * cos(radians(90 - deltaAngle))
        
        for row in range(self.nRows):
            for col in range(self.nCols):
                if (not isZero(self.grid[col][row])):
                    [x,y] = self.grid[col][row]
                    x = x - deltaX
                    y = y - deltaY
                    # we must zero out this cell first in case the new position
                    # turns out to be in the same cell
                    self.grid[col][row] = 0.0   
                    self.enterPoint(x,y)
                # end
            # end col
        # end row
        self.angle    = angle
        self.distance = dist
        pass
    # end
    
    ###########################################################################
    # getValue   
    def getValue (self, xIndex, yIndex):
        return self.grid[xIndex][yIndex]
    # end
    
    ###########################################################################
    # isZero - checks if a map cell contains no data, taking into account 
    # floating point roundoff
    def isZero (self, xIndex, yIndex):
        [x, y] = self.grid[xIndex][yIndex]
        if (x > -0.001 and x < 0.001 and y > -0.001 and y < 0.001):
            return True
        return False
    # end   

    ###########################################################################
    # printGrid - 
    def printGrid (self, str):
        print (str)
        for row in range (self.nRows-1, -1, -1):
            print ("Row %2d: " % row),
            for col in range (self.nCols):
                if (self.isZero(row, col)):
                    print ("%8s" % "   --   "),
                else:
                    [x, y] = self.getValue (row, col)
                    print (" (%2.0f,%2.0f)" % (x, y)),
                # end if
            # end for col
            print ("\n")
        # end for row
        print ""
    # end      
# end
    

###############################################################################
# Test code
###############################################################################

""" 
We need to represent an area abut 52 feet wide (2 * 16 feet for the track and
an additional 2 * 10 feet for the track veering off to the right/left during 
turns).  The 16 feet track width is included twice since we always keep the car 
centered in the X center of the grid (Xcenter,0), and then the car can be 
either all the way to the right or all the way to left.  The height of the map
corresponds to 16 feet high, the max range of the sensor. Since the grid cells 
are only bins anyway we can set the resolution value fairly large.
"""
gridResolution  = 15    # 15 cm = ~6"/cell
gridWidth       = 104   # 104 cells * 6"/cell = 52 feet wide
gridHeight      = 32    # 32 cells * 6"/cell = 16 feet high


if __name__ == '__main__':
    g = Grid(resolution=10, nCols=10, nRows=5, distance=0, angle=0)
    g.enterPoint(25.0, 5.0)
    g.enterPoint(35.0, 35.0)   
    g.printGrid("testing:")
    
    
    

###############################################################################
# Attic code
###############################################################################           
"""       
        # Find the left and right walls in the map. Search the first few rows on
        # either side for the leftmost and rightmost obstacles.
        nValsFound = 0
        sumXvals   = 0.0
        for xIndex in range(0, maxWallDist):
            for yIndex in range(0, 4):
            [x,y] = map.getValue[xIndex][yIndex]
                if (not isZero(x) and not isZero(y):
                    sumXvals += x
                    nValsFound += 1
            # end for
        # end for
        if nValsFound > 0:
            leftWallPos = sumXvals / nValsFound
        else:
            leftWallPos = 0.0
        # end
        
        # Now look for the right wall. Start at the rightmost cell and work in
        nValsFound = 0
        sumXvals   = 0.0
        for xIndex in range(gridWidth-1, gridWidth-maxWallDist, -1):
            for yIndex in range(0, 4):
            [x,y] = map.getValue[xIndex][yIndex]
                if (not isZero(x) and not isZero(y):
                    sumXvals += x
                    nValsFound += 1
            # end for
        # end for
        if nValsFound > 0:
            rightWallPos = sumXvals / nValsFound
        else:
            rightWallPos = 0.0
        # end    
        
        # Now calculate the car's position in the map reference frame
        if (not isZero (leftWallPos) and not isZero(rightWallPos) and
            not isZero (carLeftDist) and not isZero(carRightDist)):
            # we found both walls  and we have both left/right distances.  
            # Average the resulting position
            carXpos = ((leftWallPosition + carLeftDist) + 
                       (rightWallPos - carLeftDist)) / 2
"""        