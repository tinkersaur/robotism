#!/usr/bin/env python
""" Simple occupancy-grid-based mapping. 

Author: David Gutow
Version: 9/2017
"""

from math import *

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
    and 'y' increasing going up (increasing row index).
    
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
        Xpos       --  The position of the car which the map is relative to 
        Ypos       --  The position of the car which the map is relative to

    """
    
    ###########################################################################
    # __init__   Note - the position of the car when this map was created or
    # updated is always in the center and at Y = 0.0
    # 
    
    def __init__(self, resolution=10, nCols=50, nRows=50, nAngles = 72, distance=0, angle=0):
        """ 
        Construct an empty occupancy grid.              
        """
        self.resolution = resolution
        self.nCols      = nCols 
        self.nRows      = nRows       
        self.Xpos       = nCols * resolution / 2
        self.Ypos       = 0.0
        self.nAngles=nAngles
        self.clear (distance, angle)
    # end
    
    ###########################################################################
    # clear   Clears and re-initializes the grid
    #     
    
    def clear (self, distance=0, angle= 0):
        self.distance   = distance
        self.angle      = angle     

		# TODO: maybe avoid reallocation. Set to zero instead.
        self.grid = [[ [0.0, 0.0] for x in range(self.nCols)] for y in range(self.nRows)]    
        self.hist=[0.0 for a in range(self.nAngles) ]
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
        
        # Position of the car when this measurement was made
        distTravelled = carCumDist   - self.distance
        angleDiff     = carCurrAngle - self.angle
        carXpos       = (sin(radians(angleDiff)) * distTravelled)
        carYpos       = (cos(radians(angleDiff)) * distTravelled)
        carXpos       += self.Xpos
        carYpos       += self.Ypos
        
        # Now figure out where the range point is relative to the car's 
        # current position
        scanAngle     = angleDiff + scanAngle
        rangeXpos     = sin(radians(scanAngle)) * scanDist
        rangeYpos     = cos(radians(scanAngle)) * scanDist         
        
        Xpos          = rangeXpos + carXpos
        Ypos          = rangeYpos + carYpos
        self.enterPoint (Xpos, Ypos)
    # end    
    
    ###########################################################################
    # enterPoint - enters an objects position into the grid  
    
    def enterPoint(self, x, y):
        col = int (x / self.resolution)
        row = int (y / self.resolution)
        
        # Throw the point out if out of range
        if (col < 0 or col >= self.nCols):
            return
        if (row < 0 or row >= self.nRows):
            return
        
        # Replace any current value on the assumption that this value is more
        # accurate
        self.grid[row][col] = [x, y]
        #print ("Grid pos (%6.2f, %6.2f) - Point (%6.2f, %6.2f) entered at row/col %2d/%2d" % 
        #   (self.Xpos, self.Ypos, x,y,row,col))
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
        deltaY     = deltaDist * cos(radians(deltaAngle))       
        deltaX     = deltaDist * sin(radians(deltaAngle))
        #print ("Recenter - Input dist-%4.1f angle-%4.1f,  Delta X/Y %4.1f/%4.1f\n" % 
        #   (dist, angle, deltaX, deltaY)),
        
        for row in range(self.nRows):
            for col in range(self.nCols):
                if (not self.isZero(row, col)):
                    [x,y] = self.grid[row][col]
                    x = x - deltaX
                    y = y - deltaY
                    # we must zero out this cell first in case the new position
                    # turns out to be in the same cell
                    self.grid[row][col] = [0.0, 0.0]   
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
        point = self.grid[xIndex][yIndex]
        x = point[0]
        y = point[1]
        if (x > -0.001 and x < 0.001 and y > -0.001 and y < 0.001):
            return True
        return False
    # end   

    def setGoalDirection(angle):
        self.goal_dir=angle

    # angle in radians to index of the histogram:
    def angle2index(self, a):
        return int((a/pi+1)/2*self.nAngles+0.5) % self.nAngles 

    def index2angle(self, k):
        return ((k*1.0)/self.nAngles-0.5)*2*pi

    # Take the occupancy grid, and calculate
    # self.speed, and self.turn_angle
    def calcForceHist(self):

        # precalculate some constants:
        nRows=len(grid)
        nCols=len(grid[0])
        w0=1.0
        # maximum distance to the obstacle.
        d_max=hypot(self.resolution*nCols/2, self.resolution*nRows)
        dw=1.0
        da = 2*pi / self.nAngles
    
        # The treshold in the histogram for finding valleys
        # The value of 0.1 is a total guess.
        Fmax = 0.1

        # The minimal width of the valley.
        # Totally a guess at this time.
        Lmin = 5 

        # reset histogram.
        for i in range(len(hist)):
            self.hist[i]=0.0

        # Deposit weights from the grid to the hypot
        # rx,ry -- the position of the obstacle
        # d -- distance to the obstacle
        # a -- angle of the obtacle.
        # w -- the weight of the obstacle.
        # relative to the robot.
        for j in range(nRows):
            for i in range(nCols):
                rx = grid[i][j][0] - xPos
                ry = grid[i][j][1] - yPos

                a=atan2(ry,rx)
                d=hypot(rx,ry)

                k=self.angle2index(a)

                w=w0-dw*(d/d_max)
                self.hist[k]+=w

        # Smooth the histogram.

        # TODO: Do we need it??? 

        # Find the valleys
        # This array contains the array of the start of the
        # valley and its width.
        valleys = []
        for i in range(self.nAngles):
            if self.hist[i]>Fmax:
                continue

            n=1
            while self.hist[ (i+j) % self.nAngles ] > Fmax:
                n+=1

            if n>=Lmin:
                valleys.append([i,n])
                
        # Find the best direction

        # First try to find a valley that contains the
        # goal direction. If there is such a valley,
        # then go in the middle of it.

        found = False
        k = (self.goal_dir/pi+1)/2*self.nAngles
        for v in valleys:
            if k>=v[0] and k<=v[0]+v[1]-1:
                found = True
                TurnAngle = self.index2angle(v[0] + v[1]/2.0)
                break

        # If there is no valley that contains the goal direction,
        # then try to find a valley that has an edge that would
        # be the closest match.

        TurnAngle = 0.0
        if not found:
            for v in valleys:
                edge = self.index2angle(v[0]) 
                if not found :
                    TurnAngle = edge
                    found = True
                elif fabs(TurnAngle-self.goal_dir) < fabs(edge-self.goal_dir):
                    TurnAngle = edge
                    edge = self.index2angle(v[0]+v[1]) 
                elif fabs(TurnAngle-self.goal_dir) < fabs(edge-self.goal_dir):
                     TurnAngle = edge

        Vmax=1.0
        Vmin=0.1

        # Large obstacle. The value is a total guess at this point.
        Hmax = 5

        ObstacleSize = min(1.0, hist[angle2index(0)]/Hmax)

        Speed = (Vmax-Vmin)*(1.0 - ObstacleSize) + Vmin

        self.speed = Speed
        self.turn_angle = TurnAngle

    ###########################################################################
    # printGrid - 
    def printGrid (self, str):
        print (str)
        for row in range (self.nRows-1, -1, -1):
            print ("Row %2d:" % row),
            for col in range (self.nCols):
                if (self.isZero(row, col)):
                    print ("%-11s" % "    ---   "),
                else:
                    [x, y] = self.getValue (row, col)
                    print ("(%4.1f,%4.1f)" % (x, y)),
                # end if
            # end for col
            print ("\n")
        # end for row
        print ""
    # end def
    
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
    g = Grid(resolution=10, nCols=10, nRows=9, distance=0, angle=0)
    g.enterRange (35, -10, 0,  10)  # Resulting point should be at (43.92, 34.47)
    g.enterRange (00,  0, 35, -10)  # Resulting point should be at (43.92, 34.47)    
    g.enterRange (35, -10, 20, 10)  # Resulting point should be at (43.92, 54.47)  
    g.enterRange (25,  10, 20, 10)  # Resulting point should be at (61.18, 43.41)       
    g.printGrid("\nAFTER POINTS ENTERED:") 
    
    g.recenterGrid(10, 0)           # All points should shift down by -10
    g.printGrid("\nAFTER RE-CENTERING (10,0):")   
    g.recenterGrid(20, 45)          # All points should shift by -7.1, -7.1
    g.printGrid("\nAFTER RE-CENTERING (10,45):")   
    
    g.clear (distance=0, angle= 0)  # Start with a fresh slate
    g.printGrid("\nAFTER CLEARING GRID:")       
    g.enterRange  (10, 0, 10, 0)    # Resulting point should be at (50.0, 20.0)   
    g.recenterGrid(20, 45)          # point should shift to (50-14.1), 20.0-14.1)
    
    g.printGrid("AFTER CLEARING AND RE-CENTERING (10,45):") 



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