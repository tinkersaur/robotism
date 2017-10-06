#!/usr/bin/env python
""" Simple occupancy-grid-based mapping. 

Author: David Gutow
Version: 9/2017
"""

from math import *

###############################################################################
# Class Grid
###############################################################################

class FineGrid(object):
    """ 
    The Grid class stores an occupancy grid as a two dimensional array.
    Each cell in the grid can be thought of as a bin holding a position 
    of an obstacle as detected by the sensor.  Each cell only holds one 
    position; if another obstacle is detected in the same cell (in 
    same vicinity) it is assumed to be the same obstacle.  The newer
    position replaces the older one, on the assumption that the vehicle
    is closer to the obstacle and thus the measurement is more accurate.
    
    (Possible future enhancement - look at the 8 surrounding cells to 
    see if there is an entry within a set distance say 'dx' and
    if so, assume that is the original for the current point)
    
    The origin (0,0) of the grid is considered to be the lower left
    corner with 'x' increasing to the right (increasing column index)
    and 'y' increasing going up (increasing row index).
    
    The distance and angle variables
    are the vehicle cumulative distance and angle which correspond to the
    current grid. Angle is assumed to be an absolute angle in world 
    reference frame, something like we might obtain from a compass. 
    
    Public instance variables:
        nCols      --  Number of columns in the occupancy grid.
        nRows      --  Number of rows in the occupancy grid.
        distance   --  The distance (position) this grid is referenced from
                       (The cumulative distance the car has travelled)
        angle      --  The angle this grid is referenced from
        grid       --  integer array with nRows rows and nCols columns.
        (carX,carY) --  The position of the car which the map is relative to 
        carA        -- the direction of the car
        dx,dy       --  Width & height of each grid square in cm. 
        (x0,y0,dx,dy) -- the coeficients used to tranfrom the
                         grid point into real world

    """
    
    ###########################################################################

    def __init__(self):
        """ 
        Construct an empty occupancy grid.              
        """
        self.nCols      = 25 
        self.nRows      = 25      
        self.nAngles = 72
        self.dx = 20.0
        self.dy = 20.0
        # The ray width in degrees
        self.rayWidth =5
        self.carWidth = 10
        self.carLength = 15
        self.turnAngle = 0
        self.speed = 0
        self.goal_dir = 0

    def setGridSize(self, nCols=50, nRows=50):
        self.nCols      = nCols 
        self.nRows      = nRows       
    
    def setGridResolution(self, resolution=10):
        self.dx = resolution
        self.dy = resolution
    
    def setHistSize(self, nAngles=72):
        self.nAngles=nAngles

    def setRayWidth(self,  val):
        self.rayWidth = val

    def setCarSize(self, width, length):
        self.carWidth = width
        self.carLength = length

    def reset(self):
        self.distance   = 0.0
        self.angle      = 0.0     
        self.carX       = 0
        self.carY       = 0
        self.carA       = 0.0
        self.x0=-self.nCols*self.dx/2
        self.y0=-self.nRows*self.dy/2
        self.grid = [[ 0.5 for x in range(self.nCols)] for y in range(self.nRows)]    
        self.hist=[0.0 for a in range(self.nAngles) ]
        self.scanAngles = []
        self.scanDists = []

    def clearScan(self):
        self.scanAngles = []
        self.scanDists = []

    # returns ture if the balue was placed
    # successfully, and false, if the value is not
    # on the grid
    def isOnGrid(self, x, y):
        j=floor((x-self.x0)/self.dx)
        i=floor((y-self.y0)/self.dy)
        if 0<=i and i<nCol and 0<=j and j<nRows:
            return True
        else:
            return False

    # returns ture if the balue was placed
    # successfully, and false, if the value is not
    # on the grid
    def setGridVal(self, x, y, val):
        #print "grid step: ", self.dx, self.dy
        #print "grid step: ", self.x0, self.y0
        
        # TODO: round() works better then floor(). Not sure why.
        i=int(floor((x-self.x0)/self.dx))
        j=int(floor((y-self.y0)/self.dy))
        if 0<=i and i<self.nCols and 0<=j and j<self.nRows:
            self.grid[j][i]=val
            return True
        else:
            return False

    # Iterate over each point on a line, and
    # call a function cb(rowIndex, colIndex).
    # The coordinates are floats in the "Grid coord system".
    def setLineVal(self, X0, Y0, X1, Y1, val):
        kx=X1-X0
        ky=Y1-Y0
        d=hypot(kx,ky)
        h=int(ceil(1.1*max(fabs(kx/self.dx), fabs(ky/self.dy))))
        for p in range(0,h):
            x=X0 + kx*p/h
            y=Y0 + ky*p/h
            onGrid = self.setGridVal(x, y, val)
            if not onGrid:
                return

    def enterScan(self, angle, dist):
        self.scanAngles.append(angle)
        self.scanDists.append(dist)
        # We assume that the car is always on the grid.

        kx=cos(radians(self.carA + angle))*dist
        ky=sin(radians(self.carA + angle))*dist
       
        d=hypot(kx,ky)
        mx=-ky/d
        my= kx/d
       
        rw=tan(radians(self.rayWidth))*d

        #Figure out the number of steps:
        #h=int(ceil(5*max(fabs(kx/self.dx), fabs(ky/self.dy))))

        # TODO: this is not very efficient. Not sure why
        # I need 10 but it does not work very well with a lower
        # number.
        h=int(50*hypot(kx/self.dx,ky/self.dy))
        for p in range(0,h):
            x=self.carX + kx*p/h
            y=self.carY + ky*p/h
            onGrid = self.setGridVal(x,y,0.0)
            if not onGrid:
                break

            l=rw*p/h
            self.setLineVal(x,y, x+l*mx, y+l*my, 0.0)
            self.setLineVal(x,y, x-l*mx, y-l*my, 0.0)


        x=self.carX + kx
        y=self.carY + ky

        self.setLineVal(x,y, x+rw*mx, y+rw*my, 1.0)
        self.setLineVal(x,y, x-rw*mx, y-rw*my, 1.0)  
 
    
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
        #   (self.carX, self.carY, x,y,row,col))
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

    ###########################################################################

    def setGoalDirection(self, angle):
        self.goal_dir=angle

    ###########################################################################
    # angle in radians to index of the histogram:
    def angle2index(self, a):
        return (a/pi+1)/2*self.nAngles 

    ###########################################################################

    def index2angle(self, k):
        return ((k*1.0)/self.nAngles-0.5)*2*pi

    ###########################################################################

    # Take the occupancy grid, and calculate
    # self.speed, and self.turn_angle
    def calcForceHist(self):

        # precalculate some constants:
        w0=1.0
        # maximum distance to the obstacle.
        d_max=hypot(self.dx*self.nCols, \
            self.dy*self.nRows)
        da = 2*pi / self.nAngles
    
        # The treshold in the histogram for finding valleys
        # The value of 0.1 is a total guess.
        Fmax = d_max/10

        # The minimal width of the valley.
        # Totally a guess at this time.
        Lmin = 5 

        # reset histogram.
        for i in range(len(self.hist)):
            self.hist[i]=d_max

        # Deposit weights from the grid to the hypot
        # rx,ry -- the position of the obstacle
        # d -- distance to the obstacle
        # a -- angle of the obtacle.
        # w -- the weight of the obstacle.
        # relative to the robot.
        for i in range(self.nRows):
            for j in range(self.nCols):
                if self.grid[i][j]>0.3:
                    x = j * self.dx + self.x0
                    y = i * self.dy + self.y0
                    rx = x - self.carX
                    ry = y - self.carY

                    d=hypot(rx,ry)
                    if d>0:
                        a=atan2(ry,rx)
                        da = atan(hypot(self.dx,self.dy)/2/d)

                        k0=int(ceil(self.angle2index(a-da)))
                        k1=int(floor(self.angle2index(a+da)))+1

                        for k in range(k0, k1):

                            if k >= self.nAngles:
                                k -= self.nAngles

                            if k < 0:
                                k+= self.nAngles

                            if (self.hist[k]>d):
                                self.hist[k]=d

        # Find the valleys
        # This array contains the array of the start of the
        # valley and its width.
        valleys = []
        i=0
        while i<self.nAngles:
            if self.hist[i] < Fmax:
                #print "Obstacle is too close: ", self.hist[i]
                i+=1
                continue

            n=1
            while self.hist[ (i+n) % self.nAngles ] > Fmax \
                  and n < self.nAngles:
                n+=1

            # print "found valley: [",i,", ",n,"]"
            if n>=Lmin:
                valleys.append([i,n])

            i+=n

        if len(valleys) == 0:
            print "No valleys found."
            print "          Fmax: ", Fmax
            print "          Lmin: ", Lmin
            print "Maximum Forces: ", max(self.hist)
            print "Minimum Forces: ", min(self.hist)
            for i in range(0, len(self.hist)):
                print "hist[",i,"] = ", self.hist[i]

            if True:
                return

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

        ObstacleSize = self.hist[int(round(self.angle2index(0)))]/Hmax
        ObstacleSize = min(1.0, ObstacleSize) 

        Speed = (Vmax-Vmin)*(1.0 - ObstacleSize) + Vmin

        self.speed = Speed
        self.turnAngle = TurnAngle

    ###########################################################################
    def getDescription (self):
        res = "size: %d %d\n" % ( self.nCols, self.nRows) 
        for row in range (self.nRows-1, -1, -1):
            res += "grid : " + ("%03d" % row) + " "
            for col in range (self.nCols):
                v=self.grid[row][col]
                if v>0.75:
                    res += "#"
                elif v<0.25:
                    res +="."
                else:
                    res +="+"
            res +="\n"

        res += "hist-angles: "
        for i in range(0, len(self.hist)):
           res += " %f" % (self.index2angle(i) /pi*180)
        res+="\n"

        res += "hist-values: "
        for i in range(0, len(self.hist)):
           res += " %f" % self.hist[i] 
        res+="\n"
        
        if len(self.scanAngles)>0:
            res += "scan-angles: "
            for i in range(0, len(self.scanAngles)):
               res += " %f" % self.scanAngles[i] 
            res+="\n"

            res += "scan-ranges: "
            for i in range(0, len(self.scanDists)):
               res += " %f" % self.scanDists[i] 
            res+="\n"
            
        res += "grid-coords: %f %f %f %f\n" % (self.x0, self.y0, self.dx, self.dy)
        res += "car-size : %f %f\n" % (self.carWidth, self.carLength)
        res += "car-pos : %f %f %f\n" % (self.carX, self.carY, self.carA)
        res += "velocity: %f %f\n" % (self.turnAngle, self.speed)
        res += "goal-dir : %f\n" % (self.goal_dir)
        res+="draw: all\n"

        return res
    # end def getDescription
    
# end class FineGrid
    
