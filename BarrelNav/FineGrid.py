
""" Simple occupancy-grid-based mapping. 

Author: David Gutow
Version: 9/2017
"""

from math import *
from random import *

###############################################################################
# Class Grid
###############################################################################

class FineGrid(object):
    """ 
    The Grid class stores an occupancy grid as a two dimensional array.
    Each cell in the grid can be thought of as a bin holding a
    probability to find an obstacle at a position.
    
    The origin (0,0) of the grid is considered to be the lower left
    corner with 'x' increasing to the right (increasing column index)
    and 'y' increasing going up (increasing row index).
   
    Angles are stored in degrees, and distances are stored in cm.  An angle
    is assumed to be an absolute angle in the world reference frame,
    something like what we might obtain from a compass.  and independent of
    the car rotation.
    
    Public instance variables:
        nCols      --  Number of columns in the occupancy grid.
        nRows      --  Number of rows in the occupancy grid.
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
        self.valleys = []
        self.minValleyAngle = 25
        self.numRandomSteps = 9
        self.maxRandomSteps = 10
        self.lastRandomSpeedSign = 1
        self.goodRange = 100
        self.minRange = 25

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

    def setMinValleyAngle(self, val):
        self.minValleyAngle = val

    def setMinRange(self, val):
        """ If a distance to an obstacle is this far
            than the robot can maneuver in front of it at
            a mininal speed. This is also the treshold in
            the histogram for finding valleys.
        """
        self.minRange = val

    def setGoodRange(self, val):
        """ If a distance to an obstacle is this far
            than the robot can maneuver in front of it easily
            going at full speed.
        """
        self.goodRange = val

    def setMaxSpeed(self, val):
        self.maxSpeed = val

    def setMinSpeed(self, val):
        self.minSpeed = val

    def reset(self):
        self.distance   = 0.0
        self.angle      = 0.0     
        self.carX       = 0
        self.carY       = 0
        self.carA       = 0.0
        self.x0=-self.nCols*self.dx/2
        self.y0=-self.nRows*self.dy/2
        self.grid = [[ 0.5 for x in range(self.nCols)]\
                for y in range(self.nRows)]    
        self.hist=[0.0 for a in range(self.nAngles) ]
        self.scanAngles = []
        self.scanDists = []

    def setCarPos(self, x, y):
        self.carX = x
        self.carY = y

    def setCarAngle(self, val):
        self.carA = val

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

        kx=cos(radians(angle))*dist
        ky=sin(radians(angle))*dist
       
        d=hypot(kx,ky)
        mx=-ky/d
        my= kx/d
       
        rw=tan(radians(self.rayWidth/2))*d

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
 
    
    def scroll(self, di, dj):
        old = self.grid
        self.grid = [ [ 0.0 for j in range(0, self.nCols) ] \
            for i in range(0, self.nRows) ]
        for i in range(0, self.nRows):
            for j in range(0, self.nCols):
                i1 = i+di
                j1 = j+dj
                if 0<=i1 and i1<self.nRows and \
                    0<=j1 and j1<self.nCols:
                    self.grid[i][j] = old[i1][j1]
                else:
                    self.grid[i][j] = 0.5
        self.x0 += dj * self.dx
        self.y0 += di * self.dy
            

    def recenter(self):
        dj = int(round( self.carX / self.dx))
        di = int(round( self.carY / self.dy))
        if fabs(di)>=2 or fabs(dj)>=2:
            self.scroll(di, dj)
    
    ###########################################################################
    # getValue   
    def getValue (self, xIndex, yIndex):
        return self.grid[xIndex][yIndex]
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
    
        # The minimal width of the valley.
        Lmin = int(ceil(self.minValleyAngle/(360.0/self.nAngles)))

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
        self.valleys = []
        i=0
        while i<self.nAngles:
            if self.hist[i] < self.minRange:
                #print "Obstacle is too close: ", self.hist[i]
                i+=1
                continue

            n=1
            while self.hist[ (i+n) % self.nAngles ] > self.minRange \
                  and n < self.nAngles:
                n+=1

            # print "found valley: [",i,", ",n,"]"
            if n>=Lmin:
                self.valleys.append([i,n])

            i+=n

        if len(self.valleys) == 0:
            print "No valleys found."
            print "     goodRange: ", self.goodRange
            print "     minRange: ", self.minRange
            print "          Lmin: ", Lmin
            print "Maximum Forces: ", max(self.hist)
            print "Minimum Forces: ", min(self.hist)
            for i in range(0, len(self.hist)):
                print "hist[",i,"] = ", self.hist[i]

            if True:
                return

    def chooseVelocityOld(self):
        # Find the best direction

        # First try to find a valley that contains the
        # goal direction. If there is such a valley,
        # then go in the middle of it.

        found = False
        k = (self.goal_dir/pi+1)/2*self.nAngles
        for v in self.valleys:
            if k>=v[0] and k<=v[0]+v[1]-1:
                found = True
                TurnAngle = self.index2angle(v[0] + v[1]/2.0)
                break

        # If there is no valley that contains the goal direction,
        # then try to find a valley that has an edge that would
        # be the closest match.

        TurnAngle = 0.0
        if not found:
            for v in self.valleys:
                edge = self.index2angle(v[0]) 
                if not found :
                    TurnAngle = edge
                    found = True
                elif fabs(TurnAngle-self.goal_dir) < fabs(edge-self.goal_dir):
                    TurnAngle = edge
                    edge = self.index2angle(v[0]+v[1]) 
                elif fabs(TurnAngle-self.goal_dir) < fabs(edge-self.goal_dir):
                     TurnAngle = edge


        ObstacleSize = self.hist[int(round(self.angle2index(0)))]/self.goodRange
        ObstacleSize = min(1.0, ObstacleSize) 

        self.speed = (self.maxSpeed-self.minSpeed)* \
                (1.0 - ObstacleSize) + self.minSpeed

        self.turnAngle = TurnAngle

    def chooseVelocity(self):

        # Find the best direction

        Lmin = int(ceil(self.minValleyAngle/2/(360.0/self.nAngles)))

        found = False
        best_score = 0
        for v in self.valleys:
            k = v[0]+Lmin
            while k <= v[0] + v[1] - Lmin:
                score = cos(self.index2angle(k)-self.goal_dir) * self.hist[k]
                if not found or score>best_score:
                    best_score = score
                    best_dir = k
                    print "best_dir = ", best_dir
                    print "best_score = ", best_score
                
                found = True
                k+=1

        if found:
            self.turnAngle = self.index2angle(best_dir)
            ObstacleSize = self.hist[best_dir]/self.goodRange
            ObstacleSize = min(1.0, ObstacleSize) 

            self.speed = (self.maxSpeed-self.minSpeed)* \
                    (1.0 - ObstacleSize) + self.minSpeed

        else:
            # Generate a direction randomly:
            print "no good direction found. Choosing random direction."
            self.numRandomSteps += 1
            if (self.numRandomSteps >= self.maxRandomSteps):
               self.numRandomSteps = 0
               self.turnAngle = random()*180-90
               self.speed = - self.minSpeed * self.lastRandomSpeedSign
               self.lastRandomSpeedSign = - self.lastRandomSpeedSign

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
    

