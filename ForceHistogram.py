from math import *

class ForceHistogram:

    def __init__(self):
        self.setHistParam(72)
        self.gdir=0

    # Set the number of elemts in the histogram
    def setHistParam(self, na):
        self.na=na
        self.hist=[0.0 for x in range(self.na) ]


    def setGridParam(self, dx, dy, x0, y0):
        self.dx=dx
        self.dy=dy
        self.x0=x0
        self.y0=y0

    def setRobotPos(x, y):
        self.x=x
        self.y=y

    #

    def setGoalDirection(angle):
        self.gdir


    def setGrid(self, grid):
        self.grid=grid

    # angle in radians to index of the histogram:
    def angle2index(self, a):
        return int((a/pi+1)/2*self.na+0.5) % self.na 

    def index2angle(self, k):
        return ((k*1.0)/self.na-0.5)*2*pi

    # return [Speed, TurnAngle]
    def navigate:

        # precalculate some constants:
        ny=len(grid)
        nx=len(grid[0])
        w0=1.0
        // maximum distance to the obstacle.
        d_max=hypot(self.dx*nx/2, self.dy*ny)
        dw=1.0
        da = 2*pi / self.na
    
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
        for j in range(ny):
            for i in range(nx):
                rx = grid[i][j][0] - x
                ry = grid[i][j][1] - y

                a=atan2(y,x)
                d=hypot(x,y)

                k=self.angle2index(a)

                w=w0-dw*(d/d_max)
                self.hist[k]+=w

        # Smooth the histogram.

        # TODO: Do we need it??? 

        # Find the valleys
        # This array contains the array of the start of the
        # valley and its width.
        valleys = []
        for i in range(self.na):
            if self.hist[i]>Fmax:
                continue

            n=1
            while self.hist[ (i+j) % self.na ] > Fmax:
                n+=1

            if n>=Lmin:
                valleys.append([i,n])
                
        # Find the best direction

        # First try to find a valley that contains the
        # goal direction. If there is such a valley,
        # then go in the middle of it.

        found = False
        k = (self.gdir/pi+1)/2*self.na
        for v in valleys:
            if k>=v[0] && k<=v[0]+v[1]-1:
                found = True
                TurnAngle = self.index2angle(v[0] + v[1]/2.0)
                break

        # If there is no valley that contains the goal direction,
        # then try to find a valley that has an edge that would
        # be the closest match.

        TurnAngle = 0
        if (!found):
            for v in valleys:
                edge = self.index2angle(v[0]) 
                if (!found):
                    TurnAngle = edge
                    found = True
                elif fabs(TurnAngle-self.gdir) < \
                     fabs(edge-self.gdir):
                    TurnAngle = edge

                edge = self.index2angle(v[0]+v[1]) 
                elif fabs(TurnAngle-self.gdir) < \
                     fabs(edge-self.gdir):
                    TurnAngle = edge

        Vmax=1.0
        Vmin=0.1

        # Large obstacle. The value is a total guess at this point.
        Hmax = 5

        ObstacleSize = min(1.0, hist[angle2index(0)]/Hmax)

        Speed = (Vmax-Vmin)*(1.0 - ObstacleSize) + Vmin

        return [Speed, TurnAngle]
