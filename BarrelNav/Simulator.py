class Simulator:

    def __init__(self):
        self.carX = 0
        self.carY = 0
        self.carWidth = 20
        self.carLength = 10
        self.steerAngle = 0
        self.speed = 0
        self.time_step = 0.1
        self.robotBrain = None
        self.nRows = 20
        self.nCols = 20
        self._realloc()

    def _realloc(self):
        self.grid = [ [ 0.0 for i in range(0,nCols) ] for j in range(0,nRows)] 
    
    def setCarPos(self, x, y):
        pass

    def setCarSize(self, width, length):
        pass

    def setCarVelocity(self, steer_angle, speed):
        pass

    def setTimeStep(self, time_step):
        pass

    def setRobotBrain(self, brain)
        pass

    def advanceTimeStep(self, dt):
        pass

    def loadWorld(self, file_name):
        pass

    def _parseLine(self, line):
        tokens = re.split(" +: +", line)
        if token[0] == 'size':
            pass
        elif token[0] = 'grid':
            pass


    def rayScan(self, file_name):
        pass
        #return dist

    def display(self):
        pass

    def simulate():
        # loop:
        #    advance physics
        #    let the robot make a decision
        #    display 
