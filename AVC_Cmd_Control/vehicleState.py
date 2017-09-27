# vehicleState.py
"""
 vehicleState.py 
 Class to store the current state of the robot
 Class to store all the possible run modes of the robot
 
 Written by David Gutow 8/2017
"""
from rangeSensorPair import rangeSensorPair

############################################################################### 
# an enumeration of the possible Modes of the vehicle
class Modes(object):

    ###########################################################################
    # The enumerations of each mode:
    NONE            = 0     # Not defined
    INITIALIZATION  = 1     # robot being initialized
    WAIT_FOR_START  = 2     # waiting for the start signal
    
    RACE_STRAIGHT   = 3     # racing in normal Modes
    RACE_CURVE      = 4     # racing around a curve
    NEGOT_CROSSING  = 5     # negotiating the intersection
    
    APPR_STOPSIGN   = 10    # spotted a stop sign
    NEGOT_STOPSIGN  = 11    # negotiating stop sign
    
    APPR_HOOP       = 20    # spotted the hoop
    NEGOT_HOOP      = 21    # negotiating the hoop
    
    APPR_BARRELS    = 30    # spotted the barrels
    NEGOT_BARRELS   = 31    # negotiating the barrels
    RECOV_BARRELS   = 32    # recovery from barrels
    
    APPR_RAMP       = 40    # spotted the ramp
    NEGOT_RAMP      = 41    # negotiating the ramp
    
    APPR_PED        = 50    # spotted the pedestrian
    NEGOT_PED       = 51    # negotiating the pedestrian
    
    WAIT_FOR_END    = 60    # waiting for signal of end of course
    NEGOT_END       = 61    # ending race
    
    TERMINATE       = 100   # 
    ERROR           = 101   # An error occured
    
    ###########################################################################    
    # The values stored in this class
    
    currMode        = NONE
    prevMode        = NONE  # The previous mode we were in
    modeCount       = 0     # Count of how long we've been in this mode
 
    ###########################################################################
    # setMode - Sets a new mode
    #   
    def setMode(self, mode):
        self.prevMode = self.currMode
        self.currMode = mode
        modeCount     = 0
    # end
        
        
    ###########################################################################
    # newMode - Returns true if we've just switched to a mode.  Also increments
    # the modeCount value each time it's called.
    #   
    def newMode(self):
        self.modeCount += 1      # First time around it'll be equal to '1'
        if self.modeCount <= 1:
            return True     # We are in a new mode
        # end
        return False
    # end
    
    ###########################################################################
    # toString - converts a mode to a string
    #     
    def toString(self, mode):
        if mode == self.NONE:
            return "NONE"
        elif mode == self.WAIT_FOR_BIST:
            return "WAIT_FOR_BIST"       
        elif mode == self.WAIT_FOR_START:
            return "WAIT_FOR_START"                
        elif mode == self.RACE_STRAIGHT:
            return "RACE_STRAIGHT"               
        elif mode == self.RACE_CURVE:
            return "RACE_CURVE"  
        elif mode == self.NEGOT_CROSSING:
            return "NEGOT_CROSSING"                       
        elif mode == self.APPR_STOPSIGN:
            return "APPR_STOPSIGN"     
        elif mode == self.NEGOT_STOPSIGN:
            return "NEGOT_STOPSIGN"              
        elif mode == self.APPR_HOOP:
            return "APPR_HOOP"      
        elif mode == self.NEGOT_HOOP:
            return "NEGOT_HOOP"                
        elif mode == self.APPR_BARRELS:
            return "APPR_BARRELS"   
        elif mode == self.NEGOT_BARRELS:
            return "NEGOT_BARRELS"    
        elif mode == self.RECOV_BARRELS:
            return "RECOV_BARRELS"     
        elif mode == self.APPR_RAMP:
            return "APPR_RAMP"    
        elif mode == self.NEGOT_RAMP:
            return "NEGOT_RAMP"               
        elif mode == self.APPR_PED:
            return "APPR_PED"     
        elif mode == self.NEGOT_PED:
            return "NEGOT_PED"         
        elif mode == self.WAIT_FOR_END:
            return "WAIT_FOR_END"     
        elif mode == self.NEGOT_END:
            return "NEGOT_END"                  
        elif mode == self.TERMINATE:
            return "TERMINATE"   
        elif mode == self.ERROR:
            return "ERROR"  
        else:           
            return "UNKNOWN"    
    # end toString
    
    def printMode(self, str):
        print ("%s currMode = %2d - %s" % (str, self.mode , self.toString(self.mode)))
    # end printMode
# end
 
###############################################################################
class iopModes (object):
    NO_MODE      = -1    # No mode yet
    BIST_MODE    =  0    # In BIST/RESET mode
    NORMAL_MODE  =  1    # In Normal mode
    ESTOP_MODE   =  2    # In emergency stop mode
# end class    
    
###############################################################################
class vehicleState (object):

    mode            = Modes()  # The current mode of the system
    
    timeSinceStart     = 0.0   # Seconds since the start signal arrived    
    timeAtStart        = 0.0   # Time when start signal arrived
    distAtStart        = 0.0   # Distance measured when start signal arrived
    compassAtStart     = 0.0   # Compass angle when start signal arrived
    
    # Telemetry coming from the IOP processor
    iopTime            = 0     # IOP current time
    iopMode            = iopModes.NO_MODE  # Mode of the IOP processor
    iopCmdAcceptCnt    = 0     # Number of accepted commands                               
    iopBistStatus      = 0xFF  # This is the IOP BIST/ESTOP word                                

    currSpeed          = 0.0   # Current speed (cm/sec)  
    currSteerAngle     = 0.0   # Current angle of steering
    distSinceStart     = 0.0   # Distance covered since start (cm)
     
    leftRangeSensors   = rangeSensorPair(50, 10, 0, 100, 500, False)
    rightRangeSensors  = rangeSensorPair(50, 10, 0, 100, 500, True)
    
    iopBumperStatus    = 0x00  # Bitfield of status of each bump switch
    iopStartSwStatus   = False # Start switch been pushed   
   
    iopFlSensorAngle   = 0.0   # Angle of front looking sensor
    iopFlSensorInUse   = 0     # Front looking sensor in use (1:20-150, 2:100-550)
    iopFlSensorRange   = 0     # Range reported by the front looking sensor
    
    iopBattVolt1       = 0.0   # Voltage of battery 1
    iopBattVolt2       = 0.0   # Voltage of battery 2
    
    iopAccelVert       = 0.0   # Value of the vertical accelerometer
    iopGyroHoriz       = 0.0   # Gyro value in horizontal plane
    iopCompassAngle    = 0.0   # Compass angle
    
    # These are the results of the two range sensors above
    leftWallAngle      = 0.0   # The angle of the vehicle (relative to left wall)
    leftWallDist       = 0.0   # The calculated distance to left wall
    rightWallAngle     = 0.0   # The angle of the vehicle (relative to right wall)    
    rightWallDist      = 0.0   # The calculated distance to right wall
    
    # Desires coming out of the wall follower controller
    dsrdSpeed          = 0.0   # Desired speed of vehicle
    dsrdWallAngle      = 0.0   # Desired angle of vehicle (relative to walls)
    dsrdLeftWallDist   = 0.0
    dsrdRightWallDist  = 0.0
    
    # Booleans indicating whether we passed/negotiated each obstacle
    finishedStopSign   = False # Completed the stop sign
    finishedPedestrian = False # Completed pedestrian
    finishedRamp       = False # Completed the ramp
    finishedHoop       = False # Completed the hoop
    
    # Heartbeat going to the IOP
    currHeartBeat      = 0
    
    # Bling
    currSoundInPlay    = 0
    currLightScene     = 0
    
    # Error handling
    errorString        = ""
    
    def __init__(self):
        self.mode.setMode(Modes.NONE)
        self.timeSinceStart  = 0.0 
        self.distSinceStart  = 0.0   
        self.currCompassDir  = 0.0  
    #end   
# end class    
###############################################################################
