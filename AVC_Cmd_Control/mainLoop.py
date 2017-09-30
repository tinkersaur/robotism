"""
 mainLoop.py 

 
 Written by David Gutow 8/2017
"""

import time
import threading
import struct
from Queue           import Queue

from simulator       import *

from vehicleState    import *
from stateControl    import stateControl
from rangeSensorPair import rangeSensorPair
from raceModes       import raceModes
from rangeClass      import Range
from OccupGrid       import Grid

###############################################################################
# Constants used throughout the code

""" What we know abut the track
"""
trWidth         = (16 * 12 * 2.54)      # Width between walls - 16 feet (cm)
trLength        = 13116                 # Approx length of track (cm)

trObstacle1     = obstacle.PEDESTRIAN   # The order we will face the obstacles
trObstacle2     = obstacle.STOPSIGN
trObstacle3     = obstacle.BARRELS
trObstacle4     = obstacle.HOOP
trObstacle5     = obstacle.RAMP 

""" Occupancy Grid constants:  We need to represent an area at least 52 feet wide 
(2 * 16 feet for the track and an additional 2 * 10 feet for the track veering
off to the right/left during turns).  The 16 feet track width is included twice 
since we always keep the car centered in the X center of the grid (Xcenter,0), 
and then the car can be either all the way to the right or all the way to left.  
The height of the map corresponds to 16 feet high, the max range of the sensor. 
Since the grid cells are only bins we can set the resolution value fairly large.
"""
ogResolution    = 15    # Size of each cell in the occupancy grid (cm)
ogNcols         = 104   # At 15 cm (6") this corresponds to 52 feet wide
ogNrows         = 32    # At 15 cm cells this is 16 feet (the range of the sensors)
ogStartDist     = 0.0   # Initial cumulative distance when system intitialized
ogStartAngle    = 0.0   # Initial angle of the vehicle when system initialized

""" Steering control constants:
"""



""" The two rangeSensorPair constants:  
Note - all angles are the angle of the sensors from a line perpindicular  
to the vehicle sides.
"""
rsLeftFrontAng  = 30    # The angle of the left front sensor (deg)
rsLeftRearAng   = 5     # The angle of the left rear sensor (deg)
rsRightFrontAng = 30    # The anlge of the right front sensor (deg)
rsRightRearAng  = 5     # The angle of the right rear sensor (deg)
rsMinDistance   = 100   # The min distance accepted (cm)
rsMaxDistance   = 500   # The max distance accepted (cm)
rsLeftSide      = False # Used to distinguish the left side sensor pair
rsRigthSide     = True  # Used to distinguish the left side sensor pair
rsFRspacing     = 20    # The spacing between the front and rear sensors (cm)
rsLRspacing     = 15    # The spacing between the left and right sensor pairs

""" The various speed values:  
"""
speedMax        = 100   # Maximum speed we'll ever go
speedApproach   = 50    # The speed we'll approach obstacle with
speedHoop       = 50    # The speed we'll negotiate the hoop obstacle
speedRamp       = 60    # The speed we'll jump the ramp
speedPed        = 40    # The speed we'll negotiate the pedestrian
speedBarrels    = 30    # The speed we'll negotiate the barrels
speedMin        = 20    # The minimum speed (except zero) we'll ever go
speedZero       = 0     # Stopped

###############################################################################

# Vehicle State holds everything known about the current vehicle state
vehState        = vehicleState()

# The two side IT range sensor pairs
rangeLeftPair   = rangeSensorPair(initFrontAng   = rsLeftFrontAng, 
                                  initRearAng    = rsLeftRearAng, 
                                  initSensorDist = rsFRspacing, 
                                  initMinDist    = rsMinDistance,
                                  initMaxDist    = rsMaxDistance, 
                                  rightSide      = rsLeftSide)
                                  
rangeRightPair  = rangeSensorPair(initFrontAng   = rsRightFrontAng, 
                                  initRearAng    = rsRightRearAng, 
                                  initSensorDist = rsFRspacing, 
                                  initMinDist    = rsMinDistance,
                                  initMaxDist    = rsMaxDistance, 
                                  rightSide      = rsRigthSide)                                  

# The vehicle occupancy grid
occGrid       = Grid (ogResolution, ogNrows, ogNcols, ogStartDist, ogStartAngle)

# TelemetryQueue is used to pass telemetry packets from the serial port thread
telemetryQueue  = Queue(40)

# VisionQueue is the queue to pass packets from the image processing task
visionQueue     = Queue(40)

# Serial port setup and support
#serialPort = serial.Serial(port='/dev/ttyUSB', baudrate=115000)
serialPortFlag  = True      # Use to kill the serial port thread

############################################################################### 
# Initialize the entire system
###############################################################################

def initializations():
    # Create the telemetry queue
    
    # start the telemetry thread and give it some time to start up
    t = threading.Thread(group=None, target=serialPortThread, name = "serialPortThread" )
    t.start()
    time.sleep (0.2)   
    
    # Create the image procesing queues (to/from)
    
    # Start the image processing task(s)
    
    # initialize robot state
    vehState.mode.currMode = raceModes.WAIT_FOR_BIST
# end initializations   

############################################################################## 
# This is the main loop of the system.  It loops forever (or until the state
# becomes Modes.Terminate)
##############################################################################

def mainLoop():
    loopCntr    = 0
    print ("mainLoop starting loop")
    
    while (vehState.mode.currMode != raceModes.TERMINATE and loopCntr < 20):

        vehState.mode.printMode (("MAIN_LOOP #%2d" % (loopCntr)))
        time.sleep (2.0)
        loopCntr = loopCntr + 1
        
        # Get all the telemetry msgs and parse into state structure
        get_telemetry()
        get_vision()
        # Now do all the state specific actions
        stateControl ()
        
        # Let the iop know we're alive
        send_command(ord ('H'), currHeartBeat, 0, 0)
        vehState.currHeartBeat += 1
        
        
    # end while
# end def
   
###############################################################################
# Serial Port support routines:    
###############################################################################
# SerialPortThread(state)

def serialPortThread():
    #serialPort = serial.Serial(port='/dev/ttyUSB', baudrate=115000)
    print ("serialPortThread starting loop")   
    cnt = 0
    
    while (serialPortFlag and cnt < 30):
        """
        data = serialPort.read(48)    # length of a telemetry message
        telemetryQueue.put(data)
        """
        
        # instead of getting data from serial port we'll call the simulator
        time.sleep (0.7)                # DAG turn off when we get real port        
        data = vehSimulator()  
        telemetryQueue.put(data)
        
        cnt += 1
    # end while
    print ("serialPortThread terminating")   
#end def
   
###############################################################################
# send_command(command, param1, param2, param3)   
###############################################################################

def send_command(commandChar, param1, param2, param3):
    # pack the command into a struct
    packedArray  = struct.pack('>hhhh', ord(command), param1, param2, param3)
    
    # send it along
    """
    nbytes = serialPort.write(packedArray)    # length of a telemetry message    
    if nbytes != 8:
        flag an error
    # end
    
    """
    if not serialPortNewCmd:
        serialPortCmd = packedArray
        serialPortNewCmd = True
    # end   
#end
    
################################################################################
# get_telemetry(state)
################################################################################

def get_telemetry():
    global telemetryQueue
    
    # Get the last message put onto the queue
    tlm_cnt = 0
    # Keep looping until we get the last message put onto the queue    
    while (not telemetryQueue.empty()):
        msg = telemetryQueue.get_nowait()
        time = process_telemetry(msg)        
        tlm_cnt += 1
    # end while
    
    if (tlm_cnt > 0):
        print ("GET TELEMETRY - received %d new pkts , time = %d" % (tlm_cnt, time))
    else:
        print "GET TELEMETRY - no new telemetry "       
# end    
   
################################################################################
# process_telemetry
################################################################################

def process_telemetry (data):
        # print "PARSE: Length of data is ", len(data)
        telemArray  = struct.unpack('>Lhhhhhhhhhhhhhhhhhhhhhh', data)
        
        vehState.iopTime        = telemArray[0]
        vehState.iopMode        = telemArray[1]
        vehState.iopAcceptCnt   = telemArray[2]
        vehState.iopBistStatus  = telemArray[3]
        vehState.iopSpeed       = telemArray[4]
        vehState.iopSteerAngle  = telemArray[5]         
        vehState.iopCumDistance = telemArray[6]
        
        # Enter the side IR sensors into the two rangeSensorPairs
        irLF_Range              = telemArray[7]
        irLR_Range              = telemArray[8]        
        irRF_Range              = telemArray[9]
        irRR_Range              = telemArray[10]
        
        vehState.iopSwitchStatus= telemArray[11]  
        vehState.iopStartSwitch = telemArray[11] & 0x01 
        
        measScanAngle           = telemArray[12]
        measScanSensor          = telemArray[13]
        measScanDist            = telemArray[14]

        vehState.iopBattVolt1   = telemArray[15]
        vehState.iopBattVolt2   = telemArray[16]
        vehState.iopAccelVert   = telemArray[17]
        vehState.iopGyroHoriz   = telemArray[18]
        vehState.iopCompAngle   = telemArray[19]
        vehState.iopSpare1      = telemArray[20]        
        vehState.iopSpare2      = telemArray[21]
        vehState.iopSpare3      = telemArray[22]   
        
        #######################################################################
        # Let's make sure we're always operating on compass angles which range
        # -180 to +180, rather than 0 to 360 degrees.
        if (vehState.iopCompAngle > 180):
            vehState.iopCompAngle -= 360

        #######################################################################  
        # Enter the scanner info into the iopRanges buffer
        vehState.iopRanges.enterRange(time   = vehState.iopTime, 
                                      angle  = measScanAngle, 
                                      ranger = measScanSensor, 
                                      range  = measScanDist) 
                                      
        #######################################################################                                      
        # Enter the scanner info into the occupGrid - DAG should we 
        # enter the data if we are using the short range sensor?
        occGrid.enterRange ( carCumDist   = vehState.iopCumDistance, 
                             carCurrAngle = vehState.iopCompAngle, 
                             scanDist     = measScanDist, 
                             scanAngle    = measScanAngle)       
                             
        #######################################################################        
        # Enter the side IR sensor data into the two rangeSensorPairs
        rangeLeftPair.newMeasurement (measFrontRange = irLF_Range, 
                                      measRearRange  = irLR_Range)                                     
        rangeRightPair.newMeasurement(measFrontRange = irRF_Range, 
                                      measRearRange  = irRR_Range)        
        
        return vehState.iopTime
# end

################################################################################
# get_vision()
################################################################################

def get_vision():
    global visionQueue
    
    # Get the last message put onto the queue
    new_data = False
    # Keep looping until we get the last message put onto the queue    
    while (not visionQueue.empty()):
        msg = visionQueue.get_nowait()
        new_data = True
    # end while
    
    if new_data:
        #time = parse_telemetry(msg)
        print "GET VISION - received new telemetry, time = ", time
    else:
        print "GET VISION - no new telemetry "       
# end    

###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":
    ##### TEST # 1 
    initializations()
    mainLoop()
    serialPortFlag = False
    
# end    