"""
 mainLoop.py - The main loop of the racing vehicle.
 
 Written by David Gutow 8/2017
"""

import time
import threading
import struct
from Queue           import Queue

from simulator       import *       # Only for the simulator

from vehicleState    import *       # Everything we know about the vehicle
from constants       import *       # Vehicle and course constants
from stateMachine    import stateMachine
from rangeSensorPair import rangeSensorPair
from raceModes       import raceModes
from rangeClass      import Range
from OccupGrid       import Grid
from serialClass     import serialClass


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
serialPort  = serialClass (telemetryQueue)

############################################################################### 
# Initialize the entire system
###############################################################################

def initializations():
    # Create the 
    # Create the telemetry queue
    
 
    
    # Create the image procesing queues (to/from)
    
    # Start the image processing task(s)
    
    # initialize robot state
    vehState.mode.currMode = raceModes.WAIT_FOR_BIST
    
    # Create the vehicle simulator
    startVehSimulator()
    time.sleep(2) 
    
# end initializations   

############################################################################## 
# This is the main loop of the system.  It loops forever (or until the state
# becomes Modes.Terminate)
##############################################################################

def mainLoop():
    loopCntr    = 0
    print ("mainLoop starting loop")
    
    while (vehState.mode.currMode != raceModes.TERMINATE and loopCntr < 50):   
        # Get all the telemetry msgs and parse into state structure
        get_telemetry()
        get_vision()
        
        # Now do all the state specific actions
        stateMachine (vehState, serialPort)
        
        # Let the iop know we're alive
        serialPort.sendCommand ('H', vehState.currHeartBeat, 0, 0)
        vehState.currHeartBeat += 1
        
        # Send the vehicle state every once in a while
        # sendStatus(loopCntr)
        
        time.sleep (0.5)
        loopCntr = loopCntr + 1         
        if loopCntr % 4 == 0:
            vehState.mode.printMode (("MAIN_LOOP #%2d" % (loopCntr)))        
    # end while
    
    # Kill the simulator by sending an unknown command
    serialPort.sendCommand ('Z', 0, 0, 0)    
# end def
      

    
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
        #global vehState
        
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
        vehState.iopCameraAngle = telemArray[20]        
        vehState.iopSpare2      = telemArray[21]
        vehState.iopSpare3      = telemArray[22]   
        
        # print "PROCESS_TELEM - Time %5d, Mode %1d, AccCnt %2d, Switch %2d/%2d" % (
            # vehState.iopTime, vehState.iopMode, vehState.iopAcceptCnt, 
            # vehState.iopSwitchStatus, vehState.iopStartSwitch)      
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
        pass #print "GET VISION - no new telemetry "       
# end    

###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":
    ##### TEST # 1 
    initializations()
    mainLoop()
    serialPort.killThread()
    
# end    