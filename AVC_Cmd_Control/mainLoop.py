"""
 mainLoop.py 

 
 Written by David Gutow 8/2017
"""

import time
import threading
import struct

from vehicleState   import *
from Queue          import Queue
from simulator      import *


# Vehicle State holds everything we know about the current state of the vehicle
state           = vehicleState()

# TelemetryQueue is the queue to pass telemetry packets from the telemetry thread
telemetryQueue  = Queue(40)

# Serial port setup and support
#serialPort = serial.Serial(port='/dev/ttyUSB', baudrate=115000)
serialPortFlag  = True      # Use to control the serial port thread


############################################################################### 
# Initialize the entire system
 
def initializations():
    # Create the telemetry queue
    
    # start the telemetry thread and give it some time to start up
    t = threading.Thread(group=None, target=serialPortThread, name = "serialPortThread" )
    t.start()
    time.sleep (0.2)   
    
    # Create the image procesing queues (to/from)
    
    # Start the image processing task(s)
    
    # initialize robot state
    state.currMode.mode = Modes.INITIALIZATION
# end initializations   

############################################################################### 
# This is the main loop of the system.  It loops forever (or until the state
# becomes Modes.Terminate)

def mainLoop():
    loopCntr    = 0
    print ("mainLoop starting loop")
    
    while (state.currMode.mode != Modes.TERMINATE and loopCntr < 20):

        state.currMode.printMode (("Loop #%2d" % (loopCntr)))
        time.sleep (1.0)
        loopCntr = loopCntr + 1
        
        # Get all the telemetry msgs and parse into state structure
        get_telemetry()
        get_vision()
        # Now do all the state specific actions
        stateControl ()
        
    # end while
# end def
    
#
  
############################################################################### 
# stateControl - choose what to do depending on our current state

def stateControl ():
        if state.currMode.mode == Modes.NONE:
            state.currMode.mode = Modes.INITIALIZATION
            
        elif state.currMode.mode == Modes.INITIALIZATION:
            state.currMode.mode = Modes.WAIT_FOR_START   
            
        elif state.currMode.mode == Modes.WAIT_FOR_START:
            state.currMode.mode = Modes.RACE_STRAIGHT      
           
        elif state.currMode.mode == Modes.RACE_STRAIGHT:
            state.currMode.mode = Modes.RACE_CURVE       
            
        elif state.currMode.mode == Modes.RACE_CURVE:
            state.currMode.mode = Modes.NEGOT_CROSSING 
            
        elif state.currMode.mode == Modes.NEGOT_CROSSING:
            state.currMode.mode = Modes.APPR_STOPSIGN  
                      
        elif state.currMode.mode == Modes.APPR_STOPSIGN:
            state.currMode.mode = Modes.NEGOT_STOPSIGN   
            
        elif state.currMode.mode == Modes.NEGOT_STOPSIGN:
            state.currMode.mode = Modes.APPR_HOOP   
            
        elif state.currMode.mode == Modes.APPR_HOOP:
            state.currMode.mode = Modes.NEGOT_HOOP    
            
        elif state.currMode.mode == Modes.NEGOT_HOOP:
            state.currMode.mode = Modes.APPR_BARRELS     
            
        elif state.currMode.mode == Modes.APPR_BARRELS:
            state.currMode.mode = Modes.NEGOT_BARRELS  
            
        elif state.currMode.mode == Modes.NEGOT_BARRELS:
            state.currMode.mode = Modes.APPR_RAMP    
            
        elif state.currMode.mode == Modes.APPR_RAMP:
            state.currMode.mode = Modes.NEGOT_RAMP  
            
        elif state.currMode.mode == Modes.NEGOT_RAMP:
            state.currMode.mode = Modes.APPR_PED   
            
        elif state.currMode.mode == Modes.APPR_PED:
            state.currMode.mode = Modes.NEGOT_PED   
            
        elif state.currMode.mode == Modes.NEGOT_PED:
            state.currMode.mode = Modes.TERMINATE        
        # endif

# end def
   
################################################################################
# Serial Port support routines:    
################################################################################
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
   
################################################################################
# send_command(command, param1, param2, param3)   
   
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
        print ("GET TELEMETRY - received %d new pkts , time = " % (tlm_cnt, time))
    else:
        print "GET TELEMETRY - no new telemetry "       
# end    
   
################################################################################
# process_telemetry

def process_telemetry (data):
        # print "PARSE: Length of data is ", len(data)
        telemArray  = struct.unpack('>Lhhhhhhhhhhhhhhhhhhhhhh', data)
        
        Time        = telemArray[0]
        Mode        = telemArray[1]
        numAccepts  = telemArray[2]
        bist_estop  = telemArray[3]
        velocity    = telemArray[4]
        steerAngle  = telemArray[5]         
        cumDist     = telemArray[6]
        irLF_Dist   = telemArray[7]
        irLR_Dist   = telemArray[8]        
        irRF_Dist   = telemArray[9]
        irRR_Dist   = telemArray[10]
        switches    = telemArray[11]    
        irFr_Angle  = telemArray[12]
        irFr_InUse  = telemArray[13]
        irFr_Dist   = telemArray[14]
        battVolt1   = telemArray[15]
        battVolt2   = telemArray[16]
        Accel       = telemArray[17]
        RotRate     = telemArray[18]
        Compass     = telemArray[19]
        spare1      = telemArray[20]        
        spare2      = telemArray[21]
        spare3      = telemArray[22]   
        
        return Time
# end

################################################################################
# get_vision()

def get_vision():
    global telemetryQueue
    
    # Get the last message put onto the queue
    new_data = False
    # Keep looping until we get the last message put onto the queue    
    while (not telemetryQueue.empty()):
        msg = telemetryQueue.get_nowait()
        new_data = True
    # end while
    
    if new_data:
        time = parse_telemetry(msg)
        print "GET TELEMETRY - received new telemetry, time = ", time
    else:
        print "GET TELEMETRY - no new telemetry "       
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