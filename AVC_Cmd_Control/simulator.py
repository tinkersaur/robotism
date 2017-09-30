#
"""
 simulator.py 
 A simple simulator for the AVC vehicle.  Used to test and debug
 the vehicle command & control code
 
 Written by David Gutow 9/2017
"""

import struct

from mainLoop   import *
from raceModes  import raceModes

# the internal states of the vehicle simulator
vehTimeCntr  = 0
vehNaccepts  = 0

# vehicle simulator stuff
serialPortNewCmd = False    # indicates a new command in the 'buffer'.  Flag
                            # set by send_command, and reset by the simulator.
                            
serialPortCmd    = ""       # holds the command string

################################################################################
# create_telemetry(state) 

def vehSimulator():
    global vehTimeCntr
    global vehNaccepts    
    global serialPortNewCmd    
    global serialPortCmd    
    
    # Check to see if we have a new command
    if (serialPortNewCmd):
        # Grab the command and parse
        cmdFields = struct.unpack('>hhhh', serialPortCmd)
        serialPortNewCmd = False
        parseCmdFields(cmdFields)
    # end
    
    # update the state of the simulator
    vehUpdateState()

    # Send out telemetry packet
    telemArray = createTelemetry(vehTimeCntr)
    #telemetryQueue.put(telemArray)   
    print "VehicleSimulator - returning telemetry with count ", vehTimeCntr
    return telemArray
# end    

################################################################################
# vehUpdateState 

def vehUpdateState():
    global vehTimeCntr
    global vehNaccepts  
    
    vehTimeCntr += 1
# end 
    
################################################################################
# parseCmdFields 

def parseCmdFields(cmdFields):
    print ("VehicleSimulator - received cmd ", chr(cmdFields[0]))
    
# end 
    
################################################################################
# create_telemetry(state)   

def createTelemetry(value):
    telemArray = []
    telemArray.append(value)    # 0 time 
    telemArray.append(1)        # 1 mode   
    telemArray.append(23)       # 2 naccepts   
    telemArray.append(0)        # 3 bist  
    telemArray.append(22)       # 4 vel   
    telemArray.append(10)       # 5 angle           
    telemArray.append(56)       # 6 cumDist   
    telemArray.append(200)      # 7 irLF   
    telemArray.append(200)      # 8 irLR           
    telemArray.append(200)      # 9 irRF  
    telemArray.append(200)      # 10 irRR  
    telemArray.append(0)        # 11 switches     
    telemArray.append(0)        # 12 irangle  
    telemArray.append(2)        # 13 iruse  
    telemArray.append(550)      # 14 irFront  
    telemArray.append(128)      # 15 batt1  
    telemArray.append(128)      # 16 batt2 
    telemArray.append(7)        # 17 accel  
    telemArray.append(89)       # 18 rot rate 
    telemArray.append(0)        # 19 compass  
    telemArray.append(0)        # 20 spare        
    telemArray.append(0)        # 21 spare           
    telemArray.append(0)        # 22 spare     
    
    # print "CREATETELEM: Length of telemArray is ", len(telemArray)
    
    packedArray  = struct.pack('>Lhhhhhhhhhhhhhhhhhhhhhh', *telemArray)
    # print "CREATETELEM: Length of packedArray is ", len(packedArray)
    return packedArray
# end
################################################################################
if __name__ == "__main__":
    ##### TEST # 1 
    serialPortNewCmd = False
    vehSimulator()
    vehSimulator()
    vehSimulator()    
# end 