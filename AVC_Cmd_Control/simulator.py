#
"""
 simulator.py 
 A simple simulator for the AVC vehicle.  Used to test and debug
 the vehicle command & control code
 
 Written by David Gutow 9/2017
"""

from Queue           import Queue
import threading
import struct
import time

#from mainLoop   import telemetryQueue

# the internal states of the vehicle simulator
simCurrTime     = 0
simCurrMode     = 0     # 0 - BIST, 1 - Normal, 2 - Estop
simAccCntr      = 0
simBist         = 0
simCurrSpeed    = 0
simCurrSteering = 0
simCumDist      = 0
simIrLF         = 250
simIrLR         = 200
simIrRF         = 200
simIrRR         = 250
simSwitches     = 0
simSensorAngle  = 0
simSensor       = 0
simSensorDist   = 0
simVolt1        = 0
simVolt2        = 0
simAccel        = 0
simGyro         = 0
simCompass      = 0
simCamAngle     = 0

simEstopReason  = 0     # 0-none 1-commanded 2-estop button 3-bump 4-heartbeat
simTimeLastHB   = 0     # Time since last heartbeat

simulatorFlag   = True  # Simulator Thread will die when this is false
    
# Command Queue is used to pass command packets to the simulator
commandQueue    = Queue(20)

# USB Queue used to mimic the USB communication line for telemetry
usbQueue        = Queue(50)

################################################################################
# Start the vehicle simulator thread
################################################################################
def startVehSimulator():
    sim = threading.Thread(group=None, target=vehSimulator, name = "Vehicle Simulator" )
    simulatorFlag   = True          # Thread will die when this is false
    sim.start()
# end

################################################################################
# The vehicle simulator thread
################################################################################

def vehSimulator():
    global simCurrTime
    global simCurrMode
    global simAccCntr
    global simBist
    global simCurrSpeed
    global simCurrSteering
    global simCumDist
    global simIrLF
    global simIrLR
    global simIrRF
    global simIrRR
    global simSwitches
    global simSensorAngle
    global simSensor
    global simSensorDist
    global simVolt1
    global simVolt2
    global simAccel
    global simGyro
    global simCompass
    global simCamAngle
    global simEstopReason
    global simTimeLastHB
    global simulatorFlag
    global commandQueue
    global usbQueue

    print ("------------> VEHICLE SIMULATOR - starting loop")   
    cmdCnt      = 0
    loopCnt     = 0
    sleepTime   = 100     # set simulator period (millisecs)
    
    while (simulatorFlag and loopCnt < 600):
        # update the state of the simulator
        vehUpdateState(sleepTime)    
    
        # Check to see if we have any new commands
        while (not commandQueue.empty()):
            cmd = commandQueue.get_nowait()
            processCommand(cmd)        
            cmdCnt += 1
        # end while
    
        # Send out telemetry packet but not at first
        if simCurrTime > 1500:
            telemArray = createTelemetry()
            if usbQueue.qsize() < 40:
                usbQueue.put_nowait(telemArray)   
        
        time.sleep (sleepTime/1000.0)   #Goodnight  
        loopCnt += 1
        
        if ((loopCnt % 50) == 0):   
            print "------------> VEHICLE SIMULATOR - looping ", loopCnt
    # end while
    
    print "------------> VEHICLE SIMULATOR - terminating"
# end    

################################################################################
# processCommand
################################################################################
def processCommand (cmd):
    global simCurrTime
    global simCurrMode
    global simAccCntr
    global simBist
    global simCurrSpeed
    global simCurrSteering
    global simCumDist
    global simIrLF
    global simIrLR
    global simIrRF
    global simIrRR
    global simSwitches
    global simSensorAngle
    global simSensor
    global simSensorDist
    global simVolt1
    global simVolt2
    global simAccel
    global simGyro
    global simCompass
    global simCamAngle
    global simEstopReason
    global simTimeLastHB
    global simulatorFlag
    global commandQueue
    global usbQueue
    
    # Grab the command and parse
    cmdFields = struct.unpack('>hhhh', cmd)
    command = chr(cmdFields[0])
    print "------------> VEHICLE SIMULATOR - received cmd ", command 

    simAccCntr += 1             # inc the accept counter
    
    # Execute the command....
    if command == 'M':          # move command
        if simCurrMode == 1:    # Must be in normal mode
            simCurrSpeed    = cmdFields[1]
            cmddist         = cmdFields[2]
        else:
            simAccCntr -= 1     # else don't inc acc counter 
        # end        
    elif command == 'T':        # Turn command
        if simCurrMode == 1:     # Must be in normal mode
            simCurrSteering =  cmdFields[1]
        else:
            simAccCntr -= 1     # else don't inc acc counter 
        # end          
    elif command == 'E':        # Estop
        simCurrMode = 2   
        simEstopReason = 1          
    elif command == 'H':        # heartbeat
        simAccCntr -= 1         # don't inc acc counter on heartbeat
        simTimeLastHB = simCurrTime    
    elif command == 'L':        # lightscene
        pass
    elif command == 'P':        # PIDs
        pass
    elif command == 'Q':        # PIDs
        pass
    elif command == 'D':        # Set Mode
        simCurrMode = cmdFields[1]
        if simCurrMode == 2:      # Commanded into estop?
            simEstopReason = 1
        # end   
    elif command == 'N':        # NOP
        simAccCntr -= 1  
    elif command == 'S':        # scan speed
        pass
    elif command == 'A':        # scan angles
        pass        
    elif command == 'V':        # set camera angle
        simCamAngle = cmdFields[1]
    elif command == 'C':        # set which scan sensor
        simSensor = cmdFields[1]       
    else:                       # unrecognized command
        simAccCntr   -= 1       # don't inc acc counter on unrecog command   
        simulatorFlag = False   # Kill the Simulator Thread        
    # end if
  
# end 
################################################################################
# vehUpdateState - Update all our dynamic state variables...

def vehUpdateState(sleepTime): 
    global simCurrTime
    global simCurrMode
    global simAccCntr
    global simBist
    global simCurrSpeed
    global simCurrSteering
    global simCumDist
    global simIrLF
    global simIrLR
    global simIrRF
    global simIrRR
    global simSwitches
    global simSensorAngle
    global simSensor
    global simSensorDist
    global simVolt1
    global simVolt2
    global simAccel
    global simGyro
    global simCompass
    global simCamAngle
    global simEstopReason
    global simTimeLastHB
    global simulatorFlag
    global commandQueue
    global usbQueue
    
    simCurrTime += sleepTime  
    simCumDist += simCurrSpeed * sleepTime/1000
    
    # Let's go through a scenario
    if simSwitches == 0 and simCurrTime > 5000:     # after 5 seconds start
        print "------------> VEHICLE SIMULATOR - setting START BUTTON "
        simSwitches = 1                             # start button pushed
# end 
    
################################################################################

# end 
    
################################################################################
# create_telemetry(state)   

def createTelemetry():   
    telemArray = []
    telemArray.append(simCurrTime)      # 0 CurrTime 
    telemArray.append(simCurrMode)      # 1 currModemode   
    telemArray.append(simAccCntr)       # 2 naccepts   
    telemArray.append(simBist)          # 3 bist  
    telemArray.append(simCurrSpeed)     # 4 curr speed   
    telemArray.append(simCurrSteering)  # 5 curr steeringangle           
    telemArray.append(simCumDist)       # 6 cumDist   
    telemArray.append(simIrLF)          # 7 irLF   
    telemArray.append(simIrLR)          # 8 irLR           
    telemArray.append(simIrRF)          # 9 irRF  
    telemArray.append(simIrRR)          # 10 irRR  
    telemArray.append(simSwitches)      # 11 switches     
    telemArray.append(simSensorAngle)   # 12 IR angle  
    telemArray.append(simSensor)        # 13 IR use  
    telemArray.append(simSensorDist)    # 14 IR Range  
    telemArray.append(simVolt1)         # 15 batt1  
    telemArray.append(simVolt2)         # 16 batt2 
    telemArray.append(simAccel)         # 17 accel  
    telemArray.append(simGyro)          # 18 rot rate 
    telemArray.append(simCompass)       # 19 compass  
    telemArray.append(simCamAngle)      # 20 camera vertical angle        
    telemArray.append(0)                # 21 spare           
    telemArray.append(0)                # 22 spare     
    
    # print "CREATETELEM: Length of telemArray is ", len(telemArray)
    
    packedArray  = struct.pack('>Lhhhhhhhhhhhhhhhhhhhhhh', *telemArray)
    # print "vehSimulator:CREATETELEM - Length of packedArray is ", len(packedArray)
    return packedArray
# end

################################################################################
if __name__ == "__main__":
    startVehSimulator()  
# end 