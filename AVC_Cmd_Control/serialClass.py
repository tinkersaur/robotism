"""
 serialClass.py - All the serial port support routines

 
 Written by David Gutow 9/2017
"""

import time
import threading
import struct
from Queue           import Queue
from simulator       import commandQueue
from simulator       import usbQueue

###############################################################################
# class serialClass - serial port support
###############################################################################

class serialClass (object):
    #serialPort     = serial.Serial(port='/dev/ttyUSB', baudrate=115000)
    serialPortFlag  = True      # Use to kill the serial port thread   
        
    ########################################################################### 
    # __init__
    ###########################################################################
    def __init__(self, telemQueue):
    
        self.telemQueue = telemQueue
        self.serialPortFlag  = True 
        
        # start the telemetry thread and give it some time to start up
        serThread = threading.Thread (
            group=None, target=self.serialPortThread, name = "serialPortThread" )
        serThread.start()
        time.sleep (0.2)  
    # end __init__
    
    ########################################################################### 
    # SerialPortThread(state)
    ###########################################################################
    def serialPortThread(self):
        #serialPort = serial.Serial(port='/dev/ttyUSB', baudrate=115000)
        print ("SERIAL PORT THREAD starting loop")   
        cnt = 0
    
        while (self.serialPortFlag and cnt < 300):
            """
            data = serialPort.read(48)    # length of a telemetry message
            self.telemQueue.put(data)
            """
        
            # instead of getting data from serial port we'll get it from 
            # the simulators usbQueue
            while (not usbQueue.empty()):
                telemetryPkt = usbQueue.get_nowait()
                self.telemQueue.put(telemetryPkt)            
            # end while
        
            time.sleep (0.1)                # DAG turn off when we get real port       
            cnt += 1
        # end while
        print ("serialPortThread terminating")   
    #end def    
    
    ###########################################################################
    # sendCommand (command, param1, param2, param3)   
    ###########################################################################
    def sendCommand (self, commandChar, param1, param2, param3):
        # pack the command into a struct
        packedArray  = struct.pack('>hhhh', ord(commandChar), param1, param2, param3)
    
        # send it along
        """
        nbytes = serialPort.write(packedArray)    # length of a telemetry message    
        if nbytes != 8:
            flag an error
        # end
        """
        commandQueue.put(packedArray)  
    #end  
    
    ###########################################################################
    # killThread - stops the serial port thread  
    ########################################################################### 
    def killThread (self):
        self.serialPortFlag = False
    # end
    
# End class    
    
###############################################################################
    
if __name__ == "__main__":
    telemetryQueue  = Queue(40)
    usbQueue        = Queue(40)
    
    sc = serialClass(telemetryQueue)  
    time.sleep (5.0)
    sc.killThread()
    
# end
    
    
    
    
    
    
    
    