"""
 statusServer.py 
 Simply sends the vehicle status every once in a while

 
 Written by David Gutow 8/2017
"""

import os
from socket         import *
# from mainLoop       import *
from vehicleState   import *

############################################################################## 
# class sendStatus - sends vehicle state every once in while
##############################################################################
class statusServer (object):
    host        = "192.168.0.1"     # IP addr of target puter
    port        = 13000             # Port number to send to
    addr        = (host, port)
    howOften    = 10
    
    ###########################################################################
    def __init__(self, host = "192.168.0.1", port = 13000, howOften = 10):
        self.howOften = howOften
        self.host     = host
        self.port     = port
        addr          = (host, port)
        
        UDPSock = socket(AF_INET, SOCK_DGRAM)
        #UDPSock.bind(addr)       
    # end __init__
    
    ###########################################################################
    def send(self, loopCntr, vehState):
        if (loopCntr % self.howOften == 0):
            data = self.assembleMsg(vehState)
            UDPSock.sendto(data, addr)     
        # end if
    # end def
    
     ###########################################################################
     # mode, timeSinceStart, timeAtStart, distAtStart, compAtStart, iopTime, iopMode,
     #      iopAcceptCnt, iopBistStatus, iopSpeed, iopSteerAngle, iopCumDist, iopStartSw,
     #      iopRanges.getRange(0), iopCompassAngle
    def assembleMsg(self, vs):   
        #       mode   timSS  timAS  dstAS  cmpAS  iopTm  iopMd iopAC iopBS 
        data1 = "%-20s, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %02d, %02s, 0x%2X, " % (
              vs.mode.toString(vs.mode.currMode), vs.timeSinceStart, vs.timeAtStart,
              vs.distAtStart, vs.compassAtStart, vs.iopTime, vs.iopMode, vs.iopAcceptCnt,
              vs.iopBistStats)
        return data1
    # end def
    
    
        
        
        
"""
# Save as server.py 
# Message Receiver
import os
from socket import *
host = ""
port = 13000
buf = 1024
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)
UDPSock.bind(addr)
print "Waiting to receive messages..."
while True:
    (data, addr) = UDPSock.recvfrom(buf)
    print "Received message: " + data
    if data == "exit":
        break
UDPSock.close()
os._exit(0)

# Save as client.py 
# Message Sender
import os
from socket import *
host = "127.0.0.1" # set to IP address of target computer
port = 13000
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)
while True:
    data = raw_input("Enter message to send or type 'exit': ")
    UDPSock.sendto(data, addr)
    if data == "exit":
        break
UDPSock.close()
os._exit(0)
"""    
       