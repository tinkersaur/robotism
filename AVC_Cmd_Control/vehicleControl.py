"""
 vehicleControl.py 
 Called by mainLoop every 100 mSec.  Controls the vehicle during the various 
 raceModes
 
 Written by David Gutow 9/2017
"""

 


from mainLoop       import *  
from raceModes      import raceModes

############################################################################### 
# normalVehicleControl
###############################################################################
def normalVehicleControl (leftWallDist, vehicleAngle):
    
# end normalVehicleControl


############################################################################### 
# withinTol - helper function. Returns true if the two values are within the
# given tolerance.
###############################################################################
def withinTol (value1, value2, tolerance):
    if ( value1 > (value2 - tolerance) and value1 < (value2 + tolerance) ):
        return True
    return False
# end within

############################################################################### 
# getVehPosition - get vehicle position using the two IR range sensor pairs
###############################################################################
def getVehPosition ():
    # If both sensor pairs report valid ranges and their angles correspond
    # and their distances add up then use both sets of data
    leftVehAngle    = rangeLeftPair.getPlatformAng()
    leftVehDist     = rangeLeftPair.calcDistFront  
    leftDataValid   = rangeLeftPair.dataValid
    
    rightVehAngle   = rangeRightPair.getPlatformAng()
    rightVehDist    = rangeRigthPair.calcDistFront
    rightDataValid  = rangerightPair.dataValid
    
    if (leftDataValid and rightDataValid):
        if (withinTol (leftVehAngle,rightVehAngle, 5)):
            if (withinTol (leftVehDist + rightVehDist + rsLRspacing, )


