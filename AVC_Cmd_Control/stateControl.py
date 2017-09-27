"""
 stateControl.py 
 Called by mainLoop every 100 mSec.  Determines what need to happen in each
 state
 
 Written by David Gutow 9/2017
"""

    
def playSound (n):  
    print ("Playing Sound %d\n" % (n))
# end



from vehicleState import *  # We get vehState here

# The complete vehicle state structure.  
vehState = vehicleState()

simMaxCnt   = 20            # max count for use during simulations
InitMaxCnt  = 20            # Max time without an IOP msg before we signal a problem
BistMaxCnt  = 30            # Max time IOP is in BIST befoe we signal a problem

maxSpeed    = 100           # Maximum speed for use during normal racing
apprSpeed   = 50            # Speed to use when approaching an obstacle

############################################################################### 
# stateControl - choose what to do depending on our current state

def stateControl ():
    ################################################# Modes.NONE
    if vehState.mode.currMode == Modes.NONE:
        if vehState.mode.newMode():
            playSound (0)   
            
        if (vehState.iopMode == iopModes.NO_MODE):    
            # We haven't received anything from IOP yet
            if (vehState.mode.modeCount >= InitMaxCnt):
                vehState.errorString = "IOP NOT COMMUNICATING"
                vehState.mode.setMode (Modes.ERROR) 
            # end if           
        else:
            vehState.mode.setMode (Modes.WAIT_FOR_BIST)  
        # end if
        
    ################################################# Modes.WAIT_FOR_BIST        
    elif vehState.currMode.mode == Modes.WAIT_FOR_BIST:
        if vehState.mode.newMode():   
            playSound (1)     
         
        # Check if the IOP BIST is complete
        if (vehState.iopStatus != 0x00):
            if (vehState.mode.modeCount > BistMaxCnt):
                vehState.errorString = "IOP BIST FAILURE"
                vehState.mode.setMode (Modes.ERROR)  
        else:
            vehState.currMode.setMode(Modes.WAIT_FOR_START)       
        # end if             
        
    ################################################# Modes.WAIT_FOR_START       
    elif vehState.currMode.mode == Modes.WAIT_FOR_START:
        if vehState.mode.newMode():   
            playSound (2)   
            send_command ('D', 0, 0, 0)

        if (vehState.iopStartSwStatus):     # We're Off!
            vehState.currMode.setMode(Modes.RACE_STRAIGHT) 
            
    ################################################# Modes.RACE_STRAIGHT     
    elif vehState.currMode.mode == Modes.RACE_STRAIGHT:
        if vehState.mode.newMode():   
            playSound (3)   
            
        if (vehState.mode.modeCount >= simMaxCnt):    
            vehState.currMode.setMode(Modes.RACE_CURVE)   
        
    ################################################# Modes.RACE_CURVE         
    elif vehState.currMode.mode == Modes.RACE_CURVE:
        if vehState.mode.newMode():   
            playSound (4)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.NEGOT_CROSSING) 
        
    ################################################# Modes.NEGOT_CROSSING         
    elif vehState.currMode.mode == Modes.NEGOT_CROSSING:
        if vehState.mode.newMode():   
            playSound (5)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.APPR_STOPSIGN)  
        
    ################################################# Modes.APPR_STOPSIGN                   
    elif vehState.currMode.mode == Modes.APPR_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (6)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.NEGOT_STOPSIGN) 
        
    ################################################# Modes.NEGOT_STOPSIGN         
    elif vehState.currMode.mode == Modes.NEGOT_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (7)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.APPR_HOOP)   
        
    ################################################# Modes.APPR_HOOP        
    elif vehState.currMode.mode == Modes.APPR_HOOP:
        if vehState.mode.newMode():   
            playSound (8)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.NEGOT_HOOP)    
        
    ################################################# Modes.NEGOT_HOOP
    elif vehState.currMode.mode == Modes.NEGOT_HOOP:
        if vehState.mode.newMode():   
            playSound (9)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.APPR_BARRELS)   
        
    ################################################# Modes.APPR_BARRELS    
    elif vehState.currMode.mode == Modes.APPR_BARRELS:
        if vehState.mode.newMode():   
            playSound (10)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.NEGOT_BARRELS)  
        
    ################################################# Modes.NEGOT_BARRELS    
    elif vehState.currMode.mode == Modes.NEGOT_BARRELS:
        if vehState.mode.newMode():   
            playSound (4)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.APPR_RAMP)    
        
    ################################################# Modes.APPR_RAMP    
    elif vehState.currMode.mode == Modes.APPR_RAMP:
        if vehState.mode.newMode():   
            playSound (11)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.NEGOT_RAMP)  
        
    ################################################# Modes.NEGOT_RAMP    
    elif vehState.currMode.mode == Modes.NEGOT_RAMP:
        if vehState.mode.newMode():   
            playSound (12)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.APPR_PED)   
        
    ################################################# Modes.APPR_PED    
    elif vehState.currMode.mode == Modes.APPR_PED:
        if vehState.mode.newMode():   
            playSound (13)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.NEGOT_PED)   
        
    ################################################# Modes.NEGOT_PED   
    elif vehState.currMode.mode == Modes.NEGOT_PED:
        if vehState.mode.newMode():   
            playSound (14)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.TERMINATE)   
        
    ################################################# Modes.ERROR
    elif vehState.currMode.mode == Modes.ERROR:
        if vehState.mode.newMode():   
            playSound (15)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.currMode.setMode(Modes.ERROR)   
        
    ################################################# Modes.
    else:
        playSound (16)       
        vehState.mode.errorString = "UNRECOGNIZED MODE"
        vehState.currMode.setMode(Modes.ERROR)    
    # endif
    
# end def

###############################################################################
# Test code
###############################################################################
if __name__ == '__main__':
    stateControl ()
    
    
