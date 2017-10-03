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


        