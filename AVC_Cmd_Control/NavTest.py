from OccupGrid import *
import zmq
import time

if __name__ == '__main__':

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    g = Grid(resolution=10, nCols=10, nRows=9, nAngles=72, distance=0, angle=0)
    g.setGoalDirection(0)
    g.enterRange (35, -10, 0,  10)  # Resulting point should be at (43.92, 34.47)
    g.enterRange (00,  0, 35, -10)  # Resulting point should be at (43.92, 34.47)    
    g.enterRange (35, -10, 20, 10)  # Resulting point should be at (43.92, 54.47)  
    g.enterRange (25,  10, 20, 10)  # Resulting point should be at (61.18, 43.41)       
    g.calcForceHist()

    # print(g.getDescription());
    while True:
        socket.send_string(g.getDescription())
        time.sleep(0.5);

    g.printGrid("\nAFTER POINTS ENTERED:") 
    
    g.recenterGrid(10, 0)           # All points should shift down by -10
    g.printGrid("\nAFTER RE-CENTERING (10,0):")   
    g.recenterGrid(20, 45)          # All points should shift by -7.1, -7.1
    g.printGrid("\nAFTER RE-CENTERING (10,45):")   
    
    g.clear (distance=0, angle= 0)  # Start with a fresh slate
    g.printGrid("\nAFTER CLEARING GRID:")       
    g.enterRange  (10, 0, 10, 0)    # Resulting point should be at (50.0, 20.0)   
    g.recenterGrid(20, 45)          # point should shift to (50-14.1), 20.0-14.1)
    
    g.printGrid("AFTER CLEARING AND RE-CENTERING (10,45):") 
