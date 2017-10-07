###############################################################################
# Test code
###############################################################################

import zmq
import time
import sys

from FineGrid import *

def publish(grid):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")
    while True:
        socket.send_string(grid.getDescription())
        time.sleep(0.5);

def test_setGridVal():
    g = FineGrid()
    g.reset()
    g.setGridVal(0, 0, 1.0)
    print g.getDescription()
    publish(g)
    
def test_setLineVal():
    g = FineGrid()
    g.reset()
    g.setLineVal(0.0, 0.0, 60, 120, 1.0)
    print g.getDescription()
    publish(g)


def test_enterScan():
    g = FineGrid()
    g.setGridSize(100, 100)
    g.setGridResolution(5)
    g.setCarSize(20,40)
    g.setRayWidth(10)
    g.reset()
    g.enterScan(10,150)
    g.enterScan(-20,80)
    #print g.getDescription()
    publish(g)


def test_calcForceHist():
    g = FineGrid()
    g.setGridSize(100, 100)
    g.setGridResolution(5)
    g.setRayWidth(10)
    g.setGoodRange(25)
    g.setMinSpeed(5)
    g.setMaxSpeed(60)
    g.reset()
    g.enterScan(-20,80)
    g.enterScan(10,150)
    g.enterScan(15,60)
    g.enterScan(25,55)
    g.enterScan(30,70)
    g.enterScan(35,75)
    g.enterScan(40,80)
    g.enterScan(45,130)
    g.calcForceHist()
    g.chooseVelocity()
    print g.getDescription()
    publish(g)


def test_calcForceHistRotated():
    g = FineGrid()
    g.setGridSize(100, 100)
    g.setGridResolution(5)
    g.setRayWidth(10)
    g.setGoodRange(25)
    g.setMinSpeed(5)
    g.setMaxSpeed(60)
    g.reset()
    g.setCarAngle(45)
    g.enterScan(-20,80)
    g.enterScan(10,150)
    g.enterScan(15,60)
    g.enterScan(25,55)
    g.enterScan(30,70)
    g.enterScan(35,75)
    g.enterScan(40,80)
    g.enterScan(45,130)
    g.calcForceHist()
    g.chooseVelocity()
    print g.getDescription()
    publish(g)


def test_recenter():
    g = FineGrid()
    g.setGridSize(100, 100)
    g.setGridResolution(5)
    g.setRayWidth(10)
    g.setGoodRange(25)
    g.setMinSpeed(5)
    g.setMaxSpeed(60)
    g.reset()
    g.setCarPos(40, 60)
    g.enterScan(-20,80)
    g.enterScan(10,150)
    g.enterScan(15,60)
    g.enterScan(25,55)
    g.enterScan(30,70)
    g.enterScan(35,75)
    g.enterScan(40,80)
    g.enterScan(45,130)
    g.calcForceHist()
    g.chooseVelocity()
    g.recenter()
    print g.getDescription()
    publish(g)

if __name__ == '__main__':
    #test_setGridVal()
    #test_setLineVal()
    #test_enterScan()
    # test_calcForceHistRotated()
    # test_calcForceHist()
    test_recenter()
