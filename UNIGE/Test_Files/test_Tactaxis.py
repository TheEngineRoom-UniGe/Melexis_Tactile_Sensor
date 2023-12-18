# -*- coding: utf-8 -*-
"""
Created on Thu May  5 15:33:43 2022

@author: tls
"""

import sys
import os
from pathlib import Path
import time

cwdpath = Path(os.getcwd())
ParentPath = cwdpath.parent.absolute()
LibraryPath = str(ParentPath)+"\\Library"
sys.path.append(LibraryPath)
import Tactaxis

########################################################################
port = 10 #enter the correct port number
########################################################################

print("Enabling connection to the board")
with Tactaxis.Tactaxis('COM'+str(port)) as Sensor: 
    pathToPattern =LibraryPath+'\\TactAxis_Seq_App.txt'
    Pat = Sensor.formatPattern(pathToPattern)
    dataReadout = Sensor.loadPattern(Pat,ack=True)  #set ack to True if there are some connectivity issue (might take more time)
    time.sleep(1)
    Sensor.flushInput()
    Sensor.flushOutput()
    time.sleep(1)
    print("Connection established")
    B = Sensor.getValuesSingleChip()
    MeasString = "Magnetic field measured is \nBx00 = {Bx00},\nBz00 = {Bz00},\nBx01 = {Bx01},\nBz01 = {Bz01},\nBx10 = {Bx10},\nBz10 = {Bz10},\nBx11 = {Bx11},\nBz11 = {Bz11}".format(Bx00 = B[0],Bz00 = B[1],Bx01 = B[2],Bz01 = B[3],Bx10 = B[4],Bz10 = B[5],Bx11 = B[6],Bz11 = B[7])
    print(MeasString)
    print("Temperature is :"+str(B[-2])+" and " + str(B[-1]))
    
print("End")
