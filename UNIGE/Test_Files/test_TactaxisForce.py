# -*- coding: utf-8 -*-
"""
Created on Thu May  5 17:23:31 2022

@author: tls
"""

import sys
import os
from pathlib import Path
import time

cwdpath = Path(os.getcwd())
ParentPath = cwdpath.parent.absolute()
LibraryPath = str(ParentPath)+"/UNIGE/Library"
print(LibraryPath)
sys.path.append(LibraryPath)
import TactaxisForce

########################################################################
port = '/dev/ttyACM0' #enter the correct port number
########################################################################

print("Enabling connection to the board")
with TactaxisForce.TactaxisForce(str(port),pathDataset=str(ParentPath)+"/UNIGE/Data/Calib.csv") as Sensor:
    print("Connection established")
    while True:
        B,F = Sensor.getForce(dual=True)
        MeasString = "Magnetic field measured is: \nBx00 = {Bx00},\nBz00 = {Bz00},\nBx01 = {Bx01},\nBz01 = {Bz01},\nBx10 = {Bx10},\nBz10 = {Bz10},\nBx11 = {Bx11},\nBz11 = {Bz11}".format(Bx00 = B[0],Bz00 = B[1],Bx01 = B[2],Bz01 = B[3],Bx10 = B[4],Bz10 = B[5],Bx11 = B[6],Bz11 = B[7])
        print(MeasString)
        print("Temperature is :"+str(B[-2])+" and " + str(B[-1]))
        print("Force measured is: \nFx = {Fx},\nFy = {Fy},\nFz = {Fz}".format(Fx= F[0],Fy=F[1],Fz=F[2]))
    print("End")