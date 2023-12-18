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
LibraryPath = str(ParentPath)+"\\Library"
sys.path.append(LibraryPath)
import TactaxisForce

########################################################################
port = 56 #enter the correct port number
########################################################################

print("Enabling connection to the board")
with TactaxisForce.TactaxisForce(str(port),activeDUT=[0,1,2,4,5,8,9,10],pathDataset=str(ParentPath)+"\\Data\\Calib.csv",ack=True) as Sensor:
    print("Connection established")
    Btot,Ftot = Sensor.getMultForceCust(dual=True)
    for k in range(4):
        B = Btot[k]
        F = Ftot[k]
        print(f"Sensor {k}")
        MeasString = "Magnetic field measured is: \nBx00 = {Bx00},\nBz00 = {Bz00},\nBx01 = {Bx01},\nBz01 = {Bz01},\nBx10 = {Bx10},\nBz10 = {Bz10},\nBx11 = {Bx11},\nBz11 = {Bz11}".format(Bx00 = B[0],Bz00 = B[1],Bx01 = B[2],Bz01 = B[3],Bx10 = B[4],Bz10 = B[5],Bx11 = B[6],Bz11 = B[7])
        print(MeasString)
        print("Temperature is :"+str(B[-2])+" and " + str(B[-1]))
        print("Force measured is: \nFx = {Fx},\nFy = {Fy},\nFz = {Fz}".format(Fx= F[0],Fy=F[1],Fz=F[2]))
        print("")
    print("End")
