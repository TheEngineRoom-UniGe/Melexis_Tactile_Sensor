# -*- coding: utf-8 -*-
"""
Created on Thu May  5 14:39:05 2022

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
import Melexio_Base

########################################################################
port = 7 #enter the correct port number
########################################################################

print("Enabling connection to the board")
with Melexio_Base.MelexIO('COM'+str(port)) as Board: 
    print("Connection established")
    frequencyTck    = 40
    voltageAPPDac   = 4000
    voltageETMDac   = 4000
    voltageRSTDac   = 0
    delayTon        = 0
    delayToff       = 0
    BytesToReadout  = 6
    S2M_MODE        = 0x03

    print("ID of the board is:")
    print(Board.Get_IDN())
    
    print("Initailization of the board")
    Board.setTckFrequency(frequencyTck)
    Board.setVoltageEnterTM(voltageAPPDac)
    Board.setVoltageReset(voltageETMDac)
    Board.setVoltageApp(voltageRSTDac)
    Board.setDelaySupplyOn(delayTon)
    Board.setDelaySupplyOff(delayToff)
    Board.setNumberTDOBytes(BytesToReadout)                                          
    Board.setTckFrequency(frequencyTck)
    Board.setS2MMode(S2M_MODE)
    Board.setActiveDUT()
    
    pathToPattern =LibraryPath+'\\TactAxis_Seq_App.txt'
    Pat = Board.formatPattern(pathToPattern)
    dataReadout = Board.loadPattern(Pat,ack=False)  #set ack to True if there are some connectivity issue (might take more time)
    Board.flushInput()
    Board.flushOutput()
    
    print("Initialization done")
    print("Reading the buffers:")
    Res1 = Board.runPattern1(BytesToReadout)
    print("Pattern1")
    print(Res1[2])
    Board.close()
print("End")
