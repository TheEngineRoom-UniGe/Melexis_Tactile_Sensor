# -*- coding: utf-8 -*-
"""
Created on Thu May  5 14:37:07 2022

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
import USB_VCP

########################################################################
port = 18 #enter the correct port number
########################################################################

print("Enabling connection to the board")
with USB_VCP.USB_VCP('COM'+str(port)) as Board: 
    print("Connection established")
    Board.sendCMD("CMD_IDN")
    Id = Board.readDataTimeOut()
    print("ID of the board is: "+str(Id[1]))