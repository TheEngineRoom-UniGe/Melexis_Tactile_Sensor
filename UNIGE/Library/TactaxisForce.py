# -*- coding: utf-8 -*-
"""
Created on Mon May  2 17:04:24 2022

@author: tls
"""


from Tactaxis import Tactaxis
from forceComputation import forceComputation
import os
import numpy as np
from pathlib import Path
import time

class TactaxisForce():
    
    def __init__(self,port,pathDataset,activeDUT=[8,9]):
        print("Conneting to the sensor...")
        # self.Sensor = Tactaxis('COM'+str(port),activeDUT=activeDUT)
        self.Sensor = Tactaxis(port,activeDUT=activeDUT)
        path = Path(__file__).parent.absolute()
        pathToPattern =str(path)+'/TactAxis_Seq_App.txt'
        Pat = self.Sensor.formatPattern(pathToPattern)
        dataReadout = self.Sensor.loadPattern(Pat,ack=True) ## Set to True for safe mode
        time.sleep(1)
        self.Sensor.flushInput()
        self.Sensor.flushOutput()
        time.sleep(1)
        print("Sensor connected")
        print("Force computation initialization...")
        self.fC = forceComputation(pathDataset)
        print("Done")
            
    
    def getField(self,av=1):
        B = self.Sensor.getValuesSingleChip(av=av)
        return B
    
    def getForce(self,av=1,dual=False):
        Val = self.getField(av=av)
        BT = Val[:10]
        F = self.fC.InferenceForce(BT[:8])
        if dual:
            return BT,F
        return F
    
    def getMultForce(self,av=1,dual=False):
        Val = self.Sensor.getValuesMultiChip(av=av)
        F = np.zeros((len(Val),3))
        for i,v in enumerate(Val):
            F[i] = self.fC.InferenceForce(v[:8])
        if dual:
            return Val[:,:10],F
        return F
        
    def __enter__(self):
        self.Sensor.__enter__()
        return self
    
    def __exit__(self,etype, value, traceback):
        print("Closing the connection")
        self.Sensor.close()
    
        
    

