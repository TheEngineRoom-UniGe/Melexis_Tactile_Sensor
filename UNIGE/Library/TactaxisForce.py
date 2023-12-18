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
    
    def __init__(self,port,pathDataset,activeDUT=[8,9],ack=True,sfi=True): #activeDUT=[8,9] for single taxel, activeDUT=[0,1,2,4,5,8,9,10] for 4 taxel module
        print("Conneting to the sensor...")
        self.Sensor = Tactaxis(str(port),activeDUT=activeDUT)
        path = Path(__file__).parent.absolute()
        pathToPattern =str(path)+'/TactAxis_Seq_App.txt'
        Pat = self.Sensor.formatPattern(pathToPattern)
        dataReadout = self.Sensor.loadPattern(Pat,ack=ack)
        time.sleep(1)
        self.Sensor.flushInput()
        self.Sensor.flushOutput()
        time.sleep(1)
        print("Sensor connected")
        print("Force computation initialization...")
        self.fC = forceComputation(pathDataset,sfi=sfi)
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
    
    def getMultForceCust(self,av=1,dual=False):
        ValRaw = np.swapaxes(self.Sensor.getValuesmTAv(av=av),0,1)
        Val = np.zeros((4,10))
        MappingCust = np.array([[5,6],[2,7],[3,4],[1,0]])
        F = np.zeros((4,3))
        for k in range(4):
            Val[k][:4] = ValRaw[MappingCust[k][0]][:4]
            Val[k][4:8] = ValRaw[MappingCust[k][1]][:4]
            Val[k][8] = ValRaw[MappingCust[k][0]][4]
            Val[k][9] = ValRaw[MappingCust[k][1]][4]

        out = np.copy(Val)
        out[:,0] =    Val[:,6]
        out[:,1] =    Val[:,7]
        out[:,2] = -1*Val[:,4]
        out[:,3] = -1*Val[:,5]
        out[:,4] = -1*Val[:,2]
        out[:,5] =    Val[:,3]
        out[:,6] =    Val[:,0]
        out[:,7] = -1*Val[:,1]

        for k in range(4):
            F[k] = self.fC.InferenceForce(out[k][:8])
        if dual:
            return out,F
        return F
        
    def __enter__(self):
        self.Sensor.__enter__()
        return self
    
    def __exit__(self,etype, value, traceback):
        print("Closing the connection")
        self.Sensor.close()
    
        
    

